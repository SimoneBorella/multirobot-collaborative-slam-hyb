#include "mapping.h"


namespace multirobot_slam
{
    Mapping::Mapping()
        : map_updated_(true), ewfd_first_(true), frontier_map_updated_(true), mapping_thread_running_(false)
    {
    }

    Mapping::Mapping(MappingParams &params)
        : params_(params), map_updated_(true), ewfd_first_(true), frontier_map_updated_(true), mapping_thread_running_(false)
    {
    }

    Mapping::~Mapping()
    {
        mapping_thread_running_.store(false);
        if (mapping_thread_.joinable())
        {
            mapping_thread_.join();
        }
    }


    MappingParams Mapping::params_from_yaml(std::string &params_path)
    {
        MappingParams p;

        try
        {
            YAML::Node config = YAML::LoadFile(params_path);

            if (config["mapping_rate"])
                p.mapping_rate = config["mapping_rate"].as<double>();

            if (config["map_resolution"])
                p.map_resolution = config["map_resolution"].as<double>();
            if (config["map_width"])
                p.map_width = config["map_width"].as<double>();
            if (config["map_height"])
                p.map_height = config["map_height"].as<double>();

            if (config["free_belief"])
                p.free_belief = config["free_belief"].as<double>();
            if (config["occ_belief"])
                p.occ_belief = config["occ_belief"].as<double>();
            if (config["distance_belief_factor"])
                p.distance_belief_factor = config["distance_belief_factor"].as<double>();

            if (config["noise_radius"])
                p.noise_radius = config["noise_radius"].as<double>();
            if (config["noise_std_dev"])
                p.noise_std_dev = config["noise_std_dev"].as<double>();

            if (config["log_odds_min"])
                p.log_odds_min = config["log_odds_min"].as<double>();
            if (config["log_odds_max"])
                p.log_odds_max = config["log_odds_max"].as<double>();

            if (config["obstacle_threshold"])
                p.obstacle_threshold = config["obstacle_threshold"].as<double>();
            if (config["free_threshold"])
                p.free_threshold = config["free_threshold"].as<double>();
            if (config["frontier_del_obstacles_radius"])
                p.frontier_del_obstacles_radius = config["frontier_del_obstacles_radius"].as<double>();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
            std::cerr << "Using default parameters.\n";
        }

        return p;
    }


    void Mapping::init(MappingParams &params)
    {
        params_ = params;

        free_belief_log_odds_ = std::log(params_.free_belief / (1.0 - params_.free_belief));
        occupied_belief_log_odds_ = std::log(params_.occ_belief / (1.0 - params_.occ_belief));

        // Initialize maps
        map_.resolution = static_cast<float>(params_.map_resolution);
        map_.width = static_cast<int>(params_.map_width / params_.map_resolution);
        map_.height = static_cast<int>(params_.map_height / params_.map_resolution);
        map_.origin_position = Eigen::Vector3d(-params_.map_width / 2.0, -params_.map_height / 2.0, 0.0);
        map_.origin_orientation = Eigen::Quaterniond::Identity();
        map_.data.assign(map_.width * map_.height, -1);

        map_log_odds_update_.resolution = map_.resolution;
        map_log_odds_update_.width = map_.width;
        map_log_odds_update_.height = map_.height;
        map_log_odds_update_.origin_position = map_.origin_position;
        map_log_odds_update_.origin_orientation = map_.origin_orientation;

        map_log_odds_data_.assign(map_.width * map_.height, 0.0);

        filtered_map_.resolution = map_.resolution;
        filtered_map_.width = map_.width;
        filtered_map_.height = map_.height;
        filtered_map_.origin_position = map_.origin_position;
        filtered_map_.origin_orientation = map_.origin_orientation;
        filtered_map_.data.resize(map_.width * map_.height, -1);

        frontier_map_.resolution = map_.resolution;
        frontier_map_.width = map_.width;
        frontier_map_.height = map_.height;
        frontier_map_.origin_position = map_.origin_position;
        frontier_map_.origin_orientation = map_.origin_orientation;
        frontier_map_.data.resize(map_.width * map_.height, -1);

        frontier_map_update_.resolution = map_.resolution;
        frontier_map_update_.width = map_.width;
        frontier_map_update_.height = map_.height;
        frontier_map_update_.origin_position = map_.origin_position;
        frontier_map_update_.origin_orientation = map_.origin_orientation;

        ewfd_visited_.assign(map_.width * map_.height, false);

    }

    void Mapping::start()
    {
        if (mapping_thread_running_)
            return;

        mapping_thread_running_.store(true);

        mapping_thread_ = std::thread([this]()
                                     {
                auto period = std::chrono::milliseconds(
                    static_cast<int>(1000.0 / params_.mapping_rate));

                auto next_time = std::chrono::steady_clock::now() + period;

                while (mapping_thread_running_.load())
                {
                    mapping();

                    std::this_thread::sleep_until(next_time);
                    next_time += period;
                } });
    }

    void Mapping::add_posed_scan(const PosedScan &posed_scan)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        posed_scan_buffer_.push_back(posed_scan);
    }

    Map Mapping::get_map()
    {
        return map_;
    }

    std::optional<Map> Mapping::get_map_if_updated()
    {
        if (map_updated_)
        {
            map_updated_ = false;
            return map_;
        }
        else
        {
            return std::nullopt;
        }
    }

    Map Mapping::get_frontier_map()
    {
        return frontier_map_;
    }

    std::optional<Map> Mapping::get_frontier_map_if_updated()
    {
        if (frontier_map_updated_)
        {
            frontier_map_updated_ = false;
            return frontier_map_;
        }
        else
        {
            return std::nullopt;
        }
    }

    std::optional<MapLogOddsUpdate> Mapping::get_map_log_odds_update()
    {
        std::lock_guard<std::mutex> lock(map_log_odds_update_mutex_);

        if(map_log_odds_update_.indices.empty())
            return std::nullopt;

        MapLogOddsUpdate map_log_odds_update_copy = map_log_odds_update_;
        map_log_odds_update_.indices.clear();
        map_log_odds_update_.delta_log_odds.clear();

        return map_log_odds_update_copy;
    }


    std::optional<FrontierMapUpdate> Mapping::get_frontier_map_update()
    {
        std::lock_guard<std::mutex> lock(frontier_map_update_mutex_);

        if(frontier_map_update_.frontier_indices.empty() && frontier_map_update_.explored_indices.empty())
            return std::nullopt;

        FrontierMapUpdate frontier_map_update_copy = frontier_map_update_;
        frontier_map_update_.frontier_indices.clear();
        frontier_map_update_.explored_indices.clear();

        return frontier_map_update_copy;
    }

    double Mapping::probability_to_log_odds(int8_t prob)
    {    
        double p = static_cast<double>(prob) / 100.0;
        
        p = std::clamp(p, 0.01, 0.99);
        
        return std::log(p / (1.0 - p));
    }

    int8_t Mapping::log_odds_to_probability(double log_odds)
    {
        double p = 1.0 / (1.0 + std::exp(-log_odds));
        int8_t prob = static_cast<int8_t>(std::round(p * 100.0));

        if (prob <= 0) prob = 0;
        if (prob >= 100) prob = 100;

        return prob;
    }



    void Mapping::bresenham_raytrace(int x0, int y0, int x1, int y1, bool hit_point)
    {
        int robot_x = x0;
        int robot_y = y0;

        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true)
        {
            if (x0 == x1 && y0 == y1)
                break;

            if (x0 >= 0 && x0 < map_.width && y0 >= 0 && y0 < map_.height)
            {
                int idx = y0 * map_.width + x0;
                // Free cell
                if (map_.data[idx] == -1)
                {
                    map_.data[idx] = 50;
                    map_log_odds_data_[idx] = probability_to_log_odds(50);
                }

                double cell_distance = std::hypot(x0 - robot_x, y0 - robot_y) * map_.resolution;

                double delta_log_odds = free_belief_log_odds_ * (1.0 - params_.distance_belief_factor * cell_distance);
                local_log_odds_delta_[idx] += delta_log_odds;
                // map_log_odds_data_[idx] += delta_log_odds;
                
                // map_log_odds_data_[idx] = std::clamp(map_log_odds_data_[idx] + delta_log_odds, params_.log_odds_min, params_.log_odds_max);
                // map_.data[idx] = log_odds_to_probability(map_log_odds_data_[idx]);
            }

            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }

        // Occupied cell with noise model
        if (!hit_point)
            return;

        int radius = std::ceil(params_.noise_radius / map_.resolution);
        float std_dev = params_.noise_std_dev / map_.resolution;

        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dx = -radius; dx <= radius; ++dx)
            {
                if ((dx * dx + dy * dy) * map_.resolution * map_.resolution > params_.noise_radius * params_.noise_radius)
                    continue;

                int nx = x1 + dx;
                int ny = y1 + dy;

                if (nx >= 0 && nx < map_.width && ny >= 0 && ny < map_.height)
                {
                    float dist2 = dx * dx + dy * dy;
                    float weight = std::exp(-dist2 / (2.0f * std_dev * std_dev));

                    int idx = ny * map_.width + nx;

                    if (map_.data[idx] == -1)
                    {
                        map_.data[idx] = 50;
                        map_log_odds_data_[idx] = probability_to_log_odds(50);
                    }

                    double cell_distance = std::hypot(nx - robot_x, ny - robot_y) * map_.resolution;

                    double delta_log_odds = (occupied_belief_log_odds_ * weight) * (1.0 - params_.distance_belief_factor * cell_distance);
                    local_log_odds_delta_[idx] += delta_log_odds;
                    // map_log_odds_data_[idx] += delta_log_odds;
                    
                    // map_log_odds_data_[idx] = std::clamp(map_log_odds_data_[idx] + delta_log_odds, params_.log_odds_min, params_.log_odds_max);
                    // map_.data[idx] = log_odds_to_probability(map_log_odds_data_[idx]);
                }
            }
        }
    }



    void Mapping::expanding_wavefront_frontier_cells_detection(int rx, int ry, double active_area_radius)
    {
        const int dx_table[4] = {0, 1, 0, -1};
        const int dy_table[4] = {1, 0, -1, 0};

        std::queue<int> queue;

        double sq_active_area_radius = active_area_radius * active_area_radius;
        int robot_idx = ry * frontier_map_.width + rx;
        
        if(ewfd_first_)
        {
            queue.push(robot_idx);
            ewfd_first_ = false;
        }
        else
        {
            for (int i = 0; i < frontier_map_.width * frontier_map_.height; i++)
            {
                if (frontier_map_.data[i] == 100)
                {
                    int x = i % frontier_map_.width;
                    int y = i / frontier_map_.width;

                    double x_dist = (x - rx) * frontier_map_.resolution;
                    double y_dist = (y - ry) * frontier_map_.resolution;
                    double sq_dist = x_dist * x_dist + y_dist * y_dist;

                    if (sq_dist > sq_active_area_radius)
                        continue;

                    queue.push(i);
                }
            }
        }


        while (!queue.empty())
        {
            int idx = queue.front();
            queue.pop();

            if (filtered_map_.data[idx] != 0)
                continue;

            int x = idx % frontier_map_.width;
            int y = idx / frontier_map_.width;

            bool is_frontier = false;

            for (int i=0; i<4; i++)
            {
                int dx = dx_table[i];
                int dy = dy_table[i];

                int nx = x + dx;
                int ny = y + dy;

                int nidx = ny * frontier_map_.width + nx;

                if (nx < 0 || nx >= frontier_map_.width || ny < 0 || ny >= frontier_map_.height)
                    continue;
                    
                if (ewfd_visited_[nidx])
                    continue;

                double nx_dist = (nx - rx) * frontier_map_.resolution;
                double ny_dist = (ny - ry) * frontier_map_.resolution;
                double sq_dist = nx_dist * nx_dist + ny_dist * ny_dist;

                if (sq_dist > sq_active_area_radius)
                    continue;

                if (filtered_map_.data[nidx] == -1)
                {
                    is_frontier = true;
                }
                if (filtered_map_.data[nidx] == 0)
                {
                    queue.push(nidx);
                    ewfd_visited_[nidx] = true;
                }
            }

            if (is_frontier)
            {
                if (frontier_map_.data[idx] != 100) {
                    frontier_map_.data[idx] = 100;
                    frontier_map_update_.frontier_indices.push_back(idx);
                }
            }
            else
            {
                if (frontier_map_.data[idx] != 0) {
                    frontier_map_.data[idx] = 0;
                    frontier_map_update_.explored_indices.push_back(idx);
                }
            }

        }
    }


    void Mapping::mapping()
    {
        std::deque<PosedScan> posed_scan_buffer;

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            posed_scan_buffer = posed_scan_buffer_;

            posed_scan_buffer_.clear();
        }

        if(posed_scan_buffer.empty())
            return;

        
        local_log_odds_delta_.clear();

        for(PosedScan& posed_scan : posed_scan_buffer)
        {
            int robot_map_x = static_cast<int>((posed_scan.position.x() - map_.origin_position.x()) / map_.resolution);
            int robot_map_y = static_cast<int>((posed_scan.position.y() - map_.origin_position.y()) / map_.resolution);

            double angle = posed_scan.angle_min;
            
            for (size_t i = 0; i < posed_scan.ranges.size(); ++i)
            {   
                bool hit_point = true;
                double range = posed_scan.ranges[i];

                
                if (range < posed_scan.range_min || std::isnan(range))
                {
                    angle += posed_scan.angle_increment;
                    continue;
                }
                if (range > posed_scan.range_max)
                {
                    range = posed_scan.range_max;
                    hit_point = false;
                }
                
                // Local laser point
                double laser_x = range * cos(angle);
                double laser_y = range * sin(angle);


                // Transform to global
                double yaw = std::atan2(
                    2.0 * (posed_scan.orientation.w() * posed_scan.orientation.z() +
                        posed_scan.orientation.x() * posed_scan.orientation.y()),
                    1.0 - 2.0 * (posed_scan.orientation.y() * posed_scan.orientation.y() +
                                posed_scan.orientation.z() * posed_scan.orientation.z())
                );

                double global_laser_x = posed_scan.position.x() + laser_x * cos(yaw) - laser_y * sin(yaw);
                double global_laser_y = posed_scan.position.y() + laser_x * sin(yaw) + laser_y * cos(yaw);

                // Map indices of endpoint
                int global_laser_map_x = static_cast<int>((global_laser_x - map_.origin_position.x()) / map_.resolution);
                int global_laser_map_y = static_cast<int>((global_laser_y - map_.origin_position.y()) / map_.resolution);

                bresenham_raytrace(robot_map_x, robot_map_y, global_laser_map_x, global_laser_map_y, hit_point);

                angle += posed_scan.angle_increment;
            }
        }


        for (auto &[idx, delta] : local_log_odds_delta_)
        {
            map_log_odds_data_[idx] =
                std::clamp(map_log_odds_data_[idx] + delta,
                        params_.log_odds_min,
                        params_.log_odds_max);

            map_.data[idx] = log_odds_to_probability(map_log_odds_data_[idx]);
        }

        // Set map updated flag
        map_updated_ = true;


        // Update map log odds update
        {
            std::lock_guard<std::mutex> lock(map_log_odds_update_mutex_);

            for (auto &[idx, delta] : local_log_odds_delta_) {
                map_log_odds_update_.indices.push_back(idx);
                map_log_odds_update_.delta_log_odds.push_back(delta);
            }
    
            local_log_odds_delta_.clear();
        }
    


        // Create filtered map
        for (size_t i = 0; i < map_.data.size(); i++)
        {
            if (map_.data[i] == -1)
                continue;

            double val = map_.data[i] / 100.0;
            if (val > params_.obstacle_threshold)
                filtered_map_.data[i] = 100;
            else if (val < params_.free_threshold)
                filtered_map_.data[i] = 0;
            else
                filtered_map_.data[i] = -1;
        }


        // Remove frontier cells around obstacles

        {
            std::lock_guard<std::mutex> lock(frontier_map_update_mutex_);

            int radius = static_cast<int>(std::ceil(params_.frontier_del_obstacles_radius / filtered_map_.resolution));

            for (size_t i = 0; i < filtered_map_.data.size(); i++) {
                if (filtered_map_.data[i] == -1)
                    continue;

                if (filtered_map_.data[i] == 100) {
                    int x = i % filtered_map_.width;
                    int y = i / filtered_map_.width;

                    for (int dx = -radius; dx <= radius; dx++) {
                        for (int dy = -radius; dy <= radius; dy++) {
                            if ((dx * dx + dy * dy) * filtered_map_.resolution * filtered_map_.resolution > params_.frontier_del_obstacles_radius * params_.frontier_del_obstacles_radius)
                                continue;

                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx < 0 || ny < 0 || nx >= filtered_map_.width || ny >= filtered_map_.height)
                                continue;

                            int idx = ny * filtered_map_.width + nx;

                            if (frontier_map_.data[idx] != 0) {
                                frontier_map_.data[idx] = 0;
                                frontier_map_update_.explored_indices.push_back(idx);
                            }
                        }
                    }
                }
            }

            // Detect frontiers cells in the active area

            PosedScan posed_scan_front = posed_scan_buffer.front();
            PosedScan posed_scan_back = posed_scan_buffer.back();

            double displacement = (posed_scan_back.position - posed_scan_front.position).norm();

            int robot_map_x = static_cast<int>((posed_scan_back.position.x() - map_.origin_position.x()) / map_.resolution);
            int robot_map_y = static_cast<int>((posed_scan_back.position.y() - map_.origin_position.y()) / map_.resolution);

            double active_area_radius = posed_scan_back.range_max + displacement + 1.0;

            expanding_wavefront_frontier_cells_detection(robot_map_x, robot_map_y, active_area_radius);
        }

        // Set frontier map update flag
        frontier_map_updated_ = true;
    }   
}