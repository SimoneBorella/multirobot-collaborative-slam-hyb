#include "submap_manager.h"

namespace multirobot_slam
{
    SubmapManager::SubmapManager()
    {
    }

    SubmapManager::SubmapManager(SubmapManagerParams &params)
        : params_(params)
    {
    }

    SubmapManagerParams SubmapManager::params_from_yaml(std::string &params_path)
    {
        SubmapManagerParams p;

        try
        {
            YAML::Node config = YAML::LoadFile(params_path);
            YAML::Node submap_manager_config = config["submap_manager"];

            if (submap_manager_config["submap_mapping_rate"])
                p.submap_mapping_rate = submap_manager_config["submap_mapping_rate"].as<double>();

            if (submap_manager_config["submap_resolution"])
                p.submap_resolution = submap_manager_config["submap_resolution"].as<double>();

            if (submap_manager_config["submap_width"])
                p.submap_width = submap_manager_config["submap_width"].as<double>();

            if (submap_manager_config["submap_height"])
                p.submap_height = submap_manager_config["submap_height"].as<double>();

            if (submap_manager_config["free_belief"]) 
                p.free_belief = submap_manager_config["free_belief"].as<double>();

            if (submap_manager_config["occ_belief"]) 
                p.occ_belief = submap_manager_config["occ_belief"].as<double>();

            if (submap_manager_config["distance_belief_factor"]) 
                p.distance_belief_factor = submap_manager_config["distance_belief_factor"].as<double>();

            if (submap_manager_config["log_odds_min"]) 
                p.log_odds_min = submap_manager_config["log_odds_min"].as<double>();

            if (submap_manager_config["log_odds_max"]) 
                p.log_odds_max = submap_manager_config["log_odds_max"].as<double>();

            if (submap_manager_config["noise_radius"]) 
                p.noise_radius = submap_manager_config["noise_radius"].as<double>();

            if (submap_manager_config["noise_std_dev"]) 
                p.noise_std_dev = submap_manager_config["noise_std_dev"].as<double>();

            if (submap_manager_config["obstacle_threshold"]) 
                p.obstacle_threshold = submap_manager_config["obstacle_threshold"].as<double>();

            if (submap_manager_config["free_threshold"]) 
                p.free_threshold = submap_manager_config["free_threshold"].as<double>();

        
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
            std::cerr << "Using default parameters.\n";
        }

        return p;
    }

    void SubmapManager::init(SubmapManagerParams &params)
    {
        params_ = params;

        free_belief_log_odds_ = std::log(params_.free_belief / (1.0 - params_.free_belief));
        occupied_belief_log_odds_ = std::log(params_.occ_belief / (1.0 - params_.occ_belief));
    }

    void SubmapManager::update_keyframes(const std::vector<KeyFrame> &keyframes)
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        keyframes_ = keyframes;
    }

    void SubmapManager::add_posed_scan(const PosedScan &posed_scan)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        posed_scan_buffer_.push_back(posed_scan);
    }

    Submap SubmapManager::create_submap_from_keyframe(const KeyFrame& keyframe)
    {
        Submap submap;
        submap.keyframe_id = keyframe.keyframe_id;

        submap.resolution = params_.submap_resolution;
        submap.width = static_cast<int>(params_.submap_width / params_.submap_resolution);
        submap.height = static_cast<int>(params_.submap_height / params_.submap_resolution);

        submap.origin_position = keyframe.pose.position;
        submap.origin_orientation = keyframe.pose.orientation;

        submap.log_odds.resize(submap.width * submap.height, 0.0f);

        submap.timestamp_start = keyframe.timestamp;
        submap.timestamp_end = std::numeric_limits<double>::infinity();

        return submap;
    }

    int SubmapManager::idx(int x, int y, int width)
    {
        return y * width + x;
    }

    double SubmapManager::probability_to_log_odds(int8_t prob)
    {    
        double p = static_cast<double>(prob) / 100.0;
        
        p = std::clamp(p, 0.01, 0.99);
        
        return std::log(p / (1.0 - p));
    }

    int8_t SubmapManager::log_odds_to_probability(double log_odds)
    {
        double p = 1.0 / (1.0 + std::exp(-log_odds));
        int8_t prob = static_cast<int8_t>(std::round(p * 100.0));

        if (prob <= 0) prob = 0;
        if (prob >= 100) prob = 100;

        return prob;
    }

    void SubmapManager::bresenham_raytrace_submap(Submap& submap, int x0, int y0, int x1, int y1, bool hit_point)
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

            if (x0 >= 0 && x0 < submap.width && y0 >= 0 && y0 < submap.height)
            {
                int id = idx(x0, y0, submap.width);

                double cell_distance = std::hypot(x0 - robot_x, y0 - robot_y) * submap.resolution;
                double delta_log_odds = free_belief_log_odds_ * (1.0 - params_.distance_belief_factor * cell_distance);

                submap.log_odds[id] = std::clamp(
                    submap.log_odds[id] + delta_log_odds,
                    params_.log_odds_min,
                    params_.log_odds_max
                );
            }

            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }

        if (!hit_point)
            return;

        int radius = std::ceil(params_.noise_radius / submap.resolution);
        float std_dev = params_.noise_std_dev / submap.resolution;

        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dx = -radius; dx <= radius; ++dx)
            {
                if ((dx*dx + dy*dy) * submap.resolution * submap.resolution > params_.noise_radius * params_.noise_radius)
                    continue;

                int nx = x1 + dx;
                int ny = y1 + dy;

                if (nx >= 0 && nx < submap.width && ny >= 0 && ny < submap.height)
                {
                    float dist2 = dx*dx + dy*dy;
                    float weight = std::exp(-dist2 / (2.0f * std_dev * std_dev));

                    int id = idx(nx, ny, submap.width);
                    double cell_distance = std::hypot(nx - robot_x, ny - robot_y) * submap.resolution;

                    double delta_log_odds = (occupied_belief_log_odds_ * weight) * (1.0 - params_.distance_belief_factor * cell_distance);

                    submap.log_odds[id] = std::clamp(
                        submap.log_odds[id] + delta_log_odds,
                        params_.log_odds_min,
                        params_.log_odds_max
                    );
                }
            }
        }
    }



    void SubmapManager::submaps_mapping()
    {
        std::vector<PosedScan> posed_scan_buffer;

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            posed_scan_buffer = posed_scan_buffer_;

            posed_scan_buffer_.clear();
        }

        if(posed_scan_buffer.empty())
            return;


        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);

            if (keyframes_.size() > submaps_.size())
            {
                for (size_t i = submaps_.size(); i < keyframes_.size(); i++)
                {
                    const KeyFrame& keyframe = keyframes_[i];
                    Submap submap = create_submap_from_keyframe(keyframe);

                    if(!submaps_.empty())
                        submaps_.back().timestamp_end = keyframe.timestamp;

                    submaps_.push_back(submap);
                    submaps_updated_.push_back(true);
                    submap_mutexes_.push_back(std::make_unique<std::mutex>());
                }
            }
        }

        if(submaps_.empty())
            return;


        for (PosedScan& posed_scan : posed_scan_buffer)
        {
            size_t submap_id = submaps_.size();

            for (int i = static_cast<int>(submaps_.size()) - 1; i >= 0; i--)
            {
                if (posed_scan.timestamp >= submaps_[i].timestamp_start &&
                    posed_scan.timestamp <  submaps_[i].timestamp_end)
                {
                    submap_id = i;
                    break;
                }
            }

            if (submap_id == submaps_.size())
                continue;

            std::lock_guard<std::mutex> lock(*submap_mutexes_[submap_id]);

            Submap& submap = submaps_[submap_id];

            double yaw_scan = std::atan2(
                2.0 * (posed_scan.orientation.w() * posed_scan.orientation.z() +
                    posed_scan.orientation.x() * posed_scan.orientation.y()),
                1.0 - 2.0 * (posed_scan.orientation.y() * posed_scan.orientation.y() +
                            posed_scan.orientation.z() * posed_scan.orientation.z())
            );

            double yaw_submap = std::atan2(
                2.0 * (submap.origin_orientation.w() * submap.origin_orientation.z() +
                    submap.origin_orientation.x() * submap.origin_orientation.y()),
                1.0 - 2.0 * (submap.origin_orientation.y() * submap.origin_orientation.y() +
                            submap.origin_orientation.z() * submap.origin_orientation.z())
            );

            double yaw = yaw_scan - yaw_submap;

            Eigen::Vector2d p_world(
                posed_scan.position.x() - submap.origin_position.x(),
                posed_scan.position.y() - submap.origin_position.y()
            );

            double c0 = std::cos(-yaw_submap);
            double s0 = std::sin(-yaw_submap);

            Eigen::Vector2d robot_submap;
            robot_submap.x() = c0 * p_world.x() - s0 * p_world.y();
            robot_submap.y() = s0 * p_world.x() + c0 * p_world.y();

            int robot_x = static_cast<int>(robot_submap.x() / submap.resolution + submap.width  / 2);
            int robot_y = static_cast<int>(robot_submap.y() / submap.resolution + submap.height / 2);

            if (robot_x < 0 || robot_x >= submap.width ||
                robot_y < 0 || robot_y >= submap.height)
                continue;

            double angle = posed_scan.angle_min;

            double cs = std::cos(yaw);
            double sn = std::sin(yaw);

            for (size_t k = 0; k < posed_scan.ranges.size(); ++k)
            {
                bool hit_point = true;
                double range = posed_scan.ranges[k];

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

                // punto laser nel frame robot
                double lx = range * std::cos(angle);
                double ly = range * std::sin(angle);

                // robot → submap
                Eigen::Vector2d laser_submap;
                laser_submap.x() = robot_submap.x() + cs * lx - sn * ly;
                laser_submap.y() = robot_submap.y() + sn * lx + cs * ly;

                int laser_map_x = static_cast<int>(laser_submap.x() / submap.resolution + submap.width  / 2);
                int laser_map_y = static_cast<int>(laser_submap.y() / submap.resolution + submap.height / 2);

                bresenham_raytrace_submap(
                    submap,
                    robot_x,
                    robot_y,
                    laser_map_x,
                    laser_map_y,
                    hit_point
                );

                angle += posed_scan.angle_increment;
            }

            submaps_updated_[submap_id] = true;
        }

    }

    std::vector<Map> SubmapManager::get_updated_submaps()
    {
        std::vector<Map> updated_maps;

        for(size_t i=0; i<submaps_updated_.size(); i++)
        {
            std::lock_guard<std::mutex> lock(*submap_mutexes_[i]);

            if(submaps_updated_[i])
            {
                const Submap& submap = submaps_[i];

                Map map;
                map.keyframe_id = submap.keyframe_id;
                map.resolution = submap.resolution;
                map.width = submap.width;
                map.height = submap.height;
                map.origin_position = submap.origin_position;
                map.origin_orientation = submap.origin_orientation;

                map.data.resize(map.width * map.height);

                for (size_t k = 0; k < submap.log_odds.size(); ++k)
                {
                    map.data[k] = log_odds_to_probability(submap.log_odds[k]);
                }

                // Create filtered map
                for (size_t i = 0; i < map.data.size(); i++)
                {
                    if (map.data[i] == -1)
                        continue;

                    double val = map.data[i] / 100.0;
                    if (val > params_.obstacle_threshold)
                        map.data[i] = 100;
                    else if (val < params_.free_threshold)
                        map.data[i] = 0;
                    else
                        map.data[i] = -1;
                }


                updated_maps.push_back(std::move(map));

                submaps_updated_[i] = false;
            }
        }

        return updated_maps;
    }

}