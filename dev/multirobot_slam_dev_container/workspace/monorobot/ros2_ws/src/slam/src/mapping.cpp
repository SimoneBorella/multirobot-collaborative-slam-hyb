#include "mapping.h"

Mapping::Mapping(rclcpp::Node::SharedPtr node, std::shared_ptr<Localization> localization)
{
    this->node = node;
    this->localization = localization;

    map_frame = node->get_parameter("map_frame").as_string();
    odom_frame = node->get_parameter("odom_frame").as_string();
    base_frame = node->get_parameter("base_frame").as_string();
    
    mapping_rate = node->get_parameter("mapping_rate").as_double();
    map_resolution = node->get_parameter("map_resolution").as_double();
    map_width = node->get_parameter("map_initial_width").as_double();
    map_height = node->get_parameter("map_initial_height").as_double();
    
    int map_initial_value = node->get_parameter("map_initial_value").as_int();

    use_rays_over_range_max = node->get_parameter("use_rays_over_range_max").as_bool();

    free_belief = node->get_parameter("free_belief").as_double();
    occupied_belief = node->get_parameter("occupied_belief").as_double();
    distance_belief_factor = node->get_parameter("distance_belief_factor").as_double();

    apply_threshold = node->get_parameter("apply_threshold").as_bool();
    occupied_threshold = node->get_parameter("occupied_threshold").as_double();
    free_threshold = node->get_parameter("free_threshold").as_double();

    override_active_hit_points = node->get_parameter("override_active_hit_points").as_bool();

    noise_model_radius = node->get_parameter("noise_model_radius").as_double();
    noise_model_std_dev = node->get_parameter("noise_model_std_dev").as_double();

    frontier_cells_detection_mode = node->get_parameter("frontier_cells_detection_mode").as_string();

    free_belief_log_odds = std::log(free_belief / (1.0 - free_belief));
    occupied_belief_log_odds = std::log(occupied_belief / (1.0 - occupied_belief));


    // Create publishers 
    rclcpp::QoS map_qos_profile(10);
    map_qos_profile.reliable();
    map_qos_profile.transient_local();          
    map_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos_profile);

    frontier_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("frontier", map_qos_profile);

    // Initialize map message

    width_cells = static_cast<int>(map_width / map_resolution);
    height_cells = static_cast<int>(map_height / map_resolution);

    map_grid.header.frame_id = map_frame;
    map_grid.info.resolution = map_resolution;
    map_grid.info.width = width_cells;
    map_grid.info.height = height_cells;
    map_grid.info.origin.position.x = -map_width / 2.0;
    map_grid.info.origin.position.y = -map_height / 2.0;
    map_grid.data.assign(width_cells * height_cells, map_initial_value);

    map.assign(width_cells * height_cells, map_initial_value);

    // Initialize frontier message
    frontier_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    frontier_grid_mutex = std::make_shared<std::shared_mutex>();

    frontier_grid->header.frame_id = map_frame;
    frontier_grid->info.resolution = map_resolution;
    frontier_grid->info.width = width_cells;
    frontier_grid->info.height = height_cells;
    frontier_grid->info.origin.position.x = -map_width / 2.0;
    frontier_grid->info.origin.position.y = -map_height / 2.0;
    frontier_grid->data.assign(width_cells * height_cells, -1);

    // Initialize posed scans
    auto [posed_scans_, posed_scans_mutex_] = localization->get_posed_scans_with_mutex();
    posed_scans = posed_scans_;
    posed_scans_mutex = posed_scans_mutex_;

    // Initialize threads
    mapping_running = false;
}

Mapping::~Mapping()
{
    mapping_running = false;
    if (mapping_thread && mapping_thread->joinable()) {
        mapping_thread->join();
    }
}




float Mapping::probability_to_log_odds(uint8_t prob) {    
    float p = static_cast<float>(prob) / 100.0f;
    
    if (p <= 0) p = 0.1;
    if (p >= 100) p = 99.9;
    
    return std::log(p / (1.0f - p));
}

uint8_t Mapping::log_odds_to_probability(float log_odds) {
    float p = 1.0f / (1.0f + std::exp(-log_odds));
    uint8_t prob = static_cast<uint8_t>(std::round(p * 100.0f));

    if (prob <= 0) prob = 0;
    if (prob >= 100) prob = 100;

    return prob;
}


void Mapping::bresenham_raytrace(int x0, int y0, int x1, int y1,
    std::vector<int8_t>& map_grid_data,
    std::vector<float>& map_data,
    bool hit_point)
{
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    double distance = std::sqrt((x1 - x0)*(x1 - x0) + (y1 - y0)*(y1 - y0)) * map_resolution;

    while (true)
    {
        int index = y0 * width_cells + x0;
        if (x0 >= 0 && x0 < width_cells && y0 >= 0 && y0 < height_cells)
        {
            // Free cell
            if (map_grid_data[index] == -1)
            {
                map_grid_data[index] = 50;
                map_data[index] = probability_to_log_odds(50);
            }

            map_data[index] += free_belief_log_odds * (1.0 - distance_belief_factor * distance);
            map_grid_data[index] = log_odds_to_probability(map_data[index]);
        }

        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }

    // Occupied cell with noise model
    if (!hit_point)
    return;

    int radius = std::ceil(noise_model_radius / map_resolution);
    float std_dev = noise_model_std_dev / map_resolution;

    for (int dy = -radius; dy <= radius; ++dy)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
            if ((dx * dx + dy * dy) * map_resolution * map_resolution > noise_model_radius * noise_model_radius)
                continue;

            int nx = x1 + dx;
            int ny = y1 + dy;

            if (nx >= 0 && nx < width_cells && ny >= 0 && ny < height_cells)
            {
                float dist2 = dx * dx + dy * dy;
                float weight = std::exp(-dist2 / (2.0f * std_dev * std_dev));

                int idx = ny * width_cells + nx;

                if (map_grid_data[idx] == -1)
                {
                    map_grid_data[idx] = 50;
                    map_data[idx] = probability_to_log_odds(50);
                }

                map_data[idx] += (occupied_belief_log_odds * weight) * (1.0 - distance_belief_factor * distance);
                map_grid_data[idx] = log_odds_to_probability(map_data[idx]);
            }
        }
    }
}



void Mapping::expanding_wavefront_frontier_cells_detection(int rx, int ry, double active_area_radius)
{
    static bool first = true;
    static std::vector<bool> visited(width_cells * height_cells, false);

    const int dx_table[4] = {0, 1, 0, -1};
    const int dy_table[4] = {1, 0, -1, 0};

    std::queue<int> queue;

    double sq_active_area_radius = active_area_radius * active_area_radius;
    int robot_index = ry * width_cells + rx;
    
    if(first)
    {
        queue.push(robot_index);
        first = false;
    }
    else
    {
        for (int i = 0; i < width_cells * height_cells; i++)
        {
            if (frontier_grid->data[i] == 100)
            {
                int x = i % width_cells;
                int y = i / width_cells;

                double x_dist = (x - rx) * map_resolution;
                double y_dist = (y - ry) * map_resolution;
                double sq_dist = x_dist * x_dist + y_dist * y_dist;

                if (sq_dist > sq_active_area_radius)
                    continue;

                queue.push(i);
            }
        }
    }


    while (!queue.empty())
    {
        int index = queue.front();
        queue.pop();

        if (map_grid.data[index] != 0)
            continue;

        int x = index % width_cells;
        int y = index / width_cells;

        bool is_frontier = false;

        for (int i=0; i<4; i++)
        {
            int dx = dx_table[i];
            int dy = dy_table[i];

            int nx = x + dx;
            int ny = y + dy;

            int nindex = ny * width_cells + nx;

            if (nx < 0 || nx >= width_cells || ny < 0 || ny >= height_cells)
                continue;
                
            if (visited[nindex])
                continue;

            double nx_dist = (nx - rx) * map_resolution;
            double ny_dist = (ny - ry) * map_resolution;
            double sq_dist = nx_dist * nx_dist + ny_dist * ny_dist;

            if (sq_dist > sq_active_area_radius)
                continue;

            if (map_grid.data[nindex] == -1)
            {
                is_frontier = true;
            }
            if (map_grid.data[nindex] == 0)
            {
                queue.push(nindex);
                visited[nindex] = true;
            }
            
        }

        if (is_frontier)
            frontier_grid->data[index] = 100;
        else
            frontier_grid->data[index] = -1;
    }

}




void Mapping::startMapping()
{
    mapping_running = true;
    mapping_thread = std::make_unique<std::thread>([this]() {
        rclcpp::Rate rate(mapping_rate);
        while (rclcpp::ok() && mapping_running.load()) {
            mapping();
            rate.sleep();
        }
    });
    RCLCPP_INFO_STREAM(node->get_logger(), "Mapping thread started.");
}


void Mapping::mapping()
{
#ifdef MAPPING_DEBUG
    auto start_mapping = std::chrono::high_resolution_clock::now();
#endif

    PosedScan last_posed_scan;

    bool map_updated = false;

    {
        std::unique_lock lock(*posed_scans_mutex);

        if(posed_scans->empty())
            return;

        for (PosedScan& posed_scan : *posed_scans)
        {
            if(!posed_scan.updated)
                continue;


            sensor_msgs::msg::LaserScan& scan = posed_scan.scan;
            Pose2& pose = posed_scan.pose;

            // Robot origin in map
            int rx = static_cast<int>((pose.x() - map_grid.info.origin.position.x) / map_resolution);
            int ry = static_cast<int>((pose.y() - map_grid.info.origin.position.y) / map_resolution);

            double angle = scan.angle_min;
            for (size_t i = 0; i < scan.ranges.size(); ++i)
            {
                bool hit_point = true;
                double range = scan.ranges[i];

                if(!use_rays_over_range_max)
                {
                    if (range < scan.range_min || range > scan.range_max || std::isnan(range))
                    {
                        angle += scan.angle_increment;
                        continue;
                    }
                }
                else{
                    if (range < scan.range_min || std::isnan(range))
                    {
                        angle += scan.angle_increment;
                        continue;
                    }
                    if (range > scan.range_max)
                    {
                        range = scan.range_max;
                        hit_point = false;
                    }
                }


                // Local laser point
                double lx = range * cos(angle);
                double ly = range * sin(angle);

                // Transform to global
                double gx = pose.x() + lx * cos(pose.theta()) - ly * sin(pose.theta());
                double gy = pose.y() + lx * sin(pose.theta()) + ly * cos(pose.theta());

                // Map indices of endpoint
                int mx = static_cast<int>((gx - map_grid.info.origin.position.x) / map_resolution);
                int my = static_cast<int>((gy - map_grid.info.origin.position.y) / map_resolution);

                bresenham_raytrace(rx, ry, mx, my, map_grid.data, map, hit_point);

                angle += scan.angle_increment;
            }

            posed_scan.updated = false;
            map_updated = true;
        }

        last_posed_scan = posed_scans->back();

        posed_scans->erase(
            std::remove_if(posed_scans->begin(), posed_scans->end(),[](const PosedScan& ps) { return !ps.keep; }),
            posed_scans->end()
        );
    }





    if (!map_updated)
        return;

    // Use thresholds
    if(apply_threshold)
    {
        for (size_t i = 0; i < map_grid.data.size(); i++)
        {
            if (map_grid.data[i] == -1)
                continue;
            
            float val = map_grid.data[i] / 100.0f;    
            if (val > occupied_threshold)
            {
                map_grid.data[i] = 100;
                if (frontier_grid->data[i] == 100)
                    frontier_grid->data[i] = -1; // Remove from frontier if occupied
            }
            else if (val < free_threshold)
                map_grid.data[i] = 0;
            else
                map_grid.data[i] = -1;
        }
    }

    // Override active hit points
    if(override_active_hit_points)
    {
        sensor_msgs::msg::LaserScan& scan = last_posed_scan.scan;
        Pose2& pose = last_posed_scan.pose;


        double angle = scan.angle_min;
        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            double range = scan.ranges[i];

            if (range < scan.range_min || range > scan.range_max || std::isnan(range))
            {
                angle += scan.angle_increment;
                continue;
            }
            
            // Local laser point
            double lx = range * cos(angle);
            double ly = range * sin(angle);

            // Transform to global
            double gx = pose.x() + lx * cos(pose.theta()) - ly * sin(pose.theta());
            double gy = pose.y() + lx * sin(pose.theta()) + ly * cos(pose.theta());

            // Map indices of endpoint
            int mx = static_cast<int>((gx - map_grid.info.origin.position.x) / map_resolution);
            int my = static_cast<int>((gy - map_grid.info.origin.position.y) / map_resolution);

            // Occupied cell
            if (mx >= 0 && mx < width_cells && my >= 0 && my < height_cells)
            {
                int end_index = my * width_cells + mx;
                map_grid.data[end_index] = 100;
            }

            angle += scan.angle_increment;
        }
    }
    
    map_grid.header.stamp = node->get_clock()->now();

    map_publisher->publish(map_grid);


#ifdef MAPPING_DEBUG
    auto end_mapping = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_mapping = end_mapping - start_mapping;
#endif

#ifdef MAPPING_DEBUG
    std::cout << "Mapping time: " << (duration_mapping.count() * 1000) << " ms" << std::endl;
    std::cout << std::endl;
#endif

    // Frontier cells detection

    int rx = static_cast<int>((last_posed_scan.pose.x() - map_grid.info.origin.position.x) / map_resolution);
    int ry = static_cast<int>((last_posed_scan.pose.y() - map_grid.info.origin.position.y) / map_resolution);
    
    if(frontier_cells_detection_mode == "ewfd")
    {
        std::unique_lock lock(*frontier_grid_mutex);
        double active_area_radius = last_posed_scan.scan.range_max + 1.0;
        expanding_wavefront_frontier_cells_detection(rx, ry, active_area_radius);
    }

    frontier_grid->header.stamp = node->get_clock()->now();

    frontier_publisher->publish(*frontier_grid);
}
