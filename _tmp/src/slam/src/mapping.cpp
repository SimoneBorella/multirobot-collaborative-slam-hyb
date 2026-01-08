#include "mapping.h"

Mapping::Mapping(rclcpp::Node::SharedPtr node, std::shared_ptr<Localization> localization)
{
    this->node = node;
    this->localization = localization;

    robots = node->get_parameter("robots").as_string_array();

    all_frame = node->get_parameter("all_frame").as_string();
    map_frame = node->get_parameter("map_frame").as_string();
    odom_frame = node->get_parameter("odom_frame").as_string();
    base_frame = node->get_parameter("base_frame").as_string();
    
    mapping_rate = node->get_parameter("mapping_rate").as_double();
    map_resolution = node->get_parameter("map_resolution").as_double();
    map_width = node->get_parameter("map_initial_width").as_double();
    map_height = node->get_parameter("map_initial_height").as_double();
    
    use_rays_over_range_max = node->get_parameter("use_rays_over_range_max").as_bool();

    free_belief = node->get_parameter("free_belief").as_double();
    occupied_belief = node->get_parameter("occupied_belief").as_double();
    distance_belief_factor = node->get_parameter("distance_belief_factor").as_double();

    occupied_threshold = node->get_parameter("occupied_threshold").as_double();
    free_threshold = node->get_parameter("free_threshold").as_double();

    override_active_hit_points = node->get_parameter("override_active_hit_points").as_bool();

    noise_model_radius = node->get_parameter("noise_model_radius").as_double();
    noise_model_std_dev = node->get_parameter("noise_model_std_dev").as_double();

    robot_obstacle_radius = node->get_parameter("robot_obstacle_radius").as_double();

    frontier_del_obstacles_radius = node->get_parameter("frontier_del_obstacles_radius").as_double();
    frontier_cells_detection_mode = node->get_parameter("frontier_cells_detection_mode").as_string();

    int costmap_initial_value = node->get_parameter("costmap_initial_value").as_int();
    kernel_distance = node->get_parameter("kernel_distance").as_double();
    kernel_size = static_cast<int>(2 * std::floor(kernel_distance / map_resolution));

    cost_decay_rate = node->get_parameter("cost_decay_rate").as_double();

    epsilon = node->get_parameter("epsilon").as_double();
    min_points = node->get_parameter("min_points").as_int();
    min_frontier_size = node->get_parameter("min_frontier_size").as_double();

    free_belief_log_odds = std::log(free_belief / (1.0 - free_belief));
    occupied_belief_log_odds = std::log(occupied_belief / (1.0 - occupied_belief));


    // Create publishers 
    rclcpp::QoS map_qos_profile(10);
    map_qos_profile.reliable();
    map_qos_profile.transient_local();          
    map_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos_profile);

    for(auto& robot : robots)
    {
        robot_map_publishers[robot] = node->create_publisher<nav_msgs::msg::OccupancyGrid>(robot + "/map", map_qos_profile);
    }

    frontier_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("frontier", map_qos_profile);
    frontiers_marker_publisher = node->create_publisher<visualization_msgs::msg::Marker>("frontier_centroids", 10);
    costmap_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", map_qos_profile);

    width_cells = static_cast<int>(map_width / map_resolution);
    height_cells = static_cast<int>(map_height / map_resolution);

    // Initialize map message
    map_grid.header.frame_id = all_frame;
    map_grid.info.resolution = map_resolution;
    map_grid.info.width = width_cells;
    map_grid.info.height = height_cells;
    map_grid.info.origin.position.x = -map_width / 2.0;
    map_grid.info.origin.position.y = -map_height / 2.0;
    map_grid.data.assign(width_cells * height_cells, -1);

    map.assign(width_cells * height_cells, -1);


    for(auto& robot : robots)
    {
        robot_map_grid[robot].header.frame_id = all_frame;
        robot_map_grid[robot].info.resolution = map_resolution;
        robot_map_grid[robot].info.width = width_cells;
        robot_map_grid[robot].info.height = height_cells;
        robot_map_grid[robot].info.origin.position.x = -map_width / 2.0;
        robot_map_grid[robot].info.origin.position.y = -map_height / 2.0;
        robot_map_grid[robot].data.assign(width_cells * height_cells, -1);

        robot_map[robot].assign(width_cells * height_cells, -1);
    }

    // Initialize frontier message
    frontier_grid.header.frame_id = all_frame;
    frontier_grid.info.resolution = map_resolution;
    frontier_grid.info.width = width_cells;
    frontier_grid.info.height = height_cells;
    frontier_grid.info.origin.position.x = -map_width / 2.0;
    frontier_grid.info.origin.position.y = -map_height / 2.0;
    frontier_grid.data.assign(width_cells * height_cells, -1);

    // Initialize frontiers
    frontiers = std::make_shared<std::vector<std::pair<std::pair<double, double>, int>>>();
    frontiers_mutex = std::make_shared<std::shared_mutex>();

    // Initialize costmap message
    costmap_grid_mutex = std::make_shared<std::shared_mutex>();
    costmap_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    costmap_grid->header.frame_id = all_frame;
    costmap_grid->info.resolution = map_resolution;
    costmap_grid->info.width = width_cells;
    costmap_grid->info.height = height_cells;
    costmap_grid->info.origin.position.x = -map_width / 2.0;
    costmap_grid->info.origin.position.y = -map_height / 2.0;
    costmap_grid->data.assign(width_cells * height_cells, costmap_initial_value);

    for(std::string& robot : robots)
    {
        ewfd_first[robot] = true;
        ewfd_visited[robot].assign(width_cells * height_cells, false);

        frontier_grid_data[robot].assign(width_cells * height_cells, -1);
    }

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



void Mapping::expanding_wavefront_frontier_cells_detection(std::string& robot, int rx, int ry, double active_area_radius)
{
    const int dx_table[4] = {0, 1, 0, -1};
    const int dy_table[4] = {1, 0, -1, 0};

    std::queue<int> queue;

    double sq_active_area_radius = active_area_radius * active_area_radius;
    int robot_index = ry * width_cells + rx;
    
    if(ewfd_first[robot])
    {
        queue.push(robot_index);
        ewfd_first[robot] = false;
    }
    else
    {
        for (int i = 0; i < width_cells * height_cells; i++)
        {
            if (frontier_grid_data[robot][i] == 100)
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
                
            if (ewfd_visited[robot][nindex])
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
                ewfd_visited[robot][nindex] = true;
            }
        }

        if (is_frontier)
            frontier_grid_data[robot][index] = 100;
        else
            frontier_grid_data[robot][index] = 0;
    }
}















std::vector<std::pair<int, int>> Mapping::get_neighbors(int x, int y)
{
    std::vector<std::pair<int, int>> neighbors;

    int height_cells = frontier_grid.info.height;
    int width_cells = frontier_grid.info.width;
    double map_resolution = frontier_grid.info.resolution;
    int epsilon_cells = std::ceil(epsilon / map_resolution);

    for (int dx = -epsilon_cells; dx <= epsilon_cells; dx++)
    {
        for (int dy = -epsilon_cells; dy <= epsilon_cells; dy++)
        {
            if (dx == 0 && dy == 0)
                continue;

            if ((dx * dx + dy * dy) * map_resolution * map_resolution > epsilon * epsilon)
                continue;

            int nx = x + dx;
            int ny = y + dy;

            int nindex = ny * width_cells + nx;

            if (nx >= 0 && ny >= 0 && nx < width_cells && ny < height_cells && frontier_grid.data[nindex] == 100)
            {
                neighbors.emplace_back(nx, ny);
            }
        }
    }

    return neighbors;
}




std::map<int, std::vector<std::pair<int, int>>> Mapping::dbscan_frontier_clusters_detection()
{
    std::map<std::pair<int, int>, int> labels;
    std::map<int, std::vector<std::pair<int, int>>> clusters;

    int cluster_id = 0;
    int height_cells = frontier_grid.info.height;
    int width_cells = frontier_grid.info.width;

    for (int y = 0; y < height_cells; y++) {
        for (int x = 0; x < width_cells; x++) {
            int index = y * width_cells + x;
    
            if (frontier_grid.data[index] != 100)
                continue;
    
            if (labels.find({x, y}) != labels.end())
                continue;
    
            std::vector<std::pair<int, int>> neighbors = get_neighbors(x, y);
    
            if (neighbors.size() < static_cast<size_t>(min_points)) {
                labels[{x, y}] = -1;
                continue;
            }
    
            labels[{x, y}] = cluster_id;
            clusters[cluster_id].push_back({x, y});
    
            std::queue<std::pair<int, int>> queue;
    
            for (const auto& p : neighbors)
                queue.push(p);
    
            while (!queue.empty()) {
                auto p = queue.front(); queue.pop();
    
                if (labels.find(p) != labels.end()) {
                    if (labels[p] == -1) {
                        labels[p] = cluster_id;
                        clusters[cluster_id].push_back(p);
                    }
                    continue;
                }
    
                labels[p] = cluster_id;
                clusters[cluster_id].push_back(p);
    
                auto p_neighbors = get_neighbors(p.first, p.second);
                if (p_neighbors.size() >= static_cast<size_t>(min_points)) {
                    for (const auto& n : p_neighbors)
                        queue.push(n);
                }
            }
    
            cluster_id++;
        }
    }
    

    return clusters;
}


std::vector<std::pair<std::pair<double, double>, int>> Mapping::frontier_centroids_detection(
    const std::map<int, std::vector<std::pair<int, int>>>& frontier_clusters)
{
    std::vector<std::pair<std::pair<double, double>, int>> centroids;

    double env_resolution = frontier_grid.info.resolution;
    double origin_x = frontier_grid.info.origin.position.x;
    double origin_y = frontier_grid.info.origin.position.y;

    for (const auto& [cluster_id, cluster] : frontier_clusters)
    {
        double cluster_size = cluster.size() * env_resolution * env_resolution;
        if (cluster_size < min_frontier_size)
            continue;

        double sum_x = 0.0;
        double sum_y = 0.0;

        for (const auto& point : cluster)
        {
            sum_x += point.first;
            sum_y += point.second;
        }

        double centroid_x = sum_x / cluster.size();
        double centroid_y = sum_y / cluster.size();


        // Find the point in the cluster closest to the centroid
        std::pair<int, int> closest_point;
        double min_dist_sq = std::numeric_limits<double>::max();

        for (const auto& point : cluster)
        {
            double dx = point.first - centroid_x;
            double dy = point.second - centroid_y;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                closest_point = point;
            }
        }

        // Convert closest grid cell to world coordinates
        double world_x = closest_point.first * env_resolution + origin_x + env_resolution / 2.0;
        double world_y = closest_point.second * env_resolution + origin_y + env_resolution / 2.0;

        centroids.push_back(std::make_pair(std::make_pair(world_x, world_y), cluster_size));
    }

    return centroids;
}




void Mapping::publish_frontiers_marker(std::vector<std::pair<std::pair<double, double>, int>>& frontier_centroids)
{
    visualization_msgs::msg::Marker marker = visualization_msgs::msg::Marker();
    
    marker.header.frame_id = all_frame;
    marker.header.stamp = node->get_clock()->now();
    
    marker.ns = "frontiers";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.color.r = 0.1f;
    marker.color.g = 0.4f;
    marker.color.b = 0.1f;
    marker.color.a = 1.0f;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;

    for (const auto& [frontier_centroid, frontier_size] : frontier_centroids) {
        geometry_msgs::msg::Point p;
        p.x = frontier_centroid.first;
        p.y = frontier_centroid.second;
        p.z = 0.0;
        marker.points.push_back(p);
    }

    frontiers_marker_publisher->publish(marker);
}














void Mapping::mapping()
{
#ifdef MAPPING_DEBUG
    auto start_mapping = std::chrono::high_resolution_clock::now();
#endif

    std::map<std::string, PosedScan> last_posed_scan;

    bool map_updated = false;
    
    std::map<std::string, std::vector<PosedScan>> posed_scans_;
    {
        std::unique_lock lock(*posed_scans_mutex);
        posed_scans_ = *posed_scans;

        for(auto& robot : robots)
        {
            (*posed_scans)[robot].erase(
                std::remove_if((*posed_scans)[robot].begin(), (*posed_scans)[robot].end(),[](const PosedScan& ps) { return !ps.keep; }),
                (*posed_scans)[robot].end()
            );
        }
    }

    for(std::string& robot : robots)
    {
        if(posed_scans_[robot].empty())
            return;

        for (PosedScan& posed_scan : posed_scans_[robot])
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
                bresenham_raytrace(rx, ry, mx, my, robot_map_grid[robot].data, robot_map[robot], hit_point);

                angle += scan.angle_increment;
            }

            posed_scan.updated = false;
            map_updated = true;
        }

        last_posed_scan[robot] = posed_scans_[robot].back();
    }

    



    if (!map_updated)
        return;

    // Use thresholds and remove frontier cells around obstacles

    int radius = static_cast<int>(std::ceil(frontier_del_obstacles_radius / map_resolution));

    std::vector<uint8_t> obstacle_mask(map_grid.data.size(), 0);

    for (size_t index = 0; index < map_grid.data.size(); index++) {
        if (map_grid.data[index] == -1) continue;
        
        float val = map_grid.data[index] / 100.0f;
        if (val > occupied_threshold) {
            map_grid.data[index] = 100;

            int x = index % width_cells;
            int y = index / width_cells;

            for (int dx = -radius; dx <= radius; dx++) {
                for (int dy = -radius; dy <= radius; dy++) {
                    if ((dx * dx + dy * dy) * map_resolution * map_resolution > frontier_del_obstacles_radius * frontier_del_obstacles_radius)
                        continue;

                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= width_cells || ny >= height_cells)
                        continue;

                    int nindex = nx + ny * width_cells;
                    obstacle_mask[nindex] = 1;
                }
            }
        } else if (val < free_threshold) {
            map_grid.data[index] = 0;
        } else {
            map_grid.data[index] = -1;
        }
    }

    for (std::string &robot : robots) {

        for (size_t index = 0; index < robot_map_grid[robot].data.size(); index++) {
            if (robot_map_grid[robot].data[index] == -1) continue;

            float val = robot_map_grid[robot].data[index] / 100.0f;
            if (val > occupied_threshold) {
                robot_map_grid[robot].data[index] = 100;
            } else if (val < free_threshold) {
                robot_map_grid[robot].data[index] = 0;
            } else {
                robot_map_grid[robot].data[index] = -1;
            }
        }

        for (size_t i = 0; i < obstacle_mask.size(); i++) {
            if (obstacle_mask[i]) {
                frontier_grid_data[robot][i] = 0;
            }
        }
    }
    


    // Remove obstacle points around the robots
    int robot_radius_cells = static_cast<int>(std::ceil(robot_obstacle_radius / map_resolution));

    for(std::string& robot : robots)
    {
        auto it = last_posed_scan.find(robot);
        if (it != last_posed_scan.end())
        {
            PosedScan& last_robot_posed_scan = it->second;

            int rx = static_cast<int>((last_robot_posed_scan.pose.x() - map_grid.info.origin.position.x) / map_resolution);
            int ry = static_cast<int>((last_robot_posed_scan.pose.y() - map_grid.info.origin.position.y) / map_resolution);

            for (int dy = -robot_radius_cells; dy <= robot_radius_cells; ++dy)
            {
                for (int dx = -robot_radius_cells; dx <= robot_radius_cells; ++dx)
                {
                    int nx = rx + dx;
                    int ny = ry + dy;

                    if (nx < 0 || ny < 0 || nx >= width_cells || ny >= height_cells)
                        continue;

                    int index = ny * width_cells + nx;

                    if (map_grid.data[index] == 100)
                        map_grid.data[index] = 0;
                    
                    for(auto& r : robots)
                    {
                        if (robot_map_grid[r].data[index] == 100)
                            robot_map_grid[r].data[index] = 0;
                    }
                }
            }
        }
    }


     // Costmap generation
    // static bool first = true;

    {
        // std::unique_lock lock(*costmap_grid_mutex);

        for (int y = 0; y < height_cells; y++)
        {
            for (int x = 0; x < width_cells; x++)
            {
                int index = y * width_cells + x;
                if (map_grid.data[index] != 100) continue;
        
                for (int dy = -kernel_size/2; dy <= kernel_size/2; ++dy)
                {
                    for (int dx = -kernel_size/2; dx <= kernel_size/2; ++dx)
                    {
                        int nx = x + dx;
                        int ny = y + dy;
            
                        if (nx < 0 || ny < 0 || nx >= width_cells || ny >= height_cells)
                            continue;
            
                        double distance = std::sqrt(dx * dx + dy * dy) * map_resolution;
                        if (distance > kernel_distance)
                            continue;
                        int8_t cost = static_cast<int8_t>(std::round(100 * std::exp(-cost_decay_rate * distance)));

                        // if(first)
                        // {
                        //     std::cout << int(cost) << " ";
                        // }
            
                        int n_index = ny * width_cells + nx;
                        costmap_grid->data[n_index] = std::max(costmap_grid->data[n_index], cost);
                    }

                    // if(first)
                    // {
                    //     std::cout << std::endl;
                    // }
                }
                // first = false;
            }
        }
    }
    
    costmap_grid->header.stamp = node->get_clock()->now();

    costmap_publisher->publish(*costmap_grid);


    // Override active hit points
    // if(override_active_hit_points)
    // {
    //     for(std::string& robot : robots)
    //     {
    //         auto it = last_posed_scan.find(robot);
    //         if (it != last_posed_scan.end())
    //         {
    //             PosedScan& last_robot_posed_scan = it->second;

    //             sensor_msgs::msg::LaserScan& scan = last_robot_posed_scan.scan;
    //             Pose2& pose = last_robot_posed_scan.pose;
                
    //             double angle = scan.angle_min;
    //             for (size_t i = 0; i < scan.ranges.size(); ++i)
    //             {
    //                 double range = scan.ranges[i];
        
    //                 if (range < scan.range_min || range > scan.range_max || std::isnan(range))
    //                 {
    //                     angle += scan.angle_increment;
    //                     continue;
    //                 }
                    
    //                 // Local laser point
    //                 double lx = range * cos(angle);
    //                 double ly = range * sin(angle);
        
    //                 // Transform to global
    //                 double gx = pose.x() + lx * cos(pose.theta()) - ly * sin(pose.theta());
    //                 double gy = pose.y() + lx * sin(pose.theta()) + ly * cos(pose.theta());
        
    //                 // Map indices of endpoint
    //                 int mx = static_cast<int>((gx - map_grid.info.origin.position.x) / map_resolution);
    //                 int my = static_cast<int>((gy - map_grid.info.origin.position.y) / map_resolution);
        
    //                 // Occupied cell
    //                 if (mx >= 0 && mx < width_cells && my >= 0 && my < height_cells)
    //                 {
    //                     int end_index = my * width_cells + mx;
    //                     map_grid.data[end_index] = 100;
    //                 }
        
    //                 angle += scan.angle_increment;
    //             }
    //         }
    //     }

    // }

    
    map_grid.header.stamp = node->get_clock()->now();

    map_publisher->publish(map_grid);

    for(auto& robot : robots)
    {
        robot_map_publishers[robot]->publish(robot_map_grid[robot]);
    }


#ifdef MAPPING_DEBUG
    auto end_mapping = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_mapping = end_mapping - start_mapping;
#endif

#ifdef MAPPING_DEBUG
    std::cout << "Mapping time: " << (duration_mapping.count() * 1000) << " ms" << std::endl;
    std::cout << std::endl;
#endif

    // Frontier cells detection

#ifdef FRONTIER_DETECTION_DEBUG
    auto start_frontier_detection = std::chrono::high_resolution_clock::now();
#endif


    // Frontier cells detection
    for(std::string& robot : robots)
    {
        auto it = last_posed_scan.find(robot);
        if (it != last_posed_scan.end()) {
            // Perform frontier detection for the robot
            PosedScan& last_robot_posed_scan = it->second;

            int rx = static_cast<int>((last_robot_posed_scan.pose.x() - map_grid.info.origin.position.x) / map_resolution);
            int ry = static_cast<int>((last_robot_posed_scan.pose.y() - map_grid.info.origin.position.y) / map_resolution);
    
            if(frontier_cells_detection_mode == "ewfd")
            {
                double active_area_radius = last_robot_posed_scan.scan.range_max + 2.5;
                expanding_wavefront_frontier_cells_detection(robot, rx, ry, active_area_radius);
            }

            // Merge frontier grid data
            for (size_t i = 0; i < frontier_grid.data.size(); i++)
            {
                if (frontier_grid_data[robot][i] == 100 && frontier_grid.data[i] == -1)
                {
                    frontier_grid.data[i] = 100;
                }
                else if (frontier_grid_data[robot][i] == 0)
                {
                    frontier_grid.data[i] = 0;
                }
            }
        }
    }


    frontier_grid.header.stamp = node->get_clock()->now();

    frontier_publisher->publish(frontier_grid);





    // Frontier clustering

    std::map<int, std::vector<std::pair<int, int>>> frontier_clusters = dbscan_frontier_clusters_detection();
    
    std::vector<std::pair<std::pair<double, double>, int>> frontier_centroids = frontier_centroids_detection(frontier_clusters);

    {
        // std::unique_lock lock(*frontiers_mutex);
        *frontiers = frontier_centroids;
    }
    
    publish_frontiers_marker(frontier_centroids);


#ifdef FRONTIER_DETECTION_DEBUG
    auto end_frontier_detection = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_frontier_detection = end_frontier_detection - start_frontier_detection;
#endif

#ifdef FRONTIER_DETECTION_DEBUG
    std::cout << "Frontier cells detection time: " << (duration_frontier_detection.count() * 1000) << " ms" << std::endl;
    std::cout << std::endl;
#endif



   
}
