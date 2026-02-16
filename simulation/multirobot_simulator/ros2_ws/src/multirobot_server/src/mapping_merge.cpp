#include "mapping_merge.h"


namespace multirobot_slam
{
    MappingMerge::MappingMerge()
        : map_updated_(true), costmap_updated_(true), frontiers_updated_(true), refinement_frontiers_updated_(true), mapping_merge_thread_running_(false)
    {
    }

    MappingMerge::MappingMerge(MappingMergeParams &params)
        : params_(params), map_updated_(true), costmap_updated_(true), frontiers_updated_(true), refinement_frontiers_updated_(true), mapping_merge_thread_running_(false)
    {
    }

    MappingMerge::~MappingMerge()
    {
        mapping_merge_thread_running_.store(false);
        if (mapping_merge_thread_.joinable())
        {
            mapping_merge_thread_.join();
        }
    }


    MappingMergeParams MappingMerge::params_from_yaml(std::string &params_path)
    {
        MappingMergeParams p;

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

            if (config["log_odds_min"])
                p.log_odds_min = config["log_odds_min"].as<double>();
            if (config["log_odds_max"])
                p.log_odds_max = config["log_odds_max"].as<double>();

            if (config["obstacle_threshold"])
                p.obstacle_threshold = config["obstacle_threshold"].as<double>();
            if (config["free_threshold"])
                p.free_threshold = config["free_threshold"].as<double>();

            if (config["costmap_kernel_distance"])
                p.costmap_kernel_distance = config["costmap_kernel_distance"].as<double>();
            if (config["costmap_decay_rate"])
                p.costmap_decay_rate = config["costmap_decay_rate"].as<double>();

            if (config["refinement_variance_threshold"])
                p.refinement_variance_threshold = config["refinement_variance_threshold"].as<double>();
            if (config["refinement_observations_threshold"])
                p.refinement_observations_threshold = config["refinement_observations_threshold"].as<int>();

            if (config["epsilon"])
                p.epsilon = config["epsilon"].as<double>();
            if (config["min_points"])
                p.min_points = config["min_points"].as<int>();
            if (config["min_frontier_size"])
                p.min_frontier_size = config["min_frontier_size"].as<double>();
            if (config["min_refinement_frontier_size"])
                p.min_refinement_frontier_size = config["min_refinement_frontier_size"].as<double>();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
            std::cerr << "Using default parameters.\n";
        }

        return p;
    }


    void MappingMerge::init(MappingMergeParams &params)
    {
        params_ = params;

        // Initialize maps
        map_.resolution = static_cast<float>(params_.map_resolution);
        map_.width = static_cast<int>(params_.map_width / params_.map_resolution);
        map_.height = static_cast<int>(params_.map_height / params_.map_resolution);
        map_.origin_position = Eigen::Vector3d(-params_.map_width / 2.0, -params_.map_height / 2.0, 0.0);
        map_.origin_orientation = Eigen::Quaterniond::Identity();
        map_.data.assign(map_.width * map_.height, -1);

        map_log_odds_data_.assign(map_.width * map_.height, probability_to_log_odds(0.5));

        filtered_map_.resolution = map_.resolution;
        filtered_map_.width = map_.width;
        filtered_map_.height = map_.height;
        filtered_map_.origin_position = map_.origin_position;
        filtered_map_.origin_orientation = map_.origin_orientation;
        filtered_map_.data.resize(map_.width * map_.height, -1);

        costmap_.resolution = map_.resolution;
        costmap_.width = map_.width;
        costmap_.height = map_.height;
        costmap_.origin_position = map_.origin_position;
        costmap_.origin_orientation = map_.origin_orientation;
        costmap_.data.resize(map_.width * map_.height, -1);

        frontier_map_.resolution = map_.resolution;
        frontier_map_.width = map_.width;
        frontier_map_.height = map_.height;
        frontier_map_.origin_position = map_.origin_position;
        frontier_map_.origin_orientation = map_.origin_orientation;
        frontier_map_.data.resize(map_.width * map_.height, -1);

        map_observation_count_.assign(map_.width * map_.height, 0);


        int kernel_radius = static_cast<int>(std::ceil(params_.costmap_kernel_distance / map_.resolution));
        lut_kernel_size_ = 2 * kernel_radius + 1;
        cost_lut_.assign(lut_kernel_size_ * lut_kernel_size_, 0);

        int center = kernel_radius;
        for (int dy = -kernel_radius; dy <= kernel_radius; ++dy) {
            for (int dx = -kernel_radius; dx <= kernel_radius; ++dx) {
                double distance = std::sqrt(dx * dx + dy * dy) * map_.resolution;
                if (distance <= params_.costmap_kernel_distance) {
                    int8_t cost = static_cast<int8_t>(std::round(100 * std::exp(-params_.costmap_decay_rate * distance)));
                    cost_lut_[(dy + center) * lut_kernel_size_ + (dx + center)] = cost;
                }
            }
        }
    }

    void MappingMerge::set_initial_poses(std::map<std::string, Pose> initial_poses)
    {
        initial_poses_ = initial_poses;
    }


    void MappingMerge::start()
    {
        if (mapping_merge_thread_running_)
            return;

        mapping_merge_thread_running_.store(true);

        mapping_merge_thread_ = std::thread([this]()
                                     {
                auto period = std::chrono::milliseconds(
                    static_cast<int>(1000.0 / params_.mapping_rate));

                auto next_time = std::chrono::steady_clock::now() + period;

                while (mapping_merge_thread_running_.load())
                {
                    mapping_merge();

                    std::this_thread::sleep_until(next_time);
                    next_time += period;
                } });
    }



    void MappingMerge::add_map_log_odds_update(const MapLogOddsUpdate &map_log_odds_update, const std::string& robot)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);

        const Pose& map_pose = initial_poses_.at(robot);

        double map_yaw = std::atan2(
            2.0 * (map_pose.orientation.w() * map_pose.orientation.z() +
                map_pose.orientation.x() * map_pose.orientation.y()),
            1.0 - 2.0 * (map_pose.orientation.y() * map_pose.orientation.y() +
                        map_pose.orientation.z() * map_pose.orientation.z())
        );

        Eigen::Affine2d T_world_map =
            Eigen::Translation2d(
                map_pose.position.x(),
                map_pose.position.y()) *
            Eigen::Rotation2Dd(map_yaw);

        double local_yaw = map_log_odds_update.origin_orientation.toRotationMatrix().eulerAngles(0,1,2)[2];

        Eigen::Affine2d T_map_local =
            Eigen::Translation2d(
                map_log_odds_update.origin_position.x(),
                map_log_odds_update.origin_position.y()) *
            Eigen::Rotation2Dd(local_yaw);

        Eigen::Affine2d T_world_local = T_world_map * T_map_local;

        for (size_t i = 0; i < map_log_odds_update.indices.size(); ++i)
        {
            int idx = map_log_odds_update.indices[i];
            int lx = idx % map_log_odds_update.width;
            int ly = idx / map_log_odds_update.width;

            Eigen::Vector2d p_local(
                (lx + 0.5) * map_log_odds_update.resolution,
                (ly + 0.5) * map_log_odds_update.resolution);

            Eigen::Vector2d p_world = T_world_local * p_local;

            int gx = static_cast<int>(
                (p_world.x() - map_.origin_position.x()) / map_.resolution);
            int gy = static_cast<int>(
                (p_world.y() - map_.origin_position.y()) / map_.resolution);

            if (gx < 0 || gy < 0 || gx >= map_.width || gy >= map_.height)
                continue;

            int gidx = gy * map_.width + gx;

            map_log_odds_data_[gidx] = std::clamp(
                map_log_odds_data_[gidx] + map_log_odds_update.delta_log_odds[i],
                params_.log_odds_min,
                params_.log_odds_max);

            map_.data[gidx] =
                log_odds_to_probability(map_log_odds_data_[gidx]);

            map_observation_count_[gidx] += 1;
        }
    }



    void MappingMerge::add_frontier_map_update(const FrontierMapUpdate &frontier_map_update, const std::string& robot)
    {
        std::lock_guard<std::mutex> lock(frontier_map_mutex_);

        const Pose& map_pose = initial_poses_.at(robot);

        double map_yaw = std::atan2(
            2.0 * (map_pose.orientation.w() * map_pose.orientation.z() +
                map_pose.orientation.x() * map_pose.orientation.y()),
            1.0 - 2.0 * (map_pose.orientation.y() * map_pose.orientation.y() +
                        map_pose.orientation.z() * map_pose.orientation.z())
        );

        Eigen::Affine2d T_world_map =
            Eigen::Translation2d(
                map_pose.position.x(),
                map_pose.position.y()) *
            Eigen::Rotation2Dd(map_yaw);

        double local_yaw =
            frontier_map_update.origin_orientation
                .toRotationMatrix()
                .eulerAngles(0,1,2)[2];

        Eigen::Affine2d T_map_local =
            Eigen::Translation2d(
                frontier_map_update.origin_position.x(),
                frontier_map_update.origin_position.y()) *
            Eigen::Rotation2Dd(local_yaw);

        Eigen::Affine2d T_world_local = T_world_map * T_map_local;

        // Explored cells
        for (size_t i = 0; i < frontier_map_update.explored_indices.size(); ++i)
        {
            int idx = frontier_map_update.explored_indices[i];
            int lx = idx % frontier_map_update.width;
            int ly = idx / frontier_map_update.width;

            Eigen::Vector2d p_local(
                (lx + 0.5) * frontier_map_update.resolution,
                (ly + 0.5) * frontier_map_update.resolution);

            Eigen::Vector2d p_world = T_world_local * p_local;

            int gx = static_cast<int>(
                (p_world.x() - map_.origin_position.x()) / map_.resolution);
            int gy = static_cast<int>(
                (p_world.y() - map_.origin_position.y()) / map_.resolution);

            if (gx < 0 || gy < 0 || gx >= map_.width || gy >= map_.height)
                continue;

            int gidx = gy * map_.width + gx;

            frontier_map_.data[gidx] = 0;

            global_visited_indices_.insert(gidx);
        }

        // Frontier cells
        for (size_t i = 0; i < frontier_map_update.frontier_indices.size(); ++i)
        {
            int idx = frontier_map_update.frontier_indices[i];
            int lx = idx % frontier_map_update.width;
            int ly = idx / frontier_map_update.width;

            Eigen::Vector2d p_local(
                (lx + 0.5) * frontier_map_update.resolution,
                (ly + 0.5) * frontier_map_update.resolution);

            Eigen::Vector2d p_world = T_world_local * p_local;

            int gx = static_cast<int>(
                (p_world.x() - map_.origin_position.x()) / map_.resolution);
            int gy = static_cast<int>(
                (p_world.y() - map_.origin_position.y()) / map_.resolution);

            if (gx < 0 || gy < 0 || gx >= map_.width || gy >= map_.height)
                continue;

            int gidx = gy * map_.width + gx;

            if (frontier_map_.data[gidx] != 0)
            {
                frontier_map_.data[gidx] = 100;
            }
        }
    }

    void MappingMerge::update_robot_poses(std::map<std::string, Pose> robot_poses)
    {
        std::lock_guard<std::mutex> lock(robot_poses_mutex_);
        robot_poses_ = robot_poses;
    }


    Map MappingMerge::get_map()
    {
        return map_;
    }

    std::optional<Map> MappingMerge::get_map_if_updated()
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

    Map MappingMerge::get_costmap()
    {
        return costmap_;
    }

    std::optional<Map> MappingMerge::get_costmap_if_updated()
    {
        if (costmap_updated_)
        {
            costmap_updated_ = false;
            return costmap_;
        }
        else
        {
            return std::nullopt;
        }
    }

    Map MappingMerge::get_frontier_map()
    {
        return frontier_map_;
    }

    std::optional<Map> MappingMerge::get_frontier_map_if_updated()
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


    std::vector<Frontier, Eigen::aligned_allocator<Frontier>> MappingMerge::get_frontiers()
    {
        return frontiers_;
    }

    std::optional<std::vector<Frontier, Eigen::aligned_allocator<Frontier>>> MappingMerge::get_frontiers_if_updated()
    {
        if (frontiers_updated_)
        {
            frontiers_updated_ = false;
            return frontiers_;
        }
        else
        {
            return std::nullopt;
        }
    }

    std::vector<Frontier, Eigen::aligned_allocator<Frontier>> MappingMerge::get_refinement_frontiers()
    {
        return refinement_frontiers_;
    }

    std::optional<std::vector<Frontier, Eigen::aligned_allocator<Frontier>>> MappingMerge::get_refinement_frontiers_if_updated()
    {
        if (refinement_frontiers_updated_)
        {
            refinement_frontiers_updated_ = false;
            return refinement_frontiers_;
        }
        else
        {
            return std::nullopt;
        }
    }


    double MappingMerge::probability_to_log_odds(int8_t prob)
    {    
        double p = static_cast<double>(prob) / 100.0;
        
        p = std::clamp(p, 0.01, 0.99);
        
        return std::log(p / (1.0 - p));
    }

    int8_t MappingMerge::log_odds_to_probability(double log_odds)
    {
        double p = 1.0 / (1.0 + std::exp(-log_odds));
        int8_t prob = static_cast<int8_t>(std::round(p * 100.0));

        if (prob <= 0) prob = 0;
        if (prob >= 100) prob = 100;

        return prob;
    }


    std::vector<int> MappingMerge::get_neighbors_indices(const Map& map, int index, int eps_cells, int eps_sq_cells)
    {
        std::vector<int> neighbors;
        neighbors.reserve(64);

        int w = map.width;
        int h = map.height;
        int x = index % w;
        int y = index / w;

        int x_min = std::max(0, x - eps_cells);
        int x_max = std::min(w - 1, x + eps_cells);
        int y_min = std::max(0, y - eps_cells);
        int y_max = std::min(h - 1, y + eps_cells);

        for (int ny = y_min; ny <= y_max; ++ny) {
            int row_offset = ny * w;
            int dy = ny - y;
            int dy_sq = dy * dy;

            for (int nx = x_min; nx <= x_max; ++nx) {
                int dx = nx - x;
                int dist_sq = dx * dx + dy_sq;

                if (dist_sq <= eps_sq_cells) {
                    int nidx = row_offset + nx;
                    if (map.data[nidx] == 100) {
                        neighbors.push_back(nidx);
                    }
                }
            }
        }
        return neighbors;
    }

    std::map<int, std::vector<std::pair<int, int>>> MappingMerge::dbscan_frontier_detection(Map& map, double min_points, double epsilon)
    {
        int w = map.width;
        int h = map.height;
        std::vector<int> labels(w * h, 0); 
        std::map<int, std::vector<std::pair<int, int>>> clusters;
        int cluster_id = 0;

        int eps_cells = std::ceil(epsilon / map.resolution);
        int eps_sq_cells = eps_cells * eps_cells;

        for (int i = 0; i < w * h; ++i) {
            if (map.data[i] != 100 || labels[i] != 0) continue;

            std::vector<int> neighbors = get_neighbors_indices(map, i, eps_cells, eps_sq_cells);

            if (neighbors.size() < static_cast<size_t>(min_points)) {
                labels[i] = -1;
                continue;
            }

            cluster_id++;
            labels[i] = cluster_id;
            std::queue<int> q;
            for (int n : neighbors) {
                if (labels[n] == 0) {
                    labels[n] = cluster_id;
                    q.push(n);
                }
            }

            while (!q.empty()) {
                int curr = q.front(); q.pop();
                clusters[cluster_id].push_back({curr % w, curr / h});

                std::vector<int> next_neighbors = get_neighbors_indices(map, curr, eps_cells, eps_sq_cells);
                if (next_neighbors.size() >= static_cast<size_t>(min_points)) {
                    for (int n : next_neighbors) {
                        if (labels[n] <= 0) {
                            if (labels[n] == 0) q.push(n);
                            labels[n] = cluster_id;
                        }
                    }
                }
            }
        }
        return clusters;
    }


    std::vector<Frontier, Eigen::aligned_allocator<Frontier>> MappingMerge::frontier_centroids_detection(const std::map<int, std::vector<std::pair<int, int>>>& frontier_clusters, double min_frontier_size)
    {
        std::vector<Frontier, Eigen::aligned_allocator<Frontier>> frontiers;

        for (const auto& [cluster_id, cluster] : frontier_clusters)
        {
            double cluster_size = cluster.size() * frontier_map_.resolution * frontier_map_.resolution;
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
            double world_x = closest_point.first * frontier_map_.resolution + frontier_map_.origin_position.x() + frontier_map_.resolution / 2.0;
            double world_y = closest_point.second * frontier_map_.resolution + frontier_map_.origin_position.y() + frontier_map_.resolution / 2.0;

            Frontier frontier;
            frontier.centroid = Eigen::Vector2d(world_x, world_y);
            frontier.size = cluster_size;
            
            frontiers.push_back(frontier);
        }

        return frontiers;
    }


    std::vector<Frontier, Eigen::aligned_allocator<Frontier>> MappingMerge::dbscan(const std::vector<Frontier, Eigen::aligned_allocator<Frontier>>& points, int min_points, double epsilon)
    {
        int n = points.size();
        if (n == 0) return {};

        std::vector<int> labels(n, 0);
        int cluster_id = 0;
        std::vector<Frontier, Eigen::aligned_allocator<Frontier>> results;
        double epsilon_sq = epsilon * epsilon;

        for (int i = 0; i < n; ++i) {
            if (labels[i] != 0 || !std::isfinite(points[i].centroid.x())) continue;

            std::vector<int> neighbors;
            for(int j = 0; j < n; ++j) {
                if (!std::isfinite(points[j].centroid.x())) continue;
                if ((points[i].centroid - points[j].centroid).squaredNorm() < epsilon_sq) {
                    neighbors.push_back(j);
                }
            }

            if (neighbors.size() < (size_t)min_points) {
                labels[i] = -1;
                continue;
            }

            cluster_id++;
            labels[i] = cluster_id;
            
            Eigen::Vector2d cluster_centroid_sum = points[i].centroid;
            double cluster_total_size = points[i].size;
            int cluster_count = 1;

            std::queue<int> q;
            for (int idx : neighbors) {
                if (idx == i) continue;
                if (labels[idx] == 0) {
                    labels[idx] = cluster_id;
                    q.push(idx);
                }
            }

            while (!q.empty()) {
                int curr = q.front(); q.pop();
                
                cluster_centroid_sum += points[curr].centroid;
                cluster_total_size += points[curr].size;
                cluster_count++;

                std::vector<int> next_neighbors;
                for(int j = 0; j < n; ++j) {
                    if (!std::isfinite(points[j].centroid.x())) continue;
                    if ((points[curr].centroid - points[j].centroid).squaredNorm() < epsilon_sq) {
                        next_neighbors.push_back(j);
                    }
                }

                if (next_neighbors.size() >= (size_t)min_points) {
                    for (int next_idx : next_neighbors) {
                        if (labels[next_idx] <= 0) {
                            if (labels[next_idx] == 0) {
                                q.push(next_idx);
                            }
                            labels[next_idx] = cluster_id;
                        }
                    }
                }
            }

            if (cluster_count > 0) {
                Frontier f;
                f.centroid = cluster_centroid_sum / static_cast<double>(cluster_count);
                f.size = cluster_total_size;
                
                // Verifica finale anti-NaN
                if (std::isfinite(f.centroid.x()) && std::isfinite(f.centroid.y())) {
                    results.push_back(f);
                }
            }
        }
        return results;
    }



    void MappingMerge::mapping_merge()
    {
        // auto start = std::chrono::high_resolution_clock::now();

        map_updated_ = true;

        // Create filtered map
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
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
        }

        // Create costmap

        // Costmap generation
        std::fill(costmap_.data.begin(), costmap_.data.end(), 0);

        int kernel_radius = lut_kernel_size_ / 2;
        int width = filtered_map_.width;
        int height = filtered_map_.height;

        for (int y = 0; y < height; ++y) {
            int row_offset = y * width;
            for (int x = 0; x < width; ++x) {
                if (filtered_map_.data[row_offset + x] != 100) continue;

                int y_min = std::max(0, y - kernel_radius);
                int y_max = std::min(height - 1, y + kernel_radius);
                int x_min = std::max(0, x - kernel_radius);
                int x_max = std::min(width - 1, x + kernel_radius);

                for (int ny = y_min; ny <= y_max; ++ny) {
                    int lut_y = ny - y + kernel_radius;
                    int lut_row_ptr = lut_y * lut_kernel_size_;
                    int map_row_ptr = ny * width;

                    for (int nx = x_min; nx <= x_max; ++nx) {
                        int lut_x = nx - x + kernel_radius;
                        int8_t cost = cost_lut_[lut_row_ptr + lut_x];
                        
                        if (cost > 0) {
                            int8_t& current_cost = costmap_.data[map_row_ptr + nx];
                            if (cost > current_cost) {
                                current_cost = cost;
                            }
                        }
                    }
                }
            }
        }
        costmap_updated_ = true;



























        





        // Unobservability check
        if (!refinement_frontiers_raw_.empty()) 
        {
            if (last_refinement_frontier_avg_obs_.size() != refinement_frontiers_raw_.size()) {
                last_refinement_frontier_avg_obs_.resize(refinement_frontiers_raw_.size(), 0.0);
            }

            std::lock_guard<std::mutex> lock(robot_poses_mutex_);
            
            for (size_t i = 0; i < refinement_frontiers_raw_.size(); ) 
            {
                const auto& f = refinement_frontiers_raw_[i];
                bool removed = false;

                for (const auto& [_, robot_pose] : robot_poses_) 
                {
                    double dist_to_robot = (f.centroid - Eigen::Vector2d(robot_pose.position.x(), robot_pose.position.y())).norm();
                    
                    if (dist_to_robot < 1.5) 
                    {
                        int gx = static_cast<int>((f.centroid.x() - map_.origin_position.x()) / map_.resolution);
                        int gy = static_cast<int>((f.centroid.y() - map_.origin_position.y()) / map_.resolution);
                        
                        double sum_obs = 0;
                        int valid_neighbors = 0;

                        for (int dy = -2; dy <= 2; dy++)
                        {
                            for (int dx = -2; dx <= 2; dx++)
                            {
                                int nidx = (gy + dy) * map_.width + (gx + dx);
                                
                                if (filtered_map_.data[nidx] != -1)
                                {
                                    sum_obs += map_observation_count_[nidx];
                                    valid_neighbors++;
                                }
                            }
                        }


                        
                        double avg_obs = 0.0;
                        
                        if(valid_neighbors > 0)
                            avg_obs = sum_obs/valid_neighbors;
                        

                        if (avg_obs <= last_refinement_frontier_avg_obs_[i] && last_refinement_frontier_avg_obs_[i] > 0) 
                        {
                            unobservable_zones_.push_back(f.centroid);
                            refinement_frontiers_raw_.erase(refinement_frontiers_raw_.begin() + i);
                            last_refinement_frontier_avg_obs_.erase(last_refinement_frontier_avg_obs_.begin() + i);
                            removed = true;
                            break; 
                        }
                        
                        last_refinement_frontier_avg_obs_[i] = avg_obs;
                    }
                }

                if (!removed) i++;
            }
        }











        // Check previously detected refinement frontiers
        if (!refinement_frontiers_raw_.empty()) 
        {
            for (size_t i = 0; i < refinement_frontiers_raw_.size();)
            {
                const auto& f = refinement_frontiers_raw_[i];
                int gx = static_cast<int>((f.centroid.x() - map_.origin_position.x()) / map_.resolution);
                int gy = static_cast<int>((f.centroid.y() - map_.origin_position.y()) / map_.resolution);

                bool should_remove = false;

                if (gx < 2 || gx >= map_.width - 2 || gy < 2 || gy >= map_.height - 2)
                {
                    should_remove = true;
                }
                else
                {
                    double sum_obs = 0;
                    double sum_variance = 0;
                    int valid_neighbors = 0;

                    for (int dy = -2; dy <= 2; dy++)
                    {
                        for (int dx = -2; dx <= 2; dx++)
                        {
                            int nidx = (gy + dy) * map_.width + (gx + dx);
                            if (filtered_map_.data[nidx] != -1)
                            {
                                sum_obs += map_observation_count_[nidx];
                                double p = map_.data[nidx] / 100.0;
                                sum_variance += (p * (1.0 - p));
                                valid_neighbors++;
                            }
                        }
                    }

                    if (valid_neighbors > 0)
                    {
                        double avg_obs = sum_obs / valid_neighbors;
                        double avg_var = sum_variance / valid_neighbors;

                        bool still_uncertain = (avg_var > params_.refinement_variance_threshold || avg_obs < params_.refinement_observations_threshold);
                        should_remove = !still_uncertain;
                    } else
                    {
                        should_remove = true;
                    }
                }

                if (should_remove)
                {
                    refinement_frontiers_raw_.erase(refinement_frontiers_raw_.begin() + i);
                    last_refinement_frontier_avg_obs_.erase(last_refinement_frontier_avg_obs_.begin() + i);
                } else
                {
                    i++;
                }
            }
        }


        // Monte carlo sampling around robot positions
        int n_samples = 250;
        double max_radius = 10.0;

        {
            std::lock_guard<std::mutex> lock(robot_poses_mutex_);
            for (const auto& [_, pose] : robot_poses_)
            {
                for (int i = 0; i < n_samples; ++i)
                {
    
                    double r = max_radius * std::sqrt((double)rand() / RAND_MAX);
                    double theta = 2.0 * M_PI * ((double)rand() / RAND_MAX);
                    
                    double wx = pose.position.x() + r * std::cos(theta);
                    double wy = pose.position.y() + r * std::sin(theta);

                    // Check if inside unobservable zone
                    bool in_unobservable_zone = false;
                    for (const auto& uz : unobservable_zones_) {
                        if ((Eigen::Vector2d(wx, wy) - uz).norm() < 1.5) {
                            in_unobservable_zone = true;
                            break;
                        }
                    }
                    if (in_unobservable_zone)
                        continue;
    
                    int gx = static_cast<int>((wx - map_.origin_position.x()) / map_.resolution);
                    int gy = static_cast<int>((wy - map_.origin_position.y()) / map_.resolution);
    
                    if (gx >= 2 && gx < map_.width - 2 && gy >= 2 && gy < map_.height - 2)
                    {
                        int idx = gy * map_.width + gx;
    
                        if (filtered_map_.data[idx] == 0)
                        {
                            
                            double sum_obs = 0;
                            double sum_variance = 0;
                            int valid_neighbors = 0;
                            bool near_obstacle = false;
                            bool near_unknown = false;
    
                            for(int dy = -2; dy <= 2; dy++)
                            {
                                for(int dx = -2; dx <= 2; dx++)
                                {
                                    int nidx = (gy + dy) * map_.width + (gx + dx);
    
                                    // Unknown zone detection
                                    if (filtered_map_.data[nidx] == -1)
                                    {
                                        near_unknown = true;
                                    }
                                    
                                    // Obstacle zone detection
                                    if (filtered_map_.data[nidx] == 100)
                                    {
                                        near_obstacle = true;
                                    }
    
                                    // Uncertanty statistics
                                    if (filtered_map_.data[nidx] != -1)
                                    {
                                        sum_obs += map_observation_count_[nidx];
                                        double p = map_.data[nidx] / 100.0;
                                        sum_variance += (p * (1.0 - p));
                                        valid_neighbors++;
                                    }
                                }
                            }
    
                            if (!near_unknown && valid_neighbors > 0 && near_obstacle)
                            {
                                double avg_obs = sum_obs / valid_neighbors;
                                double avg_var = sum_variance / valid_neighbors;
    
                                if (avg_var > params_.refinement_variance_threshold || avg_obs < params_.refinement_observations_threshold)
                                {
                                    Frontier cand;
                                    cand.centroid = Eigen::Vector2d(wx, wy);
                                    cand.size = -1.0;
                                    refinement_frontiers_raw_.push_back(cand);
                                    last_refinement_frontier_avg_obs_.push_back(avg_obs);
                                }
                            }
                        }
                    }
                }
            }
        }



        



        // Detect frontiers
        
        {
            std::lock_guard<std::mutex> lock(frontier_map_mutex_);
            
            auto frontier_clusters = dbscan_frontier_detection(frontier_map_, params_.min_points, params_.epsilon);
            
            
            std::map<int, std::vector<std::pair<int, int>>> discarded_frontier_clusters;
            for (const auto& [cluster_id, cluster] : frontier_clusters)
            {
                double cluster_size = cluster.size() * frontier_map_.resolution * frontier_map_.resolution;
                if (cluster_size < params_.min_frontier_size)
                {
                    discarded_frontier_clusters[cluster_id] = cluster;
                }
            }
            for (const auto& [cluster_id, cluster] : discarded_frontier_clusters)
            {
                frontier_clusters.erase(cluster_id);
            }


            frontiers_ = frontier_centroids_detection(frontier_clusters, params_.min_frontier_size);
            std::vector<Frontier, Eigen::aligned_allocator<Frontier>> discarded_frontiers_ = frontier_centroids_detection(discarded_frontier_clusters, 0);

            for (auto& df : discarded_frontiers_)
            {
                df.size = -1.0;
                refinement_frontiers_raw_.push_back(df);
                last_refinement_frontier_avg_obs_.push_back(0.0);
            }
        }




        // Dbscan on candidate raw refinemnet frontiers
        refinement_frontiers_.clear();
        if (!refinement_frontiers_raw_.empty()) {
            auto r_frontiers = dbscan(refinement_frontiers_raw_, 1, 1.2);
            double min_dist_sq = 0.5 * 0.5;

            for (const auto& rf : r_frontiers)
            {
                bool too_close = false;
                for (const auto& ff : frontiers_)
                {
                    if ((rf.centroid - ff.centroid).squaredNorm() < min_dist_sq)
                    {
                        too_close = true;
                        break;
                    }
                }

                if (!too_close)
                    refinement_frontiers_.push_back(rf);
            }
        }


        frontier_map_updated_ = true;
        frontiers_updated_ = true;
        refinement_frontiers_updated_ = true;



        
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> duration = end - start;

        // std::cout << "Mapping merge time: " << (duration.count() * 1000) << " ms (" << 1/duration.count() << " Hz)" << std::endl;
    }   
}