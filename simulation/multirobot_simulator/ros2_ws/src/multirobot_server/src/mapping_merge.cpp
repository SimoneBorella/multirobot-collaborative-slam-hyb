#include "mapping_merge.h"


namespace multirobot_slam
{
    MappingMerge::MappingMerge()
        : map_updated_(true), costmap_updated_(true), frontiers_updated_(true), mapping_merge_thread_running(false)
    {
    }

    MappingMerge::MappingMerge(MappingMergeParams &params)
        : params_(params), map_updated_(true), costmap_updated_(true), frontiers_updated_(true), mapping_merge_thread_running(false)
    {
    }

    MappingMerge::~MappingMerge()
    {
        mapping_merge_thread_running.store(false);
        if (mapping_merge_thread.joinable())
        {
            mapping_merge_thread.join();
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

            if (config["epsilon"])
                p.epsilon = config["epsilon"].as<double>();
            
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
    }

    void MappingMerge::set_initial_poses(std::map<std::string, Pose> initial_poses)
    {
        initial_poses_ = initial_poses;
    }


    void MappingMerge::start()
    {
        if (mapping_merge_thread_running)
            return;

        mapping_merge_thread_running.store(true);

        mapping_merge_thread = std::thread([this]()
                                     {
                auto period = std::chrono::milliseconds(
                    static_cast<int>(1000.0 / params_.mapping_rate));

                auto next_time = std::chrono::steady_clock::now() + period;

                while (mapping_merge_thread_running.load())
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

        for (size_t i = 0; i < map_log_odds_update.indicies.size(); ++i)
        {
            int idx = map_log_odds_update.indicies[i];
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
        }
    }





    void MappingMerge::add_frontiers(const std::vector<Frontier> &frontiers, const std::string& robot)
    {
        robot_frontiers_[robot] = frontiers;
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

    std::vector<Frontier> MappingMerge::get_frontiers()
    {
        return frontiers_;
    }

    std::optional<std::vector<Frontier>> MappingMerge::get_frontiers_if_updated()
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



    std::vector<Frontier> MappingMerge::merge_frontiers_dbscan(const std::vector<Frontier>& frontiers)
    {
        std::vector<Frontier> merged_frontiers;

        if (frontiers.empty())
            return merged_frontiers;

        const double eps_sq = params_.epsilon * params_.epsilon;

        std::vector<int> labels(frontiers.size(), -1);
        int cluster_id = 0;

        for (size_t i = 0; i < frontiers.size(); ++i)
        {
            if (labels[i] != -1)
                continue;

            std::vector<size_t> neighbors;
            for (size_t j = 0; j < frontiers.size(); ++j)
            {
                double dx = frontiers[j].centroid.x() - frontiers[i].centroid.x();
                double dy = frontiers[j].centroid.y() - frontiers[i].centroid.y();
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq <= eps_sq)
                    neighbors.push_back(j);
            }

            if (neighbors.empty())
                continue;

            // Start cluster
            std::queue<size_t> q;
            for (auto n : neighbors)
                q.push(n);

            Frontier cluster_frontier;
            Eigen::Vector2d sum_centroid(0, 0);
            double total_size = 0;

            while (!q.empty())
            {
                size_t idx = q.front();
                q.pop();

                if (labels[idx] != -1)
                    continue;

                labels[idx] = cluster_id;

                sum_centroid += frontiers[idx].centroid * frontiers[idx].size;
                total_size += frontiers[idx].size;

                for (size_t j = 0; j < frontiers.size(); ++j)
                {
                    if (labels[j] != -1) continue;

                    double dx = frontiers[j].centroid.x() - frontiers[idx].centroid.x();
                    double dy = frontiers[j].centroid.y() - frontiers[idx].centroid.y();
                    double dist_sq = dx * dx + dy * dy;
                    if (dist_sq <= eps_sq)
                        q.push(j);
                }
            }

            if (total_size > 0)
            {
                cluster_frontier.centroid = sum_centroid / total_size;
                cluster_frontier.size = total_size;
                merged_frontiers.push_back(cluster_frontier);
            }

            cluster_id++;
        }

        return merged_frontiers;
    }



    void MappingMerge::mapping_merge()
    {
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

        int kernel_size = static_cast<int>(2 * std::floor(params_.costmap_kernel_distance / filtered_map_.resolution));

        for (int y = 0; y < filtered_map_.height; y++)
        {
            for (int x = 0; x < filtered_map_.width; x++)
            {
                int idx = y * filtered_map_.width + x;
                if (filtered_map_.data[idx] != 100)
                    continue;
        
                for (int dy = -kernel_size/2; dy <= kernel_size/2; ++dy)
                {
                    for (int dx = -kernel_size/2; dx <= kernel_size/2; ++dx)
                    {
                        int nx = x + dx;
                        int ny = y + dy;
            
                        if (nx < 0 || ny < 0 || nx >= filtered_map_.width || ny >= filtered_map_.height)
                            continue;
            
                        double distance = std::sqrt(dx * dx + dy * dy) * filtered_map_.resolution;
                        if (distance > params_.costmap_kernel_distance)
                            continue;
                        int8_t cost = static_cast<int8_t>(std::round(100 * std::exp(-params_.costmap_decay_rate * distance)));
            
                        int nidx = ny * filtered_map_.width + nx;
                        costmap_.data[nidx] = std::max(costmap_.data[nidx], cost);
                    }
                }
            }
        }
        
        costmap_updated_ = true;

        // Merge frontiers

        std::vector<Frontier> current_frontiers;

        for (const auto& [robot_name, frontiers] : robot_frontiers_)
        {
            // Transform to world frame
            const Pose& map_pose = initial_poses_.at(robot_name);

            double map_yaw = std::atan2(
                2.0 * (map_pose.orientation.w() * map_pose.orientation.z() +
                    map_pose.orientation.x() * map_pose.orientation.y()),
                1.0 - 2.0 * (map_pose.orientation.y() * map_pose.orientation.y() +
                            map_pose.orientation.z() * map_pose.orientation.z())
            );

            Eigen::Affine2d T_world_map =
                Eigen::Translation2d(map_pose.position.x(), map_pose.position.y()) *
                Eigen::Rotation2Dd(map_yaw);

            for (const auto& f : frontiers)
            {
                Frontier f_world = f;
                f_world.centroid = T_world_map * f.centroid;

                // Check if frontier has unknown space around it
                bool has_unknown = false;

                int gx_center = static_cast<int>((f_world.centroid.x() - map_.origin_position.x()) / map_.resolution);
                int gy_center = static_cast<int>((f_world.centroid.y() - map_.origin_position.y()) / map_.resolution);

                int radius = static_cast<int>(std::ceil(f_world.size / map_.resolution));

                for (int dy = -radius; dy <= radius && !has_unknown; ++dy)
                {
                    for (int dx = -radius; dx <= radius; ++dx)
                    {
                        int gx = gx_center + dx;
                        int gy = gy_center + dy;

                        if (gx < 0 || gy < 0 || gx >= filtered_map_.width || gy >= filtered_map_.height)
                            continue;

                        int idx = gy * filtered_map_.width + gx;

                        if (filtered_map_.data[idx] == -1)
                        {
                            has_unknown = true;
                            break;
                        }
                    }
                }

                if (has_unknown)
                    current_frontiers.push_back(f_world);
            }
        }
        
        frontiers_ = merge_frontiers_dbscan(current_frontiers);

        frontiers_updated_ = true;
    }   
}