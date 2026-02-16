#ifndef MAPPING_MERGE_H
#define MAPPING_MERGE_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <thread>
#include <atomic>
#include <queue>
#include <deque>
#include <unordered_set>
#include <mutex>
#include <optional>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "data_types.h"

namespace multirobot_slam
{
    struct MappingMergeParams
    {
        double mapping_rate;
        double map_resolution;
        double map_width;
        double map_height;
        
        double log_odds_min;
        double log_odds_max;

        double obstacle_threshold;
        double free_threshold;

        double costmap_kernel_distance;
        double costmap_decay_rate;

        double refinement_variance_threshold;
        int refinement_observations_threshold;

        double epsilon;
        int min_points;
        double min_frontier_size;
        double min_refinement_frontier_size;

        MappingMergeParams(
            double mapping_rate = 2.0,
            double map_resolution = 0.05,
            double map_width = 40.0,
            double map_height = 40.0,
            double log_odds_min = -10.0,
            double log_odds_max = 10.0,
            double obstacle_threshold = 0.7,
            double free_threshold = 0.3,
            double costmap_kernel_distance = 0.2,
            double costmap_decay_rate = 4.0,
            double refinement_variance_threshold = 0.05,
            int refinement_observations_threshold = 8,
            double epsilon = 0.5,
            int min_points = 3,
            double min_frontier_size = 0.07,
            double min_refinement_frontier_size = 0.015)
            : mapping_rate(mapping_rate),
              map_resolution(map_resolution),
              map_width(map_width),
              map_height(map_height),
              log_odds_min(log_odds_min),
              log_odds_max(log_odds_max),
              obstacle_threshold(obstacle_threshold),
              free_threshold(free_threshold),
              costmap_kernel_distance(costmap_kernel_distance),
              costmap_decay_rate(costmap_decay_rate),
              refinement_variance_threshold(refinement_variance_threshold),
              refinement_observations_threshold(refinement_observations_threshold),
              epsilon(epsilon),
              min_points(min_points),
              min_frontier_size(min_frontier_size),
              min_refinement_frontier_size(min_refinement_frontier_size) {}
    };



    class MappingMerge
    {
    public:
        MappingMerge();
        MappingMerge(MappingMergeParams &params);
        ~MappingMerge();

        static MappingMergeParams params_from_yaml(std::string &params_path);

        void init(MappingMergeParams &params);
        void set_initial_poses(std::map<std::string, Pose> initial_poses);
        void start();
        void add_map_log_odds_update(const MapLogOddsUpdate &map_log_odds_update, const std::string& robot);
        void add_frontier_map_update(const FrontierMapUpdate &frontier_map_update, const std::string& robot);

        void update_robot_poses(std::map<std::string, Pose> robot_poses);
        Map get_map();
        std::optional<Map> get_map_if_updated();
        Map get_costmap();
        std::optional<Map> get_costmap_if_updated();
        Map get_frontier_map();
        std::optional<Map> get_frontier_map_if_updated();
        std::vector<Frontier, Eigen::aligned_allocator<Frontier>> get_frontiers();
        
        std::optional<std::vector<Frontier, Eigen::aligned_allocator<Frontier>>> get_frontiers_if_updated();
        std::vector<Frontier, Eigen::aligned_allocator<Frontier>> get_refinement_frontiers();
        std::optional<std::vector<Frontier, Eigen::aligned_allocator<Frontier>>> get_refinement_frontiers_if_updated();

    private:
        double probability_to_log_odds(int8_t prob);
        int8_t log_odds_to_probability(double log_odds);

        std::vector<int> get_neighbors_indices(const Map& map, int index, int eps_cells, int eps_sq_cells);
        std::map<int, std::vector<std::pair<int, int>>> dbscan_frontier_detection(Map& map, double min_points, double epsilon);
        std::vector<Frontier, Eigen::aligned_allocator<Frontier>> frontier_centroids_detection(const std::map<int, std::vector<std::pair<int, int>>>& frontier_clusters, double min_frontier_size);

        std::vector<Frontier, Eigen::aligned_allocator<Frontier>> dbscan(const std::vector<Frontier, Eigen::aligned_allocator<Frontier>>& points, int min_points, double epsilon);

        void mapping_merge();

        MappingMergeParams params_;

        std::map<std::string, Pose> initial_poses_;

        std::map<std::string, Pose> robot_poses_;

        Map map_;
        std::vector<double> map_log_odds_data_;
        Map filtered_map_;
        Map costmap_;
        std::vector<int8_t> cost_lut_;
        int lut_kernel_size_;
        Map frontier_map_;
        std::unordered_set<int> global_visited_indices_;
        std::vector<int> map_observation_count_;
        std::vector<Frontier, Eigen::aligned_allocator<Frontier>> frontiers_;
        std::vector<Frontier, Eigen::aligned_allocator<Frontier>> refinement_frontiers_;

        std::vector<Eigen::Vector2d> unobservable_zones_;
        std::vector<int> refinement_frontier_persistence_counters_;

        bool map_updated_;
        bool costmap_updated_;
        bool frontier_map_updated_;
        bool frontiers_updated_;
        bool refinement_frontiers_updated_;

        std::map<std::string, std::vector<Frontier, Eigen::aligned_allocator<Frontier>>> robot_frontiers_data_;
        
        std::mutex robot_poses_mutex_;

        std::mutex map_mutex_;
        std::mutex frontier_map_mutex_;

        std::atomic<bool> mapping_merge_thread_running_;
        std::thread mapping_merge_thread_;
    };
}

#endif // MAPPING_MERGE_H
