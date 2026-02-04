#ifndef MAPPING_MERGE_H
#define MAPPING_MERGE_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <thread>
#include <atomic>
#include <queue>
#include <deque>
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

        double epsilon;
        int min_points;
        double min_frontier_size;

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
            double epsilon = 0.5,
            int min_points = 3,
            double min_frontier_size = 0.07)
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
              epsilon(epsilon),
              min_points(min_points),
              min_frontier_size(min_frontier_size){}
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

        Map get_map();
        std::optional<Map> get_map_if_updated();
        Map get_costmap();
        std::optional<Map> get_costmap_if_updated();
        Map get_frontier_map();
        std::optional<Map> get_frontier_map_if_updated();
        std::vector<Frontier> get_frontiers();
        std::optional<std::vector<Frontier>> get_frontiers_if_updated();

    private:
        double probability_to_log_odds(int8_t prob);
        int8_t log_odds_to_probability(double log_odds);

        std::vector<std::pair<int, int>> get_neighbors(int x, int y);
        std::map<int, std::vector<std::pair<int, int>>> dbscan_frontier_clusters_detection();
        std::vector<Frontier> frontier_centroids_detection(const std::map<int, std::vector<std::pair<int, int>>>& frontier_clusters);

        void mapping_merge();

        MappingMergeParams params_;

        std::map<std::string, Pose> initial_poses_;

        Map map_;
        std::vector<double> map_log_odds_data_;
        Map filtered_map_;
        Map costmap_;
        Map frontier_map_;
        std::vector<Frontier> frontiers_;

        bool map_updated_;
        bool costmap_updated_;
        bool frontier_map_updated_;
        bool frontiers_updated_;

        std::map<std::string, std::vector<Frontier>> robot_frontiers_data_;

        std::mutex map_mutex_;
        std::mutex frontier_map_mutex_;

        std::atomic<bool> mapping_merge_thread_running_;
        std::thread mapping_merge_thread_;
    };
}

#endif // MAPPING_MERGE_H
