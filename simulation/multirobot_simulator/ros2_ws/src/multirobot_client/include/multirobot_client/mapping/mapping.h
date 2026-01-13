#ifndef MAPPING_H
#define MAPPING_H

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

#include "mapping_data_types.h"

namespace mapping
{
    struct MappingParams
    {
        double mapping_rate;
        double map_resolution;
        double map_width;
        double map_height;

        double free_belief;
        double occupied_belief;
        double distance_belief_factor;

        double noise_model_radius;
        double noise_model_std_dev;
        
        double log_odds_min;
        double log_odds_max;

        double obstacle_threshold;
        double free_threshold;

        double frontier_del_obstacles_radius;

        double epsilon;
        int min_points;

        double min_frontier_size;

        MappingParams(
            double mapping_rate = 2.0,
            double map_resolution = 0.05,
            double map_width = 40.0,
            double map_height = 40.0,
            double free_belief = 0.38,
            double occupied_belief = 0.80,
            double distance_belief_factor = 0.03,
            double noise_model_radius = 0.07,
            double noise_model_std_dev = 0.02,
            double log_odds_min = -10.0,
            double log_odds_max = 10.0,
            double obstacle_threshold = 0.7,
            double free_threshold = 0.3,
            double frontier_del_obstacles_radius = 0.18,
            double epsilon = 0.5,
            int min_points = 3,
            double min_frontier_size = 0.07)
            : mapping_rate(mapping_rate),
              map_resolution(map_resolution),
              map_width(map_width),
              map_height(map_height),
              free_belief(free_belief),
              occupied_belief(occupied_belief),
              distance_belief_factor(distance_belief_factor),
              noise_model_radius(noise_model_radius),
              noise_model_std_dev(noise_model_std_dev),
              log_odds_min(log_odds_min),
              log_odds_max(log_odds_max),
              obstacle_threshold(obstacle_threshold),
              free_threshold(free_threshold),
              frontier_del_obstacles_radius(frontier_del_obstacles_radius),
              epsilon(epsilon),
              min_points(min_points),
              min_frontier_size(min_frontier_size){}
    };


    class Mapping
    {
    public:
        Mapping();
        Mapping(MappingParams &params);
        ~Mapping();

        static MappingParams params_from_yaml(std::string &params_path);

        void init(MappingParams &params);
        void start();
        void add_posed_scan(const PosedScan &posed_scan);

        std::optional<MapLogOddsUpdate> get_map_log_odds_update();
        Map get_map();
        std::optional<Map> get_map_if_updated();
        Map get_frontier_map();
        std::optional<Map> get_frontier_map_if_updated();
        std::vector<Frontier> get_frontiers();
        std::optional<std::vector<Frontier>> get_frontiers_if_updated();

    private:
        double probability_to_log_odds(int8_t prob);
        int8_t log_odds_to_probability(double log_odds);

        void bresenham_raytrace(int x0, int y0, int x1, int y1, bool hit_point);
        void expanding_wavefront_frontier_cells_detection(int rx, int ry, double active_area_radius);
        std::vector<std::pair<int, int>> get_neighbors(int x, int y);
        std::map<int, std::vector<std::pair<int, int>>> dbscan_frontier_clusters_detection();
        std::vector<Frontier> frontier_centroids_detection(const std::map<int, std::vector<std::pair<int, int>>>& frontier_clusters);

        void mapping();

        MappingParams params_;

        std::deque<PosedScan> posed_scan_buffer_;
        std::mutex buffer_mutex_;

        double free_belief_log_odds;
        double occupied_belief_log_odds;

        std::unordered_map<int, float> local_log_odds_delta_;

        MapLogOddsUpdate map_log_odds_update_;
        std::mutex map_log_odds_update_mutex_;

        std::vector<double> map_log_odds_data_;
        Map map_;
        Map filtered_map_;
        Map frontier_map_;

        bool map_updated_;

        std::vector<Frontier> frontiers_;

        bool ewfd_first_;
        std::vector<bool> ewfd_visited;

        bool frontier_map_updated_;
        bool frontiers_updated_;

        std::atomic<bool> mapping_thread_running;
        std::thread mapping_thread;
    };


}


#endif // MAPPING_H
