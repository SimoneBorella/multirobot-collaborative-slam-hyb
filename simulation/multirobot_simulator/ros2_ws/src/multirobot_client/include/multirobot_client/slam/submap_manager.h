#ifndef SUBMAP_MANAGER_H
#define SUBMAP_MANAGER_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <mutex>

#include "data_types.h"

namespace multirobot_slam
{
    struct SubmapManagerParams
    {
        double submap_mapping_rate;
        double submap_resolution;
        double submap_width;
        double submap_height;

        double free_belief;
        double occ_belief;
        double distance_belief_factor;
        double log_odds_min;
        double log_odds_max;
        double noise_radius;
        double noise_std_dev;

        double obstacle_threshold;
        double free_threshold;

        SubmapManagerParams(
            double submap_mapping_rate = 1.0,
            double submap_resolution = 0.05,
            double submap_width = 12.0,
            double submap_height = 12.0,
            double free_belief = 0.4,
            double occ_belief = 0.85,
            double distance_belief_factor = 0.03,
            double log_odds_min = -100.0,
            double log_odds_max = 100.0,
            double noise_radius = 0.07,
            double noise_std_dev = 0.03,
            double obstacle_threshold = 0.70,
            double free_threshold = 0.30)
            : submap_mapping_rate(submap_mapping_rate),
              submap_resolution(submap_resolution),
              submap_width(submap_width),
              submap_height(submap_height),
              free_belief(free_belief),
              occ_belief(occ_belief),
              distance_belief_factor(distance_belief_factor),
              log_odds_min(log_odds_min),
              log_odds_max(log_odds_max),
              noise_radius(noise_radius),
              noise_std_dev(noise_std_dev),
              obstacle_threshold(obstacle_threshold),
              free_threshold(free_threshold) {}
    };

    class SubmapManager
    {
    public:
        SubmapManager();
        SubmapManager(SubmapManagerParams &params);

        static SubmapManagerParams params_from_yaml(std::string &params_path);
        void init(SubmapManagerParams &params);

        void update_keyframes(const std::vector<KeyFrame> &keyframes);
        void add_posed_scan(const PosedScan &posed_scan);
        void submaps_mapping();

        std::vector<Map> get_updated_submaps();

    private:
        Submap create_submap_from_keyframe(const KeyFrame& keyframe);
        int idx(int x, int y, int width);
        double probability_to_log_odds(int8_t prob);
        int8_t log_odds_to_probability(double log_odds);

        void bresenham_raytrace_submap(Submap& submap, int x0, int y0, int x1, int y1, bool hit_point);

        SubmapManagerParams params_;

        double free_belief_log_odds_;
        double occupied_belief_log_odds_;

        std::vector<KeyFrame> keyframes_;
        std::mutex keyframes_mutex_;

        std::vector<PosedScan> posed_scan_buffer_;
        std::mutex buffer_mutex_;

        std::vector<Submap> submaps_;
        std::vector<bool> submaps_updated_;
        std::vector<std::unique_ptr<std::mutex>> submap_mutexes_;
    };
}

#endif // SUBMAP_MANAGER_H
