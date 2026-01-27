#ifndef SUBMAP_MANAGER_H
#define SUBMAP_MANAGER_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

#include "data_types.h"

namespace multirobot_slam
{
    struct SubmapManagerParams
    {
        double submap_mapping_rate;
        double submap_resolution;
        double submap_width;
        double submap_height;

        SubmapManagerParams(
            double submap_mapping_rate = 1.0,
            double submap_resolution = 0.05,
            double submap_width = 12.0,
            double submap_height = 12.0)
            : submap_mapping_rate(submap_mapping_rate),
              submap_resolution(submap_resolution),
              submap_width(submap_width),
              submap_height(submap_height) {}
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

        Map get_map();

    private:
        Submap create_submap_from_keyframe(const KeyFrame& keyframe);
        int idx(int x, int y, int width);
        double probability_to_log_odds(int8_t prob);
        int8_t log_odds_to_probability(double log_odds);

        void SubmapManager::bresenham_raytrace_submap(
            Submap& submap,
            int x0, int y0, int x1, int y1,
            bool hit_point,
            float free_logodds,
            float occ_logodds,
            float distance_factor,
            float noise_radius,
            float noise_std_dev);

        SubmapManagerParams params_;

        std::vector<KeyFrame> keyframes_;
        std::mutex keyframes_mutex_;

        std::vector<PosedScan> posed_scan_buffer_;
        std::mutex buffer_mutex_;

        std::vector<Submap> submaps_;
    };
}

#endif // SUBMAP_MANAGER_H
