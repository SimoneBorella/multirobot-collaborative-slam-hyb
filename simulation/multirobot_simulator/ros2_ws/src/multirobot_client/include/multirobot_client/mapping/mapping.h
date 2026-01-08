#ifndef MAPPING_H
#define MAPPING_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <thread>
#include <atomic>
#include <deque>
#include <mutex>

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

        double occupied_threshold;
        double free_threshold;
        double noise_model_radius;
        double noise_model_std_dev;

        MappingParams(
            double mapping_rate = 2.0,
            double map_resolution = 0.05,
            double map_width = 40.0,
            double map_height = 40.0,
            double free_belief = 0.38,
            double occupied_belief = 0.80,
            double distance_belief_factor = 0.03,
            double occupied_threshold = 0.70,
            double free_threshold = 0.30,
            double noise_model_radius = 0.07,
            double noise_model_std_dev = 0.02)
            : mapping_rate(mapping_rate),
              map_resolution(map_resolution),
              map_width(map_width),
              map_height(map_height),
              free_belief(free_belief),
              occupied_belief(occupied_belief),
              distance_belief_factor(distance_belief_factor),
              occupied_threshold(occupied_threshold),
              free_threshold(free_threshold),
              noise_model_radius(noise_model_radius),
              noise_model_std_dev(noise_model_std_dev) {}
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
        Map get_map();

    private:
        float probability_to_log_odds(int8_t prob);
        int8_t log_odds_to_probability(float log_odds);

        void bresenham_raytrace(int x0, int y0, int x1, int y1, bool hit_point);
        void mapping();

        MappingParams params_;

        std::deque<PosedScan> posed_scan_buffer_;
        std::mutex buffer_mutex_;

        Map map_;
        std::vector<int8_t> map_log_odds_data_;

        float free_belief_log_odds;
        float occupied_belief_log_odds;

        std::atomic<bool> mapping_thread_running;
        std::thread mapping_thread;
    };


}


#endif // MAPPING_H
