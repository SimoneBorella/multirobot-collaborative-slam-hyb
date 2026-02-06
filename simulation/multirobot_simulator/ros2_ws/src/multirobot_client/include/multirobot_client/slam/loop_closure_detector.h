#ifndef LOOP_CLOSURE_DETECTOR_H
#define LOOP_CLOSURE_DETECTOR_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <Eigen/Dense>
#include <mutex>
#include <unordered_set>
#include <cmath>

#include <DBoW3/DBoW3.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include "data_types.h"

using namespace DBoW3;

namespace multirobot_slam
{
    struct LoopClosureDetectorParams
    {
        bool run_loop_closure_detection;
        double loop_closure_detection_rate;
        std::string vocabulary_path;
        double min_bow_score;
        double min_time_separation;
        double max_spatial_distance;
        int covisibility_consistency_threshold;
        size_t min_matches;
        size_t ransac_iters;
        size_t ransac_set_size;
        double inlier_threshold;
        int min_inliers;
        double min_inliers_ratio;
        double min_geometric_score;

        LoopClosureDetectorParams(
            bool run_loop_closure_detection = true,
            double loop_closure_detection_rate = 1.0,
            std::string vocabulary_path = "./vocabularies/ORBvoc.yml",
            double min_bow_score = 0.3,
            double min_time_separation = 10.0,
            double max_spatial_distance = 5.0,
            int covisibility_consistency_threshold = 3,
            size_t min_matches = 25,
            size_t ransac_iters = 100,
            size_t ransac_set_size = 8,
            double inlier_threshold = 0.2,
            int min_inliers = 15,
            double min_inliers_ratio = 0.25,
            double min_geometric_score = 0.8)
            : run_loop_closure_detection(run_loop_closure_detection),
              loop_closure_detection_rate(loop_closure_detection_rate),
              vocabulary_path(vocabulary_path),
              min_bow_score(min_bow_score),
              min_time_separation(min_time_separation),
              max_spatial_distance(max_spatial_distance),
              covisibility_consistency_threshold(covisibility_consistency_threshold),
              min_matches(min_matches),
              ransac_iters(ransac_iters),
              ransac_set_size(ransac_set_size),
              inlier_threshold(inlier_threshold),
              min_inliers(min_inliers),
              min_inliers_ratio(min_inliers_ratio),
              min_geometric_score(min_geometric_score) {}
    };

    class LoopClosureDetector
    {
    public:
        LoopClosureDetector();
        LoopClosureDetector(LoopClosureDetectorParams &params);

        static LoopClosureDetectorParams params_from_yaml(std::string &params_path);
        void init(LoopClosureDetectorParams &params);

        void add_keyframes_to_db(const std::map<int, KeyFrame>& keyframes);

        void notify_loop_closure_updated();
        std::optional<LoopClosureConstraint> detect();
        
    private:
        bool is_candidate(const KeyFrame& a, const KeyFrame& b);
        double compute_bow_score(const KeyFrame& a, const KeyFrame& b);
        int hamming_distance(const std::array<uint8_t, 32>& a, const std::array<uint8_t, 32>& b);
        bool estimate_relative_pose(const KeyFrame& a, const KeyFrame& b, Pose& T_ab, double& score);
        
        LoopClosureDetectorParams params_;

        Database orb_db_;
        
        std::map<int, KeyFrame> keyframes_;
        std::mutex keyframes_mutex_;
        
        std::unordered_map<int, int> database_to_keyframe_id_;

        int last_added_keyframe_id_;
        int last_processed_keyframe_id_;


        // Loop detector variables

        int loop_num_coincidences_;
        int loop_num_not_found_;
        bool loop_detected_;

        int current_keyframe_id_;
        int last_current_keyframe_id_;
        int matched_keyframe_id_;
    };
}

#endif // LOOP_CLOSURE_DETECTOR_H
