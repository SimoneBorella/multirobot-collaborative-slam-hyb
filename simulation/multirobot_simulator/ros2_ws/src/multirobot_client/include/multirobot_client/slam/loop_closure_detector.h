#ifndef LOOP_CLOSURE_DETECTOR_H
#define LOOP_CLOSURE_DETECTOR_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <Eigen/Dense>
#include <mutex>

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
        double loop_closure_detection_rate;
        double min_time_separation;
        double max_spatial_distance;
        std::string vocabulary_path;
        double min_bow_score;
        size_t min_matches;
        size_t ransac_iters;
        double inlier_threshold;
        size_t min_inliers;
        double min_total_score;

        LoopClosureDetectorParams(
            double loop_closure_detection_rate = 1.0,
            double min_time_separation = 10.0,
            double max_spatial_distance = 5.0,
            std::string vocabulary_path = "./vocabularies/ORBvoc.yml",
            double min_bow_score = 0.3,
            size_t min_matches = 25,
            size_t ransac_iters = 100,
            double inlier_threshold = 0.2,
            size_t min_inliers = 15,
            double min_total_score = 0.8)
            : loop_closure_detection_rate(loop_closure_detection_rate),
              min_time_separation(min_time_separation),
              max_spatial_distance(max_spatial_distance),
              vocabulary_path(vocabulary_path),
              min_bow_score(min_bow_score),
              min_matches(min_matches),
              ransac_iters(ransac_iters),
              inlier_threshold(inlier_threshold),
              min_inliers(min_inliers),
              min_total_score(min_total_score) {}
    };

    class LoopClosureDetector
    {
    public:
        LoopClosureDetector();
        LoopClosureDetector(LoopClosureDetectorParams &params);

        static LoopClosureDetectorParams params_from_yaml(std::string &params_path);
        void init(LoopClosureDetectorParams &params);

        void add_keyframes_to_db(const std::vector<KeyFrame>& keyframes);
        std::optional<LoopClosureConstraint> detect(const KeyFrame& current_kf, const std::vector<KeyFrame>& keyframes);
        
    private:
        LoopClosureDetectorParams params_;

        Database orb_db_;
        size_t curr_keyframe_id_ = 0;
        std::unordered_map<int, int> db_id_to_kf_id;


        bool is_candidate(const KeyFrame& a, const KeyFrame& b);
        double compute_bow_score(const KeyFrame& a, const KeyFrame& b);
        int hamming_distance(const std::array<uint8_t, 32>& a, const std::array<uint8_t, 32>& b);
        bool estimate_relative_pose(const KeyFrame& a, const KeyFrame& b, Pose& T_ab, double& score);
    };
}

#endif // LOOP_CLOSURE_DETECTOR_H
