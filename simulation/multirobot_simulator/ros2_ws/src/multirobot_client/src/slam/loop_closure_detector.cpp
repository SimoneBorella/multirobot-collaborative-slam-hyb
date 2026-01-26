#include "loop_closure_detector.h"

namespace multirobot_slam
{
    LoopClosureDetector::LoopClosureDetector()
    {
    }

    LoopClosureDetector::LoopClosureDetector(LoopClosureDetectorParams &params)
        : params_(params)
    {
    }

    LoopClosureDetectorParams LoopClosureDetector::params_from_yaml(std::string &params_path)
    {
        LoopClosureDetectorParams p;

        try
        {
            YAML::Node config = YAML::LoadFile(params_path);
            YAML::Node loop_closure_detector_config = config["loop_closure_detector"];

            if (loop_closure_detector_config["loop_closure_detection_rate"])
                p.loop_closure_detection_rate = loop_closure_detector_config["loop_closure_detection_rate"].as<double>();

            if (loop_closure_detector_config["min_time_separation"])
                p.min_time_separation = loop_closure_detector_config["min_time_separation"].as<double>();

            if (loop_closure_detector_config["max_spatial_distance"])
                p.max_spatial_distance = loop_closure_detector_config["max_spatial_distance"].as<double>();

            if (loop_closure_detector_config["vocabulary_path"])
                p.vocabulary_path = loop_closure_detector_config["vocabulary_path"].as<std::string>();

            if (loop_closure_detector_config["min_bow_score"])
                p.min_bow_score = loop_closure_detector_config["min_bow_score"].as<double>();

            if (loop_closure_detector_config["min_matches"])
                p.min_matches = loop_closure_detector_config["min_matches"].as<size_t>();

            if (loop_closure_detector_config["min_inliers"])
                p.min_inliers = loop_closure_detector_config["min_inliers"].as<size_t>();            
            
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
            std::cerr << "Using default parameters.\n";
        }

        return p;
    }

    void LoopClosureDetector::init(LoopClosureDetectorParams &params)
    {
        params_ = params;

        Vocabulary voc(params_.vocabulary_path);
        orb_db_ = Database(voc, false, 0);
    }

    void LoopClosureDetector::add_keyframes_to_db(const std::vector<KeyFrame>& keyframes)
    {
        for(size_t i = last_keyframe_id_; i < keyframes.size(); i++)
        {
            const KeyFrame& kf = keyframes[i];

            cv::Mat descriptors(kf.keypoints.size(), 32, CV_8U);
            for (size_t j = 0; j < kf.keypoints.size(); j++)
                memcpy(descriptors.ptr(j), kf.keypoints[j].descriptor.data(), 32);

            int db_id = orb_db_.add(descriptors);
            db_id_to_kf_id[db_id] = kf.keyframe_id;
        }

        last_keyframe_id_ = keyframes.size();
    }


    std::optional<LoopClosureConstraint> LoopClosureDetector::detect(const KeyFrame& active_keyframe, const std::vector<KeyFrame>& keyframes)
    {
        std::optional<LoopClosureConstraint> loop_closure_opt;
        double best_score = 0.0;

        // Query DB with active keyframe
        cv::Mat descriptors(active_keyframe.keypoints.size(), 32, CV_8U);
        for (size_t j = 0; j < active_keyframe.keypoints.size(); j++)
            memcpy(descriptors.ptr(j), active_keyframe.keypoints[j].descriptor.data(), 32);

        QueryResults ret;
        orb_db_.query(descriptors, ret, 10);

        // For each result, find the keyframe and verify candidates
        for (const auto& r : ret)
        {
            int kf_id = db_id_to_kf_id[r.Id];

            auto it = std::find_if(keyframes.begin(), keyframes.end(),
                [&](const KeyFrame& kf){ return kf.keyframe_id == kf_id; });

            if (it == keyframes.end())
                continue;

            const KeyFrame& kf = *it;

            if (!is_candidate(active_keyframe, kf))
                continue;

            double bow_score = r.Score;
            if (bow_score < params_.min_bow_score)
                continue;

            Pose T;
            double geom_score = 0.0;
            if (!estimate_relative_pose(active_keyframe, kf, T, geom_score))
                continue;

            double total_score = bow_score * geom_score;
            if (total_score > best_score)
            {
                best_score = total_score;
                loop_closure_opt = LoopClosureConstraint{
                    size_t(active_keyframe.keyframe_id),
                    size_t(kf.keyframe_id),
                    T,
                    total_score
                };
            }
        }

        return loop_closure_opt;
    }


    double LoopClosureDetector::compute_bow_score(const KeyFrame& a, const KeyFrame& b)
    {
        cv::Mat descriptors_a(a.keypoints.size(), 32, CV_8U);
        for (size_t i = 0; i < a.keypoints.size(); i++)
            memcpy(descriptors_a.ptr(i), a.keypoints[i].descriptor.data(), 32);

        QueryResults ret;
        orb_db_.query(descriptors_a, ret, 10);

        for (auto &r : ret)
        {
            int matched_kf_id = db_id_to_kf_id[r.Id];
            if (matched_kf_id == b.keyframe_id)
                return r.Score;
        }

        return 0.0;
    }


    bool LoopClosureDetector::is_candidate(const KeyFrame& a, const KeyFrame& b)
    {
        // Stesso keyframe
        if (a.keyframe_id == b.keyframe_id)
            return false;

        // Separazione temporale minima
        if (std::abs(a.timestamp - b.timestamp) < params_.min_time_separation)
            return false;

        // Distanza grossolana
        double dist = (a.pose.position - b.pose.position).norm();
        if (dist > params_.max_spatial_distance)
            return false;

        // Non loop con keyframe attivo
        if (b.is_active)
            return false;

        return true;
    }

    int LoopClosureDetector::hamming_distance(const std::array<uint8_t, 32>& a, const std::array<uint8_t, 32>& b)
    {
        int dist = 0;
        for (size_t i = 0; i < 32; ++i)
            dist += __builtin_popcount(a[i] ^ b[i]);
        return dist;
    }


    bool LoopClosureDetector::estimate_relative_pose(const KeyFrame& a, const KeyFrame& b, Pose& T_ab, double& score)
    {
        // std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matches;

        // for (const auto& ka : a.keypoints)
        // {
        //     int best_dist = 256;
        //     const Keypoint* best_kp = nullptr;

        //     for (const auto& kb : b.keypoints)
        //     {
        //         int d = hamming_distance(ka.descriptor, kb.descriptor);
        //         if (d < best_dist)
        //         {
        //             best_dist = d;
        //             best_kp = &kb;
        //         }
        //     }

        //     if (best_dist < params_.max_descriptor_distance && best_kp)
        //     {
        //         matches.push_back({ka.point, best_kp->point});
        //     }
        // }

        // if (matches.size() < params_.min_matches)
        //     return false;

        // // RANSAC SE(3)
        // std::vector<int> inliers;
        // Eigen::Matrix4d T;

        // bool ok = ransac_se3(matches, T, inliers,
        //                     params_.ransac_threshold,
        //                     params_.ransac_iterations);

        // if (!ok || inliers.size() < params_.min_inliers)
        //     return false;

        // // score
        // score = double(inliers.size()) / double(matches.size());

        // // output
        // T_ab = Pose::fromMatrix(T);

        return true;
    }

}