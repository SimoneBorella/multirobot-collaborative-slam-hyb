#include "loop_closure_detector.h"

namespace multirobot_slam
{
    LoopClosureDetector::LoopClosureDetector()
    : curr_keyframe_id_(0)
    {
    }

    LoopClosureDetector::LoopClosureDetector(LoopClosureDetectorParams &params)
        : params_(params), curr_keyframe_id_(0)
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

            if (loop_closure_detector_config["ransac_iters"])
                p.ransac_iters = loop_closure_detector_config["ransac_iters"].as<size_t>();

            if (loop_closure_detector_config["inlier_threshold"])
                p.inlier_threshold = loop_closure_detector_config["inlier_threshold"].as<double>();

            if (loop_closure_detector_config["min_inliers"])
                p.min_inliers = loop_closure_detector_config["min_inliers"].as<size_t>();    
            
            if (loop_closure_detector_config["min_total_score"])
                p.min_total_score = loop_closure_detector_config["min_total_score"].as<double>();            
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
        if(curr_keyframe_id_ >= keyframes.size())
            return;

        // std::cout << "Adding keyframes from " << curr_keyframe_id_ << " to " << keyframes.size() - 1 << std::endl;

        for(size_t i = curr_keyframe_id_; i < keyframes.size(); i++)
        {
            const KeyFrame& kf = keyframes[i];

            // std::cout << "Adding keyframe " << kf.keyframe_id << std::endl;

            cv::Mat descriptors(kf.keypoints.size(), 32, CV_8U);
            for (size_t j = 0; j < kf.keypoints.size(); j++)
                memcpy(descriptors.ptr(j), kf.keypoints[j].descriptor.data(), 32);

            
            int db_id = orb_db_.add(descriptors);
            db_id_to_kf_id[db_id] = kf.keyframe_id;
        }

        curr_keyframe_id_ = keyframes.size();
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
        // std::cout << "Loop closure candidates for keyframe " << active_keyframe.keyframe_id << ":" << std::endl;
        for (const auto& r : ret)
        {
            // std::cout << "  Candidate keyframe " << db_id_to_kf_id[r.Id] << " with BoW score " << r.Score << std::endl;

            int kf_id = db_id_to_kf_id[r.Id];
            auto it = std::find_if(keyframes.begin(), keyframes.end(),
                [&](const KeyFrame& kf){ return kf.keyframe_id == kf_id; });

            if (it == keyframes.end())
                continue;

            const KeyFrame& kf = *it;
            
            double bow_score = r.Score;
            if (bow_score < params_.min_bow_score)
                continue;

            if (!is_candidate(active_keyframe, kf))
                continue;

            Pose T;
            double geom_score = 0.0;
            if (!estimate_relative_pose(active_keyframe, kf, T, geom_score))
                continue;

            double total_score = bow_score * geom_score;

            if (total_score < params_.min_total_score)
            {
                std::cout << "Rejected: total score " << total_score << " below threshold." << std::endl;
                continue;
            }

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
        // Build descriptor matrices
        cv::Mat desc_a(a.keypoints.size(), 32, CV_8U);
        cv::Mat desc_b(b.keypoints.size(), 32, CV_8U);

        for (size_t i = 0; i < a.keypoints.size(); ++i)
            memcpy(desc_a.ptr(i), a.keypoints[i].descriptor.data(), 32);

        for (size_t i = 0; i < b.keypoints.size(); ++i)
            memcpy(desc_b.ptr(i), b.keypoints[i].descriptor.data(), 32);

        // ORB matching (KNN + ratio test)
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(desc_a, desc_b, knn_matches, 2);

        std::vector<cv::DMatch> good_matches;
        for (auto& m : knn_matches)
        {
            if (m.size() == 2 && m[0].distance < 0.75f * m[1].distance) // David Lowe (SIFT, 2004)
                good_matches.push_back(m[0]);
        }

        if (good_matches.size() < params_.min_matches)
            return false;

        // Build 3D-3D correspondences
        std::vector<Eigen::Vector3d> pts_a, pts_b;
        for (auto& m : good_matches)
        {
            pts_a.push_back(a.keypoints[m.queryIdx].point);
            pts_b.push_back(b.keypoints[m.trainIdx].point);
        }

        // RANSAC SE(3)
        size_t best_inliers = 0;
        Eigen::Matrix4d best_T = Eigen::Matrix4d::Identity();

        std::mt19937 rng(0);
        std::uniform_int_distribution<size_t> uni(0, pts_a.size() - 1);

        for (int iter = 0; iter < params_.ransac_iters; iter++)
        {
            // Sample 3 points
            std::vector<Eigen::Vector3d> sa, sb;
            for (int k = 0; k < 3; k++)
            {
                size_t idx = uni(rng);
                sa.push_back(pts_a[idx]);
                sb.push_back(pts_b[idx]);
            }

            // Estimate rigid transform (Umeyama)
            Eigen::Matrix3Xd A(3, 3), B(3, 3);
            for (int i = 0; i < 3; ++i)
            {
                A.col(i) = sa[i];
                B.col(i) = sb[i];
            }

            Eigen::Matrix4d T = Eigen::umeyama(A, B, false);
            if (!T.allFinite())
                continue;

            // Count inliers
            size_t inliers = 0;
            for (size_t i = 0; i < pts_a.size(); ++i)
            {
                Eigen::Vector4d pa;
                pa << pts_a[i], 1.0;
                Eigen::Vector3d pb_est = (T * pa).head<3>();

                if ((pb_est - pts_b[i]).norm() < params_.inlier_threshold)
                    inliers++;
            }

            if (inliers > best_inliers)
            {
                best_inliers = inliers;
                best_T = T;
            }
        }

        if (best_inliers < params_.min_inliers)
            return false;

        // Output
        score = static_cast<double>(best_inliers)/static_cast<double>(pts_a.size());

        Eigen::Vector3d t = best_T.block<3,1>(0,3);
        Eigen::Matrix3d R = best_T.block<3,3>(0,0);

        Eigen::Quaterniond q(R);
        q.normalize();

        T_ab = Pose(t, q);

        return true;
    }


}