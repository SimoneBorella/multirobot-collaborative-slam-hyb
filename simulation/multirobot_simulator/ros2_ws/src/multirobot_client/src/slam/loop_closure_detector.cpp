#include "loop_closure_detector.h"

namespace multirobot_slam
{
    LoopClosureDetector::LoopClosureDetector()
    : last_added_keyframe_id_(0), last_processed_keyframe_id_(0), loop_num_coincidences_(0), loop_num_not_found_(0), loop_detected_(false), current_keyframe_id_(-1), last_current_keyframe_id_(-1), matched_keyframe_id_(-1)
    {
    }

    LoopClosureDetector::LoopClosureDetector(LoopClosureDetectorParams &params)
        : params_(params), last_added_keyframe_id_(0), last_processed_keyframe_id_(0), loop_num_coincidences_(0), loop_num_not_found_(0), loop_detected_(false), current_keyframe_id_(-1), last_current_keyframe_id_(-1), matched_keyframe_id_(-1)
    {
    }

    LoopClosureDetectorParams LoopClosureDetector::params_from_yaml(std::string &params_path)
    {
        LoopClosureDetectorParams p;

        try
        {
            YAML::Node config = YAML::LoadFile(params_path);
            YAML::Node loop_closure_detector_config = config["loop_closure_detector"];

            if (loop_closure_detector_config["run_loop_closure_detection"])
                p.run_loop_closure_detection = loop_closure_detector_config["run_loop_closure_detection"].as<bool>();
                
            if (loop_closure_detector_config["loop_closure_detection_rate"])
                p.loop_closure_detection_rate = loop_closure_detector_config["loop_closure_detection_rate"].as<double>();

            if (loop_closure_detector_config["vocabulary_path"])
                p.vocabulary_path = loop_closure_detector_config["vocabulary_path"].as<std::string>();

            if (loop_closure_detector_config["min_bow_score"])
                p.min_bow_score = loop_closure_detector_config["min_bow_score"].as<double>();

            if (loop_closure_detector_config["min_time_separation"])
                p.min_time_separation = loop_closure_detector_config["min_time_separation"].as<double>();

            if (loop_closure_detector_config["max_spatial_distance"])
                p.max_spatial_distance = loop_closure_detector_config["max_spatial_distance"].as<double>();

            if (loop_closure_detector_config["covisibility_consistency_threshold"])
                p.covisibility_consistency_threshold = loop_closure_detector_config["covisibility_consistency_threshold"].as<int>();
            
            if (loop_closure_detector_config["min_matches"])
                p.min_matches = loop_closure_detector_config["min_matches"].as<size_t>();

            if (loop_closure_detector_config["ransac_iters"])
                p.ransac_iters = loop_closure_detector_config["ransac_iters"].as<size_t>();

            if (loop_closure_detector_config["ransac_set_size"])
                p.ransac_set_size = loop_closure_detector_config["ransac_set_size"].as<size_t>();

            if (loop_closure_detector_config["inlier_threshold"])
                p.inlier_threshold = loop_closure_detector_config["inlier_threshold"].as<double>();

            if (loop_closure_detector_config["min_inliers"])
                p.min_inliers = loop_closure_detector_config["min_inliers"].as<int>();   
                
            if (loop_closure_detector_config["min_inliers_ratio"])
                p.min_inliers_ratio = loop_closure_detector_config["min_inliers_ratio"].as<double>();    
            
            if (loop_closure_detector_config["min_geometric_score"])
                p.min_geometric_score = loop_closure_detector_config["min_geometric_score"].as<double>();            
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


    void LoopClosureDetector::add_keyframes_to_db(const std::map<int, KeyFrame>& keyframes)
    {
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            keyframes_ = keyframes;
        }

        while (true)
        {
            auto it = keyframes.find(last_added_keyframe_id_+1);
            if (it == keyframes.end())
                break;

            const KeyFrame& candidate_keyframe = it->second;

            cv::Mat descriptors(candidate_keyframe.keypoints.size(), 32, CV_8U);
            for (size_t j = 0; j < candidate_keyframe.keypoints.size(); j++)
                memcpy(descriptors.ptr(j), candidate_keyframe.keypoints[j].descriptor.data(), 32);

            int db_id = orb_db_.add(descriptors);
            database_to_keyframe_id_[db_id] = candidate_keyframe.keyframe_id;

            last_added_keyframe_id_++;
        }
    }




    void LoopClosureDetector::notify_loop_closure_updated()
    {
        loop_num_coincidences_ = 0;
        loop_detected_ = false;
        matched_keyframe_id_ = -1;
        last_current_keyframe_id_ = -1;
    }



    std::optional<LoopClosureConstraint> LoopClosureDetector::detect()
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);

        if(loop_detected_)
            return std::nullopt;

        std::optional<LoopClosureConstraint> loop_closure_opt;

        int next_id = last_processed_keyframe_id_ + 1;

        while (true)
        {
            auto it = keyframes_.find(next_id);
            if (it == keyframes_.end())
                break;
            
            current_keyframe_id_ = it->first;
            KeyFrame& current_keyframe = it->second;

            // BoW candidate search
            cv::Mat descriptors(current_keyframe.keypoints.size(), 32, CV_8U);
            for (size_t i = 0; i < current_keyframe.keypoints.size(); i++)
                memcpy(descriptors.ptr(i), current_keyframe.keypoints[i].descriptor.data(), 32);

            QueryResults ret;
            orb_db_.query(descriptors, ret, 5);

            double best_geom_score = 0.0;
            int best_candidate_id = -1;
            Pose best_T;

            for (auto &r : ret)
            {
                if (r.Score < params_.min_bow_score)
                    continue;

                int candidate_keyframe_id = database_to_keyframe_id_[r.Id];

                // Check if candidate exists in map
                auto cand_it = keyframes_.find(candidate_keyframe_id);
                if (cand_it == keyframes_.end())
                    continue;

                const KeyFrame& candidate_keyframe = cand_it->second;

                // Temporal and spatial consistency
                if (!is_candidate(current_keyframe, candidate_keyframe))
                    continue;

                // Geometric verification
                Pose T;
                double geometric_score = 0.0;
                bool success = estimate_relative_pose(current_keyframe, candidate_keyframe, T, geometric_score);

                if (!success)
                    continue;

                if (geometric_score > params_.min_geometric_score && geometric_score > best_geom_score)
                {
                    best_geom_score = geometric_score;
                    best_candidate_id = candidate_keyframe_id;
                    best_T = T;
                }
            }

            last_processed_keyframe_id_ = current_keyframe_id_;

            if (best_candidate_id != -1)
            {
                loop_detected_ = true;

                loop_closure_opt = LoopClosureConstraint{
                    current_keyframe_id_,
                    best_candidate_id,
                    best_T,
                    best_geom_score
                };

                std::cout << "Loop closure detected between keyframes "
                        << current_keyframe_id_ << " and "
                        << best_candidate_id
                        << " with score " << best_geom_score << std::endl;

                return loop_closure_opt;
            }

            next_id++;
        }

        return std::nullopt;
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
            int matched_kf_id = database_to_keyframe_id_[r.Id];
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

        // ORB matching
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
        int best_inliers = 0;
        double best_inlier_score = 0;
        Eigen::Matrix4d best_T = Eigen::Matrix4d::Identity();

        std::mt19937 rng(0);
        std::uniform_int_distribution<size_t> uni(0, pts_a.size() - 1);

        for (size_t iter = 0; iter < params_.ransac_iters; iter++)
        {
            // Sample points
            std::vector<Eigen::Vector3d> sa, sb;
            std::unordered_set<size_t> used;
            while (sa.size() < params_.ransac_set_size)
            {
                size_t idx = uni(rng);
                if (used.insert(idx).second)
                {
                    sa.push_back(pts_a[idx]);
                    sb.push_back(pts_b[idx]);
                }
            }


            // Estimate rigid transform (Umeyama)
            Eigen::Matrix3Xd A(3, params_.ransac_set_size), B(3, params_.ransac_set_size);
            for (size_t i = 0; i < params_.ransac_set_size; ++i)
            {
                A.col(i) = sa[i];
                B.col(i) = sb[i];
            }

            Eigen::Matrix4d T = Eigen::umeyama(A, B, false);

            Eigen::Matrix3d R = T.block<3,3>(0,0);
            Eigen::Vector3d t = T.block<3,1>(0,3);

            if (!R.allFinite())
                continue;

            // Force proper rotation
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
            R = svd.matrixU() * svd.matrixV().transpose();

            // Correct reflection
            if (R.determinant() < 0)
                R = -R;
            
            // Rebuild T with corrected R
            T.block<3,3>(0,0) = R;
            T.block<3,1>(0,3) = t;

            // Count inliers
            // double mean_dist = 0;
            // for(auto &p : pts_a)
            //     mean_dist += p.norm();
            // mean_dist /= pts_a.size();
            // double inlier_threshold = std::max(params_.inlier_threshold, 0.05 * mean_dist);

            double inlier_threshold = params_.inlier_threshold;

            int inliers = 0;
            double inlier_score = 0;
            for (size_t i = 0; i < pts_a.size(); ++i)
            {
                Eigen::Vector3d pb_est = R * pts_a[i] + t;

                double dist2 = (pb_est - pts_b[i]).squaredNorm();

                if (dist2 < inlier_threshold * inlier_threshold)
                {
                    inliers++;
                    inlier_score += std::exp(-0.5 * dist2 / (inlier_threshold*inlier_threshold));
                }
            }

            if (inliers > best_inliers)
            {
                best_inliers = inliers;
                best_inlier_score = inlier_score;
                best_T = T;
            }
        }

        double inlier_ratio = double(best_inliers) / double(pts_a.size());

        if (best_inliers < params_.min_inliers || inlier_ratio < params_.min_inliers_ratio)
            return false;


        // Output
        score = best_inlier_score / double(pts_a.size());

        Eigen::Vector3d t = best_T.block<3,1>(0,3);
        Eigen::Matrix3d R = best_T.block<3,3>(0,0);

        Eigen::Quaterniond q(R);
        q.normalize();

        T_ab = Pose(t, q);

        return true;
    }
}