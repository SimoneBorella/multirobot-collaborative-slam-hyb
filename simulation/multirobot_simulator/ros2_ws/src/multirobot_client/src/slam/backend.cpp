#include "backend.h"

namespace multirobot_slam
{
    Backend::Backend()
        : t_(0), odom_first_(true), imu_timestamp_prev_(0.0), max_timestamped_pose_queue_duration_(5.0), landmark_id_(0), state_updated_(false), last_updated_keyframe_id_(-1)
    {
    }

    Backend::Backend(BackendParams &params)
        : params_(params), t_(0), odom_first_(true), imu_timestamp_prev_(0.0), max_timestamped_pose_queue_duration_(5.0), landmark_id_(0), state_updated_(false), last_updated_keyframe_id_(-1)
    {
    }

    BackendParams Backend::params_from_yaml(std::string &params_path)
    {
        BackendParams p;

        try
        {
            YAML::Node config = YAML::LoadFile(params_path);
            YAML::Node backend_config = config["backend"];

            if (backend_config["backend_rate"])
                p.backend_rate = backend_config["backend_rate"].as<double>();

            if (backend_config["backend_keyframe_update_rate"])
                p.backend_keyframe_update_rate = backend_config["backend_keyframe_update_rate"].as<double>();
                
            if (backend_config["init_position"])
            {
                std::vector<double> vec = backend_config["init_position"].as<std::vector<double>>();
                if (vec.size() == 6)
                    p.init_position = Eigen::Map<Eigen::Matrix<double, 6, 1>>(vec.data());
                else
                    std::cerr << "Param init_position must have 6 elements, ignoring.\n";
            }

            if (backend_config["init_velocity"])
            {
                std::vector<double> vec = backend_config["init_velocity"].as<std::vector<double>>();
                if (vec.size() == 3)
                    p.init_velocity = Eigen::Map<Eigen::Vector3d>(vec.data());
                else
                    std::cerr << "Param init_velocity must have 3 elements, ignoring.\n";
            }

            if (backend_config["init_accelerometer_bias"])
            {
                std::vector<double> vec = backend_config["init_accelerometer_bias"].as<std::vector<double>>();
                if (vec.size() == 3)
                    p.init_accelerometer_bias = Eigen::Map<Eigen::Vector3d>(vec.data());
                else
                    std::cerr << "Param init_accelerometer_bias must have 3 elements, ignoring.\n";
            }

            if (backend_config["init_gyroscope_bias"])
            {
                std::vector<double> vec = backend_config["init_gyroscope_bias"].as<std::vector<double>>();
                if (vec.size() == 3)
                    p.init_gyroscope_bias = Eigen::Map<Eigen::Vector3d>(vec.data());
                else
                    std::cerr << "Param init_gyroscope_bias must have 3 elements, ignoring.\n";
            }

            if (backend_config["sigma_odom_position_noise"])
            {
                double val = backend_config["sigma_odom_position_noise"].as<double>();
                p.sigma_odom_position_noise = val;
            }

            if (backend_config["sigma_odom_orientation_noise"])
            {
                double val = backend_config["sigma_odom_orientation_noise"].as<double>();
                p.sigma_odom_orientation_noise = val;
            }

            if (backend_config["sigma_loop_closure_position_noise"])
            {
                double val = backend_config["sigma_loop_closure_position_noise"].as<double>();
                p.sigma_loop_closure_position_noise = val;
            }

            if (backend_config["sigma_loop_closure_orientation_noise"])
            {
                double val = backend_config["sigma_loop_closure_orientation_noise"].as<double>();
                p.sigma_loop_closure_orientation_noise = val;
            }

            if (backend_config["sigma_accelerometer_noise_density"])
            {
                double val = backend_config["sigma_accelerometer_noise_density"].as<double>();
                p.sigma_accelerometer_noise_density = val;
            }

            if (backend_config["sigma_gyroscope_noise_density"])
            {
                double val = backend_config["sigma_gyroscope_noise_density"].as<double>();
                p.sigma_gyroscope_noise_density = val;
            }

            if (backend_config["sigma_keypoint_noise"])
            {
                double val = backend_config["sigma_keypoint_noise"].as<double>();
                p.sigma_keypoint_noise = val;
            }

            if (backend_config["data_association_distance"])
            {
                double val = backend_config["data_association_distance"].as<double>();
                p.data_association_distance = val;
            }

            if (backend_config["keyframe_distance"])
                p.keyframe_distance = backend_config["keyframe_distance"].as<double>();

            if (backend_config["keyframe_angular_distance"])
                p.keyframe_angular_distance = backend_config["keyframe_angular_distance"].as<double>();


            if (backend_config["keyframe_update_position_threshold"])
                p.keyframe_update_position_threshold = backend_config["keyframe_update_position_threshold"].as<double>();

            if (backend_config["keyframe_update_orientation_threshold"])
                p.keyframe_update_orientation_threshold = backend_config["keyframe_update_orientation_threshold"].as<double>();

            if (backend_config["keyframe_update_cov_trace_threshold"])
                p.keyframe_update_cov_trace_threshold = backend_config["keyframe_update_cov_trace_threshold"].as<double>();

        
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
            std::cerr << "Using default parameters.\n";
        }

        return p;
    }

    void Backend::init(BackendParams &params)
    {
        params_ = params;

        // Initialize ISAM2
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        parameters.cacheLinearizedFactors = false;
        parameters.enablePartialRelinearizationCheck = true;

        isam_ = ISAM2(parameters);

        NonlinearFactorGraph new_factors;
        Values new_init_estimates;

        // Noises
        odom_noise_ = noiseModel::Diagonal::Sigmas(
            (Vector6() << params_.sigma_odom_position_noise,
                params_.sigma_odom_position_noise,
                params_.sigma_odom_position_noise,
                params_.sigma_odom_orientation_noise,
                params_.sigma_odom_orientation_noise,
                params_.sigma_odom_orientation_noise).finished());

        loop_closure_noise_ = noiseModel::Diagonal::Sigmas(
            (Vector6() << params_.sigma_loop_closure_position_noise,
                params_.sigma_loop_closure_position_noise,
                params_.sigma_loop_closure_position_noise,
                params_.sigma_loop_closure_orientation_noise,
                params_.sigma_loop_closure_orientation_noise,
                params_.sigma_loop_closure_orientation_noise).finished());

        // Pose prior
        Symbol x0('x', 0);
        Pose3 init_pose(
            Rot3::RzRyRx(params_.init_position(3), params_.init_position(4), params_.init_position(5)), // roll, pitch, yaw
            Point3(params_.init_position(0), params_.init_position(1), params_.init_position(2))        // x, y, z
        );
        noiseModel::Diagonal::shared_ptr init_pose_prior_noise = noiseModel::Diagonal::Sigmas(Vector6::Constant(1e-4));
        new_factors.add(PriorFactor<Pose3>(x0, init_pose, init_pose_prior_noise));
        new_init_estimates.insert(x0, init_pose);

        // Update new factors
        {
            std::lock_guard<std::mutex> lock(isam_mutex_);
            isam_.update(new_factors, new_init_estimates);
        }

        // Initial estimates
        pose_estimate_ = init_pose;

        t_ = 1;
    }

    void Backend::add_odom(OdomData &odom_data)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        odom_buffer_.push_back(odom_data);
    }

    void Backend::add_imu(ImuData &imu_data)
    {

    }

    void Backend::add_keypoints(KeypointsData &keypoints_data)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        if (keypoints_data.keypoints.size() == 0)
            return;

        keypoints_buffer_.push_back(keypoints_data);
    }

    State Backend::get_state()
    {
        return state_;
    }

    std::optional<State> Backend::get_state_if_updated()
    {
        if (state_updated_)
        {
            state_updated_ = false;
            return state_;
        }
        else
        {
            return std::nullopt;
        }
    }


    void Backend::update_keyframes(int update_window)
    {
        Values estimates;
        std::vector<std::pair<Symbol, int>> keys;

        // Get keyframes symbols
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);

            if (keyframes_.empty())
                return;

            int start_kf_id = 0;

            if (update_window > 0)
            {
                int newest_kf_id = keyframes_.rbegin()->first;
                int window_start_id = newest_kf_id - update_window + 1;
                start_kf_id = std::max(window_start_id, last_updated_keyframe_id_ + 1);
            }

            for (auto it = keyframes_.lower_bound(start_kf_id); it != keyframes_.end(); ++it)
            {
                const KeyFrame& kf = it->second;
                keys.emplace_back(
                    Symbol(kf.pose_symbol.first, kf.pose_symbol.second),
                    kf.keyframe_id
                );
            }
        }

        // Read isam estimates and marginals
        std::vector<Pose3> poses(keys.size());
        std::vector<Eigen::Matrix<double,6,6>> covariances(keys.size());

        {
            std::lock_guard<std::mutex> lock(isam_mutex_);
            estimates = isam_.calculateEstimate();

            for (size_t i = 0; i < keys.size(); ++i)
            {
                poses[i] = estimates.at<Pose3>(keys[i].first);
                try {
                    covariances[i] = isam_.marginalCovariance(keys[i].first);
                } catch (const std::exception& e) {
                    // Indeterminant linear system exception, high fallback covariance
                    covariances[i] = Eigen::Matrix<double, 6, 6>::Identity() * 1.0; 
                    std::cout << "Backend failed in retrieving marginal covariances." << std::endl;
                }
            }
        }

        int max_updated_kf_id = last_updated_keyframe_id_;

        // Update keyframes
        std::map<int, KeyFrame> keyframes_updates;
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);

            for (size_t i = 0; i < keys.size(); ++i)
            {
                // New estimates
                Eigen::Vector3d new_position = poses[i].translation();
                Eigen::Quaterniond new_orientation = Eigen::Quaterniond(poses[i].rotation().matrix());
                Eigen::Matrix<double, 6, 6> new_covariance = covariances[i];

                // Update keyframes
                KeyFrame& kf = keyframes_[keys[i].second];
                kf.pose.position = new_position;
                kf.pose.orientation = new_orientation;
                kf.covariance = new_covariance;
                                
                // If keyframe added for the first time then append to keyframes updates
                if (kf.keyframe_id > last_updated_keyframe_id_)
                {   
                    keyframes_updates[kf.keyframe_id] = kf;
                    keyframes_threshold_reference_[kf.keyframe_id] = kf;

                    max_updated_kf_id = std::max(max_updated_kf_id, kf.keyframe_id);
                }
                else
                {
                    // Check if thresholds exceed with respect to the reference
                    KeyFrame& ref_kf = keyframes_threshold_reference_[kf.keyframe_id];
                    double delta_position = (ref_kf.pose.position - new_position).norm();
                    double delta_yaw = Eigen::AngleAxisd(ref_kf.pose.orientation.inverse() * new_orientation).angle();
                    double delta_cov_trace = std::abs(ref_kf.covariance.trace() - new_covariance.trace());
    
                    bool threshold_exceeded =
                        delta_position > params_.keyframe_update_position_threshold ||
                        delta_yaw > params_.keyframe_update_orientation_threshold ||
                        delta_cov_trace > params_.keyframe_update_cov_trace_threshold;

                    if (threshold_exceeded)
                    {
                        ref_kf = kf;
                        keyframes_updates[kf.keyframe_id] = kf;
                    }
                }
            }
            // Update last updated keyframe id
            last_updated_keyframe_id_ = max_updated_kf_id;
        }


        {
            std::lock_guard<std::mutex> lock(keyframes_updates_mutex_);

            for (auto& [new_id, new_kf] : keyframes_updates)
            {
               keyframes_updates_[new_id] = new_kf;
            }
        }
    }






    std::map<int, KeyFrame> Backend::get_keyframes()
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        return keyframes_;
    }


    std::map<int, KeyFrame> Backend::get_keyframes_updates()
    {
        std::lock_guard<std::mutex> lock(keyframes_updates_mutex_);
        std::map<int, KeyFrame> keyframes_updates = std::move(keyframes_updates_);
        keyframes_updates_.clear();
        return keyframes_updates;
    }


    std::vector<std::pair<Symbol, double>> Backend::nearest_neighbor_data_association(const Point3 &observed_point, const Values &estimates)
    {
        std::vector<std::pair<Symbol, double>> results;

        if (landmark_symbols_.empty())
            return results;

        Symbol associated_l;
        double min_dist = std::numeric_limits<double>::infinity();
        for (const Symbol &l : landmark_symbols_)
        {
            if (estimates.exists(l))
            {
                Point3 landmark_estimate = estimates.at<Point3>(l);
                double dist = (observed_point - landmark_estimate).norm();

                if (dist < params_.data_association_distance && dist < min_dist)
                {
                    min_dist = dist;
                    associated_l = l;
                }
            }
        }

        if (std::isfinite(min_dist))
            results.emplace_back(associated_l, 1.0);

        return results;
    }

    std::vector<std::pair<Symbol, double>> Backend::probabilistic_data_association(const Point3 &observed_point, const Values &estimates, const Marginals &marginals)
    {
        std::vector<std::pair<Symbol, double>> results;

        if (landmark_symbols_.empty())
            return results;

        double total_likelihood = 0.0;
        std::vector<std::tuple<Symbol, double, double>> temp_results;

        for (const Symbol &l : landmark_symbols_)
        {
            if (!estimates.exists(l))
                continue;

            Point3 landmark_estimate = estimates.at<Point3>(l);
            Vector3 dist = observed_point - landmark_estimate;

            if (dist.norm() > params_.data_association_distance)
                continue;

            // Could throw exception if marginal is singular
            Matrix3 cov;
            try
            {
                cov = marginals.marginalCovariance(l);
            }
            catch (...)
            {
                // Assumption
                cov = Matrix3::Identity() * 0.05 * 0.05;
            }

            double cov_determinant = cov.determinant();
            if (cov_determinant < 1e-6)
            {
                cov_determinant = 1e-6;
            }

            Matrix3 cov_inv = cov.inverse();

            // Compute the Mahalanobis distance squared:
            //   d^2 = (x - mu)^T sigma^(-1) (x - mu)
            // where x = observed_point, mu = landmark_estimate, sigma = covariance matrix
            double mahalanobis_dist = std::sqrt(dist.transpose() * cov_inv * dist);

            // double mahalanobis_dist_threshold = 5.0;
            // if (mahalanobis_dist > mahalanobis_dist_threshold)
            //     continue;

            // Compute the likelihood assuming a 3D Gaussian distribution:
            //   p(x) = (1 / ((2PI)^(n/2) * |sigma|^(1/2))) * exp(-0.5 * mahalanobis_dist^2)
            // where n = 3 for 3D points
            double likelihood = (1.0 / (std::pow(2.0 * M_PI, 1.5) * std::sqrt(cov_determinant))) * std::exp(-0.5 * mahalanobis_dist * mahalanobis_dist);

            // Apply threshold to likelihood to ignore weak associations
            temp_results.push_back({l, likelihood, mahalanobis_dist});
            total_likelihood += likelihood;
        }

        // Normalize likelihoods to get probabilities
        for (const auto &[l, likelihood, _] : temp_results)
        {
            if (total_likelihood > 0.0)
            {
                double probability = likelihood / total_likelihood;
                results.emplace_back(l, probability);
            }
        }

        return results;
    }

    bool Backend::find_bounding_poses(double landmark_ts, Symbol &prev_sym, Symbol &next_sym, double &prev_ts, double &next_ts)
    {
        if (timestamped_pose_queue_.size() < 2)
            return false;

        size_t idx_prev = 0;
        for (size_t i = 0; i < timestamped_pose_queue_.size(); i++)
        {
            if (timestamped_pose_queue_[i].first <= landmark_ts)
                idx_prev = i;
            else
                break;
        }

        if (idx_prev == timestamped_pose_queue_.size() - 1)
            return false;

        prev_ts = timestamped_pose_queue_[idx_prev].first;
        next_ts = timestamped_pose_queue_[idx_prev + 1].first;

        prev_sym = timestamped_pose_queue_[idx_prev].second;
        next_sym = timestamped_pose_queue_[idx_prev + 1].second;

        return true;
    }

    void Backend::optimize()
    {
        // auto start = std::chrono::high_resolution_clock::now();

        std::deque<OdomData> odom_buffer;
        std::deque<KeypointsData> keypoints_buffer;

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);

            if (odom_buffer_.empty())
                return;

            odom_buffer = odom_buffer_;
            keypoints_buffer = keypoints_buffer_;

            odom_buffer_.clear();
            keypoints_buffer_.clear();
        }

        NonlinearFactorGraph new_factors;
        Values new_estimates;

        Symbol x_prev('x', t_ - 1);
        Symbol x_curr('x', t_);

        // Odom factors
        const OdomData &odom = odom_buffer.back();

        Pose3 odom_pose(
            Rot3::Quaternion(odom.orientation.w(), odom.orientation.x(), odom.orientation.y(), odom.orientation.z()),
            Point3(odom.position.x(), odom.position.y(), odom.position.z()));

        if (odom_first_)
        {
            last_odom_pose_ = odom_pose;
            odom_first_ = false;
            return;
        }
        else
        {
            Pose3 delta_odom = last_odom_pose_.between(odom_pose);
            new_factors.add(BetweenFactor<Pose3>(x_prev, x_curr, delta_odom, odom_noise_));
            new_estimates.insert(x_curr, pose_estimate_ * delta_odom);
            last_odom_pose_ = odom_pose;
        }

        double ts = odom.timestamp;

        timestamped_pose_queue_.emplace_back(ts, x_curr);


        // Landmarks factors
        Values estimates;
        Marginals marginals;
        {
            std::lock_guard<std::mutex> lock(isam_mutex_);
            estimates = isam_.calculateEstimate();
            // try {
            //     marginals = Marginals(isam_.getFactorsUnsafe(), estimates);
            // } catch (...) {
                
            // }
        }

        
        auto keypoint_noise = noiseModel::Diagonal::Sigmas(
            Vector3(params_.sigma_keypoint_noise,
                    params_.sigma_keypoint_noise,
                    params_.sigma_keypoint_noise));

        if (!keypoints_buffer.empty())
        {
            KeypointsData keypoints_data = keypoints_buffer.back();

            double keypoints_ts = keypoints_data.timestamp;

            Symbol prev_sym, next_sym;
            double prev_ts, next_ts;
            if (!find_bounding_poses(keypoints_ts, prev_sym, next_sym, prev_ts, next_ts))
                return;

            Pose3 prev_pose = estimates.at<Pose3>(prev_sym);

            Pose3 keypoints_pose(
                Rot3::Quaternion(keypoints_data.pose.orientation.w(), keypoints_data.pose.orientation.x(), keypoints_data.pose.orientation.y(), keypoints_data.pose.orientation.z()),
                Point3(keypoints_data.pose.position.x(), keypoints_data.pose.position.y(), keypoints_data.pose.position.z()));

            Pose3 delta_pose = prev_pose.between(keypoints_pose);

            std::vector<Keypoint, Eigen::aligned_allocator<Keypoint>> transformed_keypoints;

            for (const Keypoint &keypoint : keypoints_data.keypoints)
            {
                Eigen::Vector3d point_map = keypoints_data.pose.orientation * keypoint.point + keypoints_data.pose.position;

                Point3 keypoint_measurement(keypoint.point.x(), keypoint.point.y(), keypoint.point.z());
                keypoint_measurement = delta_pose.transformFrom(keypoint_measurement);

                // Update transformed keypoints for keyframe storage
                Keypoint kp = keypoint;
                kp.point = keypoint_measurement;
                transformed_keypoints.push_back(kp);


                Unit3 bearing(keypoint_measurement);
                double range = keypoint_measurement.norm();

                Point3 keypoint_observation(point_map.x(), point_map.y(), point_map.z());

                // Data association
                auto associations = probabilistic_data_association(keypoint_observation, estimates, marginals);
                
                if (!associations.empty())
                {
                    for (const auto &[associated_l, probability] : associations)
                    {
                        // To scale information
                        // I * p  =>  S / p => sigmas / sqrt(p)
                        auto scaled_noise = noiseModel::Robust::Create(
                            noiseModel::mEstimator::Huber::Create(1.345),
                            noiseModel::Diagonal::Sigmas(
                                keypoint_noise->sigmas() / std::sqrt(probability)));

                        new_factors.add(BearingRangeFactor<Pose3, Point3>(prev_sym, associated_l, bearing, range, scaled_noise));
                    }
                }
                else
                {
                    Symbol l('l', landmark_id_++);
                    new_estimates.insert(l, keypoint_observation);
                    landmark_symbols_.push_back(l);

                    auto huber_noise = noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.345),
                        keypoint_noise);

                    new_factors.add(BearingRangeFactor<Pose3, Point3>(prev_sym, l, bearing, range, huber_noise));
                }
            }



            // Keyframes management
            Symbol pose_symbol = prev_sym;
            Eigen::Vector3d pose_position = Eigen::Vector3d(prev_pose.x(), prev_pose.y(), prev_pose.z());
            Eigen::Quaterniond pose_orientation = Eigen::Quaterniond(
                prev_pose.rotation().toQuaternion().w(),
                prev_pose.rotation().toQuaternion().x(),
                prev_pose.rotation().toQuaternion().y(),
                prev_pose.rotation().toQuaternion().z());

            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);

                if(keyframes_.empty())
                {
                    KeyFrame keyframe;
                    keyframe.timestamp = ts;
                    keyframe.keyframe_id = 0;
                    keyframe.pose_symbol = std::make_pair(pose_symbol.chr(), pose_symbol.index());
                    keyframe.pose = Pose(pose_position, pose_orientation);
                    keyframe.keypoints = transformed_keypoints;
                    keyframe.keypoints_number = transformed_keypoints.size();
                    keyframe.is_active = true;
                    keyframes_[keyframe.keyframe_id] = keyframe;
                }
                else
                {
                    KeyFrame& last_keyframe = keyframes_.rbegin()->second;
                    double delta_keyframe_distance = (pose_position - last_keyframe.pose.position).norm();
                    double delta_keyframe_angular_distance = pose_orientation.angularDistance(last_keyframe.pose.orientation);
    
                    if (delta_keyframe_distance > params_.keyframe_distance || delta_keyframe_angular_distance > params_.keyframe_angular_distance)
                    {
                        KeyFrame keyframe;
                        keyframe.timestamp = ts;
                        keyframe.keyframe_id = last_keyframe.keyframe_id + 1;
                        keyframe.pose_symbol = std::make_pair(pose_symbol.chr(), pose_symbol.index());
                        keyframe.pose = Pose(pose_position, pose_orientation);
                        keyframe.keypoints = transformed_keypoints;
                        keyframe.keypoints_number = transformed_keypoints.size();
                        keyframe.is_active = true;
                        keyframes_[keyframe.keyframe_id] = keyframe;
        
                        last_keyframe.is_active = false;
    
                        // Debug
                        // save_keyframes(keyframes_, "./output/localization/keyframes.csv");
                    }
                }
    
            }
        }

        // Queues pruning

        size_t landmarks_max_size = 200;

        if (landmark_symbols_.size() > landmarks_max_size)
        {
            landmark_symbols_.erase(landmark_symbols_.begin(), landmark_symbols_.begin() + (landmark_symbols_.size() - landmarks_max_size));
        }

        while (!timestamped_pose_queue_.empty() && ts - timestamped_pose_queue_.front().first > max_timestamped_pose_queue_duration_)
        {
            timestamped_pose_queue_.pop_front();
        }

        // Update ISAM2
        Eigen::Matrix<double,6,6> covariance;
        try {
            std::lock_guard<std::mutex> lock(isam_mutex_);
            isam_.update(new_factors, new_estimates);
            pose_estimate_ = isam_.calculateEstimate<Pose3>(x_curr);

            try {
                covariance = isam_.marginalCovariance(x_curr);
            } catch (...) {
                // Fallback covariance as odometry covariance
                covariance = odom_noise_->covariance(); 
                std::cout << "Backend failed in retrieving marginal covariances." << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "Backend ISAM2 update failed: " << e.what() << std::endl;
            
            // Fallback to avoid system to lock, same as previous values
            covariance = state_.covariance;
        }

        // Update current state estimate
        state_.timestamp = odom.timestamp;
        state_.position = pose_estimate_.translation();
        state_.orientation = Eigen::Quaterniond(pose_estimate_.rotation().matrix());
        state_.covariance = covariance;

        state_updated_ = true;

        
        // Update discrete time
        t_++;


        // if (t_%100 == 0)
        // {
        //     std::lock_guard<std::mutex> lock(isam_mutex_);
        //     Values isam_estimates = isam_.calculateEstimate();
        //     std::cout << "Saving ISAM2 graph " << t_ << std::endl;
        //     // Marginals isam_marginals(isam_.getFactorsUnsafe(), isam_estimates);
        //     // save_graph(isam_.getFactorsUnsafe(), isam_estimates, isam_marginals, "./output/localization/isam_graph_" + std::to_string(t_) + ".txt");
        //     save_graph(isam_.getFactorsUnsafe(), isam_estimates, std::nullopt, "./output/localization/isam_graph_" + std::to_string(t_) + ".txt");
        // }

        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> duration = end - start;

        // std::cout << "Localization time: " << (duration.count() * 1000) << " ms (" << 1/duration.count() << " Hz)" << std::endl;
    }



    void Backend::add_loop_closure(const LoopClosureConstraint& loop_closure)
    {
        // Debug
        // save_loop_closure(loop_closure, "./output/localization/loop_closures.csv");

        Symbol xi, xj;
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            xi = Symbol(keyframes_[loop_closure.keyframe_i].pose_symbol.first, keyframes_[loop_closure.keyframe_i].pose_symbol.second);
            xj = Symbol(keyframes_[loop_closure.keyframe_j].pose_symbol.first, keyframes_[loop_closure.keyframe_j].pose_symbol.second);
        }

        Pose3 loop_closure_transform(
            Rot3::Quaternion(loop_closure.transform_pose.orientation.w(),
                             loop_closure.transform_pose.orientation.x(),
                             loop_closure.transform_pose.orientation.y(),
                             loop_closure.transform_pose.orientation.z()),
            Point3(loop_closure.transform_pose.position.x(),
                   loop_closure.transform_pose.position.y(),
                   loop_closure.transform_pose.position.z()));
        
        NonlinearFactorGraph new_factors;

        new_factors.add(
            BetweenFactor<Pose3>(xj, xi, loop_closure_transform, loop_closure_noise_)
        );

        {
            std::lock_guard<std::mutex> lock(isam_mutex_);
            isam_.update(new_factors);
        }
    }

    void Backend::save_keyframes(std::map<int, KeyFrame> keyframes, const std::string &filename)
    {
        std::ofstream file(filename);

        file << "timestamp,keyframe_id,symbol,x,y,z,q_w,q_x,q_y,q_z" << "\n";

        file << std::fixed << std::setprecision(3);

        for (auto& [id, kf] : keyframes)
        {
            file << kf.timestamp << ","
                << kf.keyframe_id << ","
                << kf.pose_symbol.first << kf.pose_symbol.second << ","
                << kf.pose.position.x() << ","
                << kf.pose.position.y() << ","
                << kf.pose.position.z() << ","
                << kf.pose.orientation.w() << ","
                << kf.pose.orientation.x() << ","
                << kf.pose.orientation.y() << ","
                << kf.pose.orientation.z() << "\n";
        }
    }


    void Backend::save_loop_closure(const LoopClosureConstraint& loop_closure, const std::string &filename)
    {
        static bool first_call = true;

        std::ofstream file;

        if (first_call)
        {
            // truncate file
            file.open(filename, std::ios::out);
            file << "keyframe_i,keyframe_j,x,y,z,q_w,q_x,q_y,q_z,score\n";
            first_call = false;
        }
        else
        {
            file.open(filename, std::ios::app);
        }

        if (!file.is_open())
            return;

        file << std::fixed << std::setprecision(3);

        file << loop_closure.keyframe_i << ","
            << loop_closure.keyframe_j << ","
            << loop_closure.transform_pose.position.x() << ","
            << loop_closure.transform_pose.position.y() << ","
            << loop_closure.transform_pose.position.z() << ","
            << loop_closure.transform_pose.orientation.w() << ","
            << loop_closure.transform_pose.orientation.x() << ","
            << loop_closure.transform_pose.orientation.y() << ","
            << loop_closure.transform_pose.orientation.z() << ","
            << loop_closure.score
            << "\n";
    }

    void Backend::save_graph(NonlinearFactorGraph graph, Values estimates, std::optional<gtsam::Marginals> marginals, const std::string &filename)
    {
        std::ofstream file(filename);

        if (!file.is_open())
        {
            std::cout << "Could not open file: " << filename << std::endl;
            return;
        }

        // POSE3 xn x y z roll pitch yaw Covariance(6x6)
        for (int t = 0; t < t_; t++)
        {
            Symbol x('x', t);

            if (estimates.exists(x))
            {
                Pose3 pose_estimate = estimates.at<Pose3>(x);

                Vector3 rpy = pose_estimate.rotation().rpy();

                file << "POSE3 " << x << " "
                           << pose_estimate.x() << " "
                           << pose_estimate.y() << " "
                           << pose_estimate.z() << " "
                           << rpy(0) << " " << rpy(1) << " " << rpy(2);

                if (marginals.has_value())
                {
                    auto pose_covariance = marginals.value().marginalCovariance(x);

                    for (int i = 0; i < 6; i++)
                        for (int j = 0; j < 6; j++)
                            file << " " << pose_covariance(i, j);
                }
                file << "\n";
            }
        }

        // POINT3 ln x y z Covariance(3x3)
        for (Symbol l : landmark_symbols_)
        {
            if (estimates.exists(l))
            {
                Point3 landmark_estimate = estimates.at<Point3>(l);

                file << "POINT3 " << l << " "
                           << landmark_estimate.x() << " "
                           << landmark_estimate.y() << " "
                           << landmark_estimate.z();

                if (marginals.has_value())
                {
                    auto landmark_covariance = marginals.value().marginalCovariance(l);
                    for (int i = 0; i < 3; i++)
                        for (int j = 0; j < 3; j++)
                            file << " " << landmark_covariance(i, j);
                }
                file << "\n";
            }
        }

        for (const auto &factor : graph)
        {
            // PRIORPOSE3
            if (auto priorPose = boost::dynamic_pointer_cast<PriorFactor<Pose3>>(factor))
            {
                Symbol key = priorPose->keys().at(0);
                Pose3 prior = priorPose->prior();
                Matrix6 covariance = boost::dynamic_pointer_cast<noiseModel::Gaussian>(priorPose->noiseModel())->covariance();

                Vector3 rpy = prior.rotation().rpy();

                file << "PRIORPOSE3 " << key << " "
                           << prior.x() << " " << prior.y() << " " << prior.z() << " "
                           << rpy(0) << " " << rpy(1) << " " << rpy(2);

                for (int i = 0; i < 6; i++)
                    for (int j = 0; j < 6; j++)
                        file << " " << covariance(i, j);
                file << "\n";
            }

            // PRIORPOINT3
            if (auto priorPoint = boost::dynamic_pointer_cast<PriorFactor<Point3>>(factor))
            {
                Symbol key = priorPoint->keys().at(0);
                Point3 prior = priorPoint->prior();
                Matrix3 covariance = boost::dynamic_pointer_cast<noiseModel::Gaussian>(priorPoint->noiseModel())->covariance();

                file << "PRIORPOINT3 " << key << " "
                           << prior.x() << " " << prior.y() << " " << prior.z();

                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        file << " " << covariance(i, j);
                file << "\n";
            }

            // BETWEENFACTOR3
            if (auto betweenFactor = boost::dynamic_pointer_cast<BetweenFactor<Pose3>>(factor))
            {
                Symbol pose_x1 = betweenFactor->keys().at(0);
                Symbol pose_x2 = betweenFactor->keys().at(1);

                Pose3 measurement = betweenFactor->measured();

                noiseModel::Base::shared_ptr noise_model;

                if (auto robust_noise = boost::dynamic_pointer_cast<noiseModel::Robust>(betweenFactor->noiseModel()))
                {
                    noise_model = robust_noise->noise();
                }
                else
                {
                    noise_model = betweenFactor->noiseModel();
                }

                Matrix6 covariance = boost::dynamic_pointer_cast<noiseModel::Diagonal>(betweenFactor->noiseModel())->covariance();

                Vector3 rpy = measurement.rotation().rpy();

                file << "BETWEENFACTOR3 " << pose_x1 << " " << pose_x2 << " "
                           << measurement.x() << " " << measurement.y() << " " << measurement.z() << " "
                           << rpy(0) << " " << rpy(1) << " " << rpy(2);

                for (int i = 0; i < 6; i++)
                    for (int j = 0; j < 6; j++)
                        file << " " << covariance(i, j);
                file << "\n";
            }

            // BEARINGRANGEFACTOR3
            if (auto brFactor = boost::dynamic_pointer_cast<BearingRangeFactor<Pose3, Point3>>(factor))
            {
                Symbol pose_x = brFactor->keys().at(0);
                Symbol landmark_l = brFactor->keys().at(1);
                Unit3 bearing = brFactor->measured().bearing();
                double range = brFactor->measured().range();

                noiseModel::Base::shared_ptr noise_model;

                if (auto robust_noise = boost::dynamic_pointer_cast<noiseModel::Robust>(brFactor->noiseModel()))
                {
                    noise_model = robust_noise->noise();
                }
                else
                {
                    noise_model = brFactor->noiseModel();
                }

                auto covariance = boost::dynamic_pointer_cast<noiseModel::Diagonal>(noise_model)->covariance();

                Point3 b = bearing.unitVector();

                file << "BEARINGRANGEFACTOR3 " << pose_x << " " << landmark_l << " "
                           << b.x() << " " << b.y() << " " << b.z() << " "
                           << range;

                for (int i = 0; i < 2; i++)
                    for (int j = 0; j < 2; j++)
                        file << " " << covariance(i, j);

                file << "\n";
            }
        }
    }
}