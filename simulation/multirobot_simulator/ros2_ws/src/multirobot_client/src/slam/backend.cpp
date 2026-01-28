#include "backend.h"

namespace multirobot_slam
{
    Backend::Backend()
        : t_(0), odom_first_(true), imu_timestamp_prev_(0.0), max_timestamped_pose_queue_duration_(5.0), landmark_id_(0), state_updated_(false)
    {
    }

    Backend::Backend(BackendParams &params)
        : params_(params), t_(0), odom_first_(true), imu_timestamp_prev_(0.0), max_timestamped_pose_queue_duration_(5.0), landmark_id_(0), state_updated_(false)
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

    void Backend::update_keyframe_poses()
    {
        Values estimates;
        {
            std::lock_guard<std::mutex> lock(isam_mutex_);
            estimates = isam_.calculateEstimate();
        }

        std::lock_guard<std::mutex> lock(keyframes_mutex_);

        for(KeyFrame& kf : keyframes_)
        {
            Pose3 kf_pose = isam_.calculateEstimate<Pose3>(Symbol(kf.pose_symbol.first, kf.pose_symbol.second));

            kf.pose.position = kf_pose.translation();
            kf.pose.orientation = Eigen::Quaterniond(kf_pose.rotation().matrix());
        }
    }

    std::vector<KeyFrame> Backend::get_keyframes()
    {
        return keyframes_;
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
            // marginals = Marginals(isam_.getFactorsUnsafe(), estimates);
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
                // auto associations = nearest_neighbor_data_association(keypoint_observation, estimates);
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
                    keyframe.is_active = true;
                    keyframes_.push_back(keyframe);
                }
    
                KeyFrame &last_keyframe = keyframes_.back();
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
                    keyframe.is_active = true;
                    keyframes_.push_back(keyframe);
    
                    last_keyframe.is_active = false;
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
        {
            std::lock_guard<std::mutex> lock(isam_mutex_);
            isam_.update(new_factors, new_estimates);
            pose_estimate_ = isam_.calculateEstimate<Pose3>(x_curr);
        }

        // Update current state estimate
        state_.timestamp = odom.timestamp;
        state_.position = pose_estimate_.translation();
        state_.attitude = Eigen::Quaterniond(pose_estimate_.rotation().matrix());

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
            BetweenFactor<Pose3>(xi, xj, loop_closure_transform, loop_closure_noise_)
        );

        {
            std::lock_guard<std::mutex> lock(isam_mutex_);
            isam_.update(new_factors);
        }
    }



    void Backend::save_graph(NonlinearFactorGraph graph, Values estimates, std::optional<gtsam::Marginals> marginals, const std::string &filename)
    {
        std::ofstream graph_file(filename);

        if (!graph_file.is_open())
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

                graph_file << "POSE3 " << x << " "
                           << pose_estimate.x() << " "
                           << pose_estimate.y() << " "
                           << pose_estimate.z() << " "
                           << rpy(0) << " " << rpy(1) << " " << rpy(2);

                if (marginals.has_value())
                {
                    auto pose_covariance = marginals.value().marginalCovariance(x);

                    for (int i = 0; i < 6; i++)
                        for (int j = 0; j < 6; j++)
                            graph_file << " " << pose_covariance(i, j);
                }
                graph_file << "\n";
            }
        }

        // POINT3 ln x y z Covariance(3x3)
        for (Symbol l : landmark_symbols_)
        {
            if (estimates.exists(l))
            {
                Point3 landmark_estimate = estimates.at<Point3>(l);

                graph_file << "POINT3 " << l << " "
                           << landmark_estimate.x() << " "
                           << landmark_estimate.y() << " "
                           << landmark_estimate.z();

                if (marginals.has_value())
                {
                    auto landmark_covariance = marginals.value().marginalCovariance(l);
                    for (int i = 0; i < 3; i++)
                        for (int j = 0; j < 3; j++)
                            graph_file << " " << landmark_covariance(i, j);
                }
                graph_file << "\n";
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

                graph_file << "PRIORPOSE3 " << key << " "
                           << prior.x() << " " << prior.y() << " " << prior.z() << " "
                           << rpy(0) << " " << rpy(1) << " " << rpy(2);

                for (int i = 0; i < 6; i++)
                    for (int j = 0; j < 6; j++)
                        graph_file << " " << covariance(i, j);
                graph_file << "\n";
            }

            // PRIORPOINT3
            if (auto priorPoint = boost::dynamic_pointer_cast<PriorFactor<Point3>>(factor))
            {
                Symbol key = priorPoint->keys().at(0);
                Point3 prior = priorPoint->prior();
                Matrix3 covariance = boost::dynamic_pointer_cast<noiseModel::Gaussian>(priorPoint->noiseModel())->covariance();

                graph_file << "PRIORPOINT3 " << key << " "
                           << prior.x() << " " << prior.y() << " " << prior.z();

                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        graph_file << " " << covariance(i, j);
                graph_file << "\n";
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

                graph_file << "BETWEENFACTOR3 " << pose_x1 << " " << pose_x2 << " "
                           << measurement.x() << " " << measurement.y() << " " << measurement.z() << " "
                           << rpy(0) << " " << rpy(1) << " " << rpy(2);

                for (int i = 0; i < 6; i++)
                    for (int j = 0; j < 6; j++)
                        graph_file << " " << covariance(i, j);
                graph_file << "\n";
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

                graph_file << "BEARINGRANGEFACTOR3 " << pose_x << " " << landmark_l << " "
                           << b.x() << " " << b.y() << " " << b.z() << " "
                           << range;

                for (int i = 0; i < 2; i++)
                    for (int j = 0; j < 2; j++)
                        graph_file << " " << covariance(i, j);

                graph_file << "\n";
            }
        }
    }
}