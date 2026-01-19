#include "localization.h"

namespace multirobot_slam
{
    Localization::Localization()
        : t_(0), odom_first_(true), imu_timestamp_prev_(0.0), max_timestamped_pose_queue_duration_(5.0), landmark_id_(0), state_updated_(false), localization_thread_running_(false)
    {
    }

    Localization::Localization(LocalizationParams &params)
        : params_(params), t_(0), odom_first_(true), imu_timestamp_prev_(0.0), max_timestamped_pose_queue_duration_(5.0), landmark_id_(0), state_updated_(false), localization_thread_running_(false)
    {
    }

    Localization::~Localization()
    {
        localization_thread_running_.store(false);
        if (localization_thread_.joinable())
        {
            localization_thread_.join();
        }
    }

    LocalizationParams Localization::params_from_yaml(std::string &params_path)
    {
        LocalizationParams p;

        try
        {
            YAML::Node config = YAML::LoadFile(params_path);

            if (config["localization_rate"])
                p.localization_rate = config["localization_rate"].as<double>();

            if (config["init_position"])
            {
                std::vector<double> vec = config["init_position"].as<std::vector<double>>();
                if (vec.size() == 6)
                    p.init_position = Eigen::Map<Eigen::Matrix<double, 6, 1>>(vec.data());
                else
                    std::cerr << "Param init_position must have 6 elements, ignoring.\n";
            }

            if (config["init_velocity"])
            {
                std::vector<double> vec = config["init_velocity"].as<std::vector<double>>();
                if (vec.size() == 3)
                    p.init_velocity = Eigen::Map<Eigen::Vector3d>(vec.data());
                else
                    std::cerr << "Param init_velocity must have 3 elements, ignoring.\n";
            }

            if (config["init_accelerometer_bias"])
            {
                std::vector<double> vec = config["init_accelerometer_bias"].as<std::vector<double>>();
                if (vec.size() == 3)
                    p.init_accelerometer_bias = Eigen::Map<Eigen::Vector3d>(vec.data());
                else
                    std::cerr << "Param init_accelerometer_bias must have 3 elements, ignoring.\n";
            }

            if (config["init_gyroscope_bias"])
            {
                std::vector<double> vec = config["init_gyroscope_bias"].as<std::vector<double>>();
                if (vec.size() == 3)
                    p.init_gyroscope_bias = Eigen::Map<Eigen::Vector3d>(vec.data());
                else
                    std::cerr << "Param init_gyroscope_bias must have 3 elements, ignoring.\n";
            }

            if (config["sigma_odom_position_noise"])
            {
                double val = config["sigma_odom_position_noise"].as<double>();
                p.sigma_odom_position_noise = val;
            }

            if (config["sigma_odom_orientation_noise"])
            {
                double val = config["sigma_odom_orientation_noise"].as<double>();
                p.sigma_odom_orientation_noise = val;
            }

            if (config["sigma_accelerometer_noise_density"])
            {
                double val = config["sigma_accelerometer_noise_density"].as<double>();
                p.sigma_accelerometer_noise_density = val;
            }

            if (config["sigma_gyroscope_noise_density"])
            {
                double val = config["sigma_gyroscope_noise_density"].as<double>();
                p.sigma_gyroscope_noise_density = val;
            }

            if (config["sigma_landmark_noise"])
            {
                double val = config["sigma_landmark_noise"].as<double>();
                p.sigma_landmark_noise = val;
            }

            if (config["data_association_distance"])
            {
                double val = config["data_association_distance"].as<double>();
                p.data_association_distance = val;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
            std::cerr << "Using default parameters.\n";
        }

        return p;
    }

    void Localization::init(LocalizationParams &params)
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

        // Pose prior
        Symbol x0('x', 0);
        Pose3 init_pose(
            Rot3::RzRyRx(params_.init_position(3), params_.init_position(4), params_.init_position(5)), // roll, pitch, yaw
            Point3(params_.init_position(0), params_.init_position(1), params_.init_position(2))        // x, y, z
        );
        noiseModel::Diagonal::shared_ptr init_pose_prior_noise = noiseModel::Diagonal::Sigmas(Vector6::Constant(1e-4));
        new_factors.add(PriorFactor<Pose3>(x0, init_pose, init_pose_prior_noise));
        new_init_estimates.insert(x0, init_pose);

        // Velocity prior
        Symbol v0('v', 0);
        Vector3 init_velocity(params_.init_velocity); // v_x, v_y, v_z
        noiseModel::Diagonal::shared_ptr init_vel_prior_noise = noiseModel::Diagonal::Sigmas(Vector3::Constant(1e-4));
        new_factors.add(PriorFactor<Vector3>(v0, init_velocity, init_vel_prior_noise));
        new_init_estimates.insert(v0, init_velocity);

        // Imu bias prior
        Symbol b0('b', 0);
        imuBias::ConstantBias init_bias = imuBias::ConstantBias(params_.init_accelerometer_bias, params_.init_gyroscope_bias);
        noiseModel::Diagonal::shared_ptr init_bias_prior_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(1), Vector3::Constant(1)).finished());
        new_factors.add(PriorFactor<imuBias::ConstantBias>(b0, init_bias, init_bias_prior_noise));
        new_init_estimates.insert(b0, init_bias);

        // Update new factors
        isam_.update(new_factors, new_init_estimates);

        // Imu preintegration
        auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(9.81);

        double var_acc = params_.sigma_accelerometer_noise_density *
                         params_.sigma_accelerometer_noise_density * params_.localization_rate;

        double var_gyr = params_.sigma_gyroscope_noise_density *
                         params_.sigma_gyroscope_noise_density * params_.localization_rate;

        imu_params->accelerometerCovariance = I_3x3 * var_acc; // 0.1   m/s² noise²
        imu_params->gyroscopeCovariance = I_3x3 * var_gyr;     // 0.01  rad/s² noise²
        imu_params->integrationCovariance = I_3x3 * 1e-6;      // integration uncertainty

        imu_preintegrated_ = PreintegratedImuMeasurements(imu_params, init_bias);

        // Initial estimates
        pose_estimate_ = init_pose;
        velocity_estimate_ = init_velocity;
        bias_estimate_ = init_bias;

        t_ = 1;
    }

    void Localization::start()
    {
        if (localization_thread_running_)
            return;

        localization_thread_running_.store(true);

        localization_thread_ = std::thread([this]()
                                           {
                auto period = std::chrono::milliseconds(
                    static_cast<int>(1000.0 / params_.localization_rate));

                auto next_time = std::chrono::steady_clock::now() + period;

                while (localization_thread_running_.load())
                {
                    localization();

                    std::this_thread::sleep_until(next_time);
                    next_time += period;
                } });
    }

    void Localization::add_odom_measurement(OdomData &odom_data)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        odom_buffer_.push_back(odom_data);
    }

    void Localization::add_imu_measurement(ImuData &imu_data)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        imu_buffer_.push_back(imu_data);
    }

    void Localization::add_landmarks_measurement(LandmarksData &landmarks_data)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        if (landmarks_data.points.size() == 0)
            return;

        landmarks_buffer_.push_back(landmarks_data);
    }

    State Localization::get_state()
    {
        return state_;
    }

    std::optional<State> Localization::get_state_if_updated()
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

    std::vector<std::pair<Symbol, double>> Localization::nearest_neighbor_data_association(const Point3 &observed_point, const Values &estimates)
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

    std::vector<std::pair<Symbol, double>> Localization::probabilistic_data_association(const Point3 &observed_point, const Values &estimates, const Marginals &marginals)
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

    bool Localization::find_bounding_poses(double landmark_ts, Symbol &prev_sym, Symbol &next_sym, double &prev_ts, double &next_ts)
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

    void Localization::localization()
    {
        // auto start = std::chrono::high_resolution_clock::now();

        std::deque<OdomData> odom_buffer;
        std::deque<ImuData> imu_buffer;
        std::deque<LandmarksData> landmarks_buffer;

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);

            if (odom_buffer_.empty() || imu_buffer_.empty())
                return;

            odom_buffer = odom_buffer_;
            imu_buffer = imu_buffer_;
            landmarks_buffer = landmarks_buffer_;

            odom_buffer_.clear();
            imu_buffer_.clear();
            landmarks_buffer_.clear();
        }

        NonlinearFactorGraph new_factors;
        Values new_estimates;

        Symbol x_prev('x', t_ - 1);
        Symbol v_prev('v', t_ - 1);
        Symbol b_prev('b', t_ - 1);

        Symbol x_curr('x', t_);
        Symbol v_curr('v', t_);
        Symbol b_curr('b', t_);

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
            auto odom_noise = noiseModel::Diagonal::Sigmas(
                (Vector6() << params_.sigma_odom_position_noise,
                 params_.sigma_odom_position_noise,
                 params_.sigma_odom_position_noise,
                 params_.sigma_odom_orientation_noise,
                 params_.sigma_odom_orientation_noise,
                 params_.sigma_odom_orientation_noise)
                    .finished());

            new_factors.add(BetweenFactor<Pose3>(x_prev, x_curr, delta_odom, odom_noise));
            // new_estimates.insert(x_curr, pose_estimate_ * delta_odom);
            last_odom_pose_ = odom_pose;
        }

        double ts = odom.timestamp;

        timestamped_pose_queue_.emplace_back(ts, x_curr);

        // Imu preintegration
        double total_dt = 0.0;
        for (const auto &imu_data : imu_buffer)
        {
            double imu_timestamp_curr = imu_data.timestamp;

            if (imu_timestamp_prev_ == 0.0)
            {
                imu_timestamp_prev_ = imu_timestamp_curr;
                continue;
            }

            double dt = imu_timestamp_curr - imu_timestamp_prev_;
            if (dt <= 0.0)
                continue;

            imu_preintegrated_.integrateMeasurement(imu_data.linear_acceleration,
                                                    imu_data.angular_velocity,
                                                    dt);
            total_dt += dt;
            imu_timestamp_prev_ = imu_timestamp_curr;
        }

        // Imu factor
        new_factors.add(ImuFactor(x_prev, v_prev, x_curr, v_curr, b_prev, imu_preintegrated_));

        // Bias evolution factor
        gtsam::Matrix6 bias_noise_covariance = gtsam::Matrix6::Zero();
        bias_noise_covariance.block<3, 3>(0, 0) = (params_.sigma_accelerometer_noise_density * params_.sigma_accelerometer_noise_density * total_dt) * gtsam::Matrix3::Identity();
        bias_noise_covariance.block<3, 3>(3, 3) = (params_.sigma_gyroscope_noise_density * params_.sigma_gyroscope_noise_density * total_dt) * gtsam::Matrix3::Identity();

        auto bias_noise = noiseModel::Gaussian::Covariance(bias_noise_covariance);

        new_factors.add(BetweenFactor<imuBias::ConstantBias>(b_prev, b_curr, imuBias::ConstantBias(), bias_noise));

        // Predict current state as initial estimate
        NavState predicted_state = imu_preintegrated_.predict(NavState(pose_estimate_, velocity_estimate_), bias_estimate_);

        new_estimates.insert(x_curr, predicted_state.pose());
        new_estimates.insert(v_curr, predicted_state.v());
        new_estimates.insert(b_curr, bias_estimate_);

        // Landmarks factors
        Values estimates = isam_.calculateEstimate();
        Marginals marginals = Marginals(isam_.getFactorsUnsafe(), estimates);

        auto landmark_noise = noiseModel::Diagonal::Sigmas(
            Vector3(params_.sigma_landmark_noise,
                    params_.sigma_landmark_noise,
                    params_.sigma_landmark_noise));

        if (!landmarks_buffer.empty())
        {
            LandmarksData landmarks = landmarks_buffer.back();

            double landmark_ts = landmarks.timestamp;

            Symbol prev_sym, next_sym;
            double prev_ts, next_ts;
            if (!find_bounding_poses(landmark_ts, prev_sym, next_sym, prev_ts, next_ts))
                return;

            Pose3 prev_pose = isam_.calculateEstimate<Pose3>(prev_sym);

            Pose3 landamrks_pose(
                Rot3::Quaternion(landmarks.pose.orientation.w(), landmarks.pose.orientation.x(), landmarks.pose.orientation.y(), landmarks.pose.orientation.z()),
                Point3(landmarks.pose.position.x(), landmarks.pose.position.y(), landmarks.pose.position.z()));

            Symbol k_curr('k', t_);

            Pose3 delta_pose = prev_pose.between(landamrks_pose);
            auto strong_noise = noiseModel::Diagonal::Sigmas((Vector6() << 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6).finished());

            new_factors.add(BetweenFactor<Pose3>(prev_sym, k_curr, delta_pose, strong_noise));
            new_estimates.insert(k_curr, landamrks_pose);

            for (const Eigen::Vector3d &point : landmarks.points)
            {
                Eigen::Vector3d point_map = landmarks.pose.orientation * point + landmarks.pose.position;

                Point3 landmark_measurement(point.x(), point.y(), point.z());
                Unit3 bearing(landmark_measurement);
                double range = landmark_measurement.norm();

                Point3 landmark_observation(point_map.x(), point_map.y(), point_map.z());

                // Data association
                auto associations = probabilistic_data_association(landmark_observation, estimates, marginals);
                // auto associations = nearest_neighbor_data_association(landmark_observation, estimates);

                if (!associations.empty())
                {
                    for (const auto &[associated_l, probability] : associations)
                    {
                        // To scale information
                        // I * p  =>  S / p => sigmas / sqrt(p)
                        auto scaled_noise = noiseModel::Robust::Create(
                            noiseModel::mEstimator::Huber::Create(1.345),
                            noiseModel::Diagonal::Sigmas(
                                landmark_noise->sigmas() / std::sqrt(probability)));

                        new_factors.add(BearingRangeFactor<Pose3, Point3>(k_curr, associated_l, bearing, range, scaled_noise));
                    }
                }
                else
                {
                    Symbol l('l', landmark_id_++);
                    new_estimates.insert(l, landmark_observation);
                    landmark_symbols_.push_back(l);

                    auto huber_noise = noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.345),
                        landmark_noise);

                    new_factors.add(BearingRangeFactor<Pose3, Point3>(k_curr, l, bearing, range, huber_noise));
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
        isam_.update(new_factors, new_estimates);

        // Compute current estimate
        pose_estimate_ = isam_.calculateEstimate<Pose3>(x_curr);
        velocity_estimate_ = isam_.calculateEstimate<Vector3>(v_curr);
        bias_estimate_ = isam_.calculateEstimate<imuBias::ConstantBias>(b_curr);

        state_.timestamp = odom.timestamp;
        state_.position = pose_estimate_.translation();
        state_.attitude = Eigen::Quaterniond(pose_estimate_.rotation().matrix());
        state_.velocity = velocity_estimate_;
        state_.accelerometer_bias = bias_estimate_.accelerometer();
        state_.gyroscope_bias = bias_estimate_.gyroscope();

        state_updated_ = true;

        // Reset IMU preintegration
        imu_preintegrated_.resetIntegrationAndSetBias(bias_estimate_);

        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> duration = end - start;

        // if (t_%10 == 0)
        //     std::cout << "Time: " << (duration.count() * 1000) << " ms (" << 1/duration.count() << " Hz)" << std::endl;

        // if (t_%100 == 0)
        // {
        //     Values isam_estimates = isam_.calculateEstimate();
        //     std::cout << "Saving ISAM2 graph " << t_ << std::endl;
        //     // Marginals isam_marginals(isam_.getFactorsUnsafe(), isam_estimates);
        //     // save_graph(isam_.getFactorsUnsafe(), isam_estimates, isam_marginals, "./output/localization/isam_graph_" + std::to_string(t_) + ".txt");
        //     save_graph(isam_.getFactorsUnsafe(), isam_estimates, std::nullopt, "./output/localization/isam_graph_" + std::to_string(t_) + ".txt");
        // }

        // Update discrete time
        t_++;
    }

    void Localization::save_graph(NonlinearFactorGraph graph, Values estimates, std::optional<gtsam::Marginals> marginals, const std::string &filename)
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

            Symbol k('k', t);

            if (estimates.exists(k))
            {
                Pose3 pose_estimate = estimates.at<Pose3>(k);

                Vector3 rpy = pose_estimate.rotation().rpy();

                graph_file << "POSE3 " << k << " "
                           << pose_estimate.x() << " "
                           << pose_estimate.y() << " "
                           << pose_estimate.z() << " "
                           << rpy(0) << " " << rpy(1) << " " << rpy(2);

                if (marginals.has_value())
                {
                    auto pose_covariance = marginals.value().marginalCovariance(k);

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