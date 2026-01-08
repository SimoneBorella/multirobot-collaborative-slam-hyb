#include "localization.h"

namespace localization
{
    Localization::Localization()
        : t(0), imu_timestamp_prev(0.0), odom_first_(true), gnss_first_(true), landmark_id(0), localization_thread_running(false)
    {
    }

    Localization::Localization(LocalizationParams &params)
        : params_(params), t(0), imu_timestamp_prev(0.0), odom_first_(true), gnss_first_(true), landmark_id(0), localization_thread_running(false)
    {
    }

    Localization::~Localization()
    {
        localization_thread_running.store(false);
        if (localization_thread.joinable())
        {
            localization_thread.join();
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

        isam = ISAM2(parameters);

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
        isam.update(new_factors, new_init_estimates);

        // Imu preintegration
        auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(9.81);

        double var_acc = params_.sigma_accelerometer_noise_density *
                         params_.sigma_accelerometer_noise_density * params_.localization_rate;

        double var_gyr = params_.sigma_gyroscope_noise_density *
                         params_.sigma_gyroscope_noise_density * params_.localization_rate;

        imu_params->accelerometerCovariance = I_3x3 * var_acc; // 0.1   m/s² noise²
        imu_params->gyroscopeCovariance = I_3x3 * var_gyr;     // 0.01  rad/s² noise²
        imu_params->integrationCovariance = I_3x3 * 1e-6;      // integration uncertainty

        imu_preintegrated = PreintegratedImuMeasurements(imu_params, init_bias);

        pose_estimate = init_pose;
        velocity_estimate = init_velocity;
        bias_estimate = init_bias;

        t = 1;
    }

    void Localization::start()
    {
        if (localization_thread_running)
            return;

        localization_thread_running.store(true);

        localization_thread = std::thread([this]()
                                          {
                auto period = std::chrono::milliseconds(
                    static_cast<int>(1000.0 / params_.localization_rate));

                auto next_time = std::chrono::steady_clock::now() + period;

                while (localization_thread_running.load())
                {
                    localization();

                    std::this_thread::sleep_until(next_time);
                    next_time += period;
                } });
    }

    void Localization::add_imu_measurement(ImuData &imu_data)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        imu_buffer_.push_back(imu_data);
    }

    void Localization::add_odom_measurement(OdomData &odom_data)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        odom_buffer_.push_back(odom_data);
    }

    void Localization::add_gnss_measurement(GNSSData &gnss_data)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        if (gnss_first_)
        {
            wgs84_reference_[0] = gnss_data.latitude;
            wgs84_reference_[1] = gnss_data.longitude;
            altitude_reference_ = gnss_data.altitude;

            gnss_first_ = false;
        }

        gnss_buffer_.push_back(gnss_data);
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


    std::vector<std::pair<Symbol, double>> Localization::probabilistic_data_association(const Point3& observed_point, const Values& estimates, const Marginals& marginals)
    {
        std::vector<std::pair<Symbol, double>> results;

        if (landmark_symbols.empty())
            return results;

        double total_likelihood = 0.0;
        std::vector<std::tuple<Symbol, double, double>> temp_results;

        for (const Symbol& l : landmark_symbols)
        {
            if (!estimates.exists(l))
                continue;
            
            Point3 landmark_estimate = estimates.at<Point3>(l);
            Vector3 dist = observed_point - landmark_estimate;

            double data_association_distance = 0.4;
            if (dist.norm() > data_association_distance)
                continue;

            // Matrix3 cov;
            // // Could throw exception if marginal is singular
            // try {
            //     cov = marginals.marginalCovariance(l);
            // } catch (...) {
            //     continue;
            // }

            // Constant covariance assumption to avoid marginal computation
            Matrix3 cov = Matrix3::Identity() * 0.05*0.05;


            double cov_determinant = cov.determinant();
            if (cov_determinant < 1e-6) {
                cov_determinant = 1e-6;
            }

            Matrix3 cov_inv = cov.inverse();

            // Compute the Mahalanobis distance squared:
            //   d^2 = (x - mu)^T sigma^(-1) (x - mu)
            // where x = observed_point, mu = landmark_estimate, sigma = covariance matrix
            double mahalanobis_dist = std::sqrt(dist.transpose() * cov_inv * dist);

            double mahalanobis_dist_threshold = 2.0;
            if (mahalanobis_dist > mahalanobis_dist_threshold)
                continue;

            // Compute the likelihood assuming a 3D Gaussian distribution:
            //   p(x) = (1 / ((2PI)^(n/2) * |sigma|^(1/2))) * exp(-0.5 * mahalanobis_dist^2)
            // where n = 3 for 3D points
            double likelihood = (1.0 / (std::pow(2.0 * M_PI, 1.5) * std::sqrt(cov_determinant))) * std::exp(-0.5 * mahalanobis_dist * mahalanobis_dist);

            // Apply threshold to likelihood to ignore weak associations
            temp_results.push_back({l, likelihood, mahalanobis_dist});
            total_likelihood += likelihood;
        }

        // Normalize likelihoods to get probabilities
        for (const auto& [l, likelihood, _] : temp_results)
        {
            if (total_likelihood > 0.0)
            {
                double probability = likelihood / total_likelihood;
                results.emplace_back(l, probability);
            }
        }

        return results;
    }



    void Localization::localization()
    {
        auto start = std::chrono::high_resolution_clock::now();


        std::deque<ImuData> imu_buffer;
        std::deque<OdomData> odom_buffer;
        std::deque<GNSSData> gnss_buffer;
        std::deque<LandmarksData> landmarks_buffer;

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            imu_buffer = imu_buffer_;
            odom_buffer = odom_buffer_;
            gnss_buffer = gnss_buffer_;
            landmarks_buffer = landmarks_buffer_;

            if (imu_buffer.empty() || (imu_timestamp_prev == 0.0 && imu_buffer.size() < 2))
                return;

            imu_buffer_.clear();
            odom_buffer_.clear();
            gnss_buffer_.clear();
            landmarks_buffer_.clear();
        }

        Symbol x_prev('x', t - 1);
        Symbol v_prev('v', t - 1);
        Symbol b_prev('b', t - 1);
        Symbol x_curr('x', t);
        Symbol v_curr('v', t);
        Symbol b_curr('b', t);

        NonlinearFactorGraph new_factors;
        Values new_estimates;

        // Imu preintegration
        double total_dt = 0.0;
        for (const auto &imu_data : imu_buffer)
        {
            double dt;
            double imu_timestamp_curr = imu_data.timestamp;

            if (imu_timestamp_prev == 0.0)
            {
                imu_timestamp_prev = imu_timestamp_curr;
                continue;
            }

            dt = imu_timestamp_curr - imu_timestamp_prev;
            imu_timestamp_prev = imu_data.timestamp;

            total_dt += dt;

            imu_preintegrated.integrateMeasurement(imu_data.linear_acceleration, imu_data.angular_velocity, dt);
        }

        // Imu factor
        new_factors.add(ImuFactor(x_prev, v_prev, x_curr, v_curr, b_prev, imu_preintegrated));

        // Odom factor
        if (!odom_buffer.empty())
        {
            const OdomData& odom = odom_buffer.back();

            Pose3 odom_pose(
                Rot3::Quaternion(odom.orientation.w(), odom.orientation.x(), odom.orientation.y(), odom.orientation.z()),
                Point3(odom.position.x(), odom.position.y(), odom.position.z())
            );

            if(odom_first_)
            {
                last_odom_pose_ = odom_pose;
                odom_first_ = false;
            }
            else
            {
                Pose3 delta_odom = last_odom_pose_.between(odom_pose);
    
                auto odom_noise = noiseModel::Diagonal::Sigmas((Vector6() << 0.02, 0.02, 0.02, 0.01, 0.01, 0.01).finished());
    
                new_factors.add(BetweenFactor<Pose3>(x_prev, x_curr, delta_odom, odom_noise));

                last_odom_pose_ = odom_pose;
            }
        }

        // Landmarks factors
        noiseModel::Diagonal::shared_ptr landmark_noise = noiseModel::Diagonal::Sigmas(Vector3(0.02, 0.02, 0.02));

        if (!landmarks_buffer.empty())
        {
            LandmarksData landmarks_data = landmarks_buffer.back();
        
            Values estimates = isam.calculateEstimate();
            // Marginals marginals = Marginals(isam.getFactorsUnsafe(), estimates);
            Marginals marginals;

            Pose3 current_pose = estimates.at<Pose3>(x_prev);

            for (const Eigen::Vector3d &point : landmarks_data.points)
            {
                Point3 landmark_measurement(point.x(), point.y(), point.z());
                Point3 landmark_observation = current_pose.transformFrom(landmark_measurement);

                // Unit3 bearing = current_pose.bearing(landmark_observation);
                // double range = current_pose.range(landmark_observation);

                Unit3 bearing(landmark_measurement);
                double range = landmark_measurement.norm();
    
                // Data association
                auto associations = probabilistic_data_association(landmark_observation, estimates, marginals);
                
                if (!associations.empty())
                {
                    for (const auto& [associated_l, probability] : associations)
                    {
                        // To scale information
                        // I * p  =>  S / p => sigmas / sqrt(p)
                        auto scaled_noise = noiseModel::Robust::Create(
                            noiseModel::mEstimator::Huber::Create(1.345),
                            noiseModel::Diagonal::Sigmas(
                                landmark_noise->sigmas() / std::sqrt(probability)
                            )
                        );
    
                        new_factors.add(BearingRangeFactor<Pose3, Point3>(x_curr, associated_l, bearing, range, scaled_noise));
                    }
                }
                else
                {
                    Symbol l('l', landmark_id++);
                    new_estimates.insert(l, landmark_observation);
                    landmark_symbols.push_back(l);
    
                    auto huber_noise = noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.345),
                        landmark_noise
                    );
    
                    new_factors.add(BearingRangeFactor<Pose3, Point3>(x_curr, l, bearing, range, huber_noise));
                }
            }
        }

        size_t landmarks_max_size = 150;

        if (landmark_symbols.size() > landmarks_max_size)
        {
            landmark_symbols.erase(landmark_symbols.begin(), landmark_symbols.begin() + (landmark_symbols.size() - landmarks_max_size));
        }



        // Bias evolution factor
        gtsam::Matrix6 bias_noise_covariance = gtsam::Matrix6::Zero();
        bias_noise_covariance.block<3, 3>(0, 0) = (params_.sigma_accelerometer_noise_density * params_.sigma_accelerometer_noise_density * total_dt) * gtsam::Matrix3::Identity();
        bias_noise_covariance.block<3, 3>(3, 3) = (params_.sigma_gyroscope_noise_density * params_.sigma_gyroscope_noise_density * total_dt) * gtsam::Matrix3::Identity();

        auto bias_noise = noiseModel::Gaussian::Covariance(bias_noise_covariance);

        new_factors.add(BetweenFactor<imuBias::ConstantBias>(b_prev, b_curr, imuBias::ConstantBias(), bias_noise));

        // Predict current state as initial estimate
        NavState predicted_state = imu_preintegrated.predict(NavState(pose_estimate, velocity_estimate), bias_estimate);

        new_estimates.insert(x_curr, predicted_state.pose());
        new_estimates.insert(v_curr, predicted_state.v());
        new_estimates.insert(b_curr, bias_estimate);



        // GNSS factor
        if (!gnss_buffer.empty())
        {
            GNSSData gnss_data = gnss_buffer.back();
            std::array<double, 2> wgs84_position{gnss_data.latitude, gnss_data.longitude};

            std::array<double, 2> xy = wgs84::to_cartesian(wgs84_reference_, wgs84_position);
            double z = gnss_data.altitude - altitude_reference_;

            Eigen::Vector3d gnss_position(xy[0], xy[1], z);

            Eigen::Matrix3d gnss_covariance = gnss_data.covariance;
            noiseModel::Gaussian::shared_ptr gnss_noise = noiseModel::Gaussian::Covariance(gnss_covariance);
            // noiseModel::Diagonal::shared_ptr gnss_noise = noiseModel::Diagonal::Sigmas(Vector3::Constant(0.001));

            new_factors.add(GPSFactor(x_curr, Point3(gnss_position), gnss_noise));
        }

        // Update ISAM2
        isam.update(new_factors, new_estimates);

        // Compute current estimate
        pose_estimate = isam.calculateEstimate<Pose3>(x_curr);
        velocity_estimate = isam.calculateEstimate<Vector3>(v_curr);
        bias_estimate = isam.calculateEstimate<imuBias::ConstantBias>(b_curr);

        state_.position = pose_estimate.translation();
        state_.attitude = Eigen::Quaterniond(pose_estimate.rotation().matrix());
        state_.velocity = velocity_estimate;
        state_.accelerometer_bias = bias_estimate.accelerometer();
        state_.gyroscope_bias = bias_estimate.gyroscope();

        // Reset IMU preintegration
        imu_preintegrated.resetIntegrationAndSetBias(bias_estimate);      

        // Update timestep
        t++;

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;


        // if (t%10 == 0)
        //     std::cout << "Time: " << (duration.count() * 1000) << " ms (" << 1/duration.count() << " Hz)" << std::endl;

        // if (t%10 == 0)
        //     std::cout << "Landmarks: " << landmark_symbols.size() << std::endl;
    }
}