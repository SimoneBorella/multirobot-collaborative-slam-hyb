#include "localization.h"

namespace multirobot_slam
{
    Localization::Localization()
        : t_(0), odom_first_(true), landmark_id_(0), localization_thread_running_(false)
    {
    }

    Localization::Localization(LocalizationParams &params)
        : params_(params), t_(0), odom_first_(true), landmark_id_(0), localization_thread_running_(false)
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

        // Update new factors
        isam_.update(new_factors, new_init_estimates);

        pose_estimate_ = init_pose;

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

        if (landmark_symbols_.empty())
            return results;

        double total_likelihood = 0.0;
        std::vector<std::tuple<Symbol, double, double>> temp_results;

        for (const Symbol& l : landmark_symbols_)
        {
            if (!estimates.exists(l))
                continue;
            
            Point3 landmark_estimate = estimates.at<Point3>(l);
            Vector3 dist = observed_point - landmark_estimate;

            double data_association_distance = 0.4;
            if (dist.norm() > data_association_distance)
                continue;

            // Could throw exception if marginal is singular
            // Matrix3 cov;
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
        // auto start = std::chrono::high_resolution_clock::now();

        std::deque<OdomData> odom_buffer;
        std::deque<LandmarksData> landmarks_buffer;

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            odom_buffer = odom_buffer_;
            landmarks_buffer = landmarks_buffer_;

            odom_buffer_.clear();
            landmarks_buffer_.clear();
        }

        if(odom_buffer.empty())
        {
            return;
        }

        int prev_t = t_-1;
        
        NonlinearFactorGraph new_factors;
        Values new_estimates;
            
            
        // Odom factors
        Pose3 prev_pose = last_odom_pose_;
        double prev_timestamp = state_.timestamp;
        
        Symbol x_prev('x', prev_t);
        
        for (const OdomData &odom : odom_buffer)
        {
            Pose3 odom_pose(
                Rot3::Quaternion(odom.orientation.w(), odom.orientation.x(), odom.orientation.y(), odom.orientation.z()),
                Point3(odom.position.x(), odom.position.y(), odom.position.z())
            );

            if (odom_first_)
            {
                last_odom_pose_ = odom_pose;
                prev_pose = odom_pose;
                odom_first_ = false;
                prev_timestamp = odom.timestamp;
                continue;
            }

            Pose3 delta_odom = prev_pose.between(odom_pose);

            auto odom_noise = noiseModel::Diagonal::Sigmas((Vector6() << 0.02,0.02,0.02,0.01,0.01,0.01).finished());

            Symbol x_curr('x', t_++);
            new_factors.add(BetweenFactor<Pose3>(x_prev, x_curr, delta_odom, odom_noise));

            new_estimates.insert(x_curr, pose_estimate_ * delta_odom);

            // Update previous
            prev_pose = odom_pose;
            x_prev = x_curr;
            prev_timestamp = odom.timestamp;
        }

        last_odom_pose_ = prev_pose;
        



        // Landmarks factors
        Values estimates = isam_.calculateEstimate();
        // Marginals marginals = Marginals(isam_.getFactorsUnsafe(), estimates);
        Marginals marginals;

        auto landmark_noise = noiseModel::Diagonal::Sigmas(Vector3(0.1,0.1,0.1));

        for (const LandmarksData &landmarks : landmarks_buffer)
        {
            int closest_index = 0;
            double min_time_gap = std::numeric_limits<double>::infinity();

            for (size_t i = 0; i < odom_buffer.size(); i++)
            {
                double time_gap = std::abs(odom_buffer[i].timestamp - landmarks.timestamp);
                if (time_gap < min_time_gap)
                {
                    min_time_gap = time_gap;
                    closest_index = i;
                }
            }

            int closest_t = t_ - odom_buffer.size() + closest_index;
            Symbol x_ref('x', closest_t);
            
            
            // Data association
            for (const Eigen::Vector3d &point : landmarks.points)
            {
                Eigen::Vector3d point_map = landmarks.pose.orientation * point + landmarks.pose.position;

                Point3 landmark_measurement(point.x(), point.y(), point.z());
                Unit3 bearing(landmark_measurement);
                double range = landmark_measurement.norm();

                Point3 landmark_observation(point_map.x(), point_map.y(), point_map.z());

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
    
                        new_factors.add(BearingRangeFactor<Pose3, Point3>(x_ref, associated_l, bearing, range, scaled_noise));
                    }
                }
                else
                {
                    Symbol l('l', landmark_id_++);
                    new_estimates.insert(l, landmark_observation);
                    landmark_symbols_.push_back(l);
    
                    auto huber_noise = noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.345),
                        landmark_noise
                    );
    
                    new_factors.add(BearingRangeFactor<Pose3, Point3>(x_ref, l, bearing, range, huber_noise));
                }
            }
        }

        size_t landmarks_max_size = 200;

        if (landmark_symbols_.size() > landmarks_max_size)
        {
            landmark_symbols_.erase(landmark_symbols_.begin(), landmark_symbols_.begin() + (landmark_symbols_.size() - landmarks_max_size));
        }


        // Update ISAM2
        isam_.update(new_factors, new_estimates);

        // Compute current estimate
        Symbol x_curr('x', t_-1);
        pose_estimate_ = isam_.calculateEstimate<Pose3>(x_curr);

        state_.timestamp = prev_timestamp;
        state_.position = pose_estimate_.translation();
        state_.attitude = Eigen::Quaterniond(pose_estimate_.rotation().matrix());

        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> duration = end - start;

        // if (t_%10 == 0)
        //     std::cout << "Time: " << (duration.count() * 1000) << " ms (" << 1/duration.count() << " Hz)" << std::endl;
    }
}