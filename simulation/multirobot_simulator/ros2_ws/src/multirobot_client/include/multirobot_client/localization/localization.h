#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/numericalDerivative.h>

#include "data_types.h"
#include "wgs84.h"
#include "factors.h"

using namespace gtsam;

namespace multirobot_slam
{
    struct LocalizationParams
    {
        double localization_rate;
        Eigen::Matrix<double, 6, 1> init_position; // [x, y, z, roll, pitch, yaw]
        Eigen::Vector3d init_velocity;             // m/s
        Eigen::Vector3d init_accelerometer_bias;   // m/s^2 bias
        Eigen::Vector3d init_gyroscope_bias;       // rad/s bias
        double sigma_accelerometer_noise_density;  // (m/s^2)/sqrt(s)
        double sigma_gyroscope_noise_density;      // (rad/s)/sqrt(s)

        LocalizationParams(
            double localization_rate = 10.0,
            Eigen::Matrix<double, 6, 1> init_position = Eigen::Matrix<double, 6, 1>::Zero(),
            Eigen::Vector3d init_velocity = Eigen::Vector3d::Zero(),
            Eigen::Vector3d init_accelerometer_bias = Eigen::Vector3d::Constant(1e-3),
            Eigen::Vector3d init_gyroscope_bias = Eigen::Vector3d::Constant(1e-3),
            double sigma_accelerometer_noise_density = 1e-3,
            double sigma_gyroscope_noise_density = 1e-3)
            : localization_rate(localization_rate),
              init_position(init_position),
              init_velocity(init_velocity),
              init_accelerometer_bias(init_accelerometer_bias),
              init_gyroscope_bias(init_gyroscope_bias),
              sigma_accelerometer_noise_density(sigma_accelerometer_noise_density),
              sigma_gyroscope_noise_density(sigma_gyroscope_noise_density) {}
    };

    class Localization
    {
    public:
        Localization();
        Localization(LocalizationParams &params);
        ~Localization();

        static LocalizationParams params_from_yaml(std::string &params_path);

        void init(LocalizationParams &params);
        void start();
        void add_imu_measurement(ImuData &imu_data);
        void add_odom_measurement(OdomData &odom_data);
        void add_gnss_measurement(GNSSData &gnss_data);
        void add_landmarks_measurement(LandmarksData &landmarks_data);
        State get_state();
        
    private:
        std::vector<std::pair<Symbol, double>> probabilistic_data_association(const Point3& observed_point, const Values& estimates, const Marginals& marginals);
        void localization();

        LocalizationParams params_;

        int t;
        double imu_timestamp_prev;

        std::deque<ImuData> imu_buffer_;
        std::deque<OdomData> odom_buffer_;
        std::deque<GNSSData> gnss_buffer_;
        std::deque<LandmarksData> landmarks_buffer_;
        std::mutex buffer_mutex_;

        bool odom_first_;
        Pose3 last_odom_pose_;

        bool gnss_first_;
        std::array<double, 2> wgs84_reference_;
        double altitude_reference_;

        int landmark_id_;
        std::vector<Symbol> landmark_symbols_;

        ISAM2 isam_;

        PreintegratedImuMeasurements imu_preintegrated_;

        State state_;
        Pose3 pose_estimate_;
        Vector3 velocity_estimate_;
        imuBias::ConstantBias bias_estimate_;

        std::atomic<bool> localization_thread_running_;
        std::thread localization_thread_;
    };
}

#endif // LOCALIZATION_H
