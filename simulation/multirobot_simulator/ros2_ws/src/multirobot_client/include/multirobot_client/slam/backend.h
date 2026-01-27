#ifndef BACKEND_H
#define BACKEND_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <optional>

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
    struct BackendParams
    {
        double backend_rate;
        Eigen::Matrix<double, 6, 1> init_position; // [x, y, z, roll, pitch, yaw]
        Eigen::Vector3d init_velocity;             // m/s
        Eigen::Vector3d init_accelerometer_bias;   // m/s^2 bias
        Eigen::Vector3d init_gyroscope_bias;       // rad/s bias
        double sigma_odom_position_noise;
        double sigma_odom_orientation_noise;
        double sigma_loop_closure_position_noise;
        double sigma_loop_closure_orientation_noise;
        double sigma_accelerometer_noise_density;  // (m/s^2)/sqrt(s)
        double sigma_gyroscope_noise_density;      // (rad/s)/sqrt(s)
        double sigma_keypoint_noise;
        double data_association_distance;
        double keyframe_distance;
        double keyframe_angular_distance;

        BackendParams(
            double backend_rate = 10.0,
            Eigen::Matrix<double, 6, 1> init_position = Eigen::Matrix<double, 6, 1>::Zero(),
            Eigen::Vector3d init_velocity = Eigen::Vector3d::Zero(),
            Eigen::Vector3d init_accelerometer_bias = Eigen::Vector3d::Constant(1e-3),
            Eigen::Vector3d init_gyroscope_bias = Eigen::Vector3d::Constant(1e-3),
            double sigma_odom_position_noise = 0.05,
            double sigma_odom_orientation_noise = 0.01,
            double sigma_loop_closure_position_noise = 0.1,
            double sigma_loop_closure_orientation_noise = 0.05,
            double sigma_accelerometer_noise_density = 1e-3,
            double sigma_gyroscope_noise_density = 1e-3,
            double sigma_keypoint_noise = 0.1,
            double data_association_distance = 0.2,
            double keyframe_distance = 0.5,
            double keyframe_angular_distance = 0.524)
            : backend_rate(backend_rate),
              init_position(init_position),
              init_velocity(init_velocity),
              init_accelerometer_bias(init_accelerometer_bias),
              init_gyroscope_bias(init_gyroscope_bias),
              sigma_odom_position_noise(sigma_odom_position_noise),
              sigma_odom_orientation_noise(sigma_odom_orientation_noise),
              sigma_loop_closure_position_noise(sigma_loop_closure_position_noise),
              sigma_loop_closure_orientation_noise(sigma_loop_closure_orientation_noise),
              sigma_accelerometer_noise_density(sigma_accelerometer_noise_density),
              sigma_gyroscope_noise_density(sigma_gyroscope_noise_density),
              sigma_keypoint_noise(sigma_keypoint_noise),
              data_association_distance(data_association_distance),
              keyframe_distance(keyframe_distance),
              keyframe_angular_distance(keyframe_angular_distance) {}
    };

    class Backend
    {
    public:
        Backend();
        Backend(BackendParams &params);

        static BackendParams params_from_yaml(std::string &params_path);
        void init(BackendParams &params);

        void add_odom(OdomData &odom_data);
        void add_imu(ImuData &imu_data);
        void add_keypoints(KeypointsData &keypoints_data);
        State get_state();
        std::optional<State> get_state_if_updated();
        std::vector<KeyFrame> get_keyframes();
        
        void optimize();

        void add_loop_closure(const LoopClosureConstraint& loop_closure);

        
    private:
        std::vector<std::pair<Symbol, double>> nearest_neighbor_data_association(const Point3& observed_point, const Values& estimates);
        std::vector<std::pair<Symbol, double>> probabilistic_data_association(const Point3& observed_point, const Values& estimates, const Marginals& marginals);
        bool find_bounding_poses(double landmark_ts, Symbol &prev_sym, Symbol &next_sym, double &prev_ts, double &next_ts);
        void save_graph(NonlinearFactorGraph graph, Values estimates, std::optional<gtsam::Marginals> marginals, const std::string &filename);

        BackendParams params_;

        int t_;

        std::deque<OdomData> odom_buffer_;
        std::deque<ImuData> imu_buffer_;
        std::deque<KeypointsData> keypoints_buffer_;
        std::mutex buffer_mutex_;

        bool odom_first_;
        Pose3 last_odom_pose_;

        double imu_timestamp_prev_;

        std::deque<std::pair<double, Symbol>> timestamped_pose_queue_;
        double max_timestamped_pose_queue_duration_;

        int landmark_id_;
        std::vector<Symbol> landmark_symbols_;

        ISAM2 isam_;
        std::mutex isam_mutex_;

        PreintegratedImuMeasurements imu_preintegrated_;

        State state_;
        bool state_updated_;
        Pose3 pose_estimate_;
        Vector3 velocity_estimate_;
        imuBias::ConstantBias bias_estimate_;

        std::vector<KeyFrame> keyframes_;
        std::mutex keyframes_mutex_;

        noiseModel::Diagonal::shared_ptr odom_noise_;
        noiseModel::Diagonal::shared_ptr loop_closure_noise_;
    };
}

#endif // BACKEND_H
