#ifndef LOCAL_PLANNING_H
#define LOCAL_PLANNING_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <thread>
#include <atomic>
#include <queue>
#include <deque>
#include <mutex>
#include <optional>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "data_types.h"

using namespace gtsam;

namespace multirobot_slam
{
    struct LocalPlanningParams
    {
        double local_planning_rate;

        double stop_dist_threshold;
        double dt;
        double predict_time;
        double max_vel_x;
        double min_vel_x;
        double max_vel_theta;
        double lookahead_dist;
        std::vector<double> start_prior_noise;
        std::vector<double> goal_prior_noise;
        double dynamic_noise;
        double inter_robot_noise;
        double obstacle_noise;
        double robot_dist_threshold;
        double obstacle_position_dist_threshold;
        double obstacle_orientation_dist_threshold;

        LocalPlanningParams(
            double local_planning_rate = 10.0,
            double stop_dist_threshold = 0.1,
            double dt = 0.1,
            double predict_time = 1.5,
            double max_vel_x = 0.22,
            double min_vel_x = -0.0,
            double max_vel_theta = 1.0,
            double lookahead_dist = 0.0,
            std::vector<double> start_prior_noise = std::vector<double>{0.001, 0.001, 0.001, 0.001},
            std::vector<double> goal_prior_noise = std::vector<double>{0.02, 0.02, 0.02, 0.02},
            double dynamic_noise = 0.2,
            double inter_robot_noise = 0.2,
            double obstacle_noise = 0.2,
            double robot_dist_threshold = 0.4,
            double obstacle_position_dist_threshold = 0.25,
            double obstacle_orientation_dist_threshold = 0.875)
            : local_planning_rate(local_planning_rate),
              stop_dist_threshold(stop_dist_threshold),
              dt(dt),
              predict_time(predict_time),
              max_vel_x(max_vel_x),
              min_vel_x(min_vel_x),
              max_vel_theta(max_vel_theta),
              lookahead_dist(lookahead_dist),
              start_prior_noise(start_prior_noise),
              goal_prior_noise(goal_prior_noise),
              dynamic_noise(dynamic_noise),
              inter_robot_noise(inter_robot_noise),
              obstacle_noise(obstacle_noise),
              robot_dist_threshold(robot_dist_threshold),
              obstacle_position_dist_threshold(obstacle_position_dist_threshold),
              obstacle_orientation_dist_threshold(obstacle_orientation_dist_threshold){}
    };

    class DynamicsFactor : public NoiseModelFactor2<Vector4, Vector4>
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double dt_;

    public:

        DynamicsFactor(Key key1, Key key2, const SharedNoiseModel &model, double dt)
            : NoiseModelFactor2<Vector4, Vector4>(model, key1, key2), dt_(dt) {}

        Vector evaluateError(const Vector4 &state1,
                             const Vector4 &state2,
                             boost::optional<Matrix &> H1 = boost::none,
                             boost::optional<Matrix &> H2 = boost::none) const override
        {

            // Predicted next state from current state
            Vector4 predicted;
            predicted(0) = state1(0) + state1(2) * dt_; // x + vx*dt
            predicted(1) = state1(1) + state1(3) * dt_; // y + vy*dt
            predicted(2) = state1(2);                   // vx stays same
            predicted(3) = state1(3);                   // vy stays same

            // Error = predicted - actual
            Vector4 error = predicted - state2;

            // Jacobians
            if (H1)
            {
                *H1 = Matrix::Zero(4, 4);
                (*H1)(0, 0) = 1.0;
                (*H1)(0, 2) = dt_;
                (*H1)(1, 1) = 1.0;
                (*H1)(1, 3) = dt_;
                (*H1)(2, 2) = 1.0;
                (*H1)(3, 3) = 1.0;
            }
            if (H2)
            {
                *H2 = -Matrix::Identity(4, 4);
            }

            return error;
        }
    };

    class InterRobotFactor : public NoiseModelFactor2<Vector4, Vector4>
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        double min_dist_;

    public:

        InterRobotFactor(Key key1, Key key2,
                         const SharedNoiseModel &model,
                         double min_dist)
            : NoiseModelFactor2<Vector4, Vector4>(model, key1, key2),
              min_dist_(min_dist) {}

        Vector evaluateError(const Vector4 &state1,
                             const Vector4 &state2,
                             boost::optional<Matrix &> H1 = boost::none,
                             boost::optional<Matrix &> H2 = boost::none) const override
        {
            // Relative position
            Vector2 diff(state1[0] - state2[0],
                         state1[1] - state2[1]);
            double dist = diff.norm();

            // Error definition: shrinks to 0 at min_dist
            double error_val = 1.0 - dist / min_dist_;

            // Jacobians
            if (H1 || H2)
            {
                if (dist > 1e-9)
                {
                    Vector2 dir = diff / dist;
                    Vector2 grad_pos = (-1.0 / min_dist_) * dir;

                    if (H1)
                        *H1 = (Matrix(1, 4) << grad_pos(0), grad_pos(1), 0.0, 0.0).finished();
                    if (H2)
                        *H2 = (Matrix(1, 4) << -grad_pos(0), -grad_pos(1), 0.0, 0.0).finished();
                }
                else
                {
                    if (H1)
                        *H1 = Matrix14::Zero();
                    if (H2)
                        *H2 = Matrix14::Zero();
                }
            }

            return (Vector(1) << error_val).finished();
        }
    };

    class ObstacleFactor : public NoiseModelFactor1<Vector4>
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        double resolution_;
        double origin_x_, origin_y_;
        Map &costmap_;
        double max_dist_;

    public:
        ObstacleFactor(Key key,
                       Map &costmap,
                       const SharedNoiseModel &model,
                       double max_dist)
            : NoiseModelFactor1<Vector4>(model, key),
              costmap_(costmap),
              max_dist_(max_dist)
        {
            resolution_ = costmap_.resolution;
            origin_x_ = costmap_.origin_position.x();
            origin_y_ = costmap_.origin_position.y();
        }

        Vector evaluateError(const Vector4 &state,
                             boost::optional<Matrix &> H = boost::none) const override
        {
            double x = state[0];
            double y = state[1];

            // World → grid
            int mx = static_cast<int>((x - origin_x_) / resolution_);
            int my = static_cast<int>((y - origin_y_) / resolution_);

            // Out of bounds → heavy penalty
            if (mx < 0 || my < 0 ||
                mx >= (int)costmap_.width ||
                my >= (int)costmap_.height)
            {
                if (H)
                    *H = Matrix::Zero(1, 4);
                return (Vector(1) << 10.0).finished();
            }

            // Search kernel around the cell
            int kernel = std::ceil(max_dist_ / resolution_);
            double min_dist = std::numeric_limits<double>::infinity();
            int closest_nx = -1, closest_ny = -1;

            for (int dy = -kernel; dy <= kernel; ++dy)
            {
                for (int dx = -kernel; dx <= kernel; ++dx)
                {
                    int nx = mx + dx;
                    int ny = my + dy;

                    if (nx < 0 || ny < 0 ||
                        nx >= (int)costmap_.width ||
                        ny >= (int)costmap_.height)
                        continue;

                    double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                    if (distance > max_dist_)
                        continue;

                    int cost = static_cast<int>(costmap_.data[ny * costmap_.width + nx]);
                    if (cost < 0)
                        continue;

                    if (cost >= 50 && distance < min_dist)
                    {
                        min_dist = distance;
                        closest_nx = nx;
                        closest_ny = ny;
                    }
                }
            }

            // Error value
            double penalty = 0.0;
            if (!std::isinf(min_dist))
                penalty = 1.0 - min_dist / max_dist_;

            // Jacobian
            if (H)
            {
                Eigen::Vector2d grad(0, 0);
                if (!std::isinf(min_dist) && closest_nx >= 0 && closest_ny >= 0)
                {
                    double obs_x = origin_x_ + (closest_nx + 0.5) * resolution_;
                    double obs_y = origin_y_ + (closest_ny + 0.5) * resolution_;

                    double dx = x - obs_x;
                    double dy = y - obs_y;
                    double dist = std::sqrt(dx * dx + dy * dy);

                    if (dist > 1e-6)
                    {
                        Eigen::Vector2d dir(dx / dist, dy / dist);
                        grad = (-1.0 / max_dist_) * dir;
                    }
                }
                *H = (Matrix(1, 4) << grad.x(), grad.y(), 0, 0).finished();
            }

            return (Vector(1) << penalty).finished();
        }
    };

    class VelocityLimitFactor : public NoiseModelFactor1<Vector4>
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        double v_max_;
        double v_min_;

    public:
        VelocityLimitFactor(Key key,
                            const SharedNoiseModel &model,
                            double v_min, double v_max)
            : NoiseModelFactor1<Vector4>(model, key),
              v_max_(v_max), v_min_(v_min) {}

        Vector evaluateError(const Vector4 &state,
                             boost::optional<Matrix &> H = boost::none) const override
        {
            double vx = state[2];
            double vy = state[3];
            double speed = std::sqrt(vx * vx + vy * vy);

            double error_val = 0.0;

            if (speed > v_max_)
                error_val = speed - v_max_;
            else if (speed < v_min_)
                error_val = v_min_ - speed;

            if (H)
            {
                Eigen::Vector4d grad = Eigen::Vector4d::Zero();
                if (speed > 1e-6)
                {
                    grad[2] = (vx / speed) * (speed > v_max_ ? 1.0 : -1.0);
                    grad[3] = (vy / speed) * (speed > v_max_ ? 1.0 : -1.0);
                }
                *H = (Matrix(1, 4) << grad.transpose()).finished();
            }

            return (Vector(1) << error_val).finished();
        }
    };

    class AngularVelocityLimitFactor : public NoiseModelFactor2<Vector4, Vector4>
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        double dt_;
        double max_angular_;

    public:
        AngularVelocityLimitFactor(Key key1, Key key2,
                                   const SharedNoiseModel &model,
                                   double dt, double max_ang)
            : NoiseModelFactor2<Vector4, Vector4>(model, key1, key2),
              dt_(dt), max_angular_(max_ang) {}

        Vector evaluateError(const Vector4 &state1,
                             const Vector4 &state2,
                             boost::optional<Matrix &> H1 = boost::none,
                             boost::optional<Matrix &> H2 = boost::none) const override
        {
            double heading1 = std::atan2(state1[3], state1[2]);
            double heading2 = std::atan2(state2[3], state2[2]);

            double ang_vel = (heading2 - heading1) / dt_;
            ang_vel = std::atan2(std::sin(ang_vel), std::cos(ang_vel)); // normalize

            double error_val = 0.0;
            if (std::abs(ang_vel) > max_angular_)
                error_val = std::abs(ang_vel) - max_angular_;

            if (H1 || H2)
            {
                // Jacobians can be approximated numerically or analytically (slightly more complex)
                if (H1)
                    *H1 = Matrix::Zero(1, 4);
                if (H2)
                    *H2 = Matrix::Zero(1, 4);
            }

            return (Vector(1) << error_val).finished();
        }
    };

    class LocalPlanning
    {
    public:
        LocalPlanning();
        LocalPlanning(LocalPlanningParams &params);
        ~LocalPlanning();

        static LocalPlanningParams params_from_yaml(std::string &params_path);

        void init(LocalPlanningParams &params);

        void start();

        void set_robot_pose_callback(std::function<std::map<std::string, Pose>()> callback);
        void set_send_vel_cmds_callback(std::function<void(const std::map<std::string, VelCmd, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, VelCmd>>>&)> callback);

        void update_global_paths(std::map<std::string, Path> global_paths);
        void update_costmap(Map costmap);

        std::map<std::string, VelCmd, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, VelCmd>>> get_vel_cmds();

    private:
        Matrix make_dynamics_information(double sigma, double dt);

        void local_planning();

        LocalPlanningParams params_;

        std::function<std::map<std::string, Pose>()> get_robot_poses_callback_;
        std::function<void(const std::map<std::string, VelCmd, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, VelCmd>>>&)> send_vel_cmds_callback_;

        int iter_count;

        int horizon_steps_;

        noiseModel::Diagonal::shared_ptr start_prior_noise_;
        noiseModel::Diagonal::shared_ptr goal_prior_noise_;
        noiseModel::Gaussian::shared_ptr dynamic_noise_;
        noiseModel::Gaussian::shared_ptr inter_robot_noise_;
        noiseModel::Gaussian::shared_ptr obstacle_noise_;

        // std::map<std::string, Pose> robot_poses_;
        std::map<std::string, Path> global_paths_;
        Map costmap_;

        bool costmap_received_;

        std::map<std::string, int> robot_ids_;


        std::map<std::string, VelCmd, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, VelCmd>>> last_vel_cmds_;
        std::map<std::string, VelCmd, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, VelCmd>>> vel_cmds_;

        std::mutex global_paths_mutex_;
        std::mutex costmap_mutex_;

        std::atomic<bool> local_planning_thread_running_;
        std::thread local_planning_thread_;
    };
}

#endif // LOCAL_PLANNING_H