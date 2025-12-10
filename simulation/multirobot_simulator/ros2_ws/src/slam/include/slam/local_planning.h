#ifndef LOCAL_PLANNING_H
#define LOCAL_PLANNING_H

// #define LOCAL_PLANNING_DEBUG
// #define FGP_SAVE_GRAPH_DEBUG

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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

#include <iostream>
#include <vector>
#include <cmath>

#include "mapping.h"
#include "decision_global_planning.h"

using namespace std::chrono_literals;

using namespace gtsam;




class LocalPlanner
{
public:
    virtual ~LocalPlanner() = default;

    virtual std::map<std::string, geometry_msgs::msg::Twist> computeCommands(std::map<std::string, geometry_msgs::msg::TransformStamped>& robot_poses, int local_planning_count) = 0;
};










struct Pose
{
    double x;
    double y;
    double theta;
};


class DWA : public LocalPlanner
{
public:
    DWA() = default;
    DWA(rclcpp::Node::SharedPtr node, std::pair<std::shared_ptr<nav_msgs::msg::OccupancyGrid>, std::shared_ptr<std::shared_mutex>> costmap_grid_shared, std::pair<std::shared_ptr<std::map<std::string, nav_msgs::msg::Path>>, std::shared_ptr<std::shared_mutex>> robot_paths_shared, std::pair<std::shared_ptr<std::map<std::string, geometry_msgs::msg::Twist>>, std::shared_ptr<std::mutex>> robot_twist_shared);
    std::map<std::string, geometry_msgs::msg::Twist> computeCommands(std::map<std::string, geometry_msgs::msg::TransformStamped>& robot_poses, int local_planning_count);

private:
    geometry_msgs::msg::Twist computeDWACommand(const std::string& robot, const std::map<std::string, Pose> &robot_poses, const nav_msgs::msg::Path &global_path);
    Pose transformToPose(const geometry_msgs::msg::TransformStamped &transform);
    Pose simulateTrajectory(const Pose &pose, double v, double w);
    double getScoreTrajectory(const Pose pose, const Pose predict_pose, const nav_msgs::msg::Path &global_path, const std::map<std::string, Pose> &robot_poses, const std::string &robot);

    bool keep_internal_state;

    double stop_dist_threshold;
    double dt;
    double predict_time;
    double max_vel_x;
    double min_vel_x;
    double max_vel_theta;

    double max_acc_x;
    double max_acc_theta;
    double lookahead_dist;
    double robot_dist_threshold;
    double obstacle_dist_threshold;

    double w_progress_score;
    double w_heading_score;
    double w_robot_collision_score;
    double w_costmap_score;

    std::map<std::string, geometry_msgs::msg::Twist> last_cmd_vel;
    
    std::shared_ptr<std::map<std::string, geometry_msgs::msg::Twist>> robot_twist;
    std::shared_ptr<std::mutex> robot_twist_mutex;

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_grid;
    std::shared_ptr<std::shared_mutex> costmap_grid_mutex;

    std::shared_ptr<std::map<std::string, nav_msgs::msg::Path>> robot_paths;
    std::shared_ptr<std::shared_mutex> robot_paths_mutex;

    rclcpp::Node::SharedPtr node;
};
























struct PlanningRobotMetadata
{
    PlanningRobotMetadata(){};

    std::string robot_name;
    int robot_id;
};




inline Matrix makeDynamicsInformation(double sigma, double dt) {
    using Eigen::MatrixXd;
    using Eigen::Matrix2d;
    Matrix2d I = Matrix2d::Identity();
    Matrix2d Qc_inv = (1.0 / (sigma * sigma)) * I;

    MatrixXd Qi_inv(4,4);

    MatrixXd A = 12.0 * pow(dt, -3.0) * Qc_inv;
    MatrixXd B = -6.0 * pow(dt, -2.0) * Qc_inv;
    MatrixXd C = 4.0 / dt * Qc_inv;

    Qi_inv << A(0,0), A(0,1), B(0,0), B(0,1),
              A(1,0), A(1,1), B(1,0), B(1,1),
              B(0,0), B(0,1), C(0,0), C(0,1),
              B(1,0), B(1,1), C(1,0), C(1,1);

    return Qi_inv;
}



class DynamicsFactor : public NoiseModelFactor2<Vector4, Vector4> {
    double dt_;

public:
    DynamicsFactor(Key key1, Key key2, const SharedNoiseModel& model, double dt)
        : NoiseModelFactor2<Vector4, Vector4>(model, key1, key2), dt_(dt) {}

    Vector evaluateError(const Vector4& state1,
                        const Vector4& state2,
                        boost::optional<Matrix&> H1 = boost::none,
                        boost::optional<Matrix&> H2 = boost::none) const override
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
        if (H1) {
            *H1 = Matrix::Zero(4, 4);
            (*H1)(0,0) = 1.0; (*H1)(0,2) = dt_;
            (*H1)(1,1) = 1.0; (*H1)(1,3) = dt_;
            (*H1)(2,2) = 1.0;
            (*H1)(3,3) = 1.0;
        }
        if (H2) {
            *H2 = -Matrix::Identity(4, 4);
        }

        return error;
    }
};




class InterRobotFactor : public NoiseModelFactor2<Vector4, Vector4>
{
private:
    double min_dist_;

public:
    InterRobotFactor(Key key1, Key key2,
                     const SharedNoiseModel& model,
                     double min_dist)
        : NoiseModelFactor2<Vector4, Vector4>(model, key1, key2),
          min_dist_(min_dist) {}

    Vector evaluateError(const Vector4& state1,
                                const Vector4& state2,
                                boost::optional<Matrix&> H1 = boost::none,
                                boost::optional<Matrix&> H2 = boost::none) const override
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
                    *H1 = (Matrix(1,4) << grad_pos(0), grad_pos(1), 0.0, 0.0).finished();
                if (H2)
                    *H2 = (Matrix(1,4) << -grad_pos(0), -grad_pos(1), 0.0, 0.0).finished();
            }
            else
            {
                if (H1) *H1 = Matrix14::Zero();
                if (H2) *H2 = Matrix14::Zero();
            }
        }

        return (Vector(1) << error_val).finished();
    }
};



class ObstacleFactor : public NoiseModelFactor1<Vector4>
{
private:
    double resolution_;
    double origin_x_, origin_y_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_;
    double max_dist_;

public:
    ObstacleFactor(Key key,
                   std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap,
                   const SharedNoiseModel &model,
                   double max_dist)
        : NoiseModelFactor1<Vector4>(model, key),
          costmap_(costmap),
          max_dist_(max_dist)
    {
        resolution_ = costmap_->info.resolution;
        origin_x_   = costmap_->info.origin.position.x;
        origin_y_   = costmap_->info.origin.position.y;
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
            mx >= (int)costmap_->info.width ||
            my >= (int)costmap_->info.height)
        {
            if (H) *H = Matrix::Zero(1, 4);
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
                    nx >= (int)costmap_->info.width ||
                    ny >= (int)costmap_->info.height)
                    continue;

                double distance = std::sqrt(dx*dx + dy*dy) * resolution_;
                if (distance > max_dist_)
                    continue;

                int cost = static_cast<int>(costmap_->data[ny * costmap_->info.width + nx]);
                if (cost < 0) continue;

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
            Eigen::Vector2d grad(0,0);
            if (!std::isinf(min_dist) && closest_nx >= 0 && closest_ny >= 0)
            {
                double obs_x = origin_x_ + (closest_nx + 0.5) * resolution_;
                double obs_y = origin_y_ + (closest_ny + 0.5) * resolution_;

                double dx = x - obs_x;
                double dy = y - obs_y;
                double dist = std::sqrt(dx*dx + dy*dy);

                if (dist > 1e-6)
                {
                    Eigen::Vector2d dir(dx/dist, dy/dist);
                    grad = (-1.0 / max_dist_) * dir;
                }
            }
            *H = (Matrix(1,4) << grad.x(), grad.y(), 0, 0).finished();
        }

        return (Vector(1) << penalty).finished();
    }
};



class VelocityLimitFactor : public NoiseModelFactor1<Vector4>
{
private:
    double v_max_;
    double v_min_;

public:
    VelocityLimitFactor(Key key,
                        const SharedNoiseModel& model,
                        double v_min, double v_max)
        : NoiseModelFactor1<Vector4>(model, key),
          v_max_(v_max), v_min_(v_min) {}

    Vector evaluateError(const Vector4 &state,
                                boost::optional<Matrix &> H = boost::none) const override
    {
        double vx = state[2];
        double vy = state[3];
        double speed = std::sqrt(vx*vx + vy*vy);

        double error_val = 0.0;

        if(speed > v_max_) error_val = speed - v_max_;
        else if(speed < v_min_) error_val = v_min_ - speed;

        if(H) {
            Eigen::Vector4d grad = Eigen::Vector4d::Zero();
            if(speed > 1e-6) {
                grad[2] = (vx / speed) * (speed > v_max_ ? 1.0 : -1.0);
                grad[3] = (vy / speed) * (speed > v_max_ ? 1.0 : -1.0);
            }
            *H = (Matrix(1,4) << grad.transpose()).finished();
        }

        return (Vector(1) << error_val).finished();
    }
};




class AngularVelocityLimitFactor : public NoiseModelFactor2<Vector4, Vector4>
{
private:
    double dt_;
    double max_angular_;

public:
    AngularVelocityLimitFactor(Key key1, Key key2,
                               const SharedNoiseModel& model,
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
        if(std::abs(ang_vel) > max_angular_)
            error_val = std::abs(ang_vel) - max_angular_;

        if(H1 || H2) {
            // Jacobians can be approximated numerically or analytically (slightly more complex)
            if(H1) *H1 = Matrix::Zero(1,4);
            if(H2) *H2 = Matrix::Zero(1,4);
        }

        return (Vector(1) << error_val).finished();
    }
};












class FGP : public LocalPlanner
{
public:
    FGP() = default;
    FGP(rclcpp::Node::SharedPtr node, std::pair<std::shared_ptr<nav_msgs::msg::OccupancyGrid>, std::shared_ptr<std::shared_mutex>> costmap_grid_shared, std::pair<std::shared_ptr<std::map<std::string, nav_msgs::msg::Path>>, std::shared_ptr<std::shared_mutex>> robot_paths_shared, std::pair<std::shared_ptr<std::map<std::string, geometry_msgs::msg::Twist>>, std::shared_ptr<std::mutex>> robot_twist_shared);
    std::map<std::string, geometry_msgs::msg::Twist> computeCommands(std::map<std::string, geometry_msgs::msg::TransformStamped>& robot_poses, int local_planning_count);
    void save_graph(NonlinearFactorGraph graph, Values estimates, Marginals marginals, const std::string &filename);

private:
    std::vector<std::string> robots;

    bool keep_internal_state;

    double stop_dist_threshold;
    double dt;
    double predict_time;
    double max_vel_x;
    double min_vel_x;
    double max_vel_theta;

    noiseModel::Diagonal::shared_ptr start_prior_noise;
    noiseModel::Diagonal::shared_ptr goal_prior_noise;
    noiseModel::Gaussian::shared_ptr dynamic_noise;
    noiseModel::Gaussian::shared_ptr inter_robot_noise;
    noiseModel::Gaussian::shared_ptr obstacle_noise;

    double robot_dist_threshold;
    double obstacle_dist_threshold;

    int horizon_steps;
    double lookahead_dist;

    std::map<std::string, geometry_msgs::msg::Twist> last_cmd_vel;

    std::shared_ptr<std::map<std::string, geometry_msgs::msg::Twist>> robot_twist;
    std::shared_ptr<std::mutex> robot_twist_mutex;

    std::map<std::string, PlanningRobotMetadata> metadata;

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_grid;
    std::shared_ptr<std::shared_mutex> costmap_grid_mutex;

    std::shared_ptr<std::map<std::string, nav_msgs::msg::Path>> robot_paths;
    std::shared_ptr<std::shared_mutex> robot_paths_mutex;

    rclcpp::Node::SharedPtr node;
};




















class LocalPlanning
{
public:
    LocalPlanning() = default;
    LocalPlanning(rclcpp::Node::SharedPtr node, std::shared_ptr<Mapping> mapping, std::shared_ptr<DecisionGlobalPlanning> decision_global_planning);
    ~LocalPlanning();
    void startLocalPlanning();

private:

    void local_planning();

    std::unique_ptr<LocalPlanner> local_planner;

    std::vector<std::string> robots;

    std::string all_frame;
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    double local_planning_rate;
    std::string local_planner_type;

    int local_planning_count;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> robot_cmd_publishers;

    std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> robot_odom_subscriptions;
    std::shared_ptr<std::map<std::string, geometry_msgs::msg::Twist>> robot_twist;
    std::shared_ptr<std::mutex> robot_twist_mutex;

    std::unique_ptr<std::thread> local_planning_thread;
    std::atomic<bool> local_planning_running;

    rclcpp::Node::SharedPtr node;
};

#endif // LOCAL_PLANNING_H
