#ifndef LOCALIZATION_H
#define LOCALIZATION_H

// #define LOCALIZATION_DEBUG
// #define FINE_LOCALIZATION_DEBUG
// #define LOCALIZATION_SAVE_GRAPH_DEBUG

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "interfaces/msg/point_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <flann/flann.hpp>

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

using namespace std::chrono_literals;

using namespace gtsam;

struct PosedScan
{
    PosedScan(){}

    PosedScan(Symbol x, Pose2 pose, const sensor_msgs::msg::LaserScan& scan, bool updated, bool keep = true)
        : x(x), pose(pose), scan(scan), updated(updated), keep(keep) {}

    PosedScan(Pose2 pose, const sensor_msgs::msg::LaserScan& scan, bool updated, bool keep = false)
        : pose(pose), scan(scan), updated(updated), keep(keep) {}

    Symbol x;
    Pose2 pose;
    sensor_msgs::msg::LaserScan scan;
    bool updated;
    bool keep;
};



class Localization
{
public:
    Localization() = default;
    Localization(rclcpp::Node::SharedPtr node);
    ~Localization();
    void startLocalization();
    std::shared_ptr<std::vector<PosedScan>> get_posed_scans() { return posed_scans; }
    std::shared_ptr<std::shared_mutex> get_posed_scans_mutex() { return posed_scans_mutex; }
    std::pair<std::shared_ptr<std::vector<PosedScan>>, std::shared_ptr<std::shared_mutex>> get_posed_scans_with_mutex() {return {posed_scans, posed_scans_mutex};}

private:
    void landmarks_callback(const interfaces::msg::PointArray::SharedPtr landmarks_msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void publish_map_to_odom();
    std::optional<Symbol> nearest_neighbor_data_association(const Point2& observed_point);
    std::optional<Symbol> flann_data_association(const Point2& observed_point);
    std::vector<std::pair<Symbol, double>> probabilistic_data_association(const Point2& observed_point);
    void localization();
    void fine_localization();
    void save_graph(NonlinearFactorGraph graph, Values estimates, Marginals marginals, const std::string &filename);

    std::shared_ptr<std::vector<PosedScan>> posed_scans;
    std::shared_ptr<std::shared_mutex> posed_scans_mutex;

    double isam_relinearize_threshold;
    int isam_relinearize_skip;

    std::unique_ptr<ISAM2> optimizer;
    NonlinearFactorGraph graph;
    Values graph_estimates;
    Values opt_estimates;
    
    Marginals marginals;

    int pose_id;
    Symbol last_x;
    Pose2 last_pose;
    Pose2 map_to_odom;

    int landmark_id;
    
    std::vector<std::pair<size_t, size_t>> odom_factor_indices_per_pose;
    std::vector<std::pair<size_t, size_t>> obs_factor_indices_per_pose;
    std::vector<size_t> temp_factor_indices;
    size_t factor_id_ref;

    std::vector<Pose2> window_odom_poses;

    size_t window_size;
    size_t lag;

    bool use_fixed_marginal;

    double min_pose_displacement;
    double min_pose_theta_displacement;

    std::string data_association_mode;
    bool use_all_landmarks;
    double data_association_distance;
    double mahalanobis_dist_threshold;

    std::vector<Symbol> pose_symbols;
    std::vector<Symbol> landmark_symbols;

    interfaces::msg::PointArray::SharedPtr landmarks;
    sensor_msgs::msg::LaserScan::SharedPtr scan;

    noiseModel::Diagonal::shared_ptr init_prior_noise;
    noiseModel::Diagonal::shared_ptr marginal_prior_noise;
    noiseModel::Diagonal::shared_ptr observation_noise;
    noiseModel::Diagonal::shared_ptr odometry_noise;

    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    double localization_rate;
    double fine_localization_rate;
    double map_to_odom_rate;

    bool map_to_odom_correction;

    rclcpp::Subscription<interfaces::msg::PointArray>::SharedPtr landmarks_subscription;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    std::unique_ptr<std::thread> localization_thread;
    std::unique_ptr<std::thread> fine_localization_thread;
    std::unique_ptr<std::thread> map_to_odom_thread;

    std::atomic<bool> localization_running;
    std::atomic<bool> fine_localization_running;
    std::atomic<bool> map_to_odom_running;

    rclcpp::Node::SharedPtr node;
};

#endif // LOCALIZATION_H
