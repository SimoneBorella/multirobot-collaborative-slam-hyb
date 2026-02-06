#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>
#include <message_filters/subscriber.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "interfaces/msg/key_point_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "interfaces/msg/map_log_odds_update.hpp"
#include "interfaces/msg/frontier_map_update.hpp"
#include "interfaces/msg/keyframe_update.hpp"
#include "interfaces/msg/keyframe_update_array.hpp"
#include "interfaces/msg/state.hpp"

#include "slam.h"
#include "mapping.h"

using namespace multirobot_slam;

using namespace std::chrono_literals;

class MultirobotClient : public rclcpp::Node
{
public:
    MultirobotClient()
        : Node("multirobot_client"), imu_first_(true)
    {
        ns_ = this->get_namespace();
        if (!ns_.empty() && ns_[0] == '/')
            ns_ = ns_.substr(1);

        declare_parameter("world_frame", std::string("world"));
        declare_parameter("map_frame", std::string("map"));
        declare_parameter("odom_frame", std::string("odom"));
        declare_parameter("base_frame", std::string("base_link"));

        world_frame_ = get_parameter("world_frame").as_string();
        map_frame_ = get_parameter("map_frame").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();

        double timer_rate = 50.0;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / timer_rate)),
            std::bind(&MultirobotClient::timer_callback, this));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        auto timer_interface =
            std::make_shared<tf2_ros::CreateTimerROS>(
                this->get_node_base_interface(),
                this->get_node_timers_interface());

        tf_buffer_->setCreateTimerInterface(timer_interface);

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        std::string imu_topic = "/" + ns_ + "/imu";
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, std::bind(&MultirobotClient::imu_callback, this, std::placeholders::_1));

        std::string odom_topic = "/" + ns_ + "/odom";
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&MultirobotClient::odom_callback, this, std::placeholders::_1));

        std::string keypoints_topic = "/" + ns_ + "/keypoints";
        keypoints_subscription_.subscribe(this, keypoints_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());

        keypoints_filter_ = std::make_shared<tf2_ros::MessageFilter<interfaces::msg::KeyPointArray>>(
            keypoints_subscription_,
            *tf_buffer_,
            map_frame_,
            10,
            this->get_node_logging_interface(),
            this->get_node_clock_interface());

        keypoints_filter_->registerCallback(std::bind(&MultirobotClient::keypoints_callback, this, std::placeholders::_1));

        std::string scan_topic = "/" + ns_ + "/scan";
        scan_subscription_.subscribe(this, scan_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());

        scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
            scan_subscription_,
            *tf_buffer_,
            map_frame_,
            10,
            this->get_node_logging_interface(),
            this->get_node_clock_interface());

        scan_filter_->registerCallback(std::bind(&MultirobotClient::scan_callback, this, std::placeholders::_1));

        rclcpp::QoS map_qos_profile(10);
        map_qos_profile.reliable();
        map_qos_profile.transient_local();

        // std::string map_topic = "/" + ns_ + "/map";
        // map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos_profile);
        
        std::string state_topic = "/" + ns_ + "/state";
        state_publisher_ = this->create_publisher<interfaces::msg::State>(state_topic, 10);

        std::string map_log_odds_update_topic = "/" + ns_ + "/map_log_odds_update";
        map_log_odds_update_publisher_ = this->create_publisher<interfaces::msg::MapLogOddsUpdate>(map_log_odds_update_topic, map_qos_profile);

        std::string frontier_map_update_topic = "/" + ns_ + "/frontier_map_update";
        frontier_map_update_publisher_ = this->create_publisher<interfaces::msg::FrontierMapUpdate>(frontier_map_update_topic, map_qos_profile);

        std::string keyframes_marker_topic = "/" + ns_ + "/keyframes_marker";
        keyframes_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(keyframes_marker_topic, 10);

        std::string keyframes_update_topic = "/" + ns_ + "/keyframes_update";
        keyframes_update_publisher_ = this->create_publisher<interfaces::msg::KeyframeUpdateArray>(keyframes_update_topic, 10);

        std::string submap_topic = "/" + ns_ + "/submap";
        submap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(submap_topic, map_qos_profile);
    }

    void setup()
    {
        std::string package_dir = ament_index_cpp::get_package_share_directory("multirobot_client");

        std::string slam_params_path = package_dir + "/params/slam_params.yaml";
        SLAMParams slam_params = SLAM::params_from_yaml(slam_params_path);

        std::string mapping_params_path = package_dir + "/params/mapping_params.yaml";
        MappingParams mapping_params = Mapping::params_from_yaml(mapping_params_path);

        slam_.init(slam_params);
        std::cout << "[Client " << ns_ << "]: " << "SLAM initialized." << std::endl;
        mapping_.init(mapping_params);
        std::cout << "[Client " << ns_ << "]: " << "Mapping initialized." << std::endl;

        slam_.start();
        std::cout << "[Client " << ns_ << "]: " << "SLAM started." << std::endl;
        mapping_.start();
        std::cout << "[Client " << ns_ << "]: " << "Mapping started." << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

private:
    void publish_map_to_odom(const State &state)
    {
        if (std::isnan(state.position.x()) || std::isnan(state.position.y()) || std::isnan(state.position.z()) ||
            std::isnan(state.orientation.x()) || std::isnan(state.orientation.y()) || std::isnan(state.orientation.z()) || std::isnan(state.orientation.w()) ||
            (state.orientation.x() == 0 && state.orientation.y() == 0 && state.orientation.z() == 0 && state.orientation.w() == 0))
        {
            return;
        }

        const rclcpp::Time state_time(
            static_cast<int64_t>(state.timestamp * 1e9));

        if (!tf_buffer_->canTransform(
                odom_frame_,
                base_frame_,
                state_time,
                rclcpp::Duration::from_seconds(0.5)))
        {
            // std::cout << "[Client " << ns_ << "] Could not read transform " << odom_frame_ << " -> " << base_frame_ << " at timestep " << state.timestamp << std::endl;
            return;
        }

        geometry_msgs::msg::TransformStamped odom_to_base =
            tf_buffer_->lookupTransform(
                odom_frame_,
                base_frame_,
                state_time);

        tf2::Transform T_odom_base;
        tf2::fromMsg(odom_to_base.transform, T_odom_base);

        tf2::Transform T_map_base;
        tf2::Quaternion q_map_base(
            state.orientation.x(),
            state.orientation.y(),
            state.orientation.z(),
            state.orientation.w());
        tf2::Vector3 t_map_base(
            state.position.x(),
            state.position.y(),
            state.position.z());

        T_map_base.setOrigin(t_map_base);
        T_map_base.setRotation(q_map_base);

        tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = state_time;
        map_to_odom.header.frame_id = map_frame_;
        map_to_odom.child_frame_id = odom_frame_;
        map_to_odom.transform = tf2::toMsg(T_map_odom);

        tf_broadcaster_->sendTransform(map_to_odom);
    }

    void publish_state(const State& state)
    {
        interfaces::msg::State state_msg;
        state_msg.header.stamp = this->now();
        
        // Position
        state_msg.pose.position.x = state.position.x();
        state_msg.pose.position.y = state.position.y();
        state_msg.pose.position.z = state.position.z();
        // Orientation
        state_msg.pose.orientation.x = state.orientation.x();
        state_msg.pose.orientation.y = state.orientation.y();
        state_msg.pose.orientation.z = state.orientation.z();
        state_msg.pose.orientation.w = state.orientation.w();
        // Covariance (6x6 row-major)
        for (int row = 0; row < 6; ++row)
        {
            for (int col = 0; col < 6; ++col)
            {
                state_msg.covariance[row * 6 + col] = state.covariance(row, col);
            }
        }

        state_publisher_->publish(state_msg);
    }

    void publish_map_log_odds_update(const MapLogOddsUpdate &map_log_odds_update)
    {
        interfaces::msg::MapLogOddsUpdate map_log_odds_update_msg;

        map_log_odds_update_msg.header.stamp = this->now();
        map_log_odds_update_msg.resolution = map_log_odds_update.resolution;
        map_log_odds_update_msg.width = map_log_odds_update.width;
        map_log_odds_update_msg.height = map_log_odds_update.height;

        map_log_odds_update_msg.origin.position.x = map_log_odds_update.origin_position.x();
        map_log_odds_update_msg.origin.position.y = map_log_odds_update.origin_position.y();
        map_log_odds_update_msg.origin.position.z = map_log_odds_update.origin_position.z();
        map_log_odds_update_msg.origin.orientation.w = map_log_odds_update.origin_orientation.w();
        map_log_odds_update_msg.origin.orientation.x = map_log_odds_update.origin_orientation.x();
        map_log_odds_update_msg.origin.orientation.y = map_log_odds_update.origin_orientation.y();
        map_log_odds_update_msg.origin.orientation.z = map_log_odds_update.origin_orientation.z();

        map_log_odds_update_msg.indices = map_log_odds_update.indices;
        map_log_odds_update_msg.delta_log_odds = map_log_odds_update.delta_log_odds;

        map_log_odds_update_publisher_->publish(map_log_odds_update_msg);
    }

    // void publish_map(const Map& map)
    // {
    //     nav_msgs::msg::OccupancyGrid map_msg;
    //     map_msg.header.stamp = this->now();
    //     map_msg.header.frame_id = ns_ + "/" + map_frame_;
    //     map_msg.info.resolution = map.resolution;
    //     map_msg.info.width = map.width;
    //     map_msg.info.height = map.height;
    //     map_msg.info.origin.position.x = map.origin_position.x();
    //     map_msg.info.origin.position.y = map.origin_position.y();
    //     map_msg.info.origin.position.z = map.origin_position.z();
    //     map_msg.info.origin.orientation.w = map.origin_orientation.w();
    //     map_msg.info.origin.orientation.x = map.origin_orientation.x();
    //     map_msg.info.origin.orientation.y = map.origin_orientation.y();
    //     map_msg.info.origin.orientation.z = map.origin_orientation.z();
    //     map_msg.data = map.data;
    //     map_publisher_->publish(map_msg);
    // }

    void publish_frontier_map_update(const FrontierMapUpdate &frontier_map_update)
    {
        interfaces::msg::FrontierMapUpdate frontier_map_update_msg;

        frontier_map_update_msg.header.stamp = this->now();
        frontier_map_update_msg.resolution = frontier_map_update.resolution;
        frontier_map_update_msg.width = frontier_map_update.width;
        frontier_map_update_msg.height = frontier_map_update.height;

        frontier_map_update_msg.origin.position.x = frontier_map_update.origin_position.x();
        frontier_map_update_msg.origin.position.y = frontier_map_update.origin_position.y();
        frontier_map_update_msg.origin.position.z = frontier_map_update.origin_position.z();
        frontier_map_update_msg.origin.orientation.w = frontier_map_update.origin_orientation.w();
        frontier_map_update_msg.origin.orientation.x = frontier_map_update.origin_orientation.x();
        frontier_map_update_msg.origin.orientation.y = frontier_map_update.origin_orientation.y();
        frontier_map_update_msg.origin.orientation.z = frontier_map_update.origin_orientation.z();

        frontier_map_update_msg.frontier_indices = frontier_map_update.frontier_indices;
        frontier_map_update_msg.explored_indices = frontier_map_update.explored_indices;

        frontier_map_update_publisher_->publish(frontier_map_update_msg);
    }

    void publish_keyframes(const std::map<int, KeyFrame> &keyframes)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        for (const auto &[id, kf] : keyframes)
        {
            // Keyframe pose marker
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = ns_ + "/" + map_frame_;
            marker.header.stamp = this->now();
            marker.ns = "keyframes";
            marker.id = kf.keyframe_id;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.lifetime = rclcpp::Duration(0, 0);

            marker.pose.position.x = kf.pose.position.x();
            marker.pose.position.y = kf.pose.position.y();
            marker.pose.position.z = kf.pose.position.z();

            marker.pose.orientation.x = kf.pose.orientation.x();
            marker.pose.orientation.y = kf.pose.orientation.y();
            marker.pose.orientation.z = kf.pose.orientation.z();
            marker.pose.orientation.w = kf.pose.orientation.w();

            marker.scale.x = 0.25;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

            marker_array.markers.push_back(marker);

            // Covariance ellipse marker
            Eigen::Matrix2d cov_xy;
            cov_xy << kf.covariance(0,0), kf.covariance(0,1),
                    kf.covariance(1,0), kf.covariance(1,1);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov_xy);
            if (solver.info() != Eigen::Success)
                continue;

            Eigen::Vector2d eigenvalues = solver.eigenvalues();
            Eigen::Matrix2d eigenvectors = solver.eigenvectors();

            // 95% confidence interval (chi-square, 2 DoF)
            constexpr double chi2_95 = 5.991;

            double a = std::sqrt(std::max(eigenvalues(1), 0.0) * chi2_95); // major axis
            double b = std::sqrt(std::max(eigenvalues(0), 0.0) * chi2_95); // minor axis

            Eigen::Vector2d major_axis = eigenvectors.col(1);
            double yaw = std::atan2(major_axis.y(), major_axis.x());

            visualization_msgs::msg::Marker ellipse;
            ellipse.header.frame_id = ns_ + "/" + map_frame_;
            ellipse.header.stamp = this->now();
            ellipse.ns = "keyframe_covariance";
            ellipse.id = 100000 + kf.keyframe_id;
            ellipse.action = visualization_msgs::msg::Marker::ADD;
            ellipse.type = visualization_msgs::msg::Marker::CYLINDER;
            ellipse.lifetime = rclcpp::Duration(0, 0);

            ellipse.pose.position.x = kf.pose.position.x();
            ellipse.pose.position.y = kf.pose.position.y();
            ellipse.pose.position.z = kf.pose.position.z() - 0.01;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            ellipse.pose.orientation = tf2::toMsg(q);

            ellipse.scale.x = 2.0 * a;   // diameter
            ellipse.scale.y = 2.0 * b;
            ellipse.scale.z = 0.01;

            ellipse.color.r = 0.0f;
            ellipse.color.g = 0.0f;
            ellipse.color.b = 1.0f;
            ellipse.color.a = 0.4f;

            marker_array.markers.push_back(ellipse);
        }

        keyframes_marker_publisher_->publish(marker_array);
    }


    void publish_keyframes_updates(const std::map<int, KeyFrame> &keyframes_updates)
    {
        interfaces::msg::KeyframeUpdateArray keyframes_update_array_msg;
        keyframes_update_array_msg.header.stamp = this->now();

        for (const auto &[id, kf] : keyframes_updates)
        {
            interfaces::msg::KeyframeUpdate keyframe_update_msg;
            keyframe_update_msg.keyframe_id = id;

            // Position
            keyframe_update_msg.pose.position.x = kf.pose.position.x();
            keyframe_update_msg.pose.position.y = kf.pose.position.y();
            keyframe_update_msg.pose.position.z = kf.pose.position.z();
            // Orientation
            keyframe_update_msg.pose.orientation.x = kf.pose.orientation.x();
            keyframe_update_msg.pose.orientation.y = kf.pose.orientation.y();
            keyframe_update_msg.pose.orientation.z = kf.pose.orientation.z();
            keyframe_update_msg.pose.orientation.w = kf.pose.orientation.w();
            // Covariance (6x6 row-major)
            for (int row = 0; row < 6; ++row)
            {
                for (int col = 0; col < 6; ++col)
                {
                    keyframe_update_msg.covariance[row * 6 + col] = kf.covariance(row, col);
                }
            }
            keyframes_update_array_msg.updates.push_back(keyframe_update_msg);
        }

        keyframes_update_publisher_->publish(keyframes_update_array_msg);
    }


    void publish_submaps(const std::map<int, Map> &updated_submaps)
    {
        rclcpp::Time current_time = this->now();

        for (const auto &[keyframe_id, map] : updated_submaps)
        {
            // Publish tf
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = current_time;
            tf_msg.header.frame_id = map_frame_;
            tf_msg.child_frame_id = "submap_" + std::to_string(map.keyframe_id);

            tf_msg.transform.translation.x = map.origin_position.x();
            tf_msg.transform.translation.y = map.origin_position.y();
            tf_msg.transform.translation.z = map.origin_position.z();

            tf_msg.transform.rotation.x = map.origin_orientation.x();
            tf_msg.transform.rotation.y = map.origin_orientation.y();
            tf_msg.transform.rotation.z = map.origin_orientation.z();
            tf_msg.transform.rotation.w = map.origin_orientation.w();

            tf_broadcaster_->sendTransform(tf_msg);

            // Publish occupancy grid map
            nav_msgs::msg::OccupancyGrid grid_msg;
            grid_msg.header.stamp = current_time;
            grid_msg.header.frame_id = ns_ + "/submap_" + std::to_string(map.keyframe_id);

            grid_msg.info.resolution = map.resolution;
            grid_msg.info.width = map.width;
            grid_msg.info.height = map.height;

            grid_msg.info.origin.position.x = -(map.width * map.resolution) / 2;
            grid_msg.info.origin.position.y = -(map.height * map.resolution) / 2;
            grid_msg.info.origin.position.z = 0.0;
            grid_msg.info.origin.orientation.x = 0.0;
            grid_msg.info.origin.orientation.y = 0.0;
            grid_msg.info.origin.orientation.z = 0.0;
            grid_msg.info.origin.orientation.w = 1.0;

            grid_msg.data = map.data;

            submap_publisher_->publish(grid_msg);
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Eigen::Matrix3d R_flip;
        R_flip << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;
        Eigen::Quaterniond q_flip(R_flip);

        if (imu_first_)
        {
            Eigen::Quaterniond q_init(
                msg->orientation.w,
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z);

            imu_initial_orientation_ = q_flip * q_init;
            imu_initial_orientation_.normalize();

            imu_first_ = false;
            return;
        }

        ImuData imu_data;

        imu_data.timestamp =
            msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // Orientation
        Eigen::Quaterniond q_current(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z);

        Eigen::Quaterniond q_map = q_flip * q_current;
        Eigen::Quaterniond q_relative = imu_initial_orientation_.inverse() * q_map;
        q_relative.normalize();
        imu_data.orientation = q_relative;

        // Angular velocity
        imu_data.angular_velocity = Eigen::Vector3d(
            msg->angular_velocity.x,
            -msg->angular_velocity.y,
            -msg->angular_velocity.z);

        // Linear acceleration
        imu_data.linear_acceleration = Eigen::Vector3d(
            msg->linear_acceleration.x,
            -msg->linear_acceleration.y,
            -msg->linear_acceleration.z);

        // std::cout << "[Client " << ns_ << "] IMU Linear acceleration: (x: " << msg->linear_acceleration.x << ", y: " << msg->linear_acceleration.y << ", z: " << msg->linear_acceleration.z << ")" << std::endl;
        // std::cout << "[Client " << ns_ << "] IMU Angular velocity: (x: " << msg->angular_velocity.x << ", y: " << msg->angular_velocity.y << ", z: " << msg->angular_velocity.z << ")" << std::endl;
        slam_.add_imu(imu_data);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        OdomData odom_data;
        odom_data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        odom_data.position = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        odom_data.orientation = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);

        slam_.add_odom(odom_data);
    }

    void keypoints_callback(const interfaces::msg::KeyPointArray::ConstSharedPtr msg)
    {
        KeypointsData keypoints_data;

        keypoints_data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        const rclcpp::Time keypoints_time = msg->header.stamp;

        geometry_msgs::msg::TransformStamped base_to_keypoints =
            tf_buffer_->lookupTransform(
                base_frame_,
                msg->header.frame_id,
                keypoints_time);

        tf2::Transform T_base_to_keypoints;
        tf2::fromMsg(base_to_keypoints.transform, T_base_to_keypoints);
        for (const auto &p : msg->keypoints)
        {
            tf2::Vector3 p_landmark(p.point.x, p.point.y, p.point.z);
            tf2::Vector3 p_base = T_base_to_keypoints * p_landmark;

            keypoints_data.keypoints.emplace_back(Eigen::Vector3d(p_base.x(), p_base.y(), p_base.z()), p.descriptor);
        }

        geometry_msgs::msg::TransformStamped map_to_base =
            tf_buffer_->lookupTransform(
                map_frame_,
                base_frame_,
                keypoints_time);

        keypoints_data.pose.position = Eigen::Vector3d(
            map_to_base.transform.translation.x,
            map_to_base.transform.translation.y,
            map_to_base.transform.translation.z);

        keypoints_data.pose.orientation = Eigen::Quaterniond(
            map_to_base.transform.rotation.w,
            map_to_base.transform.rotation.x,
            map_to_base.transform.rotation.y,
            map_to_base.transform.rotation.z);

        slam_.add_keypoints(keypoints_data);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        PosedScan posed_scan;
        posed_scan.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        const rclcpp::Time scan_time = msg->header.stamp;

        geometry_msgs::msg::TransformStamped map_to_scan =
            tf_buffer_->lookupTransform(
                map_frame_,
                msg->header.frame_id,
                scan_time);

        posed_scan.position = Eigen::Vector3d(
            map_to_scan.transform.translation.x,
            map_to_scan.transform.translation.y,
            map_to_scan.transform.translation.z);

        posed_scan.orientation = Eigen::Quaterniond(
            map_to_scan.transform.rotation.w,
            map_to_scan.transform.rotation.x,
            map_to_scan.transform.rotation.y,
            map_to_scan.transform.rotation.z);

        posed_scan.angle_min = msg->angle_min;
        posed_scan.angle_max = msg->angle_max;
        posed_scan.angle_increment = msg->angle_increment;
        posed_scan.time_increment = msg->time_increment;
        posed_scan.scan_time = msg->scan_time;
        posed_scan.range_min = msg->range_min;
        posed_scan.range_max = msg->range_max;
        posed_scan.ranges = msg->ranges;
        posed_scan.intensities = msg->intensities;

        slam_.add_posed_scan(posed_scan);
        mapping_.add_posed_scan(posed_scan);
    }

    void timer_callback()
    {
        const State &state = slam_.get_state();
        publish_map_to_odom(state);
        publish_state(state);


        const std::optional<MapLogOddsUpdate> &map_log_odds_update = mapping_.get_map_log_odds_update();

        if (map_log_odds_update.has_value())
        {
            publish_map_log_odds_update(map_log_odds_update.value());
        }

        // const std::optional<Map> &map = mapping_.get_map_if_updated();
        // if (map.has_value())
        // {
        //     publish_map(map.value());
        // }

        // const std::optional<Map> &frontier_map = mapping_.get_frontier_map_if_updated();
        // if (frontier_map.has_value())
        // {
        //     publish_frontier_map(frontier_map.value());
        // }

        const std::optional<FrontierMapUpdate> &frontier_map_update = mapping_.get_frontier_map_update();
        if (frontier_map_update.has_value())
        {
            publish_frontier_map_update(frontier_map_update.value());
        }

        std::map<int, KeyFrame> keyframes = slam_.get_keyframes();
        publish_keyframes(keyframes);

        std::map<int, KeyFrame> keyframes_updates = slam_.get_keyframes_updates();
        publish_keyframes_updates(keyframes_updates);

        std::map<int, Map> updated_submaps = slam_.get_updated_submaps();
        publish_submaps(updated_submaps);
    }

    std::string ns_;

    std::string world_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    SLAM slam_;
    Mapping mapping_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool imu_first_;
    Eigen::Quaterniond imu_initial_orientation_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    message_filters::Subscriber<interfaces::msg::KeyPointArray> keypoints_subscription_;
    std::shared_ptr<tf2_ros::MessageFilter<interfaces::msg::KeyPointArray>> keypoints_filter_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_subscription_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;

    // rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    
    rclcpp::Publisher<interfaces::msg::State>::SharedPtr state_publisher_;
    rclcpp::Publisher<interfaces::msg::MapLogOddsUpdate>::SharedPtr map_log_odds_update_publisher_;
    rclcpp::Publisher<interfaces::msg::FrontierMapUpdate>::SharedPtr frontier_map_update_publisher_;
    rclcpp::Publisher<interfaces::msg::KeyframeUpdateArray>::SharedPtr keyframes_update_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframes_marker_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr submap_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultirobotClient>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}