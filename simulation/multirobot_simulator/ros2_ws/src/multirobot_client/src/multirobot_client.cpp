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

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "interfaces/msg/point_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "interfaces/msg/map_log_odds_update.hpp"
#include "interfaces/msg/frontier.hpp"
#include "interfaces/msg/frontier_array.hpp"

#include "localization.h"
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
            imu_topic, 10, std::bind(&MultirobotClient::imu_callback, this, std::placeholders::_1)
        );

        std::string odom_topic = "/" + ns_ + "/odom";
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&MultirobotClient::odom_callback, this, std::placeholders::_1)
        );

        std::string landmarks_topic = "/" + ns_ + "/landmarks";
        landmarks_subscription_.subscribe(this, landmarks_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());

        landmarks_filter_ = std::make_shared<tf2_ros::MessageFilter<interfaces::msg::PointArray>>(
            landmarks_subscription_,
            *tf_buffer_,
            map_frame_,
            10,
            this->get_node_logging_interface(),
            this->get_node_clock_interface());
        
        landmarks_filter_->registerCallback(std::bind(&MultirobotClient::landmarks_callback, this, std::placeholders::_1));

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
        
        std::string map_topic = "/" + ns_ + "/map";
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos_profile);

        std::string map_log_odds_update_topic = "/" + ns_ + "/map_log_odds_update";
        map_log_odds_update_publisher_ = this->create_publisher<interfaces::msg::MapLogOddsUpdate>(map_log_odds_update_topic, map_qos_profile);
        
        std::string frontier_map_topic = "/" + ns_ + "/frontier_map";
        frontier_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(frontier_map_topic, map_qos_profile);

        std::string frontiers_topic = "/" + ns_ + "/frontiers";
        frontiers_publisher_ = this->create_publisher<interfaces::msg::FrontierArray>(frontiers_topic, 10);


        std::string frontiers_marker_topic = "/" + ns_ + "/frontiers_marker";
        frontiers_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(frontiers_marker_topic, 10);
    }

    void setup()
    {
        std::string package_dir = ament_index_cpp::get_package_share_directory("multirobot_client");

        std::string localization_params_path = package_dir + "/params/localization_params.yaml";
        LocalizationParams localization_params = Localization::params_from_yaml(localization_params_path);

        std::string mapping_params_path = package_dir + "/params/mapping_params.yaml";
        MappingParams mapping_params = Mapping::params_from_yaml(mapping_params_path);

        localization_.init(localization_params);
        std::cout << "[Client " << ns_ << "]: " << "Localization initialized." << std::endl;
        mapping_.init(mapping_params);
        std::cout << "[Client " << ns_ << "]: " << "Mapping initialized." << std::endl;

        localization_.start();
        std::cout << "[Client " << ns_ << "]: " << "Localization started." << std::endl;
        mapping_.start();
        std::cout << "[Client " << ns_ << "]: " << "Mapping started." << std::endl;
    }

private:

    void publish_map_to_odom(const State& state)
    { 
        const rclcpp::Time state_time(
            static_cast<int64_t>(state.timestamp * 1e9)
        );
        
        if (!tf_buffer_->canTransform(
            odom_frame_,
            base_frame_,
            state_time,
            rclcpp::Duration::from_seconds(0.1)))
        {
            std::cout << "[Client " << ns_ << "] Could not read transform " << odom_frame_ << " -> " << base_frame_ << " at timestep " << state.timestamp << std::endl;
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
            state.attitude.x(),
            state.attitude.y(),
            state.attitude.z(),
            state.attitude.w());
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

    void publish_map_log_odds_update(const MapLogOddsUpdate& map_log_odds_update)
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

        map_log_odds_update_msg.indicies = map_log_odds_update.indicies;
        map_log_odds_update_msg.delta_log_odds = map_log_odds_update.delta_log_odds;

        map_log_odds_update_publisher_->publish(map_log_odds_update_msg);
    }

    void publish_map(const Map& map)
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = ns_ + "/" + map_frame_;
        map_msg.info.resolution = map.resolution;
        map_msg.info.width = map.width;
        map_msg.info.height = map.height;
        map_msg.info.origin.position.x = map.origin_position.x();
        map_msg.info.origin.position.y = map.origin_position.y();
        map_msg.info.origin.position.z = map.origin_position.z();
        map_msg.info.origin.orientation.w = map.origin_orientation.w();
        map_msg.info.origin.orientation.x = map.origin_orientation.x();
        map_msg.info.origin.orientation.y = map.origin_orientation.y();
        map_msg.info.origin.orientation.z = map.origin_orientation.z();
        map_msg.data = map.data;
        map_publisher_->publish(map_msg);
    }

    void publish_frontier_map(const Map& frontier_map)
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = ns_ + "/" + map_frame_;
        map_msg.info.resolution = frontier_map.resolution;
        map_msg.info.width = frontier_map.width;
        map_msg.info.height = frontier_map.height;
        map_msg.info.origin.position.x = frontier_map.origin_position.x();
        map_msg.info.origin.position.y = frontier_map.origin_position.y();
        map_msg.info.origin.position.z = frontier_map.origin_position.z();
        map_msg.info.origin.orientation.w = frontier_map.origin_orientation.w();
        map_msg.info.origin.orientation.x = frontier_map.origin_orientation.x();
        map_msg.info.origin.orientation.y = frontier_map.origin_orientation.y();
        map_msg.info.origin.orientation.z = frontier_map.origin_orientation.z();
        map_msg.data = frontier_map.data;
        frontier_map_publisher_->publish(map_msg);
    }

    void publish_frontiers(const std::vector<Frontier>& frontiers)
    {
        interfaces::msg::FrontierArray msg;

        msg.header.stamp = this->now();
        msg.header.frame_id = ns_ + "/" + map_frame_;

        msg.frontiers.reserve(frontiers.size());

        for (size_t i=0; i<frontiers.size(); i++)
        {
            interfaces::msg::Frontier f;

            geometry_msgs::msg::Point p;
            p.x = frontiers[i].centroid.x();
            p.y = frontiers[i].centroid.y();
            p.z = 0.0;

            f.centroid = p;
            f.size = frontiers[i].size;

            msg.frontiers.push_back(f);
        }

        frontiers_publisher_->publish(msg);
    }


    void publish_frontiers_marker(const std::vector<Frontier>& frontiers)
    {
        visualization_msgs::msg::Marker frontiers_marker = visualization_msgs::msg::Marker();
        frontiers_marker.header.stamp = this->now();
        frontiers_marker.header.frame_id = ns_ + "/" + map_frame_;
    
        frontiers_marker.ns = "frontiers";
        frontiers_marker.id = 0;
        frontiers_marker.type = visualization_msgs::msg::Marker::POINTS;
        frontiers_marker.action = visualization_msgs::msg::Marker::ADD;

        frontiers_marker.color.r = 0.3f;
        frontiers_marker.color.g = 0.6f;
        frontiers_marker.color.b = 0.3f;
        frontiers_marker.color.a = 1.0f;

        frontiers_marker.scale.x = 0.2;
        frontiers_marker.scale.y = 0.2;

        for (const auto& frontier : frontiers) {
            geometry_msgs::msg::Point p;
            p.x = frontier.centroid.x();
            p.y = frontier.centroid.y();
            p.z = 0.0;
            frontiers_marker.points.push_back(p);
        }

        frontiers_marker_publisher_->publish(frontiers_marker);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        // Eigen::Matrix3d R_flip;
        // R_flip << 1, 0, 0,
        //     0, -1, 0,
        //     0, 0, -1;
        // Eigen::Quaterniond q_flip(R_flip);

        // if (imu_first_)
        // {
        //     Eigen::Quaterniond q_init(
        //         msg->orientation.w,
        //         msg->orientation.x,
        //         msg->orientation.y,
        //         msg->orientation.z);

        //     imu_initial_orientation_ = q_flip * q_init;
        //     imu_initial_orientation_.normalize();

        //     imu_first_ = false;
        //     return;
        // }

        // ImuData imu_data;

        // imu_data.timestamp =
        //     msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // // Orientation
        // Eigen::Quaterniond q_current(
        //     msg->orientation.w,
        //     msg->orientation.x,
        //     msg->orientation.y,
        //     msg->orientation.z);

        // Eigen::Quaterniond q_map = q_flip * q_current;
        // Eigen::Quaterniond q_relative = imu_initial_orientation_.inverse() * q_map;
        // q_relative.normalize();
        // imu_data.orientation = q_relative;

        // // Angular velocity
        // imu_data.angular_velocity = Eigen::Vector3d(
        //     msg->angular_velocity.x,
        //     -msg->angular_velocity.y,
        //     -msg->angular_velocity.z);

        // // Linear acceleration
        // imu_data.linear_acceleration = Eigen::Vector3d(
        //     msg->linear_acceleration.x,
        //     -msg->linear_acceleration.y,
        //     -msg->linear_acceleration.z);

        // // std::cout << "[Client " << ns_ << "] IMU Linear acceleration: (x: " << msg->linear_acceleration.x << ", y: " << msg->linear_acceleration.y << ", z: " << msg->linear_acceleration.z << ")" << std::endl;
        // // std::cout << "[Client " << ns_ << "] IMU Angular velocity: (x: " << msg->angular_velocity.x << ", y: " << msg->angular_velocity.y << ", z: " << msg->angular_velocity.z << ")" << std::endl;
        // localization_.add_imu_measurement(imu_data);
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

        localization_.add_odom_measurement(odom_data);
    }

    void landmarks_callback(const interfaces::msg::PointArray::ConstSharedPtr msg)
    {
        LandmarksData landmarks_data;
        
        landmarks_data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        const rclcpp::Time landmarks_time = msg->header.stamp;

        geometry_msgs::msg::TransformStamped base_to_landmarks =
            tf_buffer_->lookupTransform(
                base_frame_,
                msg->header.frame_id,
                landmarks_time
            );

        tf2::Transform T_base_to_landmarks;
        tf2::fromMsg(base_to_landmarks.transform, T_base_to_landmarks);

        for (const auto &p : msg->points)
        {
            tf2::Vector3 p_landmark(p.x, p.y, p.z);
            tf2::Vector3 p_base = T_base_to_landmarks * p_landmark;

            landmarks_data.points.emplace_back(p_base.x(), p_base.y(), p_base.z());
        }



        geometry_msgs::msg::TransformStamped map_to_base =
            tf_buffer_->lookupTransform(
                map_frame_,
                base_frame_,
                landmarks_time
            );
        
        landmarks_data.pose.position = Eigen::Vector3d(
            map_to_base.transform.translation.x,
            map_to_base.transform.translation.y,
            map_to_base.transform.translation.z);

        landmarks_data.pose.orientation = Eigen::Quaterniond(
            map_to_base.transform.rotation.w,
            map_to_base.transform.rotation.x,
            map_to_base.transform.rotation.y,
            map_to_base.transform.rotation.z);

        localization_.add_landmarks_measurement(landmarks_data);
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
                scan_time
            );

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

        mapping_.add_posed_scan(posed_scan);
    }


    void timer_callback()
    {
        const State &state = localization_.get_state();
        
        publish_map_to_odom(state);

        const std::optional<MapLogOddsUpdate> &map_log_odds_update = mapping_.get_map_log_odds_update();

        if (map_log_odds_update.has_value())
        {
            publish_map_log_odds_update(map_log_odds_update.value());
        }

        const std::optional<Map> &map = mapping_.get_map_if_updated();
        if (map.has_value())
        {
            publish_map(map.value());
        }

        const std::optional<Map> &frontier_map = mapping_.get_frontier_map_if_updated();
        if (frontier_map.has_value())
        {
            publish_frontier_map(frontier_map.value());
        }

        const std::optional<std::vector<Frontier>> &frontiers = mapping_.get_frontiers_if_updated();
        if (frontiers.has_value())
        {
            publish_frontiers(frontiers.value());
            publish_frontiers_marker(frontiers.value());
        }
    }

    std::string ns_;

    std::string world_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    Localization localization_;
    Mapping mapping_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool imu_first_;
    Eigen::Quaterniond imu_initial_orientation_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    message_filters::Subscriber<interfaces::msg::PointArray> landmarks_subscription_;
    std::shared_ptr<tf2_ros::MessageFilter<interfaces::msg::PointArray>> landmarks_filter_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_subscription_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Publisher<interfaces::msg::MapLogOddsUpdate>::SharedPtr map_log_odds_update_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr frontier_map_publisher_;
    rclcpp::Publisher<interfaces::msg::FrontierArray>::SharedPtr frontiers_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontiers_marker_publisher_;
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