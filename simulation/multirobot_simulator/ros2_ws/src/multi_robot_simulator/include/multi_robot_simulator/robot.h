#ifndef ROBOT_H
#define ROBOT_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/algorithm/string.hpp>
#include <thread>
#include <shared_mutex>
#include <limits>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include "types.h"
#include "sensor.h"
#include "lidar.h"
#include "camera.h"

class Robot
{
public:

    static void registerRobot(std::shared_ptr<Robot> robot)
    {
        robots.push_back(robot);
    }

    static const std::vector<std::shared_ptr<Robot>>& getAllRobots()
    {
        return robots;
    }


    Robot() = default;
    Robot(rclcpp::Node::SharedPtr node, std::shared_ptr<Environment> environment, const std::string& robot_yaml_path);
    ~Robot();

    void loadRobotConfiguration(const std::string& robot_yaml_path);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void spawnIfNotInitialized();
    void startUpdateLoop(double rate);
    void startStateUpdateLoop(double rate);
    void stateUpdate(double rate);
    void updatePosition(double rate);
    void updateIMUPosition(double rate);
    void publishOdometry();
    void publishFootprintMarker();


    void publishAllToMapTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node->get_clock()->now();
        transform.header.frame_id = "all";
        transform.child_frame_id = name + "/map";
    
        transform.transform.translation.x = init_position.x;
        transform.transform.translation.y = init_position.y;
        transform.transform.translation.z = 0.0;
    
        tf2::Quaternion q;
        q.setRPY(0, 0, init_position.theta);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        static_tf_broadcaster->sendTransform(transform);
    }



    void publishMapToOdomTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node->get_clock()->now();
        transform.header.frame_id = name + "/map";
        transform.child_frame_id = name + "/odom";

        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        static_tf_broadcaster->sendTransform(transform);
    }

    void publishOdomToBaseLinkTransform()
    {
        // {
        //     geometry_msgs::msg::TransformStamped transform;
        //     transform.header.stamp = node->get_clock()->now();
        //     transform.header.frame_id = name + "/odom";
        //     transform.child_frame_id = name + "/base_link";
        
        //     double dx = position.x - init_position.x;
        //     double dy = position.y - init_position.y;

        //     double translated_x = dx * cos(-init_position.theta) - dy * sin(-init_position.theta);
        //     double translated_y = dx * sin(-init_position.theta) + dy * cos(-init_position.theta);

        //     transform.transform.translation.x = translated_x;
        //     transform.transform.translation.y = translated_y;
        //     transform.transform.translation.z = 0.0;
        
        //     tf2::Quaternion q;
        //     double theta = position.theta - init_position.theta;

        //     theta = fmod(theta + M_PI, 2 * M_PI);
        //     if (theta < 0)
        //         theta += 2 * M_PI;
        //     theta -= M_PI;

        //     q.setRPY(0, 0, theta);
        //     transform.transform.rotation.x = q.x();
        //     transform.transform.rotation.y = q.y();
        //     transform.transform.rotation.z = q.z();
        //     transform.transform.rotation.w = q.w();

        //     tf_broadcaster->sendTransform(transform);
        // }

        {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = node->get_clock()->now();
            transform.header.frame_id = name + "/odom";
            transform.child_frame_id = name + "/base_link";
            // transform.child_frame_id = name + "/base_link_imu";
        
            double dx = imu_position.x - init_position.x;
            double dy = imu_position.y - init_position.y;

            double translated_x = dx * cos(-init_position.theta) - dy * sin(-init_position.theta);
            double translated_y = dx * sin(-init_position.theta) + dy * cos(-init_position.theta);

            transform.transform.translation.x = translated_x;
            transform.transform.translation.y = translated_y;
            transform.transform.translation.z = 0.0;
        
            tf2::Quaternion q;
            double theta = imu_position.theta - init_position.theta;

            theta = fmod(theta + M_PI, 2 * M_PI);
            if (theta < 0)
                theta += 2 * M_PI;
            theta -= M_PI;

            q.setRPY(0, 0, theta);
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            tf_broadcaster->sendTransform(transform);
        }

        {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = node->get_clock()->now();
            transform.header.frame_id = name + "/map";
            transform.child_frame_id = name + "/base_link_imu";
        
            double dx = imu_position.x - init_position.x;
            double dy = imu_position.y - init_position.y;

            double translated_x = dx * cos(-init_position.theta) - dy * sin(-init_position.theta);
            double translated_y = dx * sin(-init_position.theta) + dy * cos(-init_position.theta);

            transform.transform.translation.x = translated_x;
            transform.transform.translation.y = translated_y;
            transform.transform.translation.z = 0.0;
        
            tf2::Quaternion q;
            double theta = imu_position.theta - init_position.theta;

            theta = fmod(theta + M_PI, 2 * M_PI);
            if (theta < 0)
                theta += 2 * M_PI;
            theta -= M_PI;

            q.setRPY(0, 0, theta);
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            tf_broadcaster->sendTransform(transform);
        }
    }

    void publishBaseLinkToBaseFootprintTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node->get_clock()->now();
        transform.header.frame_id = name + "/base_link";
        transform.child_frame_id = name + "/base_footprint";
    
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
    
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        static_tf_broadcaster->sendTransform(transform);
    }


    Shape getShape() const
    {
        return shape;
    }

    bool isNotSpawned() const
    {
        return !random_spawn;
    }


    std::string toString() const
    {
        return name;
    }


    static std::vector<std::shared_ptr<Robot>> robots;

private:
    Command cmd;

    std::string name;
    bool random_spawn;
    Position position;
    Position imu_position;
    Position init_position;
    Shape shape;
    std::vector<std::shared_ptr<Sensor>> sensors;
    std::shared_ptr<Environment> environment;

    double linear_bias;
    double angular_bias;

    double linear_bias_std_dev;
    double angular_bias_std_dev;
    double alpha;
    double linear_scale_error;
    double angular_scale_error;

    std::default_random_engine generator;

    std::unique_ptr<std::thread> state_update_thread;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav_subscription;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr footprint_publisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

    rclcpp::Node::SharedPtr node;
};


// std::ostream& operator<<(std::ostream& os, const Robot& robot) {
//     os << robot.toString();
//     return os;
// };

#endif // ROBOT_H
