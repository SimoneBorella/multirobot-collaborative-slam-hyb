#ifndef SENSOR_H
#define SENSOR_H

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
#include <memory>
#include <string>
#include <boost/algorithm/string.hpp>
#include "types.h"
#include "environment.h"


class Sensor
{
public:
    Sensor(rclcpp::Node::SharedPtr node, const std::string& robot_name, Position& robot_position, std::shared_ptr<Environment> environment, const std::string& name, Position& position, double frequency)
        : robot_name(robot_name), robot_position(robot_position), name(name), position(position), frequency(frequency)
    {
        this->node = node;
        this->environment = environment;

        static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

        publishBaseLinkToSensorLinkTransform();
    }

    void startUpdateLoop()
    {
        sensor_update_thread = std::make_unique<std::thread>([this]() {
            rclcpp::Rate rate(frequency);
            while (rclcpp::ok()) {
                sensorUpdate();
                rate.sleep();
            }
        });
        sensor_update_thread->detach();

        RCLCPP_INFO_STREAM(node->get_logger(), "Sensor " << robot_name << "/" << name << " thread started.");
    }

    // void startUpdateLoop()
    // {
    //     sensor_update_thread = std::make_unique<std::thread>([this]() {
    //         rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    //         rclcpp::Duration duration(std::chrono::duration<double>(1.0 / frequency));
    //         while (rclcpp::ok()) {
    //             sensorUpdate();
    //             clock->sleep_for(duration);
    //         }
    //     });
    //     sensor_update_thread->detach();
    // }

    // void startUpdateLoop()
    // {
    //     sensor_update_thread = std::make_unique<std::thread>([this]() {
    //         rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    //         rclcpp::Duration duration(std::chrono::duration<double>(1.0 / frequency));
            
    //         auto next_time = clock->now();

    //         while (rclcpp::ok()) {
    //             sensorUpdate();
    //             next_time += duration;
    //             // RCLCPP_INFO_STREAM(node->get_logger(), boost::to_upper_copy<std::string>(name) + ": " << clock->now());
    //             clock->sleep_until(next_time);
    //         }
    //     });
    //     sensor_update_thread->detach();
    // }


    void publishBaseLinkToSensorLinkTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node->get_clock()->now();
        transform.header.frame_id = robot_name + "/base_link";
        transform.child_frame_id = robot_name + "/" + name + "_link";
    
        transform.transform.translation.x = position.x;
        transform.transform.translation.y = position.y;
        transform.transform.translation.z = 0.0;
    
        tf2::Quaternion q;
        q.setRPY(0, 0, position.theta);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        static_tf_broadcaster->sendTransform(transform);
    }

    void setRobotInitPosition(Position& robot_init_position)
    {
        this->robot_init_position = robot_init_position;
    }

    void setRobotPosition(Position& robot_position)
    {
        this->robot_position = robot_position;
    }

    void setOtherRobotsPositions(std::vector<Position>& other_robots_positions)
    {
        this->other_robots_positions = other_robots_positions;
    }

    void setOtherRobotsShapes(std::vector<Shape>& other_robots_shapes)
    {
        this->other_robots_shapes = other_robots_shapes;
    }


    virtual ~Sensor() {
        if (sensor_update_thread && sensor_update_thread->joinable()) {
            sensor_update_thread->join();
        }
    }

protected:
    virtual void sensorUpdate() = 0;

    std::string robot_name;
    Position robot_position;
    Position robot_init_position;
    std::vector<Position> other_robots_positions;
    std::vector<Shape> other_robots_shapes;
    std::shared_ptr<Environment> environment;
    std::string name;
    Position position;
    double frequency;
    std::unique_ptr<std::thread> sensor_update_thread;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
    rclcpp::Node::SharedPtr node;
};

#endif // SENSOR_H