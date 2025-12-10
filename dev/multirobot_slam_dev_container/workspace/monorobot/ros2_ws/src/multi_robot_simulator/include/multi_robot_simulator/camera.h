#ifndef CAMERA_H
#define CAMERA_H

#include <random>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "interfaces/msg/point_array.hpp"
#include "types.h"
#include "sensor.h"

class Camera : public Sensor
{
public:
    Camera(rclcpp::Node::SharedPtr node, const std::string& robot_name, Position& robot_position, std::shared_ptr<Environment> environment, const std::string& name, Position& position, double frequency, double max_range, double field_of_view, double noise_std_dev, std::string& topic)
        : Sensor(node, robot_name, robot_position, environment, name, position, frequency), max_range(max_range), field_of_view(field_of_view), noise_std_dev(noise_std_dev)
    {
        landmarks_publisher = node->create_publisher<interfaces::msg::PointArray>(topic, 10);
        landmarks_marker_publisher = node->create_publisher<visualization_msgs::msg::Marker>(topic + "/plot", 10);
    }

private:
    void sensorUpdate() override;
    bool bresenhamObstacleCheck(int x0, int y0, int x1, int y1, int occupancy_map_width, int occupancy_map_height, std::shared_ptr<std::vector<std::vector<int8_t>>> occupancy_map);
    std::vector<Point> getLandmarks();
    void publishLandmarksMarker(std::vector<Point>& landmarks);
    void publishLandmarks(std::vector<Point>& landmarks);

    double max_range;
    double field_of_view;
    double noise_std_dev;

    std::default_random_engine generator;

    rclcpp::Publisher<interfaces::msg::PointArray>::SharedPtr landmarks_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmarks_marker_publisher;
};

#endif // CAMERA_H
