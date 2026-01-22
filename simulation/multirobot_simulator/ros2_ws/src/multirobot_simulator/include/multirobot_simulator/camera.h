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
    Camera(rclcpp::Node::SharedPtr node,
           const std::string& robot_name,
           Position& robot_position,
           std::shared_ptr<Environment> environment,
           const std::string& name,
           Position& position,
           double frequency,
           double max_range,
           double field_of_view,
           double noise_std_dev,
           std::string& topic);

private:
    void sensorUpdate() override;

    bool bresenhamObstacleCheck(
        int x0, int y0,
        int x1, int y1,
        int map_w, int map_h,
        const std::vector<std::vector<int8_t>>& map);

    std::vector<Point>& getLandmarks();

    void publishLandmarksMarker(std::vector<Point>& landmarks);
    void publishLandmarks(std::vector<Point>& landmarks);

    double max_range;
    double field_of_view;
    double noise_std_dev;
    double max_range_sq_;

    std::default_random_engine generator;
    std::normal_distribution<double> noise_dist_;

    std::vector<Point> landmarks_;

    rclcpp::Publisher<interfaces::msg::PointArray>::SharedPtr landmarks_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmarks_marker_publisher;
};

#endif
