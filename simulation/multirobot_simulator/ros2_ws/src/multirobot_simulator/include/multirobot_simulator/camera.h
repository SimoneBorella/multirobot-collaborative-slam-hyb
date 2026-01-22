#ifndef CAMERA_H
#define CAMERA_H

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "interfaces/msg/key_point_array.hpp"
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
    inline void addDescriptorNoise(std::array<uint8_t, 32>& descriptor);

    void sensorUpdate() override;

    bool bresenhamObstacleCheck(
        int x0, int y0,
        int x1, int y1,
        int map_w, int map_h,
        const std::vector<std::vector<int8_t>>& map);

    std::vector<KeyPoint>& getKeyPoints();

    void publishKeyPointsMarker(std::vector<KeyPoint>& keypoints);
    void publishKeyPoints(std::vector<KeyPoint>& keypoints);

    double max_range;
    double field_of_view;
    double noise_std_dev;
    double max_range_sq_;

    std::default_random_engine generator;
    std::normal_distribution<double> noise_dist_;

    std::uniform_int_distribution<int> byte_dist_{0, 32 - 1};
    std::uniform_int_distribution<int> bit_dist_{0, 7};

    std::vector<KeyPoint> keypoints_;

    rclcpp::Publisher<interfaces::msg::KeyPointArray>::SharedPtr keypoints_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr keypoints_marker_publisher;
};

#endif
