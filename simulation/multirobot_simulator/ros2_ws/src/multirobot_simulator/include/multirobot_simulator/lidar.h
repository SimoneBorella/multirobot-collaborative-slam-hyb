#ifndef LIDAR_H
#define LIDAR_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "types.h"
#include "sensor.h"

class Lidar : public Sensor
{
public:
    Lidar(rclcpp::Node::SharedPtr node,
          const std::string& robot_name,
          Position& robot_position,
          std::shared_ptr<Environment> environment,
          const std::string& name,
          Position& position,
          double frequency,
          double min_range,
          double max_range,
          double resolution,
          int points,
          std::string& scan_topic,
          std::string& pcl_topic);

private:
    void sensorUpdate() override;

    std::vector<Point>& getLidarPoints();

    bool bresenhamRaytrace(
        int x0, int y0,
        int x1, int y1,
        int map_w, int map_h,
        const std::vector<double>& origin,
        double lidar_x, double lidar_y,
        double cos_base, double sin_base,
        std::vector<Point>& points);

    inline bool hitRobot(double wx, double wy) const;

    void publishLidarScan(std::vector<Point>& points);
    void publishLidarPointCloud2(std::vector<Point>& points);

    double min_range;
    double max_range;
    double resolution;
    int points;

    std::vector<std::vector<int8_t>> occupancy_map_aux;

    std::vector<double> ray_cos_;
    std::vector<double> ray_sin_;
    std::vector<Point> lidar_points_;

    double inv_resolution_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_plot_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_plot_publisher;
};

#endif
