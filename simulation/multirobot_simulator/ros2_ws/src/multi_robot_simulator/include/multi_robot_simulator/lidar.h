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
    Lidar(rclcpp::Node::SharedPtr node, const std::string& robot_name, Position& robot_position, std::shared_ptr<Environment> environment, const std::string& name, Position& position, double frequency, double min_range, double max_range, double resolution, int points, std::string& scan_topic, std::string& pcl_topic)
        : Sensor(node, robot_name, robot_position, environment, name, position, frequency), min_range(min_range), max_range(max_range), resolution(resolution), points(points)
    {
        scan_publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);
        pcl_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(pcl_topic, 10);

        scan_plot_publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic + "/plot", 10);
        pcl_plot_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(pcl_topic + "/plot", 10);
    
        occupancy_map_aux = environment->getOccupancyMapCopy();
    }

private:
    void sensorUpdate() override;

    std::vector<Point> getLidarPoints();
    bool bresenhamRaytrace(int x0, int y0, int x1, int y1, int occupancy_map_width, int occupancy_map_height, double env_resolution, std::vector<double>& origin, double lidar_global_x, double lidar_global_y, std::vector<Point>& lidar_points);
    void publishLidarScan(std::vector<Point>& points);
    void publishLidarPointCloud2(std::vector<Point>& points);

    double min_range;
    double max_range;
    double resolution;
    int points;

    std::vector<std::vector<int8_t>> occupancy_map_aux;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_plot_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_plot_publisher;
};



#endif // LIDAR_H