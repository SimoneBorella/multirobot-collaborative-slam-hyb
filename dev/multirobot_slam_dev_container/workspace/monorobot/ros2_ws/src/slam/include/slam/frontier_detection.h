#ifndef FRONTIER_DETECTION_H
#define FRONTIER_DETECTION_H

// #define FRONTIER_DETECTION_DEBUG

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mapping.h"

using namespace std::chrono_literals;

class FrontierDetection
{
public:
    FrontierDetection() = default;
    FrontierDetection(rclcpp::Node::SharedPtr node, std::shared_ptr<Mapping> mapping);
    ~FrontierDetection();
    void startFrontierDetection();

private:
    std::vector<std::pair<int, int>> get_neighbors(int x, int y);
    std::map<int, std::vector<std::pair<int, int>>> dbscan();
    std::vector<std::pair<double, double>> compute_frontier_centroids(const std::map<int, std::vector<std::pair<int, int>>>& frontier_clusters);
    void publish_frontiers_marker(std::vector<std::pair<double, double>>& frontier_centroids);
    void frontier_detection();

    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    double frontier_detection_rate;
    double epsilon;
    int min_points;
    int min_frontier_size;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontiers_marker_publisher;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client;

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> frontier_grid;
    std::shared_ptr<std::shared_mutex> frontier_grid_mutex;

    std::unique_ptr<std::thread> frontier_detection_thread;

    std::atomic<bool> frontier_detection_running;

    std::shared_ptr<Mapping> mapping;

    rclcpp::Node::SharedPtr node;
};

#endif // FRONTIER_DETECTION_H
