#ifndef MAPPING_H
#define MAPPING_H

// #define MAPPING_DEBUG
// #define FRONTIER_DETECTION_DEBUG

#include <cmath>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "localization.h"

using namespace std::chrono_literals;

class Mapping
{
public:
    Mapping() = default;
    Mapping(rclcpp::Node::SharedPtr node, std::shared_ptr<Localization> localization);
    ~Mapping();
    void startMapping();
    std::pair<std::shared_ptr<std::vector<std::pair<std::pair<double, double>, int>>>, std::shared_ptr<std::shared_mutex>> get_frontiers_with_mutex() {return {frontiers, frontiers_mutex};}
    std::pair<std::shared_ptr<nav_msgs::msg::OccupancyGrid>, std::shared_ptr<std::shared_mutex>> get_costmap_grid_with_mutex() {return {costmap_grid, costmap_grid_mutex};}


private:
    float probability_to_log_odds(uint8_t prob);
    uint8_t log_odds_to_probability(float log_odds);
    void bresenham_raytrace(int x0, int y0, int x1, int y1, std::vector<int8_t>& map_grid_data, std::vector<float>& map_data, bool hit_point);
    void expanding_wavefront_frontier_cells_detection(std::string& robot, int x0, int y0, double active_area_radius);
    std::vector<std::pair<int, int>> get_neighbors(int x, int y);
    std::map<int, std::vector<std::pair<int, int>>> dbscan_frontier_clusters_detection();
    std::vector<std::pair<std::pair<double, double>, int>> frontier_centroids_detection(const std::map<int, std::vector<std::pair<int, int>>>& frontier_clusters);
    void publish_frontiers_marker(std::vector<std::pair<std::pair<double, double>, int>>& frontier_centroids);
    void mapping();

    std::vector<std::string> robots;

    std::string all_frame;
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    double mapping_rate;

    bool update_only_last_scan;

    double map_resolution;
    double map_width;
    double map_height;
    int width_cells;
    int height_cells;

    bool use_rays_over_range_max;

    double free_belief;
    double occupied_belief;
    double free_belief_log_odds;
    double occupied_belief_log_odds;
    double distance_belief_factor;

    double occupied_threshold;
    double free_threshold;

    bool override_active_hit_points;

    double noise_model_radius;
    double noise_model_std_dev;

    double robot_obstacle_radius;

    double frontier_del_obstacles_radius;
    std::string frontier_cells_detection_mode;
    
    double kernel_distance;
    int kernel_size;
    double cost_decay_rate;

    double epsilon;
    int min_points;
    double min_frontier_size;


    std::map<std::string, bool> ewfd_first;
    std::map<std::string, std::vector<bool>> ewfd_visited;
    std::map<std::string, std::vector<int8_t>> frontier_grid_data;

    std::vector<float> map;
    nav_msgs::msg::OccupancyGrid map_grid;

    std::map<std::string, std::vector<float>> robot_map;
    std::map<std::string, nav_msgs::msg::OccupancyGrid> robot_map_grid;

    nav_msgs::msg::OccupancyGrid frontier_grid;

    std::shared_ptr<std::vector<std::pair<std::pair<double, double>, int>>> frontiers;
    std::shared_ptr<std::shared_mutex> frontiers_mutex;

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_grid;
    std::shared_ptr<std::shared_mutex> costmap_grid_mutex;


    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
    std::map<std::string, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> robot_map_publishers;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr frontier_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontiers_marker_publisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher;

    std::unique_ptr<std::thread> mapping_thread;

    std::atomic<bool> mapping_running;

    std::shared_ptr<std::map<std::string, std::vector<PosedScan>>> posed_scans;
    std::shared_ptr<std::shared_mutex> posed_scans_mutex;

    std::shared_ptr<Localization> localization;

    rclcpp::Node::SharedPtr node;
};

#endif // MAPPING_H
