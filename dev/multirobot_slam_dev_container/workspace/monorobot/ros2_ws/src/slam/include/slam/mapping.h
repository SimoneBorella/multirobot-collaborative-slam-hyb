#ifndef MAPPING_H
#define MAPPING_H

// #define MAPPING_DEBUG

#include <cmath>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "localization.h"

using namespace std::chrono_literals;

class Mapping
{
public:
    Mapping() = default;
    Mapping(rclcpp::Node::SharedPtr node, std::shared_ptr<Localization> localization);
    ~Mapping();
    void startMapping();
    std::pair<std::shared_ptr<nav_msgs::msg::OccupancyGrid>, std::shared_ptr<std::shared_mutex>> get_frontier_grid_with_mutex() {return {frontier_grid, frontier_grid_mutex};}


private:
    float probability_to_log_odds(uint8_t prob);
    uint8_t log_odds_to_probability(float log_odds);
    void bresenham_raytrace(int x0, int y0, int x1, int y1, std::vector<int8_t>& map_grid_data, std::vector<float>& map_data, bool hit_point);
    void expanding_wavefront_frontier_cells_detection(int x0, int y0, double active_area_radius);
    void mapping();

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

    bool apply_threshold;
    double occupied_threshold;
    double free_threshold;

    bool override_active_hit_points;

    double noise_model_radius;
    double noise_model_std_dev;

    std::string frontier_cells_detection_mode;

    std::vector<float> map;
    nav_msgs::msg::OccupancyGrid map_grid;

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> frontier_grid;
    std::shared_ptr<std::shared_mutex> frontier_grid_mutex;


    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr frontier_publisher;

    std::unique_ptr<std::thread> mapping_thread;

    std::atomic<bool> mapping_running;

    std::shared_ptr<std::vector<PosedScan>> posed_scans;
    std::shared_ptr<std::shared_mutex> posed_scans_mutex;

    std::shared_ptr<Localization> localization;

    rclcpp::Node::SharedPtr node;
};

#endif // MAPPING_H
