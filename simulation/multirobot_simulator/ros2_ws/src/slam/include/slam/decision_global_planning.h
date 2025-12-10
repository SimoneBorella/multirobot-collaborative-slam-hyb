#ifndef DECISION_GLOBAL_PLANNING_H
#define DECISION_GLOBAL_PLANNING_H

// #define DECISION_MAKING_DEBUG
// #define GLOBAL_PLANNING_DEBUG

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/discrete/DiscreteValues.h>

#include <iostream>
#include <vector>
#include <cmath>

#include "mapping.h"

using namespace std::chrono_literals;

using namespace gtsam;



struct AStarNode {
    int x, y;
    float g, h;
    std::shared_ptr<AStarNode> parent;

    AStarNode(int x, int y, float g = 0, float h = 0, std::shared_ptr<AStarNode> parent = nullptr)
        : x(x), y(y), g(g), h(h), parent(parent) {}

    float f() const {
        return g + h;
    }
};

struct CompareAStarNode {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
        return a->f() > b->f();
    }
};



class DecisionGlobalPlanning
{
public:
    DecisionGlobalPlanning() = default;
    DecisionGlobalPlanning(rclcpp::Node::SharedPtr node, std::shared_ptr<Mapping> mapping);
    ~DecisionGlobalPlanning();
    void startDecisionGlobalPlanning();
    std::pair<std::shared_ptr<std::map<std::string, nav_msgs::msg::Path>>, std::shared_ptr<std::shared_mutex>> get_robot_paths_with_mutex() {return {robot_paths, robot_paths_mutex};}

private:
    std::map<std::string, std::pair<double, double>> decision_making_frontier_assignment(std::vector<std::pair<std::pair<double, double>, int>>& frontier_centroids, std::map<std::string, geometry_msgs::msg::TransformStamped>& robot_transforms);
    float heuristic(int x1, int y1, int x2, int y2);
    template <typename T> nav_msgs::msg::Path reconstructPath(std::shared_ptr<T> last);
    nav_msgs::msg::Path aStar(int sx, int sy, int gx, int gy);
    void decision_global_planning();

    std::vector<std::string> robots;

    std::map<std::string, std::pair<double, double>> initial_positions;

    std::string all_frame;
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    double decision_global_planning_rate;
    double alpha_distance;
    double beta_dimension;
    double conflict_penalty;
    std::string heuristic_type;
    std::string global_planner_type;

    std::vector<std::pair<int, int>> directions;
    std::vector<float> directions_cost;

    std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> robot_path_publishers;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    std::shared_ptr<std::vector<std::pair<std::pair<double, double>, int>>> frontiers;
    std::shared_ptr<std::shared_mutex> frontiers_mutex;

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_grid;
    std::shared_ptr<std::shared_mutex> costmap_grid_mutex;

    std::shared_ptr<std::map<std::string, nav_msgs::msg::Path>> robot_paths;
    std::shared_ptr<std::shared_mutex> robot_paths_mutex;

    std::unique_ptr<std::thread> decision_global_planning_thread;

    std::atomic<bool> decision_global_planning_running;

    std::shared_ptr<Mapping> mapping;

    rclcpp::Node::SharedPtr node;
};

#endif // DECISION_GLOBAL_PLANNING_H
