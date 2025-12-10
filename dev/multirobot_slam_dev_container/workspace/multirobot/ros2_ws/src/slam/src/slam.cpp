#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "localization.h"
#include "mapping.h"
#include "decision_global_planning.h"
#include "local_planning.h"


using namespace std::chrono_literals;

class SLAM : public rclcpp::Node
{
public:
    SLAM()
        : Node("slam")
    {
        declare_parameter("robots", std::vector<std::string>());
        declare_parameter("initial_poses", std::vector<double>());

        declare_parameter("all_frame", "all");
        declare_parameter("map_frame", "map");
        declare_parameter("odom_frame", "odom");
        declare_parameter("base_frame", "base_link");

        // Localization parameters
        declare_parameter("localization_rate", 50.0);
        declare_parameter("fine_localization_rate", 0.2);
        declare_parameter("map_to_odom_rate", 100.0);
        declare_parameter("map_to_odom_correction", true);
        declare_parameter("isam_relinearize_threshold", 0.01);
        declare_parameter("isam_relinearize_skip", 1);
        declare_parameter("init_prior_noise", std::vector<double>());
        declare_parameter("marginal_prior_noise", std::vector<double>());
        declare_parameter("observation_noise", std::vector<double>());
        declare_parameter("odometry_noise", std::vector<double>());
        declare_parameter("window_size", 10);
        declare_parameter("lag", 10);
        declare_parameter("use_fixed_marginal", false);
        declare_parameter("min_pose_displacement", 0.05);
        declare_parameter("min_pose_theta_displacement", 10.0);
        declare_parameter("data_association_mode", std::string("nn"));
        declare_parameter("use_all_landmarks", false);
        declare_parameter("data_association_distance", 0.2);
        declare_parameter("mahalanobis_dist_threshold", 2.0);
        

        // Mapping parameters
        declare_parameter("mapping_rate", 1.0);
        declare_parameter("map_resolution", 0.05);
        declare_parameter("map_initial_width", 40.0);
        declare_parameter("map_initial_height", 40.0);
        declare_parameter("use_rays_over_range_max", false);
        declare_parameter("free_belief", 0.35);
        declare_parameter("occupied_belief", 0.75);
        declare_parameter("distance_belief_factor", 0.05);
        declare_parameter("occupied_threshold", 0.7);
        declare_parameter("free_threshold", 0.4);
        declare_parameter("override_active_hit_points", false);
        declare_parameter("noise_model_radius", 0.05);
        declare_parameter("noise_model_std_dev", 0.02);
        declare_parameter("robot_obstacle_radius", 0.3);
        declare_parameter("frontier_del_obstacles_radius", 0.2);
        declare_parameter("frontier_cells_detection_mode", std::string("ewfd"));
        declare_parameter("costmap_initial_value", 10);
        declare_parameter("kernel_distance", 0.8);
        declare_parameter("cost_decay_rate", 8.0);


        // Decision making and global planning parameters
        declare_parameter("decision_global_planning_rate", 0.5);
        declare_parameter("epsilon", 0.2);
        declare_parameter("min_points", 2);
        declare_parameter("min_frontier_size", 0.1);
        declare_parameter("alpha_distance", 1.0);
        declare_parameter("beta_dimension", 0.1);
        declare_parameter("conflict_penalty", 0.01);
        declare_parameter("global_planner_type", std::string("a_star"));
        declare_parameter("heuristic_type", std::string("euclidean"));

        // Local planning parameters
        declare_parameter("local_planning_rate", 15.0);
        declare_parameter("local_planner_type", std::string("dwa"));
        declare_parameter("keep_internal_state", false);
        declare_parameter("stop_dist_threshold", 0.2);
        declare_parameter("dt", 0.1);
        declare_parameter("predict_time", 2.0);
        declare_parameter("max_vel_x", 0.22);
        declare_parameter("min_vel_x", -0.1);
        declare_parameter("max_vel_theta", 2.80);


    
        declare_parameter("max_acc_x", 0.5);
        declare_parameter("max_acc_theta", 2.5);
        declare_parameter("lookahead_dist", 2.0);
        declare_parameter("w_progress_score", 1.0);
        declare_parameter("w_heading_score", 1.0);
        declare_parameter("w_robot_collision_score", 1.0);
        declare_parameter("w_costmap_score", 1.0);
        
        
        
        declare_parameter("start_prior_noise", std::vector<double>());
        declare_parameter("goal_prior_noise", std::vector<double>());
        declare_parameter("dynamic_noise", 0.1);
        declare_parameter("inter_robot_noise", 0.01);
        declare_parameter("obstacle_noise", 0.01);
        declare_parameter("robot_dist_threshold", 0.5);
        declare_parameter("obstacle_dist_threshold", 0.2);
    }

    void setup()
    {
        localization = std::make_shared<Localization>(shared_from_this());
        mapping = std::make_shared<Mapping>(shared_from_this(), localization);
        decision_global_planning = std::make_shared<DecisionGlobalPlanning>(shared_from_this(), mapping);
        local_planning = std::make_shared<LocalPlanning>(shared_from_this(), mapping, decision_global_planning);

        localization->startLocalization();
        mapping->startMapping();
        decision_global_planning->startDecisionGlobalPlanning();
        local_planning->startLocalPlanning();
    }

    std::shared_ptr<Localization> localization;
    std::shared_ptr<Mapping> mapping;
    std::shared_ptr<DecisionGlobalPlanning> decision_global_planning;
    std::shared_ptr<LocalPlanning> local_planning;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SLAM>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}