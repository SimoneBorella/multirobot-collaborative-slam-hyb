#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/map_log_odds_update.hpp"
#include "interfaces/msg/point_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "interfaces/msg/frontier.hpp"
#include "interfaces/msg/frontier_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mapping_merge.h"
#include "task_planning.h"
#include "global_planning.h"

using namespace multirobot_slam;

using namespace std::chrono_literals;

class MultirobotServer : public rclcpp::Node
{
public:
    MultirobotServer()
        : Node("multirobot_server")
    {
        declare_parameter("world_frame", "world");
        declare_parameter("map_frame", "map");
        declare_parameter("odom_frame", "odom");
        declare_parameter("base_frame", "base_link");
        
        declare_parameter("robots", std::vector<std::string>());
        declare_parameter("initial_poses", std::vector<double>());

        world_frame_ = get_parameter("world_frame").as_string();
        map_frame_ = get_parameter("map_frame").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();

        robots_ = this->get_parameter("robots").as_string_array();
        initial_poses_ = this->get_parameter("initial_poses").as_double_array();

        double timer_rate = 50.0;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / timer_rate)),
            std::bind(&MultirobotServer::timer_callback, this));

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        rclcpp::QoS map_qos_profile(10);
        map_qos_profile.reliable();
        map_qos_profile.transient_local();

        for(const auto &robot : robots_)
        {
            std::string robot_map_log_odds_update_topic = "/" + robot + "/map_log_odds_update";
    
            robot_map_log_odds_update_subscriptions_[robot] = this->create_subscription<interfaces::msg::MapLogOddsUpdate>(
                robot_map_log_odds_update_topic, 
                map_qos_profile, 
                [this, robot](interfaces::msg::MapLogOddsUpdate::SharedPtr msg) {
                    map_log_odds_update_callback(msg, robot);
                }
            );

            std::string robot_frontiers_topic = "/" + robot + "/frontiers";
    
            robot_frontiers_subscriptions_[robot] = this->create_subscription<interfaces::msg::FrontierArray>(
                robot_frontiers_topic,
                10, 
                [this, robot](interfaces::msg::FrontierArray::SharedPtr msg) {
                    frontiers_callback(msg, robot);
                }
            );

            std::string robot_global_path_topic = "/" + robot + "/global_path";
            robot_global_path_publishers_[robot] = this->create_publisher<nav_msgs::msg::Path>(robot_global_path_topic, 10);
            
            std::string robot_cmd_topic = "/" + robot + "/cmd_vel";
            robot_cmd_publishers_[robot] = this->create_publisher<geometry_msgs::msg::Twist>(robot_cmd_topic, 10);
        }

        std::string map_topic = "/map";
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos_profile);

        std::string costmap_topic = "/costmap";
        costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic, map_qos_profile);

        std::string frontiers_marker_topic = "/frontiers_marker";
        frontiers_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(frontiers_marker_topic, 10);


        rclcpp::on_shutdown([this]() {
            publishZeroCmd();
        });
    }

    
    void publish_world_to_map(std::string robot, Eigen::Vector3d initial_pose)
    {
        geometry_msgs::msg::TransformStamped world_to_map;
        world_to_map.header.stamp = this->now();
        world_to_map.header.frame_id = "world";
        world_to_map.child_frame_id = robot + "/map";
    
        world_to_map.transform.translation.x = initial_pose.x();
        world_to_map.transform.translation.y = initial_pose.y();
        world_to_map.transform.translation.z = 0.0;
    
        tf2::Quaternion q;
        q.setRPY(0, 0, initial_pose.z());
        world_to_map.transform.rotation.x = q.x();
        world_to_map.transform.rotation.y = q.y();
        world_to_map.transform.rotation.z = q.z();
        world_to_map.transform.rotation.w = q.w();

        static_tf_broadcaster_->sendTransform(world_to_map);
    }

    void publishZeroCmd()
    {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.linear.y = 0.0;
        stop_msg.linear.z = 0.0;
        stop_msg.angular.x = 0.0;
        stop_msg.angular.y = 0.0;
        stop_msg.angular.z = 0.0;

        for(int i=0; i<10; i++)
        {
            for (auto &[robot, cmd_publisher] : robot_cmd_publishers_)
            {
                cmd_publisher->publish(stop_msg);
            }
        }
    }


    void map_log_odds_update_callback(const interfaces::msg::MapLogOddsUpdate::SharedPtr msg, const std::string &robot)
    {
        MapLogOddsUpdate map_log_odds_update;
        map_log_odds_update.resolution = msg->resolution;
        map_log_odds_update.width = msg->width;
        map_log_odds_update.height = msg->height;

        map_log_odds_update.origin_position =
            Eigen::Vector3d(
                msg->origin.position.x,
                msg->origin.position.y,
                msg->origin.position.z
            );
        map_log_odds_update.origin_orientation =
            Eigen::Quaterniond(
                msg->origin.orientation.w,
                msg->origin.orientation.x,
                msg->origin.orientation.y,
                msg->origin.orientation.z
            );

        map_log_odds_update.indicies = msg->indicies;
        map_log_odds_update.delta_log_odds = msg->delta_log_odds;

        mapping_merge_.add_map_log_odds_update(map_log_odds_update, robot);
    }


    void frontiers_callback(const interfaces::msg::FrontierArray::SharedPtr msg, const std::string &robot)
    {
        std::vector<Frontier> frontiers;

        for(auto& f : msg->frontiers)
        {
            Frontier frontier;
            frontier.centroid = Eigen::Vector2d(f.centroid.x, f.centroid.y);
            frontier.size = f.size;

            frontiers.push_back(frontier);
        }

        mapping_merge_.add_frontiers(frontiers, robot);
    }



    void setup()
    {
        std::map<std::string, Pose> initial_poses;

        for (size_t i = 0; i < robots_.size(); ++i)
        {
            double x = initial_poses_[3 * i];
            double y = initial_poses_[3 * i + 1];
            double yaw = initial_poses_[3 * i + 2]; 

            Eigen::Vector3d initial_pose(x, y, yaw);
            
            publish_world_to_map(robots_[i], initial_pose);

            initial_poses[robots_[i]].position = 
                Eigen::Vector3d(
                    initial_poses_[3 * i],
                    initial_poses_[3 * i + 1],
                    0.0
                );
            
            initial_poses[robots_[i]].orientation =
                Eigen::Quaterniond(
                    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                );
        }

        std::string package_dir = ament_index_cpp::get_package_share_directory("multirobot_server");

        std::string mapping_merge_params_path = package_dir + "/params/mapping_merge_params.yaml";
        MappingMergeParams mapping_merge_params = MappingMerge::params_from_yaml(mapping_merge_params_path);

        std::string task_planning_params_path = package_dir + "/params/task_planning_params.yaml";
        TaskPlanningParams task_planning_params = TaskPlanning::params_from_yaml(task_planning_params_path);

        std::string global_planning_params_path = package_dir + "/params/global_planning_params.yaml";
        GlobalPlanningParams global_planning_params = GlobalPlanning::params_from_yaml(global_planning_params_path);

        mapping_merge_.init(mapping_merge_params);
        mapping_merge_.set_initial_poses(initial_poses);
        std::cout << "[Server]: " << "Mapping merge initialized." << std::endl;

        task_planning_.init(task_planning_params);
        std::cout << "[Server]: " << "Task planning initialized." << std::endl;

        global_planning_.init(global_planning_params);
        std::cout << "[Server]: " << "Global planning initialized." << std::endl;

        mapping_merge_.start();
        std::cout << "[Server]: " << "Mapping merge started." << std::endl;
    }


    bool get_robot_pose(const std::string& robot, Pose& pose_out)
    {
        std::string base_frame = robot + "/" + base_frame_;

        try
        {
            geometry_msgs::msg::TransformStamped world_to_base =
                tf_buffer_->lookupTransform(
                    world_frame_,
                    base_frame,
                    tf2::TimePointZero
                );

            pose_out.position = Eigen::Vector3d(
                world_to_base.transform.translation.x,
                world_to_base.transform.translation.y,
                world_to_base.transform.translation.z
            );

            pose_out.orientation = Eigen::Quaterniond(
                world_to_base.transform.rotation.w,
                world_to_base.transform.rotation.x,
                world_to_base.transform.rotation.y,
                world_to_base.transform.rotation.z
            );

            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            return false;
        }
    }


    void publish_map(const Map& map)
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = world_frame_;
        map_msg.info.resolution = map.resolution;
        map_msg.info.width = map.width;
        map_msg.info.height = map.height;
        map_msg.info.origin.position.x = map.origin_position.x();
        map_msg.info.origin.position.y = map.origin_position.y();
        map_msg.info.origin.position.z = map.origin_position.z();
        map_msg.info.origin.orientation.w = map.origin_orientation.w();
        map_msg.info.origin.orientation.x = map.origin_orientation.x();
        map_msg.info.origin.orientation.y = map.origin_orientation.y();
        map_msg.info.origin.orientation.z = map.origin_orientation.z();
        map_msg.data = map.data;
        map_publisher_->publish(map_msg);
    }


    void publish_costmap(const Map& costmap)
    {
        nav_msgs::msg::OccupancyGrid costmap_msg;
        costmap_msg.header.stamp = this->now();
        costmap_msg.header.frame_id = world_frame_;
        costmap_msg.info.resolution = costmap.resolution;
        costmap_msg.info.width = costmap.width;
        costmap_msg.info.height = costmap.height;
        costmap_msg.info.origin.position.x = costmap.origin_position.x();
        costmap_msg.info.origin.position.y = costmap.origin_position.y();
        costmap_msg.info.origin.position.z = costmap.origin_position.z();
        costmap_msg.info.origin.orientation.w = costmap.origin_orientation.w();
        costmap_msg.info.origin.orientation.x = costmap.origin_orientation.x();
        costmap_msg.info.origin.orientation.y = costmap.origin_orientation.y();
        costmap_msg.info.origin.orientation.z = costmap.origin_orientation.z();
        costmap_msg.data = costmap.data;
        costmap_publisher_->publish(costmap_msg);
    }


    void publish_frontiers_marker(const std::vector<Frontier>& frontiers)
    {
        visualization_msgs::msg::Marker frontiers_marker = visualization_msgs::msg::Marker();
        frontiers_marker.header.stamp = this->now();
        frontiers_marker.header.frame_id = world_frame_;
    
        frontiers_marker.ns = "frontiers";
        frontiers_marker.id = 0;
        frontiers_marker.type = visualization_msgs::msg::Marker::POINTS;
        frontiers_marker.action = visualization_msgs::msg::Marker::ADD;

        frontiers_marker.color.r = 0.1f;
        frontiers_marker.color.g = 0.4f;
        frontiers_marker.color.b = 0.1f;
        frontiers_marker.color.a = 1.0f;

        frontiers_marker.scale.x = 0.2;
        frontiers_marker.scale.y = 0.2;

        for (const auto& frontier : frontiers) {
            geometry_msgs::msg::Point p;
            p.x = frontier.centroid.x();
            p.y = frontier.centroid.y();
            p.z = 0.0;
            frontiers_marker.points.push_back(p);
        }

        frontiers_marker_publisher_->publish(frontiers_marker);
    }



    void publish_global_paths(std::map<std::string, Path> global_paths)
    {
        for (const auto& [robot, path] : global_paths)
        {
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = world_frame_;

            for (const auto& pose : path.poses)
            {
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header = path_msg.header;

                pose_stamped.pose.position.x = pose.position.x();
                pose_stamped.pose.position.y = pose.position.y();
                pose_stamped.pose.position.z = pose.position.z();

                pose_stamped.pose.orientation.w = pose.orientation.w();
                pose_stamped.pose.orientation.x = pose.orientation.x();
                pose_stamped.pose.orientation.y = pose.orientation.y();
                pose_stamped.pose.orientation.z = pose.orientation.z();

                path_msg.poses.push_back(pose_stamped);
            }

            robot_global_path_publishers_[robot]->publish(path_msg);
        }
    }



    void timer_callback()
    {
        const std::optional<Map> &map = mapping_merge_.get_map_if_updated();

        if (map.has_value())
        {
            publish_map(map.value());
        }

        const std::optional<Map> &costmap = mapping_merge_.get_costmap_if_updated();

        if (costmap.has_value())
        {
            publish_costmap(costmap.value());

            global_planning_.update_costmap(costmap.value());
        }

        const std::optional<std::vector<Frontier>> &frontiers = mapping_merge_.get_frontiers_if_updated();

        if (frontiers.has_value())
        {
            publish_frontiers_marker(frontiers.value());

            std::map<std::string, Pose> robot_poses;

            for (const auto& robot : robots_)
            {
                Pose pose;
                if (get_robot_pose(robot, pose))
                {
                    robot_poses[robot] = pose;
                }
            }

            std::map<std::string, Frontier> tasks = task_planning_.plan_tasks(robot_poses, frontiers.value());

            std::map<std::string, Path> global_paths = global_planning_.plan_global_path(robot_poses, tasks);

            publish_global_paths(global_paths);

            // Update global path to local planning
        }

        // Get and publish veocity commands for each robot
        
    }

    std::string world_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    std::vector<std::string> robots_;
    std::vector<double> initial_poses_;

    MappingMerge mapping_merge_;
    TaskPlanning task_planning_;
    GlobalPlanning global_planning_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    std::map<std::string, rclcpp::Subscription<interfaces::msg::MapLogOddsUpdate>::SharedPtr> robot_map_log_odds_update_subscriptions_;
    std::map<std::string, rclcpp::Subscription<interfaces::msg::FrontierArray>::SharedPtr> robot_frontiers_subscriptions_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontiers_marker_publisher_;
    std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> robot_global_path_publishers_;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> robot_cmd_publishers_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultirobotServer>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}