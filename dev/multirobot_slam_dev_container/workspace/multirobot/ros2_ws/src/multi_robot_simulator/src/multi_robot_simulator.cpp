#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>


#include "environment.h"
#include "robot.h"

using namespace std::chrono_literals;

class MultiRobotSimulator : public rclcpp::Node
{
public:
    MultiRobotSimulator()
        : Node("multi_robot_simulator")
    {
		declare_parameter("rate", 100.0);
		declare_parameter("map", "indoor_map_0");
		declare_parameter("robots", std::vector<std::string>());
		declare_parameter("landmarks_density", 2.5);
    }

    void setup()
    {
        double rate = get_parameter("rate").as_double();
        std::string map_filename = get_parameter("map").as_string();
        std::vector<std::string> robots_filenames = get_parameter("robots").as_string_array();  
        double landmarks_density = get_parameter("landmarks_density").as_double();

        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();

        // Environment initialization
        environment = std::make_shared<Environment>(
            shared_from_this(),
            ament_index_cpp::get_package_share_directory("multi_robot_simulator") + "/maps/" + map_filename + ".yaml",
            ament_index_cpp::get_package_share_directory("multi_robot_simulator") + "/maps/" + map_filename + ".pgm",
            landmarks_density
        );
        
        initializeRobots(robots_filenames);
        spawnRobots();

        // Timer initialization
        timer = this->create_wall_timer(std::chrono::duration<double>(0.5), std::bind(&MultiRobotSimulator::timer_callback, this));
        
        auto robots = Robot::getAllRobots();
        for (const auto& robot : robots)
            robot->startUpdateLoop(rate);
        
    }

    
private:
    void initializeRobots(std::vector<std::string>& robots_filenames)
    {
        for (const std::string &robot_filename : robots_filenames)
        {
            auto robot = std::make_shared<Robot>(
                shared_from_this(),
                environment,
                ament_index_cpp::get_package_share_directory("multi_robot_simulator") + "/robots/" + robot_filename + ".yaml"
            );

            Robot::registerRobot(robot);
        }
    }

    void spawnRobots()
    {
        auto robots = Robot::getAllRobots();
        for (const auto& robot : robots)
            robot->spawnIfNotInitialized();
    }

    void timer_callback()
    {
        environment->publishMapOccupancyGridIfNewSubscriber();
        environment->publishLandmarksMarkerIfNewSubscriber();
    }

    std::shared_ptr<Environment> environment;

    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiRobotSimulator>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}