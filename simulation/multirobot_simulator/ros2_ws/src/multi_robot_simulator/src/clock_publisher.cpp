#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <rosgraph_msgs/msg/clock.hpp>


using namespace std::chrono_literals;

class ClockPublihser : public rclcpp::Node
{
public:
    ClockPublihser()
        : Node("clock_publisher")
    {
		declare_parameter("clock_speed_multiplier", 1.0);   
		declare_parameter("update_period", 1.0);   

        clock_speed_multiplier = get_parameter("clock_speed_multiplier").as_double();
        update_period = get_parameter("update_period").as_double();

        // Publisher initialization
        clock_publisher = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        // Timer initialization
        clock_timer = this->create_wall_timer(std::chrono::duration<double>(update_period), std::bind(&ClockPublihser::clock_timer_callback, this));
    }

private:
    void clock_timer_callback()
    {
        auto msg = rosgraph_msgs::msg::Clock();

        sim_time += clock_speed_multiplier * update_period;

        int64_t seconds = static_cast<int64_t>(sim_time);
        int64_t nanoseconds = static_cast<int64_t>((sim_time - seconds) * 1e9);

        msg.clock.sec = seconds;
        msg.clock.nanosec = nanoseconds;

        // RCLCPP_INFO_STREAM(this->get_logger(), "Publishing time: " << seconds << "." << nanoseconds);

        clock_publisher->publish(msg);
    }


    double update_period;
    double sim_time;
    double clock_speed_multiplier;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher;
    rclcpp::TimerBase::SharedPtr clock_timer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClockPublihser>());
    rclcpp::shutdown();
    return 0;
}