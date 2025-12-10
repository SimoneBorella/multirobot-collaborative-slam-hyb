#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <unordered_map>

class VisualizationPublisher : public rclcpp::Node {
public:
    VisualizationPublisher() : Node("visualization_publisher")
    {
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "multi_robot_simulator");

        while (!param_client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(get_logger(), "Waiting for parameter service...");
        }

        robots = param_client->get_parameter<std::vector<std::string>>("robots");

        for (const auto &robot : robots) {
            std::string topic_name = robot + "/map";
            std::string topic_name_plot = robot + "/map/plot";

            map_subscribers[robot] = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                topic_name, 10, 
                [this, robot](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { 
                    this->map_callback(msg, robot); 
                });

            map_publishers[robot] = this->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name_plot, 10);
        }
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, const std::string& robot) {
        auto modified_msg = *msg;
        modified_msg.header.frame_id = robot + "/map";
        map_publishers[robot]->publish(modified_msg);
    }   

    std::vector<std::string> robots;
    std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_publishers;
    std::unordered_map<std::string, rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_subscribers;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualizationPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
