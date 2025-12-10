#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_set>
#include <algorithm>

class TfPublisher : public rclcpp::Node {
public:
    TfPublisher() : Node("tf_publisher")
    {

        this->declare_parameter<std::string>("robot_name", "");
        robot_name = this->get_parameter("robot_name").as_string();
        
        // Create subscriptions
        std::string tf_subscription_topic = robot_name + "/tf";
        tf_subscription = this->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_subscription_topic, rclcpp::SensorDataQoS(), std::bind(&TfPublisher::tf_callback, this, std::placeholders::_1)
        );

        // Initialize broadcasters using shared_from_this
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto &transform : msg->transforms) {
            std::string frame_id = transform.header.frame_id;
            std::string child_frame_id = transform.child_frame_id;

            if(frame_id == "map" && child_frame_id == "odom") {
                geometry_msgs::msg::TransformStamped new_transform = transform;
                new_transform.header.frame_id = robot_name + "/map";
                new_transform.child_frame_id = robot_name + "/odom";
                tf_broadcaster->sendTransform(new_transform);
            }
        }
    }

    std::string robot_name;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
