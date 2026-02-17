#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_set>
#include <algorithm>

class TfPublisher : public rclcpp::Node
{
public:
    TfPublisher() : Node("tf_publisher")
    {
        declare_parameter("client_namespace", std::string());

        ns_ = get_parameter("client_namespace").as_string();

        if (!ns_.empty() && ns_[0] == '/')
            ns_ = ns_.substr(1);

        // Create subscriptions
        tf_subscription = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/" + ns_ + "/tf", rclcpp::SensorDataQoS(), [this](tf2_msgs::msg::TFMessage::SharedPtr msg)
            { tf_callback(msg, false); });

        rclcpp::QoS static_qos = rclcpp::QoS(rclcpp::KeepAll()).transient_local();
        tf_static_subscription = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/" + ns_ + "/tf_static", static_qos, [this](tf2_msgs::msg::TFMessage::SharedPtr msg)
            { tf_callback(msg, true); });

        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

private:
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static)
    {
        for (const auto &transform : msg->transforms)
        {
            std::string frame_id = transform.header.frame_id;
            std::string child_frame_id = transform.child_frame_id;

            if (!is_static && (frame_id != "map"))
                continue;

            geometry_msgs::msg::TransformStamped new_transform = transform;
            new_transform.header.frame_id = ns_ + "/" + frame_id;
            new_transform.child_frame_id = ns_ + "/" + child_frame_id;

            if (is_static)
            {
                tf_static_broadcaster->sendTransform(new_transform);
            }
            else
            {
                tf_broadcaster->sendTransform(new_transform);
            }
        }
    }

    std::string ns_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_subscription;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
