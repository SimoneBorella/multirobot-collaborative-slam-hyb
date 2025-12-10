#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_set>
#include <algorithm>

#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"

class TfPublisher : public rclcpp::Node {
public:
    TfPublisher() : Node("tf_publisher")
    {
        this->declare_parameter<std::string>("all_frame", "all");
        this->declare_parameter<bool>("verbose", false);

        all_frame = this->get_parameter("all_frame").as_string();
        verbose = this->get_parameter("verbose").as_bool();
        
        // Create subscriptions
        rigid_bodies_subscription = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
            "/rigid_bodies", 10, std::bind(&TfPublisher::rigid_bodies_callback, this, std::placeholders::_1)
        );

        // Initialize broadcasters using shared_from_this
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

private:
    void rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
    {
        for(mocap4r2_msgs::msg::RigidBody& rb : msg->rigidbodies)
        {
            if(rb.pose.position.x == 0.0 && rb.pose.position.y == 0.0 && rb.pose.position.z == 0.0)
                continue;

            size_t pos = rb.rigid_body_name.find('.');
            std::string rb_name;
            if (pos != std::string::npos) { // found
                rb_name = rb.rigid_body_name.substr(0, pos);
            } else {
                rb_name = rb.rigid_body_name;
            }

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = msg->header.stamp;
            tf_msg.header.frame_id = all_frame;
            tf_msg.child_frame_id = rb_name + "/ground_truth";

            tf_msg.transform.translation.x = rb.pose.position.x;
            tf_msg.transform.translation.y = rb.pose.position.y;
            tf_msg.transform.translation.z = rb.pose.position.z;

            tf_msg.transform.rotation = rb.pose.orientation;


            if (verbose)
            {
                double x = rb.pose.position.x;
                double y = rb.pose.position.y;
                double z = rb.pose.position.z;

                double qx = rb.pose.orientation.x;
                double qy = rb.pose.orientation.y;
                double qz = rb.pose.orientation.z;
                double qw = rb.pose.orientation.w;

                tf2::Quaternion q(qx, qy, qz, qw);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                // Print with std::cout
                std::cout << "RigidBody: " << rb_name
                        << " | Position [x, y, z]: (" << x << ", " << y << ", " << z << ")"
                        << " | Orientation [r, p, y]: (" << roll << ", " << pitch << ", " << yaw << ")"
                        << std::endl << std::endl;
            }


            tf_broadcaster->sendTransform(tf_msg);
        }
    }

    std::string all_frame;
    bool verbose;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr rigid_bodies_subscription;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
