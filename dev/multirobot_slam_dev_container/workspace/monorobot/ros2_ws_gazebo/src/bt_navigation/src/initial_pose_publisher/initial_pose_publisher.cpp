
#include <chrono>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node
{
public:
	InitialPosePublisher()
		: Node("initial_pose_publisher")
	{
		// Declare parameters
		declare_parameter("x_init", 0.0);
		declare_parameter("y_init", 0.0);
		declare_parameter("z_init", 0.0);
		declare_parameter("R_init", 0.0);
		declare_parameter("P_init", 0.0);
		declare_parameter("Y_init", 0.0);

		declare_parameter("pose_cov_init", std::vector<double>());

		// Get parameters
		x_init_ = get_parameter("x_init").as_double();
		y_init_ = get_parameter("y_init").as_double();
		z_init_ = get_parameter("z_init").as_double();
		R_init_ = get_parameter("R_init").as_double();
		P_init_ = get_parameter("P_init").as_double();
		Y_init_ = get_parameter("Y_init").as_double();

		std::vector<double> pose_cov_init_v = get_parameter("pose_cov_init").as_double_array();

		std::copy(pose_cov_init_v.begin(), pose_cov_init_v.end(), pose_cov_init_.begin());

		// Publishers
		initialpose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

		while(initialpose_publisher_->get_subscription_count() == 0)
		{
			RCLCPP_INFO(this->get_logger(), "Waiting for subscriber to connect");
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}

		RCLCPP_INFO(this->get_logger(), "Subscriber connected");
		send_init_pose();
	}

private:

	void send_init_pose() {

		geometry_msgs::msg::PoseWithCovarianceStamped msg;
		msg.header.frame_id = "map";
		msg.pose.pose.position.x = x_init_;
		msg.pose.pose.position.y = y_init_;
		msg.pose.pose.position.z = z_init_;
		
		tf2::Quaternion quat;
		quat.setRPY(R_init_, P_init_, Y_init_);
		msg.pose.pose.orientation.w = quat.getW();
		msg.pose.pose.orientation.x = quat.getX();
		msg.pose.pose.orientation.y = quat.getY();
		msg.pose.pose.orientation.z = quat.getZ();

		msg.pose.covariance = pose_cov_init_;

		initialpose_publisher_->publish(msg);
	}

	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_publisher_;
	
	double x_init_;
	double y_init_;
	double z_init_;
	double R_init_;
	double P_init_;
	double Y_init_;

	std::array<double, 36> pose_cov_init_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<InitialPosePublisher>();
	// rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}