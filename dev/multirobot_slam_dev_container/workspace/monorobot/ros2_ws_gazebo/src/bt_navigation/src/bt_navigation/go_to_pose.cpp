#include "go_to_pose.h"

GoToPose::GoToPose(const std::string &name,
				   const BT::NodeConfiguration &config,
				   rclcpp::Node::SharedPtr node_ptr)
	: BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
	action_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_ptr_, "/navigate_to_pose");
	done_flag_ = false;
}

BT::PortsList GoToPose::providedPorts()
{
	return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart()
{
	// Get location key from port and read YAML file
	BT::Optional<std::string> loc = getInput<std::string>("loc");
	// const std::string location_file = node_ptr_->get_parameter("location_file").as_string();
	const std::string location_file = ament_index_cpp::get_package_share_directory("bt_navigation") + "/config" + "/locations.yaml";
	
	YAML::Node locations = YAML::LoadFile(location_file);

	std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

	// setup action client
	auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
	send_goal_options.result_callback = std::bind(&GoToPose::result_callback, this, std::placeholders::_1);

	// make pose
	auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
	goal_msg.pose.header.frame_id = "map";
	goal_msg.pose.pose.position.x = pose[0];
	goal_msg.pose.pose.position.y = pose[1];

	tf2::Quaternion q;
	q.setRPY(0, 0, pose[2]);
	q.normalize();
	goal_msg.pose.pose.orientation = tf2::toMsg(q);

	// send pose
	done_flag_ = false;
	action_client_ptr_->async_send_goal(goal_msg, send_goal_options);

	RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "[ " << this->name() << " ] Requested location " << "(" << pose[0] << ", " << pose[1] << ", " << pose[2] << ")");
	
	return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning()
{
	if (done_flag_)
	{
		RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "[ " << this->name() << " ] Location reached");
		return BT::NodeStatus::SUCCESS;
	}
	else
	{
		return BT::NodeStatus::RUNNING;
	}
}

void GoToPose::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
	// If there is a result, we consider navigation completed.
	// bt_navigator only sends an empty message without status. Idk why though.

	if (result.result)
	{
		done_flag_ = true;
	}
}