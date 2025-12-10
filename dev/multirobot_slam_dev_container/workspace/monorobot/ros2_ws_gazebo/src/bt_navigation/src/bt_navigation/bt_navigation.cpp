#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "go_to_pose.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

class BTNavigation : public rclcpp::Node
{
public:
	BTNavigation() : Node("bt_navigation")
	{
		
	}

	void setup()
	{
		RCLCPP_INFO(get_logger(), "BT setup started");

		BT::BehaviorTreeFactory factory;

		BT::NodeBuilder builder = [=](const std::string &name, const BT::NodeConfiguration &config)
		{
			return std::make_unique<GoToPose>(name, config, shared_from_this());
		};
		factory.registerBuilder<GoToPose>("GoToPose", builder);

		const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("bt_navigation") + "/bt_xml";
		tree_ = factory.createTreeFromFile(bt_xml_dir + "/tree.xml");

		RCLCPP_INFO(get_logger(), "BT setup finished");

		timer_ = this->create_wall_timer(500ms, std::bind(&BTNavigation::update_behavior_tree, this));

		rclcpp::spin(shared_from_this());
		rclcpp::shutdown();
	}

	void update_behavior_tree()
	{
		BT::NodeStatus tree_status = tree_.tickRoot();

		if (tree_status == BT::NodeStatus::RUNNING)
		{
			return;
		}
		else if (tree_status == BT::NodeStatus::SUCCESS)
		{
			RCLCPP_INFO(this->get_logger(), "Navigation Succeeded");
			timer_->cancel();
			rclcpp::shutdown();
		}
		else if (tree_status == BT::NodeStatus::FAILURE)
		{
			RCLCPP_INFO(this->get_logger(), "Navigation Failed");
			timer_->cancel();
			rclcpp::shutdown();
		}
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	BT::Tree tree_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<BTNavigation>();
  	node->setup();
	return 0;
}