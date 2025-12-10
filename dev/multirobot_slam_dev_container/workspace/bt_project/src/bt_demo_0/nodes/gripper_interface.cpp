#include "gripper_interface.h"

GripperInterface::GripperInterface() : _gripper_open(false)
{
}

BT::NodeStatus GripperInterface::open()
{
    _gripper_open = true;
    std::cout << "Gripper opened" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::close()
{
    _gripper_open = false;
    std::cout << "Gripper closed" << std::endl;
    return BT::NodeStatus::SUCCESS;
}
