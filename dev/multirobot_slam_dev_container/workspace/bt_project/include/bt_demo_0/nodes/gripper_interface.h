#ifndef GRIPPER_INTERFACE_H
#define GRIPPER_INTERFACE_H

#include <behaviortree_cpp_v3/bt_factory.h>
#include <iostream>

class GripperInterface
{
public:
    GripperInterface();

    BT::NodeStatus open();
    BT::NodeStatus close();

private:
    bool _gripper_open;
};

#endif // GRIPPER_INTERFACE_H
