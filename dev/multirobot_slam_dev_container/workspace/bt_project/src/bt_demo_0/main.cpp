#include <iostream>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "approach_object.h"
#include "check_battery.h"
#include "gripper_interface.h"

int main(int argc, char **argv)
{
    BT::BehaviorTreeFactory factory;

    GripperInterface gripper;

    factory.registerNodeType<CheckBattery>("CheckBattery");
    factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &gripper));
    factory.registerNodeType<ApproachObject>("ApproachObject");

    BT::Tree tree = factory.createTreeFromFile("bt_xml/tree_demo_0.xml");

    tree.tickRootWhileRunning();
    return 0;
}
