#include "check_battery.h"
#include <iostream>

CheckBattery::CheckBattery(const std::string &name)
    : BT::ConditionNode(name, {})
{
}

BT::NodeStatus CheckBattery::tick()
{
    std::cout << "Battery ok" << std::endl;
    return BT::NodeStatus::SUCCESS;
    
    // std::cout << "Battery error" << std::endl;
    // return BT::NodeStatus::FAILURE;
}
