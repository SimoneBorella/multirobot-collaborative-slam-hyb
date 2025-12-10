#include "check_battery.h"
#include <iostream>

CheckBattery::CheckBattery(const std::string &name)
    : BT::ConditionNode(name, {})
{
}

BT::NodeStatus CheckBattery::tick()
{
    std::cout << "[ " << this->name() << " ] : Battery ok" << std::endl;
    return BT::NodeStatus::SUCCESS;
    
    // std::cout << "Battery error" << std::endl;
    // return BT::NodeStatus::FAILURE;
}
