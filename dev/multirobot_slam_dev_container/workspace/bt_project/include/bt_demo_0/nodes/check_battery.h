#ifndef CHECK_BATTERY_H
#define CHECK_BATTERY_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <string>

class CheckBattery : public BT::ConditionNode
{
public:
    explicit CheckBattery(const std::string &name);

    BT::NodeStatus tick() override;
};

#endif // CHECK_BATTERY_H