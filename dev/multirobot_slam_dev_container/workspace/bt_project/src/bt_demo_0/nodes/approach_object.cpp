#include "approach_object.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

ApproachObject::ApproachObject(const std::string &name)
    : BT::SyncActionNode(name, {})
{
}

BT::NodeStatus ApproachObject::tick()
{
    std::cout << "Approaching object" << std::endl;
    std::this_thread::sleep_for(5s);
    std::cout << "Object approached" << std::endl;

    return BT::NodeStatus::SUCCESS;
}
