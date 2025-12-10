#ifndef APPROACH_OBJECT_H
#define APPROACH_OBJECT_H

#include <behaviortree_cpp_v3/action_node.h>
#include <string>

class ApproachObject : public BT::SyncActionNode
{
public:
    explicit ApproachObject(const std::string &name);

    BT::NodeStatus tick() override;
};

#endif // APPROACH_OBJECT_H

