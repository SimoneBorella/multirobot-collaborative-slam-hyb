#ifndef MOVE_H
#define MOVE_H

#include <behaviortree_cpp_v3/action_node.h>
#include <pose.h>
#include <string>
#include <chrono>

namespace chr = std::chrono;

class Move : public BT::StatefulActionNode
{
public:
  // Any TreeNode with ports must have a constructor with this signature
  Move(const std::string &name, const BT::NodeConfiguration &config)
      : StatefulActionNode(name, config)
  {
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Pose>("goal")};
  }

  // this function is invoked once at the beginning.
  BT::NodeStatus onStart() override;

  // If onStart() returned RUNNING, we will keep calling
  // this method until it return something different from RUNNING
  BT::NodeStatus onRunning() override;

  // callback to execute if the action was aborted by another node
  void onHalted() override;

private:
  Pose _goal;
  chr::system_clock::time_point _completion_time;
};

#endif // MOVE_H
