#include "move.h"

BT::NodeStatus Move::onStart()
{
  if (!getInput<Pose>("goal", _goal))
  {
    throw BT::RuntimeError("missing required input [goal]");
  }

  std::cout << "[ " << this->name() << " ] : Requested goal (" << _goal.x << ", " << _goal.y << ", " << _goal.theta << ")" << std::endl;

  // We use this counter to simulate an action that takes a certain
  // amount of time to be completed (200 ms)
  _completion_time = chr::system_clock::now() + chr::milliseconds(220);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Move::onRunning()
{
  // Pretend that we are checking if the reply has been received
  // you don't want to block inside this function too much time.
  std::this_thread::sleep_for(chr::milliseconds(10));

  // Pretend that, after a certain amount of time,
  // we have completed the operation
  if (chr::system_clock::now() >= _completion_time)
  {
    std::cout << "[ " << this->name() << " ] : Finished" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void Move::onHalted()
{
  std::cout << "[ " << this->name() << " ] : Aborted" << std::endl;
}