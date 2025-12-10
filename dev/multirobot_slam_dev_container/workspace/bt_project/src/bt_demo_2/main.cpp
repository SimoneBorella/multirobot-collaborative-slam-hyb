#include <iostream>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "check_battery.h"
#include "move.h"


int main(int argc, char **argv)
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<Move>("Move");
    factory.registerNodeType<CheckBattery>("CheckBattery");
    
    BT::Tree tree = factory.createTreeFromFile("./bt_xml/tree_demo_2.xml");

    // Here, instead of tree.tickRootWhileRunning(),
    // we prefer our own loop.
    std::cout << ">>> Ticking\n";
    BT::NodeStatus status = tree.tickRoot();
    std::cout << ">>> Status: " << BT::toStr(status) << "\n\n";

    while (status == BT::NodeStatus::RUNNING)
    {
        // Sleep to avoid busy loops.
        // do NOT use other sleep functions!
        // Small sleep time is OK, here we use a large one only to
        // have less messages on the console.
        tree.sleep(std::chrono::milliseconds(100));

        std::cout << ">>> Ticking\n";
        status = tree.tickRoot();
        std::cout << ">>> Status: " << BT::toStr(status) << "\n\n";
    }

    return 0;
}
