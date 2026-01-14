#ifndef GLOBAL_PLANNING_H
#define GLOBAL_PLANNING_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <thread>
#include <atomic>
#include <queue>
#include <deque>
#include <mutex>
#include <optional>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "data_types.h"


namespace multirobot_slam
{
    struct GlobalPlanningParams
    {
        std::string heuristic;

        GlobalPlanningParams(
            std::string heuristic = "euclidean")
            : heuristic(heuristic) {}
    };


    struct AStarNode {
        int x, y;
        float g, h;
        std::shared_ptr<AStarNode> parent;

        AStarNode(int x, int y, float g = 0, float h = 0, std::shared_ptr<AStarNode> parent = nullptr)
            : x(x), y(y), g(g), h(h), parent(parent) {}

        float f() const {
            return g + h;
        }
    };

    struct CompareAStarNode {
        bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
            return a->f() > b->f();
        }
    };



    class GlobalPlanning
    {
    public:
        GlobalPlanning();
        GlobalPlanning(GlobalPlanningParams &params);

        static GlobalPlanningParams params_from_yaml(std::string &params_path);
        void init(GlobalPlanningParams &params);

        void update_costmap(Map costmap);
        std::map<std::string, Path> plan_global_path(std::map<std::string, Pose> robot_poses, std::map<std::string, Frontier> tasks);

    private:
        double heuristic(int x1, int y1, int x2, int y2);
        Path reconstructPath(std::shared_ptr<AStarNode> last);
        Path aStar(int sx, int sy, int gx, int gy);

        GlobalPlanningParams params_;

        std::vector<std::pair<int, int>> directions_;
        std::vector<float> directions_cost_;

        Map costmap_;
        bool costmap_received_;
    };
}

#endif // GLOBAL_PLANNING_H