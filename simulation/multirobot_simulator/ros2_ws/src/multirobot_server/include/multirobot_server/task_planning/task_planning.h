#ifndef TASK_PLANNING_H
#define TASK_PLANNING_H

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

#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/inference/Symbol.h>

#include "data_types.h"

using namespace gtsam;

namespace multirobot_slam
{
    struct TaskPlanningParams
    {
        double w_distance = 0.4;
        double w_orientation = 0.3;
        double w_frontier_switch = 0.2;
        double w_frontier_size = 0.2;

        double conflict_penalty = 0.001;

        TaskPlanningParams(
            double w_distance = 0.4,
            double w_orientation = 0.2,
            double w_frontier_switch = 0.2,
            double w_frontier_size = 0.2,
            double conflict_penalty = 0.001)
            : w_distance(w_distance),
            w_orientation(w_orientation),
            w_frontier_switch(w_frontier_switch),
            w_frontier_size(w_frontier_size),
            conflict_penalty(conflict_penalty) {}
    };



    class TaskPlanning
    {
    public:
        TaskPlanning();
        TaskPlanning(TaskPlanningParams &params);

        static TaskPlanningParams params_from_yaml(std::string &params_path);
        void init(TaskPlanningParams &params);

        std::map<std::string, Frontier> plan_tasks(std::map<std::string, Pose> robot_poses, std::vector<Frontier> frontiers);
    private:

        TaskPlanningParams params_;

        std::map<std::string, Frontier> last_planned_tasks_;
    };
}

#endif // TASK_PLANNING_H