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

#include "mapping_data_types.h"

using namespace mapping;

using namespace gtsam;

namespace task_planning
{
    struct TaskPlanningParams
    {
        double alpha_distance;
        double beta_dimension;
        double conflict_penalty;

        TaskPlanningParams(
            double alpha_distance = 1.0,
            double beta_dimension = 0.01,
            double conflict_penalty = 0.001)
            : alpha_distance(alpha_distance),
              beta_dimension(beta_dimension),
              conflict_penalty(conflict_penalty){}
    };


    class TaskPlanning
    {
    public:
        TaskPlanning();
        TaskPlanning(TaskPlanningParams &params);

        static TaskPlanningParams params_from_yaml(std::string &params_path);
        void init(TaskPlanningParams &params);

        std::map<std::string, Frontier> task_planning(std::map<std::string, Pose> robot_poses, std::vector<Frontier> frontiers);
    private:

        TaskPlanningParams params_;
    };
}

#endif // TASK_PLANNING_H