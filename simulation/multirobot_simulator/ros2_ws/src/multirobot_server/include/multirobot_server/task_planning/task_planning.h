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
        double w_distance;
        double w_orientation;
        double w_frontier_switch;
        double w_frontier_size;

        double w_coverage;
        double coverage_scale;

        double conflict_penalty;

        bool uncertainty_reduction_mode;

        double d_optimality_threshold;

        double w_mahalanobis;
        double w_cost_to_go;

        TaskPlanningParams(
            double w_distance = 0.4,
            double w_orientation = 0.2,
            double w_frontier_switch = 0.2,
            double w_frontier_size = 0.2,
            double w_coverage = 0.5,
            double coverage_scale = 1.0,
            double conflict_penalty = 0.001,
            bool uncertainty_reduction_mode = true,
            double d_optimality_threshold = 1.0,
            double w_mahalanobis = 0.7,
            double w_cost_to_go = 0.3)
            : w_distance(w_distance),
              w_orientation(w_orientation),
              w_frontier_switch(w_frontier_switch),
              w_frontier_size(w_frontier_size),
              w_coverage(w_coverage),
              coverage_scale(coverage_scale),
              conflict_penalty(conflict_penalty),
              uncertainty_reduction_mode(uncertainty_reduction_mode),
              d_optimality_threshold(d_optimality_threshold),
              w_mahalanobis(w_mahalanobis),
              w_cost_to_go(w_cost_to_go) {}
    };

    class TaskPlanning
    {
    public:
        TaskPlanning();
        TaskPlanning(TaskPlanningParams &params);

        static TaskPlanningParams params_from_yaml(std::string &params_path);
        void init(TaskPlanningParams &params);

        void update_robot_state(State state, std::string robot);
        void update_robot_keyframes(std::map<int, KeyFrame>& keyframes, std::string robot);

        Pose compose_global_pose(const Pose& initial, const Pose& local);
        bool task_reached(const Pose& current, const Task& target);
        std::map<std::string, Task> plan_tasks(std::map<std::string, Pose> robot_poses, std::vector<Frontier> frontiers);
    private:
        TaskPlanningParams params_;
        std::map<std::string, Pose> robot_initial_poses_;
        std::map<std::string, Task> robot_last_planned_tasks_;

        std::map<std::string, State> robot_state_;
        std::map<std::string, std::map<int, KeyFrame>> robot_keyframes_;

        std::map<std::string, Task> active_uncertainty_tasks_;
    };
}

#endif // TASK_PLANNING_H