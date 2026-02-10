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

        bool information_gain_mode;

        double d_opt_threshold_soft;
        double d_opt_threshold_hard;

        int min_keypoints_number;

        double w_cost_to_go;
        double w_mahalanobis;
        double w_keyframe_switch;

        double mahalanobis_sigma_scale;

        TaskPlanningParams(
            double w_distance = 0.4,
            double w_orientation = 0.2,
            double w_frontier_switch = 0.2,
            double w_frontier_size = 0.2,
            double w_coverage = 0.5,
            double coverage_scale = 1.0,
            double conflict_penalty = 0.001,
            bool information_gain_mode = true,
            double d_opt_threshold_soft = -10.0,
            double d_opt_threshold_hard = -5.0,
            int min_keypoints_number = 25,
            double w_cost_to_go = 0.3,
            double w_mahalanobis = 0.7,
            double w_keyframe_switch = 0.2,
            double mahalanobis_sigma_scale = 5.0)
            : w_distance(w_distance),
              w_orientation(w_orientation),
              w_frontier_switch(w_frontier_switch),
              w_frontier_size(w_frontier_size),
              w_coverage(w_coverage),
              coverage_scale(coverage_scale),
              conflict_penalty(conflict_penalty),
              information_gain_mode(information_gain_mode),
              d_opt_threshold_soft(d_opt_threshold_soft),
              d_opt_threshold_hard(d_opt_threshold_hard),
              min_keypoints_number(min_keypoints_number),
              w_cost_to_go(w_cost_to_go),
              w_mahalanobis(w_mahalanobis),
              w_keyframe_switch(w_keyframe_switch),
              mahalanobis_sigma_scale(mahalanobis_sigma_scale) {}
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

        std::map<std::string, Task> active_hard_information_gain_tasks_;
    };
}

#endif // TASK_PLANNING_H