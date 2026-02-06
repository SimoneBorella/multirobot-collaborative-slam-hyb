#include "task_planning.h"

namespace multirobot_slam
{
    TaskPlanning::TaskPlanning()
    {
    }

    TaskPlanning::TaskPlanning(TaskPlanningParams &params)
        : params_(params)
    {
    }

    TaskPlanningParams TaskPlanning::params_from_yaml(std::string &params_path)
    {
        TaskPlanningParams p;

        try
        {
            YAML::Node config = YAML::LoadFile(params_path);

            if (config["w_distance"]) p.w_distance = config["w_distance"].as<double>();
            if (config["w_orientation"]) p.w_orientation = config["w_orientation"].as<double>();
            if (config["w_frontier_switch"]) p.w_frontier_switch = config["w_frontier_switch"].as<double>();
            if (config["w_frontier_size"]) p.w_frontier_size = config["w_frontier_size"].as<double>();
            if (config["w_coverage"]) p.w_coverage = config["w_coverage"].as<double>();
            if (config["coverage_scale"]) p.coverage_scale = config["coverage_scale"].as<double>();
            if (config["conflict_penalty"]) p.conflict_penalty = config["conflict_penalty"].as<double>();
            if (config["uncertainty_reduction_mode"]) p.uncertainty_reduction_mode = config["uncertainty_reduction_mode"].as<bool>();
            if (config["d_optimality_threshold"]) p.d_optimality_threshold = config["d_optimality_threshold"].as<double>();
            if (config["w_mahalanobis"]) p.w_mahalanobis = config["w_mahalanobis"].as<double>();
            if (config["w_cost_to_go"]) p.w_cost_to_go = config["w_cost_to_go"].as<double>();

        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
            std::cerr << "Using default parameters.\n";
        }

        return p;
    }


    void TaskPlanning::init(TaskPlanningParams &params)
    {
        params_ = params;

        double sumw = params_.w_distance + params_.w_orientation + params_.w_frontier_switch + params_.w_frontier_size;
        if (sumw > 0) {
            params_.w_distance /= sumw;
            params_.w_orientation /= sumw;
            params_.w_frontier_switch /= sumw;
            params_.w_frontier_size /= sumw;
        }

    }


    void TaskPlanning::update_robot_state(State state, std::string robot)
    {
        robot_state_[robot] = state;
    }

    void TaskPlanning::update_robot_keyframes(std::map<int, KeyFrame>& keyframes, std::string robot)
    {
        std::map<int, KeyFrame>& robot_keyframes = robot_keyframes_[robot];

        for(auto& [id, kf] : keyframes)
        {
            robot_keyframes[id] = kf;
        }
    }



    std::map<std::string, Task> TaskPlanning::plan_tasks(std::map<std::string, Pose> robot_poses, std::vector<Frontier> frontiers)
    {
        std::map<std::string, Task> tasks;

        if (robot_poses.empty())
            return tasks;

        const size_t n_frontiers = frontiers.size();

        // Store initial poses
        for (const auto& [robot, pose] : robot_poses)
        {
            if (!robot_initial_poses_.count(robot))
                robot_initial_poses_[robot] = pose;
        }

        // If no frontiers return to initial pose
        if (n_frontiers == 0)
        {
            for (const auto& [robot, _] : robot_poses)
            {
                Task t;
                t.pose = robot_initial_poses_[robot];
                t.oriented = true;
                tasks[robot] = t;
            }
            return tasks;
        }

        // Compute max distance and size
        double max_distance = 0.0;
        double max_size = 0.0;

        for (const auto& [_, pose] : robot_poses)
        {
            Eigen::Vector2d robot_position(pose.position.x(), pose.position.y());
            for (const auto& f : frontiers)
            {
                max_distance = std::max(max_distance, (f.centroid - robot_position).norm());
                max_size = std::max(max_size, f.size);
            }
        }

        max_distance = std::max(max_distance, 1e-6);
        max_size = std::max(max_size, 1e-6);


        // Define robots mode
        std::vector<std::string> explorative_robots;
        std::vector<std::string> uncertainty_reduction_robots;

        for (const auto& [robot, pose] : robot_poses)
        {
            if(params_.uncertainty_reduction_mode)
            {
                Eigen::Matrix<double, 6, 6> cov = robot_state_[robot].covariance;
                
                double det = cov.determinant();
                det = std::max(det, 1e-12);
                double log_det = std::log(det);
                
                std::cout << "Log det:" << log_det << std::endl;

                if(log_det > params_.d_optimality_threshold)
                {
                    uncertainty_reduction_robots.push_back(robot);
                }
                else
                {
                    explorative_robots.push_back(robot);
                }
            }
            else
            {
                explorative_robots.push_back(robot);
            }
        }

        // Exploration tasks

        // Build discrete factor graph
        DiscreteFactorGraph graph;

        // Build discrete keys
        std::map<std::string, DiscreteKey> robot_keys;

        for (const auto& robot : explorative_robots)
        {
            int robot_id = std::stoi(robot.substr(std::string("robot_").length()));
            robot_keys[robot] = DiscreteKey(Symbol('x', robot_id), n_frontiers);
        }

        // Unary factors
        for (const auto& robot : explorative_robots)
        {
            Pose pose = robot_poses[robot];

            Eigen::Vector2d robot_pos(pose.position.x(), pose.position.y());
            std::vector<double> values(n_frontiers);

            double robot_yaw =
                Eigen::AngleAxisd(pose.orientation).angle() *
                Eigen::AngleAxisd(pose.orientation).axis().z();

            for (size_t f = 0; f < n_frontiers; ++f)
            {
                const auto& frontier = frontiers[f];
                Eigen::Vector2d delta = frontier.centroid - robot_pos;

                double dist = delta.norm();
                double bearing = std::atan2(delta.y(), delta.x());

                double orientation_dist = std::abs(std::atan2(
                    std::sin(bearing - robot_yaw),
                    std::cos(bearing - robot_yaw)));

                double switch_dist = 0.0;
                if (robot_last_planned_tasks_.count(robot))
                {
                    const auto& old = robot_last_planned_tasks_[robot];
                    switch_dist = (frontier.centroid - Eigen::Vector2d(old.pose.position.x(), old.pose.position.y())).norm();
                }

                // Normalization
                double score =
                    params_.w_distance * (1.0 - std::min(dist / max_distance, 1.0)) +
                    params_.w_orientation * (1.0 - std::min(orientation_dist / M_PI, 1.0)) +
                    params_.w_frontier_switch * (1.0 - std::min(switch_dist / max_distance, 1.0)) +
                    params_.w_frontier_size * std::min(frontier.size / max_size, 1.0);

                // Weighted sum
                double sumw =
                    params_.w_distance +
                    params_.w_orientation +
                    params_.w_frontier_switch +
                    params_.w_frontier_size;

                if (sumw > 0.0)
                    score /= sumw;

                values[f] = std::max(score, 1e-6);
            }

            graph.add(DecisionTreeFactor(robot_keys[robot], values));
        }

        // Pairwise factors
        for (size_t i = 0; i < explorative_robots.size(); ++i)
        {
            for (size_t j = i + 1; j < explorative_robots.size(); ++j)
            {
                const auto& r1 = explorative_robots[i];
                const auto& r2 = explorative_robots[j];

                // Conflict factor
                std::vector<double> conflict(n_frontiers * n_frontiers, 1.0);
                for (size_t f = 0; f < n_frontiers; ++f)
                    conflict[f * n_frontiers + f] = params_.conflict_penalty;

                graph.add(DecisionTreeFactor(
                    {robot_keys[r1], robot_keys[r2]}, conflict));

                // Coverage factor
                std::vector<double> coverage(n_frontiers * n_frontiers, 1.0);
                for (size_t f1 = 0; f1 < n_frontiers; ++f1)
                {
                    for (size_t f2 = 0; f2 < n_frontiers; ++f2)
                    {
                        double d =
                            (frontiers[f1].centroid -
                            frontiers[f2].centroid).norm();

                        double raw = 1.0 - std::exp(-d / params_.coverage_scale);
                        coverage[f1 * n_frontiers + f2] =
                            (1.0 - params_.w_coverage) +
                            params_.w_coverage * raw;
                    }
                }

                graph.add(DecisionTreeFactor(
                    {robot_keys[r1], robot_keys[r2]}, coverage));
            }
        }

        // Optimize
        DiscreteValues result = graph.optimize();

        // Decode solution
        for (const auto& robot : explorative_robots)
        {
            size_t assignment = result[robot_keys[robot].first];

            Task t;
            t.pose.position.x() = frontiers[assignment].centroid.x();
            t.pose.position.y() = frontiers[assignment].centroid.y();
            t.oriented = false;

            tasks[robot] = t;
        }



        // Uncertainty reduction tasks

        for (const auto& robot : uncertainty_reduction_robots)
        {
            const Pose& pose = robot_poses[robot];
            const Eigen::Matrix<double,6,6>& Sigma_i = robot_state_[robot].covariance;

            Eigen::Vector2d robot_pos(
                pose.position.x(),
                pose.position.y()
            );

            // Discrete graph for robot
            DiscreteFactorGraph graph;

            const auto& keyframes = robot_keyframes_[robot];
            if (keyframes.empty())
                continue;

            int robot_id = std::stoi(robot.substr(std::string("robot_").length()));
            DiscreteKey robot_key(Symbol('u', robot_id), keyframes.size());

            std::vector<double> values(keyframes.size(), std::numeric_limits<double>::infinity());

            // Iteration in keyframe order
            Eigen::Vector2d prev_kf_pos = robot_pos;
            double accumulated_cost_to_go = 0.0;

            size_t idx = 0;
            for (const auto& [kf_id, kf] : keyframes)
            {
                Eigen::Vector2d kf_pos(
                    kf.pose.position.x(),
                    kf.pose.position.y()
                );

                // Mahalanobis distance
                Eigen::Matrix<double,6,6> Sigma_j = kf.covariance;
                Eigen::Matrix<double,6,6> Sigma_ij = Sigma_i + Sigma_j;

                Eigen::Matrix<double,6,1> dx;
                dx.setZero();
                dx(0) = pose.position.x() - kf.pose.position.x();
                dx(1) = pose.position.y() - kf.pose.position.y();

                double mahalanobis = dx.transpose() * Sigma_ij.inverse() * dx;

                // Chi-square gating (2 DoF approx)
                // if (mahalanobis > params_.mahalanobis_chi2_gate)
                // {
                //     values[idx++] = 1e9;
                //     continue;
                // }

                // Cost to go
                double step_cost = (kf_pos - prev_kf_pos).norm();
                accumulated_cost_to_go += step_cost;
                prev_kf_pos = kf_pos;

                // Score
                double score =
                    params_.w_mahalanobis * mahalanobis +
                    params_.w_cost_to_go * accumulated_cost_to_go;

                values[idx++] = std::max(score, 1e-6);
            }

            // Add unary factor
            graph.add(DecisionTreeFactor(robot_key, values));

            // Optimize
            DiscreteValues result = graph.optimize();
            size_t best_idx = result[robot_key.first];

            // Retrieve selected keyframe
            auto it = keyframes.begin();
            std::advance(it, best_idx);

            const KeyFrame& best_kf = it->second;

            // Assign task
            Task t;
            t.pose = best_kf.pose;
            t.oriented = true;

            tasks[robot] = t;
        }


        robot_last_planned_tasks_ = tasks;
        return tasks;
    }


}


