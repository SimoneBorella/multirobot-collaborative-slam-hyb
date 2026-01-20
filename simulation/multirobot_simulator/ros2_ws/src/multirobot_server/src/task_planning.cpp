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

            if (config["alpha_distance"])
                p.alpha_distance = config["alpha_distance"].as<double>();
            if (config["beta_dimension"])
                p.beta_dimension = config["beta_dimension"].as<double>();
            if (config["conflict_penalty"])
                p.conflict_penalty = config["conflict_penalty"].as<double>();
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
    }

    std::map<std::string, Frontier> TaskPlanning::plan_tasks(std::map<std::string, Pose> robot_poses, std::vector<Frontier> frontiers)
    {
        std::map<std::string, Frontier> tasks;

        DiscreteFactorGraph graph;
        
        std::vector<std::string> robots;
        std::vector<Pose> poses;

        for(const auto& [robot, pose] : robot_poses)
        {
            robots.push_back(robot);
            poses.push_back(pose);
        }

        if (poses.empty() || frontiers.empty()) {
            return tasks;
        }

        size_t n_frontiers = frontiers.size();
        size_t n_robots = poses.size();



        // Create discrete keys for each robot
        std::vector<DiscreteKey> robot_keys;
        robot_keys.reserve(n_robots);

        for(size_t i = 0; i < n_robots; i++)
        {
            const std::string& robot = robots[i];
            std::string prefix = "robot_";
            int robot_id = std::stoi(robot.substr(prefix.length()));
            DiscreteKey robot_key(Symbol('x', robot_id), n_frontiers);
            robot_keys.push_back(robot_key);
        }


        // Add unary factors for each robot
        for (size_t i = 0; i < n_robots; i++)
        {
            Eigen::Vector2d robot_position = Eigen::Vector2d(poses[i].position.x(), poses[i].position.y());

            std::vector<double> values(n_frontiers);

            for (size_t j = 0; j < n_frontiers; j++)
            {
                Eigen::Vector2d frontier_centroid = frontiers[j].centroid;
                double size = frontiers[j].size;

                Eigen::Vector2d delta_position = frontier_centroid - robot_position;

                double euclidean_dist = delta_position.norm();

                double robot_yaw = Eigen::AngleAxisd(poses[i].orientation).angle() * Eigen::AngleAxisd(poses[i].orientation).axis().z();
                double frontier_bearing = std::atan2(delta_position.y(), delta_position.x());

                double orientation_dist = std::abs(std::atan2(
                    std::sin(frontier_bearing - robot_yaw),
                    std::cos(frontier_bearing - robot_yaw)
                ));

                double frontier_switch_dist = 0.0;

                if (last_planned_tasks_.count(robots[i]))
                {
                    const Frontier& old_frontier = last_planned_tasks_[robots[i]];
                    frontier_switch_dist = (frontier_centroid - old_frontier.centroid).norm();
                }

                values[j] = std::exp(std::min(-params_.alpha_distance * (euclidean_dist + orientation_dist*5 + frontier_switch_dist) + params_.beta_dimension * size, 0.0));
            }

            DecisionTreeFactor unary_factor(robot_keys[i], values);
            graph.add(unary_factor);
        }


        // Add pairwise factors between robots and frontiers
        for (size_t i = 0; i < n_robots; i++)
        {
            for (size_t j = i + 1; j < n_robots; j++)
            {
                std::vector<double> table(n_frontiers * n_frontiers, 1.0);
                for (size_t f = 0; f < n_frontiers; f++)
                {
                    table[f * n_frontiers + f] = params_.conflict_penalty; 
                }
                DecisionTreeFactor conflict_factor({robot_keys[i], robot_keys[j]}, table);
                graph.add(conflict_factor);
            }
        }

        DiscreteValues result = graph.optimize();

        for (size_t i = 0; i < poses.size(); i++)
        {
            const std::string& robot = robots[i];

            size_t assignment = result[robot_keys[i].first];
            tasks[robot] = frontiers[assignment];
        }

        last_planned_tasks_ = tasks;

        return tasks;
    }
}


