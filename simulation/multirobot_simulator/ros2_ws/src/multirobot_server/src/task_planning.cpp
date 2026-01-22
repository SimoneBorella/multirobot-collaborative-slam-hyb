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
            if (config["conflict_penalty"]) p.conflict_penalty = config["conflict_penalty"].as<double>();
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

        // Create dynamic max distance & size
        double max_distance = 0.0;
        double max_size = 0.0;

        for (const auto& pose : poses) {
            Eigen::Vector2d robot_position(pose.position.x(), pose.position.y());
            for (const auto& f : frontiers) {
                double d = (f.centroid - robot_position).norm();
                if (d > max_distance) max_distance = d;
                if (f.size > max_size) max_size = f.size;
            }
        }

        // Avoid division by zero
        max_distance = std::max(max_distance, 1e-6);
        max_size = std::max(max_size, 1e-6);

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
            Eigen::Vector2d robot_position(poses[i].position.x(), poses[i].position.y());

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

                // Normalize each contribution [0,1]
                double distance_score = 1.0 - std::min(euclidean_dist / max_distance, 1.0);
                double orientation_score = 1.0 - std::min(orientation_dist / M_PI, 1.0);
                double frontier_switch_score = 1.0 - std::min(frontier_switch_dist / max_distance, 1.0);
                double frontier_size_score = std::min(size / max_size, 1.0);

                // Weighted sum
                double score =
                    params_.w_distance * distance_score +
                    params_.w_orientation * orientation_score +
                    params_.w_frontier_switch * frontier_switch_score +
                    params_.w_frontier_size * frontier_size_score;

                // Normalize weights
                double sumw = params_.w_distance + params_.w_orientation + params_.w_frontier_switch + params_.w_frontier_size;
                if (sumw > 0)
                    score /= sumw;

                values[j] = std::max(score, 1e-6);
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


