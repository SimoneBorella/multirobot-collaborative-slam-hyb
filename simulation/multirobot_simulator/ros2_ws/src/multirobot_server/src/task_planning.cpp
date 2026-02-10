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
            if (config["information_gain_mode"]) p.information_gain_mode = config["information_gain_mode"].as<bool>();
            if (config["d_opt_threshold_soft"]) p.d_opt_threshold_soft = config["d_opt_threshold_soft"].as<double>();
            if (config["d_opt_threshold_hard"]) p.d_opt_threshold_hard = config["d_opt_threshold_hard"].as<double>();
            if (config["min_keypoints_number"]) p.min_keypoints_number = config["min_keypoints_number"].as<int>();
            if (config["w_cost_to_go"]) p.w_cost_to_go = config["w_cost_to_go"].as<double>();
            if (config["w_mahalanobis"]) p.w_mahalanobis = config["w_mahalanobis"].as<double>();
            if (config["w_keyframe_switch"]) p.w_keyframe_switch = config["w_keyframe_switch"].as<double>();
            if (config["mahalanobis_sigma_scale"]) p.mahalanobis_sigma_scale = config["mahalanobis_sigma_scale"].as<double>();
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

    Pose TaskPlanning::compose_global_pose(const Pose& initial, const Pose& local)
    {
        Pose out;

        double yaw0 = std::atan2(
            2.0 * (initial.orientation.w() * initial.orientation.z() +
                initial.orientation.x() * initial.orientation.y()),
            1.0 - 2.0 * (initial.orientation.y() * initial.orientation.y() +
                        initial.orientation.z() * initial.orientation.z())
        );

        double yaw_l = std::atan2(
            2.0 * (local.orientation.w() * local.orientation.z() +
                local.orientation.x() * local.orientation.y()),
            1.0 - 2.0 * (local.orientation.y() * local.orientation.y() +
                        local.orientation.z() * local.orientation.z())
        );


        Eigen::Rotation2Dd R(yaw0);

        Eigen::Vector2d p0(
            initial.position.x(),
            initial.position.y());

        Eigen::Vector2d pl(
            local.position.x(),
            local.position.y());

        Eigen::Vector2d pg = p0 + R * pl;

        out.position.x() = pg.x();
        out.position.y() = pg.y();
        out.position.z() = initial.position.z();

        double yaw_g = yaw0 + yaw_l;

        out.orientation =
            Eigen::AngleAxisd(yaw_g, Eigen::Vector3d::UnitZ());

        return out;
    }



    bool TaskPlanning::task_reached(const Pose& current, const Task& target)
    {
        Eigen::Vector2d p_cur(
            current.position.x(),
            current.position.y());

        Eigen::Vector2d p_tgt(
            target.pose.position.x(),
            target.pose.position.y());

        double pos_err = (p_cur - p_tgt).norm();

        double yaw_cur = std::atan2(
            2.0 * (current.orientation.w() * current.orientation.z() +
                current.orientation.x() * current.orientation.y()),
            1.0 - 2.0 * (current.orientation.y() * current.orientation.y() +
                        current.orientation.z() * current.orientation.z())
        );

        double yaw_tgt = std::atan2(
            2.0 * (target.pose.orientation.w() * target.pose.orientation.z() +
                target.pose.orientation.x() * target.pose.orientation.y()),
            1.0 - 2.0 * (target.pose.orientation.y() * target.pose.orientation.y() +
                        target.pose.orientation.z() * target.pose.orientation.z())
        );


        double yaw_err = std::abs(std::atan2(
            std::sin(yaw_cur - yaw_tgt),
            std::cos(yaw_cur - yaw_tgt)));

        return (pos_err < 0.1) &&(yaw_err < 0.0785);
    }




    std::map<std::string, Task> TaskPlanning::plan_tasks(std::map<std::string, Pose> robot_poses, std::vector<Frontier> frontiers)
    {
        std::map<std::string, Task> tasks;

        if (robot_poses.empty())
            return tasks;

        // Store initial poses
        for (const auto& [robot, pose] : robot_poses)
        {
            if (!robot_initial_poses_.count(robot))
                robot_initial_poses_[robot] = pose;
        }

        // Compute max frontier distance, max frontier size and max keyframe distance for further cost normalization
        double max_frontier_distance = 0.0;
        double max_frontier_size = 0.0;
        double max_keyframe_distance = 0.0;


        for (const auto& [robot, pose] : robot_poses)
        {
            Eigen::Vector2d robot_position(pose.position.x(), pose.position.y());

            for (const auto& f : frontiers)
            {
                max_frontier_distance = std::max(max_frontier_distance, (f.centroid - robot_position).norm());
                max_frontier_size = std::max(max_frontier_size, f.size);
            }

            for (const auto& [_, kf] : robot_keyframes_[robot])
            {
                Pose kf_global_pose = compose_global_pose(
                    robot_initial_poses_[robot],
                    kf.pose
                );

                Eigen::Vector2d kf_pos(
                    kf_global_pose.position.x(),
                    kf_global_pose.position.y()
                );

                max_keyframe_distance = std::max(max_keyframe_distance, (kf_pos - robot_position).norm());
            }
        }

        max_frontier_distance = std::max(max_frontier_distance, 1e-6);
        max_frontier_size = std::max(max_frontier_size, 1e-6);
        max_keyframe_distance = std::max(max_keyframe_distance, 1e-6);


        // Define number of frontiers
        const size_t n_frontiers = frontiers.size();

        // Define robots mode
        std::vector<std::string> explorative_robots;
        std::vector<std::string> hard_information_gain_robots;

        std::map<std::string, double> lambda_r;

        for (const auto& [robot, pose] : robot_poses)
        {
            if(params_.information_gain_mode)
            {
                // If tasks are not reached continue hard information gain tasks
                if (active_hard_information_gain_tasks_.count(robot))
                {
                    const Task& active_task = active_hard_information_gain_tasks_[robot];

                    if (!task_reached(pose, active_task))
                    {
                        tasks[robot] = active_task;
                        continue;
                    }
                    else
                    {
                        active_hard_information_gain_tasks_.erase(robot);
                    }
                    
                }

                const Eigen::Matrix<double,6,6>& cov = robot_state_[robot].covariance;

                Eigen::Matrix3d Sigma_pose;

                Sigma_pose(0,0) = cov(0,0); // x
                Sigma_pose(0,1) = cov(0,1); 
                Sigma_pose(0,2) = cov(0,5); 

                Sigma_pose(1,0) = cov(1,0);
                Sigma_pose(1,1) = cov(1,1); // y
                Sigma_pose(1,2) = cov(1,5);

                Sigma_pose(2,0) = cov(5,0);
                Sigma_pose(2,1) = cov(5,1);
                Sigma_pose(2,2) = cov(5,5); // yaw

                double det = std::max(Sigma_pose.determinant(), 1e-12);
                double d_opt = std::log(det);

                // std::cout << "Determinant:" << det << std::endl;
                // std::cout << "D-opt:" << d_opt << std::endl;

                // D opt based choice informationgain/eploration
                if(d_opt >= params_.d_opt_threshold_hard)
                {
                    hard_information_gain_robots.push_back(robot);
                    // std::cout << "HARD INFORMATION GAIN MODE" << std::endl;
                }
                else if (n_frontiers == 0)
                {
                    Task t;
                    t.pose = robot_initial_poses_[robot];
                    t.oriented = true;
                    tasks[robot] = t;
                    // std::cout << "GO HOME MODE" << std::endl;
                }
                else if (d_opt >= params_.d_opt_threshold_soft && d_opt < params_.d_opt_threshold_hard)
                {
                    lambda_r[robot] = std::clamp(
                        (d_opt - params_.d_opt_threshold_soft) /
                        (params_.d_opt_threshold_hard - params_.d_opt_threshold_soft),
                        0.0, 1.0
                    );
                    explorative_robots.push_back(robot);
                    // std::cout << "SOFT INFORMATION GAIN MODE" << std::endl;
                    // std::cout << "Lambda: " << lambda_r[robot] << std::endl;
                }
                else
                {
                    lambda_r[robot] = 0.0;
                    explorative_robots.push_back(robot);
                    // std::cout << "EXPLORATION MODE" << std::endl;
                }
            }
            else
            {
                if (n_frontiers == 0)
                {
                    Task t;
                    t.pose = robot_initial_poses_[robot];
                    t.oriented = true;
                    tasks[robot] = t;
                }

                lambda_r[robot] = 0.0;
                explorative_robots.push_back(robot);
            }
        }

        // Define number of robot keyframes
        std::map<std::string, size_t> n_robot_keyframes;
        for (const auto& robot : explorative_robots)
        {
            n_robot_keyframes[robot] = robot_keyframes_[robot].size();
        }





        // Exploration tasks

        // Build discrete factor graph
        DiscreteFactorGraph graph;

        // Build discrete keys
        std::map<std::string, DiscreteKey> robot_keys;

        for (const auto& robot : explorative_robots)
        {
            int robot_id = std::stoi(robot.substr(std::string("robot_").length()));
            robot_keys[robot] = DiscreteKey(Symbol('x', robot_id), n_frontiers + n_robot_keyframes[robot]);
        }

        // Unary factors
        for (const auto& robot : explorative_robots)
        {
            Pose pose = robot_poses[robot];
            const Eigen::Matrix<double,6,6>& Sigma_i = robot_state_[robot].covariance;

            Eigen::Vector2d robot_pos(pose.position.x(), pose.position.y());
            
            double robot_yaw = std::atan2(
                2.0 * (pose.orientation.w() * pose.orientation.z() +
                    pose.orientation.x() * pose.orientation.y()),
                1.0 - 2.0 * (pose.orientation.y() * pose.orientation.y() +
                            pose.orientation.z() * pose.orientation.z())
            );
            
            std::vector<double> values(n_frontiers + n_robot_keyframes[robot], 1e-6);
            for (size_t f = 0; f < n_frontiers; ++f)
            {
                const auto& frontier = frontiers[f];
                Eigen::Vector2d delta = frontier.centroid - robot_pos;

                double dist = delta.norm();
                double bearing = std::atan2(delta.y(), delta.x());
                double orientation_dist = std::abs(std::atan2(std::sin(bearing - robot_yaw), std::cos(bearing - robot_yaw)));

                double switch_dist = 0.0;
                if (robot_last_planned_tasks_.count(robot))
                {
                    const auto& old = robot_last_planned_tasks_[robot];
                    switch_dist = (frontier.centroid - Eigen::Vector2d(old.pose.position.x(), old.pose.position.y())).norm();
                }

                // Score normalized
                double score =
                    params_.w_distance * (1.0 - std::min(dist / max_frontier_distance, 1.0)) +
                    params_.w_orientation * (1.0 - std::min(orientation_dist / M_PI, 1.0)) +
                    params_.w_frontier_switch * (1.0 - std::min(switch_dist / max_frontier_distance, 1.0)) +
                    params_.w_frontier_size * std::min(frontier.size / max_frontier_size, 1.0);

                // Weighted sum
                double sumw =
                    params_.w_distance +
                    params_.w_orientation +
                    params_.w_frontier_switch +
                    params_.w_frontier_size;

                if (sumw > 0.0)
                    score /= sumw;

                // Blending lambda exploration/information gain factor
                score *= (1-lambda_r[robot]);

                values[f] = std::max(score, 1e-6);
            }


            if (robot_keyframes_[robot].size() != 0)
            {
                size_t k = 0;
                for (const auto& [kf_id, kf] : robot_keyframes_[robot])
                {
                    if (kf.keypoints_number < params_.min_keypoints_number)
                    {
                        values[n_frontiers + k] = 1e-6;
                        k++;
                        continue;
                    }
    
    
                    Pose kf_global_pose = compose_global_pose(
                        robot_initial_poses_[robot],
                        kf.pose
                    );
    
                    Eigen::Vector2d kf_pos(
                        kf_global_pose.position.x(),
                        kf_global_pose.position.y()
                    );
    
                    // Distance
                    double dist = (kf_pos - robot_pos).norm();

                    if(dist < 2.0)
                    {
                        values[n_frontiers + k] = 1e-6;
                        k++;
                        continue;
                    }
    
                    // Mahalanobis distance
                    Eigen::Matrix<double,6,6> Sigma_j = kf.covariance;
                    
                    Eigen::Matrix3d Sigma_i_pose, Sigma_j_pose;
    
                    Sigma_i_pose <<
                        Sigma_i(0,0), Sigma_i(0,1), Sigma_i(0,5),
                        Sigma_i(1,0), Sigma_i(1,1), Sigma_i(1,5),
                        Sigma_i(5,0), Sigma_i(5,1), Sigma_i(5,5);
    
                    Sigma_j_pose <<
                        Sigma_j(0,0), Sigma_j(0,1), Sigma_j(0,5),
                        Sigma_j(1,0), Sigma_j(1,1), Sigma_j(1,5),
                        Sigma_j(5,0), Sigma_j(5,1), Sigma_j(5,5);
    
                    Eigen::Matrix3d Sigma_ij = Sigma_i_pose + Sigma_j_pose;
                    Sigma_ij += 1e-6 * Eigen::Matrix3d::Identity();
    
    
                    Eigen::Vector3d dx;
    
                    double yaw_i = std::atan2(
                        2.0 * (pose.orientation.w() * pose.orientation.z() +
                            pose.orientation.x() * pose.orientation.y()),
                        1.0 - 2.0 * (pose.orientation.y() * pose.orientation.y() +
                                    pose.orientation.z() * pose.orientation.z())
                    );

                    double yaw_j = std::atan2(
                        2.0 * (kf_global_pose.orientation.w() * kf_global_pose.orientation.z() +
                            kf_global_pose.orientation.x() * kf_global_pose.orientation.y()),
                        1.0 - 2.0 * (kf_global_pose.orientation.y() * kf_global_pose.orientation.y() +
                                    kf_global_pose.orientation.z() * kf_global_pose.orientation.z())
                    );
    
                    dx(0) = pose.position.x() - kf_global_pose.position.x();
                    dx(1) = pose.position.y() - kf_global_pose.position.y();
                    dx(2) = std::atan2(std::sin(yaw_i - yaw_j), std::cos(yaw_i - yaw_j));
    
                    // double mahalanobis = dx.transpose() * Sigma_ij.inverse() * dx;
                    double mahalanobis = dx.dot(Sigma_ij.ldlt().solve(dx));
    
    
                    double normalized_mahalanobis_dist = std::exp(-mahalanobis/params_.mahalanobis_sigma_scale);



                    double switch_dist = 0.0;
                    if (robot_last_planned_tasks_.count(robot))
                    {
                        const auto& old = robot_last_planned_tasks_[robot];
                        switch_dist = (kf_pos - Eigen::Vector2d(old.pose.position.x(), old.pose.position.y())).norm();
                    }
        
                    // Score normalized
                    double score =
                        params_.w_cost_to_go * (1.0 - std::min(dist / max_keyframe_distance, 1.0)) +
                        params_.w_mahalanobis * (std::min(normalized_mahalanobis_dist, 1.0)) +
                        params_.w_keyframe_switch * (1.0 - std::min(switch_dist / max_keyframe_distance, 1.0));
    
                    // Weighted sum
                    double sumw =
                        params_.w_cost_to_go +
                        params_.w_mahalanobis +
                        params_.w_keyframe_switch;
    
                    if (sumw > 0.0)
                        score /= sumw;
    
                    // Blending lambda exploration/information gain factor
                    score *= lambda_r[robot];
    
                    values[n_frontiers + k] = std::max(score, 1e-6);
                    k++;


                    // std::cout << kf_id << ": dist - " << params_.w_cost_to_go * (1.0 - std::min(dist / max_keyframe_distance, 1.0)) << " - mahalanobis - " << params_.w_mahalanobis * (std::min(normalized_mahalanobis_dist, 1.0)) << " - lambda - " << lambda_r[robot] << std::endl;
                }
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

        // Retrieve solution
        for (const auto& robot : explorative_robots)
        {
            size_t assignment = result[robot_keys[robot].first];

            Task t;

            if (assignment < n_frontiers)
            {
                // Frontier assignment
                t.pose.position.x() = frontiers[assignment].centroid.x();
                t.pose.position.y() = frontiers[assignment].centroid.y();
                t.oriented = false;
            }
            else
            {
                // Keyframe assignment
                size_t kf_index = assignment - n_frontiers;
                auto it = robot_keyframes_[robot].begin();
                std::advance(it, kf_index);

                Pose kf_global_pose = compose_global_pose(
                    robot_initial_poses_[robot],
                    it->second.pose
                );

                t.pose = kf_global_pose;
                t.oriented = true;
            }

            tasks[robot] = t;
        }






        // Uncertainty reduction tasks

        for (const auto& robot : hard_information_gain_robots)
        {
            // Discrete graph for robot
            DiscreteFactorGraph graph;


            const Pose& pose = robot_poses[robot];
            const Eigen::Matrix<double,6,6>& Sigma_i = robot_state_[robot].covariance;

            Eigen::Vector2d robot_pos(pose.position.x(), pose.position.y());


            const auto& keyframes = robot_keyframes_[robot];
            if (keyframes.empty())
                continue;

            int robot_id = std::stoi(robot.substr(std::string("robot_").length()));
            DiscreteKey robot_key(Symbol('u', robot_id), keyframes.size());

            std::vector<double> values(keyframes.size(), 1e-6);

            if (keyframes.size() != 0)
            {
                size_t k = 0;
                for (const auto& [kf_id, kf] : keyframes)
                {
                    if (kf.keypoints_number < params_.min_keypoints_number)
                    {
                        values[k] = 1e-6;
                        k++;
                        continue;
                    }

                    Pose kf_global_pose = compose_global_pose(
                        robot_initial_poses_[robot],
                        kf.pose
                    );

                    Eigen::Vector2d kf_pos(
                        kf_global_pose.position.x(),
                        kf_global_pose.position.y()
                    );

                    // Distance
                    double dist = (kf_pos - robot_pos).norm();

                    if(dist < 2.0)
                    {
                        values[n_frontiers + k] = 1e-6;
                        k++;
                        continue;
                    }

                    // Mahalanobis distance
                    Eigen::Matrix<double,6,6> Sigma_j = kf.covariance;
                    
                    Eigen::Matrix3d Sigma_i_pose, Sigma_j_pose;

                    Sigma_i_pose <<
                        Sigma_i(0,0), Sigma_i(0,1), Sigma_i(0,5),
                        Sigma_i(1,0), Sigma_i(1,1), Sigma_i(1,5),
                        Sigma_i(5,0), Sigma_i(5,1), Sigma_i(5,5);

                    Sigma_j_pose <<
                        Sigma_j(0,0), Sigma_j(0,1), Sigma_j(0,5),
                        Sigma_j(1,0), Sigma_j(1,1), Sigma_j(1,5),
                        Sigma_j(5,0), Sigma_j(5,1), Sigma_j(5,5);

                    Eigen::Matrix3d Sigma_ij = Sigma_i_pose + Sigma_j_pose;
                    Sigma_ij += 1e-6 * Eigen::Matrix3d::Identity();


                    Eigen::Vector3d dx;

                    double yaw_i = std::atan2(
                        2.0 * (pose.orientation.w() * pose.orientation.z() +
                            pose.orientation.x() * pose.orientation.y()),
                        1.0 - 2.0 * (pose.orientation.y() * pose.orientation.y() +
                                    pose.orientation.z() * pose.orientation.z())
                    );


                    double yaw_j = std::atan2(
                        2.0 * (kf_global_pose.orientation.w() * kf_global_pose.orientation.z() +
                            kf_global_pose.orientation.x() * kf_global_pose.orientation.y()),
                        1.0 - 2.0 * (kf_global_pose.orientation.y() * kf_global_pose.orientation.y() +
                                    kf_global_pose.orientation.z() * kf_global_pose.orientation.z())
                    );

                    dx(0) = pose.position.x() - kf_global_pose.position.x();
                    dx(1) = pose.position.y() - kf_global_pose.position.y();
                    dx(2) = std::atan2(std::sin(yaw_i - yaw_j), std::cos(yaw_i - yaw_j));

                    // double mahalanobis = dx.transpose() * Sigma_ij.inverse() * dx;
                    double mahalanobis = dx.dot(Sigma_ij.ldlt().solve(dx));

                    double normalized_mahalanobis_dist = std::exp(-mahalanobis/params_.mahalanobis_sigma_scale);

                    // Score normalized
                    double score =
                        params_.w_cost_to_go * (1.0 - std::min(dist / max_keyframe_distance, 1.0)) +
                        params_.w_mahalanobis * (std::min(normalized_mahalanobis_dist, 1.0));

                    // Weighted sum
                    double sumw =
                        params_.w_cost_to_go +
                        params_.w_mahalanobis;

                    if (sumw > 0.0)
                        score /= sumw;

                    values[k] = std::max(score, 1e-6);
                    k++;
                }
            }

            // Add unary factor
            graph.add(DecisionTreeFactor(robot_key, values));

            // Optimize
            DiscreteValues result = graph.optimize();
            size_t assignment = result[robot_key.first];

            // Retrieve solution
            auto it = keyframes.begin();
            std::advance(it, assignment);

            Pose kf_global_pose = compose_global_pose(
                robot_initial_poses_[robot],
                it->second.pose
            );

            Task t;
            t.pose = kf_global_pose;
            t.oriented = true;

            tasks[robot] = t;
            active_hard_information_gain_tasks_[robot] = t;
        }

        // std::cout << std::endl;

        robot_last_planned_tasks_ = tasks;
        return tasks;
    }
}


