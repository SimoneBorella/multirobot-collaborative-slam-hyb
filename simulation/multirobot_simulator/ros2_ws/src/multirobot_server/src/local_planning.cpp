#include "local_planning.h"


namespace multirobot_slam
{
    LocalPlanning::LocalPlanning()
        : iter_count(0), costmap_received_(false), local_planning_thread_running_(false)
    {
    }

    LocalPlanning::LocalPlanning(LocalPlanningParams &params)
        : params_(params), iter_count(0), costmap_received_(false), local_planning_thread_running_(false)
    {
    }

    LocalPlanning::~LocalPlanning()
    {
        local_planning_thread_running_.store(false);
        if (local_planning_thread_.joinable())
        {
            local_planning_thread_.join();
        }
    }


    LocalPlanningParams LocalPlanning::params_from_yaml(std::string &params_path)
    {
        LocalPlanningParams p;

        try
        {
            YAML::Node config = YAML::LoadFile(params_path);

            if (config["local_planning_rate"])
                p.local_planning_rate = config["local_planning_rate"].as<double>();

            if (config["stop_dist_threshold"])
                p.stop_dist_threshold = config["stop_dist_threshold"].as<double>();

            if (config["dt"])
                p.dt = config["dt"].as<double>();

            if (config["predict_time"])
                p.predict_time = config["predict_time"].as<double>();

            if (config["max_vel_x"])
                p.max_vel_x = config["max_vel_x"].as<double>();

            if (config["min_vel_x"])
                p.min_vel_x = config["min_vel_x"].as<double>();

            if (config["max_vel_theta"])
                p.max_vel_theta = config["max_vel_theta"].as<double>();

            if (config["lookahead_dist"])
                p.lookahead_dist = config["lookahead_dist"].as<double>();

            if (config["start_prior_noise"])
                p.start_prior_noise = config["start_prior_noise"].as<std::vector<double>>();

            if (config["goal_prior_noise"])
                p.goal_prior_noise = config["goal_prior_noise"].as<std::vector<double>>();

            if (config["dynamic_noise"])
                p.dynamic_noise = config["dynamic_noise"].as<double>();

            if (config["inter_robot_noise"])
                p.inter_robot_noise = config["inter_robot_noise"].as<double>();

            if (config["obstacle_noise"])
                p.obstacle_noise = config["obstacle_noise"].as<double>();

            if (config["robot_dist_threshold"])
                p.robot_dist_threshold = config["robot_dist_threshold"].as<double>();

            if (config["obstacle_position_dist_threshold"])
                p.obstacle_position_dist_threshold = config["obstacle_position_dist_threshold"].as<double>();

            if (config["obstacle_orientation_dist_threshold"])
                p.obstacle_orientation_dist_threshold = config["obstacle_orientation_dist_threshold"].as<double>();
            
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
            std::cerr << "Using default parameters.\n";
        }

        return p;
    }


    Matrix LocalPlanning::make_dynamics_information(double sigma, double dt)
    {
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
        Eigen::Matrix2d Qc_inv = (1.0 / (sigma * sigma)) * I;

        Eigen::MatrixXd Qi_inv(4,4);

        Eigen::MatrixXd A = 12.0 * pow(dt, -3.0) * Qc_inv;
        Eigen::MatrixXd B = -6.0 * pow(dt, -2.0) * Qc_inv;
        Eigen::MatrixXd C = 4.0 / dt * Qc_inv;

        Qi_inv << A(0,0), A(0,1), B(0,0), B(0,1),
                A(1,0), A(1,1), B(1,0), B(1,1),
                B(0,0), B(0,1), C(0,0), C(0,1),
                B(1,0), B(1,1), C(1,0), C(1,1);

        return Qi_inv;
    }


    void LocalPlanning::init(LocalPlanningParams &params)
    {
        params_ = params;

        if (params_.lookahead_dist <= 0)
            params_.lookahead_dist = params_.predict_time * params_.max_vel_x;

        horizon_steps_ = static_cast<int>(std::floor(params_.predict_time / params_.dt));

        start_prior_noise_ = noiseModel::Diagonal::Sigmas(Vector4(params_.start_prior_noise[0], params_.start_prior_noise[1], params_.start_prior_noise[2], params_.start_prior_noise[3]));
        goal_prior_noise_ = noiseModel::Diagonal::Sigmas(Vector4(params_.goal_prior_noise[0], params_.goal_prior_noise[1], params_.goal_prior_noise[2], params_.goal_prior_noise[3]));

        dynamic_noise_ = gtsam::noiseModel::Gaussian::Information(make_dynamics_information(params_.dynamic_noise, params_.dt));
        inter_robot_noise_ = noiseModel::Diagonal::Sigmas(Vector1(params_.inter_robot_noise));
        obstacle_noise_ = noiseModel::Diagonal::Sigmas(Vector1(params_.obstacle_noise));
    }

    void LocalPlanning::set_robot_pose_callback(std::function<std::map<std::string, Pose>()> callback)
    {
        get_robot_poses_callback_ = std::move(callback);
    }

    void LocalPlanning::set_send_vel_cmds_callback(std::function<void(const std::map<std::string, VelCmd>&)> callback)
    {
        send_vel_cmds_callback_ = std::move(callback);
    }

    void LocalPlanning::update_global_paths(std::map<std::string, Path> global_paths)
    {
        std::lock_guard<std::mutex> lock(global_paths_mutex_);
        global_paths_ = global_paths;
    }

    void LocalPlanning::update_costmap(Map costmap)
    {
        std::lock_guard<std::mutex> lock(costmap_mutex_);
        costmap_ = costmap;
        costmap_received_ = true;
    }

    std::map<std::string, VelCmd> LocalPlanning::get_vel_cmds()
    {
        return vel_cmds_;
    }

    void LocalPlanning::start()
    {
        if (local_planning_thread_running_)
            return;

        local_planning_thread_running_.store(true);

        local_planning_thread_ = std::thread([this]()
                                     {
                auto period = std::chrono::milliseconds(
                    static_cast<int>(1000.0 / params_.local_planning_rate));

                auto next_time = std::chrono::steady_clock::now() + period;

                while (local_planning_thread_running_.load())
                {
                    local_planning();

                    std::this_thread::sleep_until(next_time);
                    next_time += period;
                } });
    }



    void LocalPlanning::local_planning()
    {
        // auto start = std::chrono::high_resolution_clock::now();

        if(!get_robot_poses_callback_)
            return;

        // std::lock_guard<std::mutex> lock(global_paths_mutex_);
        // std::lock_guard<std::mutex> cm_lock(costmap_mutex_);

        std::map<std::string, Pose> robot_poses = get_robot_poses_callback_();

        if(robot_poses.empty() || global_paths_.empty() || !costmap_received_)
            return;

        // Initialization of new robots
        for (const auto& [robot, pose] : robot_poses)
        {
            if (robot_ids_.find(robot) == robot_ids_.end())
            {
                std::string prefix = "robot_";
                robot_ids_[robot] = std::stoi(robot.substr(prefix.length()));

                last_vel_cmds_[robot] = VelCmd();
            }
        }

        std::vector<std::string> robots;

        for (const auto& [robot, _] : robot_poses)
        {
            if (global_paths_.find(robot) != global_paths_.end())
                robots.push_back(robot);
        }




        // Optimization problem
        NonlinearFactorGraph graph;
        Values estimates;

        std::vector<std::string> current_robots;


        for (std::string& robot : robots)
        {
            Eigen::Vector3d robot_position = robot_poses[robot].position;
            Eigen::Quaterniond robot_orientation = robot_poses[robot].orientation;

            robot_position.z() = 0.0;
            Eigen::Vector3d robot_goal_position = global_paths_[robot].poses.back().position;
            Eigen::Quaterniond robot_goal_orientation = global_paths_[robot].final_orientation;

            double robot_goal_position_dist = (robot_goal_position - robot_position).norm();

            Eigen::Quaterniond q_err = robot_goal_orientation * robot_orientation.inverse();
            double yaw_error = std::atan2(2*(q_err.w()*q_err.z() + q_err.x()*q_err.y()),
                                                            1 - 2*(q_err.y()*q_err.y() + q_err.z()*q_err.z()));

            if (robot_goal_position_dist < params_.stop_dist_threshold)
            {
                VelCmd stop_cmd;
                stop_cmd.linear.setZero();

                if(std::abs(yaw_error) < params_.obstacle_orientation_dist_threshold)
                {
                    stop_cmd.angular.setZero();
                }
                else
                {
                    double yaw_rate = 1.5 * yaw_error;

                    yaw_rate = std::clamp(
                        yaw_rate,
                        -params_.max_vel_theta,
                        params_.max_vel_theta
                    );

                    stop_cmd.angular.setZero();
                    stop_cmd.angular.z() = yaw_rate;
                }

                vel_cmds_[robot] = stop_cmd;
                last_vel_cmds_[robot] = stop_cmd;
            }
            else
            {
                current_robots.push_back(robot);
            }
        }




        // Add start priors
        std::map<std::string, Vector4> robot_start_state;

        for (std::string &robot : current_robots)
        {
            double state_x = robot_poses[robot].position.x();
            double state_y = robot_poses[robot].position.y();
            // double state_z = robot_poses[robot].position.z();

            const Eigen::Quaterniond &orientation = robot_poses[robot].orientation;
            double state_yaw = std::atan2(
                2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
                1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z())
            );

            double state_vel = last_vel_cmds_[robot].linear.x();

            double state_vel_x = state_vel * std::cos(state_yaw);
            double state_vel_y = state_vel * std::sin(state_yaw);

            Vector4 start_state(state_x, state_y, state_vel_x, state_vel_y);

            Symbol x('x', 1e6 * robot_ids_[robot]);

            graph.addPrior(x, start_state, start_prior_noise_);

            estimates.insert(x, start_state);

            robot_start_state[robot] = start_state;
        }






        // Add goal priors
        for (std::string &robot : current_robots)
        {
            Pose lookahead_pose = global_paths_[robot].poses.back();
            Pose closest_pose = global_paths_[robot].poses.front();

            double min_dist = std::numeric_limits<double>::infinity();
            bool lookahead_pose_found = false;

            for (int i = 0; i < static_cast<int>(global_paths_[robot].poses.size()); i++)
            {
                Eigen::Vector3d robot_position = robot_poses[robot].position;
                robot_position.z() = 0.0;

                double dist = (global_paths_[robot].poses[i].position - robot_position).norm();

                int rx = static_cast<int>((robot_position.x() - costmap_.origin_position.x()) / costmap_.resolution);
                int ry = static_cast<int>((robot_position.y() - costmap_.origin_position.y()) / costmap_.resolution);

                bool valid_pose = false;
                if (rx >= 0 && ry >= 0 && rx < costmap_.width && ry < costmap_.height)
                {
                    int idx = ry * costmap_.width + rx;
                    int8_t cost = costmap_.data[idx];
                    valid_pose = (cost == 0);
                }

                if (dist < params_.lookahead_dist && valid_pose)
                {
                    lookahead_pose = global_paths_[robot].poses[i];
                    lookahead_pose_found = true;
                }

                if (dist < min_dist && valid_pose)
                {
                    min_dist = dist;
                    closest_pose = global_paths_[robot].poses[i];
                }
            }

            if (!lookahead_pose_found)
            {
                lookahead_pose = closest_pose;
            }

            Vector4 goal_state(lookahead_pose.position.x(), lookahead_pose.position.y(), 0.0, 0.0);

            Symbol x('x', horizon_steps_ + 1e6 * robot_ids_[robot]);

            graph.addPrior(x, goal_state, goal_prior_noise_);
            estimates.insert(x, goal_state);
        }


        // Add dynamic factors, obstacle factors and inter robot factors
        for (int step = 0; step < horizon_steps_; step++)
        {
            std::map<std::string, Vector4> step_estimates;

            for (std::string &robot : current_robots)
            {
                // Add dynamic factors between poses
                Symbol first_x('x', step + 1e6 * robot_ids_[robot]);
                Symbol second_x('x', step + 1 + 1e6 * robot_ids_[robot]);

                graph.add(boost::make_shared<DynamicsFactor>(first_x, second_x, dynamic_noise_, params_.dt));

                if (step != (horizon_steps_ - 1))
                {
                    Vector4 estimate;

                    Vector4 start_state = robot_start_state[robot];

                    estimate[0] = start_state[0] + start_state[2] * params_.dt; // x + vx*dt
                    estimate[1] = start_state[1] + start_state[3] * params_.dt; // y + vy*dt
                    estimate[2] = start_state[2];                               // vx stays same
                    estimate[3] = start_state[3];                               // vy stays same

                    estimates.insert(second_x, estimate);
                    step_estimates[robot] = estimate;
                }

                // // Velocity limit factor
                // double velocity_noise_val = 0.06;
                // auto velocity_noise = noiseModel::Diagonal::Sigmas(Vector1(velocity_noise_val));
                // graph.add(boost::make_shared<VelocityLimitFactor>(second_x, velocity_noise, min_vel_x, max_vel_x));

                // // Angular velocity factor
                // double angular_velocity_noise_val = 0.06;
                // auto angular_velocity_noise = noiseModel::Diagonal::Sigmas(Vector1(angular_velocity_noise_val));
                // graph.add(boost::make_shared<AngularVelocityLimitFactor>(first_x, second_x, angular_velocity_noise, params_.dt, max_vel_theta));


                // Add obstacle factors
                graph.add(boost::make_shared<ObstacleFactor>(second_x, costmap_, obstacle_noise_, params_.obstacle_position_dist_threshold));
            }

            // Add inter robot factors
            if (step > 0 && step < horizon_steps_)
            {
                for (size_t i = 0; i < current_robots.size(); i++)
                {
                    for (size_t j = i + 1; j < current_robots.size(); j++)
                    {
                        std::string robot = current_robots[i];
                        std::string other_robot = current_robots[j];

                        // Get estimates and compute distance, if distance is under a threshold then
                        Vector4 estimate = step_estimates[robot];
                        Vector4 other_estimate = step_estimates[other_robot];

                        double dist = std::sqrt((estimate[0] - other_estimate[0]) * (estimate[0] - other_estimate[0]) + (estimate[1] - other_estimate[1]) * (estimate[1] - other_estimate[1]));

                        if (dist < params_.robot_dist_threshold)
                        {
                            Symbol first_x('x', step + 1e6 * robot_ids_[robot]);
                            Symbol second_x('x', step + 1e6 * robot_ids_[other_robot]);

                            graph.add(boost::make_shared<InterRobotFactor>(
                                first_x, second_x,
                                noiseModel::Gaussian::Covariance(inter_robot_noise_->covariance() * (step * params_.dt) * (step * params_.dt)),
                                params_.robot_dist_threshold));
                        }
                    }
                }
            }
        }


        Values result;

        try {
            LevenbergMarquardtOptimizer optimizer(graph, estimates);
            result = optimizer.optimize();
        } catch (const std::exception& e) {
            std::cout << "Local planning optimization failed." << std::endl;

            for (std::string &robot : current_robots)
            {
                VelCmd vel_cmd;
                vel_cmd.linear.x() = 0;
                vel_cmd.angular.z() = 0;

                vel_cmds_[robot] = vel_cmd;
                last_vel_cmds_[robot] = vel_cmd;
            }

            send_vel_cmds_callback_(vel_cmds_);

            return;
        }



        // if(local_planning_count%50 == 0)
        // {
        //     std::cout << "Saving graph " << local_planning_count << std::endl;
        //     Marginals graph_marginals(graph, result);
        //     save_graph(graph, result, graph_marginals, "./slam/local_planning/graphs/graph_" + std::to_string(local_planning_count) + ".txt");
        //     std::cout << "Graph saved" << std::endl;
        // }


        // for (std::string &robot : current_robots)
        // {
        //     Symbol next_x('x', 1 + 1e6 * robot_ids_[robot]);

        //     if (!result.exists(next_x))
        //     {
        //         std::cout << "No optimized state found for " << robot.c_str() << std::endl;
        //         continue;
        //     }

        //     Vector4 next_state = result.at<Vector4>(next_x);
        //     Symbol curr_x('x', 1e6 * robot_ids_[robot]);
        //     Vector4 curr_state = result.at<Vector4>(curr_x);

        //     // Compute displacement between timesteps
        //     double dx = next_state[0] - curr_state[0];
        //     double dy = next_state[1] - curr_state[1];

        //     // Forward velocity = displacement / dt
        //     double v = std::sqrt(dx * dx + dy * dy) / params_.dt;
        //     v = std::clamp(v, params_.min_vel_x, params_.max_vel_x);

        //     // Heading at current pose, from velocity components
        //     double heading_curr = std::atan2(curr_state[3], curr_state[2]);

        //     // Heading from displacement
        //     double heading_next = std::atan2(dy, dx);

        //     // Angular velocity
        //     double dtheta = heading_next - heading_curr;
        //     dtheta = std::atan2(std::sin(dtheta), std::cos(dtheta));
        //     double r = dtheta / params_.dt;
        //     r = std::clamp(r, -params_.max_vel_theta, params_.max_vel_theta);

        //     VelCmd vel_cmd;
        //     vel_cmd.linear.x() = v;
        //     vel_cmd.angular.z() = r;

        //     vel_cmds_[robot] = vel_cmd;
        //     last_vel_cmds_[robot] = vel_cmd;
        // }

        for (std::string &robot : current_robots)
        {
            Symbol x0('x', 1e6 * robot_ids_[robot]);
            Symbol x1('x', 1 + 1e6 * robot_ids_[robot]);

            if (!result.exists(x1)) continue;

            Vector4 s0 = result.at<Vector4>(x0);
            Vector4 s1 = result.at<Vector4>(x1);

            double dx = s1[0] - s0[0];
            double dy = s1[1] - s0[1];
            double dist = std::sqrt(dx*dx + dy*dy);

            double v = dist / params_.dt;
            v = std::clamp(v, params_.min_vel_x, params_.max_vel_x);

            const Eigen::Quaterniond& q = robot_poses[robot].orientation;
            double current_yaw = std::atan2(2.0*(q.w()*q.z() + q.x()*q.y()), 1.0 - 2.0*(q.y()*q.y() + q.z()*q.z()));

            double heading_target = (dist > 1e-4) ? std::atan2(dy, dx) : current_yaw;
            
            double dtheta = std::atan2(std::sin(heading_target - current_yaw), std::cos(heading_target - current_yaw));
            double r = std::clamp(dtheta / params_.dt, -params_.max_vel_theta, params_.max_vel_theta);

            VelCmd cmd;
            cmd.linear.x() = v;
            cmd.angular.z() = r;
            vel_cmds_[robot] = cmd;
            last_vel_cmds_[robot] = cmd;
        }

        send_vel_cmds_callback_(vel_cmds_);
        
        iter_count++;

        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> duration = end - start;

        // std::cout << "Local planning time: " << (duration.count() * 1000) << " ms (" << 1/duration.count() << " Hz)" << std::endl;
    }   
}