#include "local_planning.h"

LocalPlanning::LocalPlanning(rclcpp::Node::SharedPtr node, std::shared_ptr<Mapping> mapping, std::shared_ptr<DecisionGlobalPlanning> decision_global_planning)
{
    this->node = node;

    robots = node->get_parameter("robots").as_string_array();

    all_frame = node->get_parameter("all_frame").as_string();
    map_frame = node->get_parameter("map_frame").as_string();
    odom_frame = node->get_parameter("odom_frame").as_string();
    base_frame = node->get_parameter("base_frame").as_string();

    local_planning_rate = node->get_parameter("local_planning_rate").as_double();
    local_planner_type = node->get_parameter("local_planner_type").as_string();

    // Initialize tf buffer and listener
    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    robot_twist = std::make_shared<std::map<std::string, geometry_msgs::msg::Twist>>();
    robot_twist_mutex = std::make_shared<std::mutex>();

    // Initialize publishers and subscribers for each robot
    for (const auto &robot : robots)
    {
        robot_cmd_publishers[robot] = node->create_publisher<geometry_msgs::msg::Twist>("/" + robot + "/cmd_vel", 10);

        robot_odom_subscriptions[robot] = node->create_subscription<nav_msgs::msg::Odometry>(
            "/" + robot + "/odom", 10,
            [this, robot](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                // std::unique_lock<std::mutex> lock(*robot_twist_mutex);
                (*robot_twist)[robot] = msg->twist.twist;
            });
    }

    // Initialize costmap grid
    auto costmap_grid_shared = mapping->get_costmap_grid_with_mutex();

    // Initialize robot paths
    auto robot_paths_shared = decision_global_planning->get_robot_paths_with_mutex();

    if (local_planner_type == "dwa")
    {
        local_planner = std::make_unique<DWA>(node, costmap_grid_shared, robot_paths_shared, std::make_pair(robot_twist, robot_twist_mutex));
    }
    else if (local_planner_type == "fgp")
    {
        local_planner = std::make_unique<FGP>(node, costmap_grid_shared, robot_paths_shared, std::make_pair(robot_twist, robot_twist_mutex));
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Unknown local planner type: %s", local_planner_type.c_str());
        return;
    }

    local_planning_count = 0;

    // Initialize threads
    local_planning_running = false;
}

LocalPlanning::~LocalPlanning()
{
    local_planning_running = false;
    if (local_planning_thread && local_planning_thread->joinable())
    {
        local_planning_thread->join();
    }
}

void LocalPlanning::startLocalPlanning()
{
    local_planning_running = true;
    local_planning_thread = std::make_unique<std::thread>([this]()
                                                          {
        rclcpp::Rate rate(local_planning_rate);
        while (rclcpp::ok() && local_planning_running.load()) {
            local_planning();
            rate.sleep();
        } });
    RCLCPP_INFO_STREAM(node->get_logger(), "Local planning thread started.");
}

void LocalPlanning::local_planning()
{
#ifdef LOCAL_PLANNING_DEBUG
    auto start_local_planning = std::chrono::high_resolution_clock::now();
#endif

    // Get robots poses
    std::map<std::string, geometry_msgs::msg::TransformStamped> robot_transforms;

    for (const auto &robot : robots)
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer->lookupTransform(all_frame, robot + "/" + base_frame, tf2::TimePointZero);
            robot_transforms[robot] = transform;
        }
        catch (const tf2::TransformException &ex)
        {
            // RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }

    // Local path planning
    std::map<std::string, geometry_msgs::msg::Twist> robot_commands = local_planner->computeCommands(robot_transforms, local_planning_count);

    for (const auto &[robot, cmd] : robot_commands)
    {
        robot_cmd_publishers[robot]->publish(cmd);
    }

    local_planning_count++;

#ifdef LOCAL_PLANNING_DEBUG
    auto end_local_planning = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_local_planning = end_local_planning - start_local_planning;
    std::cout << "Local planning time: " << (duration_local_planning.count() * 1000) << " ms" << std::endl;
    std::cout << std::endl;
#endif
}











































DWA::DWA(rclcpp::Node::SharedPtr node, std::pair<std::shared_ptr<nav_msgs::msg::OccupancyGrid>, std::shared_ptr<std::shared_mutex>> costmap_grid_shared, std::pair<std::shared_ptr<std::map<std::string, nav_msgs::msg::Path>>, std::shared_ptr<std::shared_mutex>> robot_paths_shared, std::pair<std::shared_ptr<std::map<std::string, geometry_msgs::msg::Twist>>, std::shared_ptr<std::mutex>> robot_twist_shared)
{
    this->node = node;
    this->robot_twist = robot_twist_shared.first;
    this->robot_twist_mutex = robot_twist_shared.second;

    std::vector<std::string> robots = node->get_parameter("robots").as_string_array();

    keep_internal_state = node->get_parameter("keep_internal_state").as_bool();

    stop_dist_threshold = node->get_parameter("stop_dist_threshold").as_double();
    dt = node->get_parameter("dt").as_double();
    predict_time = node->get_parameter("predict_time").as_double();

    max_vel_x = node->get_parameter("max_vel_x").as_double();
    min_vel_x = node->get_parameter("min_vel_x").as_double();
    max_vel_theta = node->get_parameter("max_vel_theta").as_double();

    max_acc_x = node->get_parameter("max_acc_x").as_double();
    max_acc_theta = node->get_parameter("max_acc_theta").as_double();
    lookahead_dist = node->get_parameter("lookahead_dist").as_double();
    robot_dist_threshold = node->get_parameter("robot_dist_threshold").as_double();
    obstacle_dist_threshold = node->get_parameter("obstacle_dist_threshold").as_double();

    w_progress_score = node->get_parameter("w_progress_score").as_double();
    w_heading_score = node->get_parameter("w_heading_score").as_double();
    w_robot_collision_score = node->get_parameter("w_robot_collision_score").as_double();
    w_costmap_score = node->get_parameter("w_costmap_score").as_double();

    // Initialize costmap grid
    costmap_grid = costmap_grid_shared.first;
    costmap_grid_mutex = costmap_grid_shared.second;

    // Initialize robot paths
    robot_paths = robot_paths_shared.first;
    robot_paths_mutex = robot_paths_shared.second;

    // Initialize last command velocities
    for(auto& robot : robots)
    {
        last_cmd_vel[robot] = geometry_msgs::msg::Twist();
    }
}

std::map<std::string, geometry_msgs::msg::Twist> DWA::computeCommands(std::map<std::string, geometry_msgs::msg::TransformStamped> &robot_transforms, int local_planning_count)
{
    std::map<std::string, geometry_msgs::msg::Twist> robot_commands;

    std::map<std::string, Pose> robot_poses;

    for (auto &[robot, transform] : robot_transforms)
    {
        robot_poses[robot] = transformToPose(transform);
    }
    
    std::map<std::string, nav_msgs::msg::Path> paths;
    {
        // std::shared_lock<std::shared_mutex> lock(*robot_paths_mutex);
        paths = *robot_paths;
    }


    // std::shared_lock lock(*costmap_grid_mutex);
    
    for (const auto &[robot, global_path] : paths)
    {
        if (robot_transforms.find(robot) == robot_transforms.end())
            continue;

        geometry_msgs::msg::Twist cmd = computeDWACommand(robot, robot_poses, global_path);

        robot_commands[robot] = cmd;
    }
    

    return robot_commands;
}


geometry_msgs::msg::Twist DWA::computeDWACommand(const std::string &robot, const std::map<std::string, Pose> &robot_poses, const nav_msgs::msg::Path &global_path)
{
    {
        double dx = global_path.poses[global_path.poses.size() - 1].pose.position.x - robot_poses.at(robot).x;
        double dy = global_path.poses[global_path.poses.size() - 1].pose.position.y - robot_poses.at(robot).y;
        double goal_dist = std::sqrt(dx * dx + dy * dy);

        if (goal_dist < stop_dist_threshold)
        {
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;

            return stop_cmd;
        }
    }

    double best_score = -std::numeric_limits<double>::infinity();
    geometry_msgs::msg::Twist best_cmd;

    geometry_msgs::msg::Twist twist;

    if (!keep_internal_state)
    {
        // std::unique_lock<std::mutex> lock(*robot_twist_mutex);
        twist = (*robot_twist).at(robot);
    }
    else
    {
        twist = last_cmd_vel[robot];
    }

    double actual_v = twist.linear.x;
    double actual_w = twist.angular.z;

    for (double v = actual_v - max_acc_x * dt; v <= actual_v + max_acc_x * dt; v += max_acc_x * dt)
    {
        for (double w = actual_w - max_acc_theta * dt; w <= actual_w + max_acc_theta * dt; w += max_acc_theta * dt)
        {

            Pose pose = robot_poses.at(robot);
            Pose predict_pose = simulateTrajectory(pose, v, w);

            double score = getScoreTrajectory(pose, predict_pose, global_path, robot_poses, robot);

            if (score > best_score)
            {
                best_score = score;
                best_cmd.linear.x = std::clamp(v, -min_vel_x, max_vel_x);
                best_cmd.angular.z = std::clamp(w, -max_vel_theta, max_vel_theta);
            }
        }
    }

    last_cmd_vel[robot] = best_cmd;
    
    return best_cmd;
}

Pose DWA::simulateTrajectory(const Pose &pose, double v, double w)
{
    Pose predict_pose;

    if (fabs(w) > 1e-6)
    {
        // Arc motion
        predict_pose.x = pose.x + (v / w) * (sin(pose.theta + w * predict_time) - sin(pose.theta));
        predict_pose.y = pose.y - (v / w) * (cos(pose.theta + w * predict_time) - cos(pose.theta));
    }
    else
    {
        // Straight line motion
        predict_pose.x = pose.x + v * cos(pose.theta) * predict_time;
        predict_pose.y = pose.y + v * sin(pose.theta) * predict_time;
    }

    predict_pose.theta = pose.theta + w * predict_time;

    // Optional: normalize theta to [-pi, pi]
    while (predict_pose.theta > M_PI)
        predict_pose.theta -= 2.0 * M_PI;
    while (predict_pose.theta < -M_PI)
        predict_pose.theta += 2.0 * M_PI;

    return predict_pose;
}

double DWA::getScoreTrajectory(const Pose pose, const Pose predict_pose, const nav_msgs::msg::Path &global_path, const std::map<std::string, Pose> &robot_poses, const std::string &robot)
{
    if (global_path.poses.empty())
        return 0.0;

    // Progress score to lookahead pose
    geometry_msgs::msg::PoseStamped lookahead_pose = global_path.poses[global_path.poses.size() - 1];
    geometry_msgs::msg::PoseStamped heading_pose = global_path.poses[0];
    double min_dist = std::numeric_limits<double>::infinity();

    for (int i = 0; i < static_cast<int>(global_path.poses.size()); i++)
    {
        double dx = global_path.poses[i].pose.position.x - pose.x;
        double dy = global_path.poses[i].pose.position.y - pose.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < lookahead_dist)
        {
            lookahead_pose = global_path.poses[i];
        }

        if (dist < min_dist)
        {
            min_dist = dist;
            heading_pose = global_path.poses[i];
        }
    }

    Pose target_pose;
    target_pose.x = lookahead_pose.pose.position.x;
    target_pose.y = lookahead_pose.pose.position.y;

    double pose_target_dist = std::sqrt((pose.x - target_pose.x) * (pose.x - target_pose.x) + (pose.y - target_pose.y) * (pose.y - target_pose.y));
    double predict_pose_target_dist = std::sqrt((predict_pose.x - target_pose.x) * (predict_pose.x - target_pose.x) + (predict_pose.y - target_pose.y) * (predict_pose.y - target_pose.y));

    double progress_dist = pose_target_dist - predict_pose_target_dist;

    double progress_score = progress_dist / pose_target_dist;



    // Heading score

    // double target_dx = lookahead_pose.pose.position.x - predict_pose.x;
    // double target_dy = lookahead_pose.pose.position.y - predict_pose.y;
    // double target_theta = std::atan2(target_dy, target_dx);

    double target_roll, target_pitch, target_theta;

    tf2::Quaternion quat_tf;
    tf2::fromMsg(heading_pose.pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(target_roll, target_pitch, target_theta);

    double heading_diff = target_theta - predict_pose.theta;
    while (heading_diff > M_PI) heading_diff -= 2*M_PI;
    while (heading_diff < -M_PI) heading_diff += 2*M_PI;

    double heading_score = std::max(0.0, 1.0 - (std::abs(heading_diff) / M_PI));


    // Robot collision score

    double robot_collision_score = 0.0;
    double min_robot_dist = std::numeric_limits<double>::infinity();

    for (const auto &[other_robot, other_pose] : robot_poses)
    {
        if (other_robot == robot)
            continue;

        double dx = other_pose.x - predict_pose.x;
        double dy = other_pose.y - predict_pose.y;
        double robot_dist = std::sqrt(dx * dx + dy * dy);

        if (robot_dist < min_robot_dist)
        {
            min_robot_dist = robot_dist;
        }
    }

    if (min_robot_dist < robot_dist_threshold)
    {
        robot_collision_score = -(1 - min_robot_dist/robot_dist_threshold);
    }

    // Costmap score
    double costmap_score = 0.0;

    int rx = static_cast<int>((predict_pose.x - costmap_grid->info.origin.position.x) / costmap_grid->info.resolution);
    int ry = static_cast<int>((predict_pose.y - costmap_grid->info.origin.position.y) / costmap_grid->info.resolution);

    if (!(rx < 0 || ry < 0 || rx >= (int)costmap_grid->info.width || ry >= (int)costmap_grid->info.height))
    {
        // Search kernel around the cell
        int kernel = std::ceil(obstacle_dist_threshold / costmap_grid->info.resolution);
        double min_dist = std::numeric_limits<double>::infinity();

        for (int dy = -kernel; dy <= kernel; ++dy)
        {
            for (int dx = -kernel; dx <= kernel; ++dx)
            {
                int nx = rx + dx;
                int ny = ry + dy;

                if (nx < 0 || ny < 0 ||
                    nx >= (int)costmap_grid->info.width ||
                    ny >= (int)costmap_grid->info.height)
                    continue;

                double distance = std::sqrt(dx*dx + dy*dy) * costmap_grid->info.resolution;
                if (distance > obstacle_dist_threshold)
                    continue;

                int cost = static_cast<int>(costmap_grid->data[ny * costmap_grid->info.width + nx]);
                if (cost < 0) continue;

                if (cost >= 50 && distance < min_dist)
                    min_dist = distance;
            }
        }

        if (!std::isinf(min_dist))
            costmap_score = -(1 - min_dist / obstacle_dist_threshold);
    }


    return w_progress_score * progress_score + w_heading_score * heading_score + w_robot_collision_score * robot_collision_score + w_costmap_score * costmap_score;
}



Pose DWA::transformToPose(const geometry_msgs::msg::TransformStamped &tf)
{
    Pose p;
    p.x = tf.transform.translation.x;
    p.y = tf.transform.translation.y;
    tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    p.theta = yaw;
    return p;
}





































FGP::FGP(rclcpp::Node::SharedPtr node, std::pair<std::shared_ptr<nav_msgs::msg::OccupancyGrid>, std::shared_ptr<std::shared_mutex>> costmap_grid_shared, std::pair<std::shared_ptr<std::map<std::string, nav_msgs::msg::Path>>, std::shared_ptr<std::shared_mutex>> robot_paths_shared, std::pair<std::shared_ptr<std::map<std::string, geometry_msgs::msg::Twist>>, std::shared_ptr<std::mutex>> robot_twist_shared)
{
    this->node = node;
    this->robot_twist = robot_twist_shared.first;
    this->robot_twist_mutex = robot_twist_shared.second;

    robots = node->get_parameter("robots").as_string_array();

    keep_internal_state = node->get_parameter("keep_internal_state").as_bool();

    stop_dist_threshold = node->get_parameter("stop_dist_threshold").as_double();
    dt = node->get_parameter("dt").as_double();
    predict_time = node->get_parameter("predict_time").as_double();
    max_vel_x = node->get_parameter("max_vel_x").as_double();
    min_vel_x = node->get_parameter("min_vel_x").as_double();
    max_vel_theta = node->get_parameter("max_vel_theta").as_double();

    lookahead_dist = node->get_parameter("lookahead_dist").as_double();

    if (lookahead_dist <= 0)
    {
        lookahead_dist = predict_time * max_vel_x;
    }

    horizon_steps = static_cast<int>(std::floor(predict_time / dt));

    std::vector<double> start_prior_noise_vec = node->get_parameter("start_prior_noise").as_double_array();
    std::vector<double> goal_prior_noise_vec = node->get_parameter("goal_prior_noise").as_double_array();
    double dynamic_noise_val = node->get_parameter("dynamic_noise").as_double();
    double inter_robot_noise_val = node->get_parameter("inter_robot_noise").as_double();
    double obstacle_noise_val = node->get_parameter("obstacle_noise").as_double();

    start_prior_noise = noiseModel::Diagonal::Sigmas(Vector4(start_prior_noise_vec[0], start_prior_noise_vec[1], start_prior_noise_vec[2], start_prior_noise_vec[3]));
    goal_prior_noise = noiseModel::Diagonal::Sigmas(Vector4(goal_prior_noise_vec[0], goal_prior_noise_vec[1], goal_prior_noise_vec[2], goal_prior_noise_vec[3]));

    dynamic_noise = gtsam::noiseModel::Gaussian::Information(makeDynamicsInformation(dynamic_noise_val, dt));
    inter_robot_noise = noiseModel::Diagonal::Sigmas(Vector1(inter_robot_noise_val));
    obstacle_noise = noiseModel::Diagonal::Sigmas(Vector1(obstacle_noise_val));






    robot_dist_threshold = node->get_parameter("robot_dist_threshold").as_double();
    obstacle_dist_threshold = node->get_parameter("obstacle_dist_threshold").as_double();

    for (std::string &robot : robots)
    {
        PlanningRobotMetadata robot_metadata;

        robot_metadata.robot_name = robot;

        std::string prefix = "robot_";
        robot_metadata.robot_id = std::stoi(robot.substr(prefix.length()));

        metadata[robot] = robot_metadata;
    }

    // Initialize costmap grid
    costmap_grid = costmap_grid_shared.first;
    costmap_grid_mutex = costmap_grid_shared.second;

    // Initialize robot paths
    robot_paths = robot_paths_shared.first;
    robot_paths_mutex = robot_paths_shared.second;


    // Initialize last command velocities
    for(auto& robot : robots)
    {
        last_cmd_vel[robot] = geometry_msgs::msg::Twist();
    }
}









std::map<std::string, geometry_msgs::msg::Twist> FGP::computeCommands(std::map<std::string, geometry_msgs::msg::TransformStamped> &robot_transforms, int local_planning_count)
{
    std::map<std::string, geometry_msgs::msg::Twist> robot_commands;

    NonlinearFactorGraph graph;
    Values estimates;

    // Find set of current robots
    std::vector<std::string> current_robots;

    std::map<std::string, nav_msgs::msg::Path> paths;
    {
        // std::shared_lock<std::shared_mutex> lock(*robot_paths_mutex);
        paths = *robot_paths;
    }
    


    // If distance from goal under a threshold set cmd as zero, else add to current robots
    for (auto &[robot, robot_path] : paths)
    {
        if (robot_transforms.find(robot) == robot_transforms.end())
            continue;

        double dx = robot_path.poses[robot_path.poses.size() - 1].pose.position.x - robot_transforms[robot].transform.translation.x;
        double dy = robot_path.poses[robot_path.poses.size() - 1].pose.position.y - robot_transforms[robot].transform.translation.y;
        double goal_dist = std::sqrt(dx * dx + dy * dy);

        if (goal_dist < stop_dist_threshold)
        {
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;

            robot_commands[robot] = stop_cmd;
        }

        current_robots.push_back(robot);
    }




    // Add start priors

    std::map<std::string, Vector4> robot_start_state;

    for (std::string &robot : current_robots)
    {
        double state_x = robot_transforms[robot].transform.translation.x;
        double state_y = robot_transforms[robot].transform.translation.y;
        // double state_z = robot_transforms[robot].transform.translation.z;

        double state_roll, state_pitch, state_yaw;
        tf2::Quaternion quat_tf;
        tf2::fromMsg(robot_transforms[robot].transform.rotation, quat_tf);
        tf2::Matrix3x3(quat_tf).getRPY(state_roll, state_pitch, state_yaw);

        geometry_msgs::msg::Twist twist;
        if (!keep_internal_state)
        {
            // std::unique_lock<std::mutex> lock(*robot_twist_mutex);
            twist = (*robot_twist).at(robot);
        }
        else
        {
            twist = last_cmd_vel[robot];
        }

        double state_vel = twist.linear.x;
        double state_vel_x = state_vel * std::cos(state_yaw);
        double state_vel_y = state_vel * std::sin(state_yaw);

        Vector4 start_state(state_x, state_y, state_vel_x, state_vel_y);

        Symbol x('x', 1e6 * metadata[robot].robot_id);

        graph.addPrior(x, start_state, start_prior_noise);

        estimates.insert(x, start_state);

        robot_start_state[robot] = start_state;
    }


    // Add goal priors
    for (std::string &robot : current_robots)
    {
        geometry_msgs::msg::PoseStamped lookahead_pose = paths[robot].poses[paths[robot].poses.size() - 1];
        geometry_msgs::msg::PoseStamped closest_pose = paths[robot].poses[0];
        double min_dist = std::numeric_limits<double>::infinity();
        bool lookahead_pose_found = false;

        for (int i = 0; i < static_cast<int>(paths[robot].poses.size()); i++)
        {
            double dx = paths[robot].poses[i].pose.position.x - robot_transforms[robot].transform.translation.x;
            double dy = paths[robot].poses[i].pose.position.y - robot_transforms[robot].transform.translation.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            int rx = static_cast<int>((paths[robot].poses[i].pose.position.x - costmap_grid->info.origin.position.x) / costmap_grid->info.resolution);
            int ry = static_cast<int>((paths[robot].poses[i].pose.position.y - costmap_grid->info.origin.position.y) / costmap_grid->info.resolution);

            bool valid_pose = false;
            if (rx >= 0 && ry >= 0 && rx < (int)costmap_grid->info.width && ry < (int)costmap_grid->info.height)
            {
                int idx = ry * costmap_grid->info.width + rx;
                int8_t cost = costmap_grid->data[idx];
                valid_pose = (cost == 0);
            }

            if (dist < lookahead_dist && valid_pose)
            {
                lookahead_pose = paths[robot].poses[i];
                lookahead_pose_found = true;
            }

            if (dist < min_dist && valid_pose)
            {
                min_dist = dist;
                closest_pose = paths[robot].poses[i];
            }
        }

        if (!lookahead_pose_found)
        {
            lookahead_pose = closest_pose;
        }

        Vector4 goal_state(lookahead_pose.pose.position.x, lookahead_pose.pose.position.y, 0.0, 0.0);

        Symbol x('x', horizon_steps + 1e6 * metadata[robot].robot_id);

        graph.addPrior(x, goal_state, goal_prior_noise);
        estimates.insert(x, goal_state);
    }



    // Add dynamic factors, obstacle factors and inter robot factors
    for (int step = 0; step < horizon_steps; step++)
    {
        std::map<std::string, Vector4> step_estimates;

        for (std::string &robot : current_robots)
        {
            // Add dynamic factors between poses
            Symbol first_x('x', step + 1e6 * metadata[robot].robot_id);
            Symbol second_x('x', step + 1 + 1e6 * metadata[robot].robot_id);

            graph.add(boost::make_shared<DynamicsFactor>(first_x, second_x, dynamic_noise, dt));

            if (step != (horizon_steps - 1))
            {
                Vector4 estimate;

                Vector4 start_state = robot_start_state[robot];

                estimate[0] = start_state[0] + start_state[2] * dt; // x + vx*dt
                estimate[1] = start_state[1] + start_state[3] * dt; // y + vy*dt
                estimate[2] = start_state(2);                       // vx stays same
                estimate[3] = start_state(3);                       // vy stays same

                estimates.insert(second_x, estimate);
                step_estimates[robot] = estimate;
            }

            // // Velocity limit factor
            // double velocity_noise_val = 0.1;
            // auto velocity_noise = noiseModel::Diagonal::Sigmas(Vector1(velocity_noise_val));
            // graph.add(boost::make_shared<VelocityLimitFactor>(second_x, velocity_noise, min_vel_x, max_vel_x));

            // // Angular velocity factor
            // double angular_velocity_noise_val = 0.1;
            // auto angular_velocity_noise = noiseModel::Diagonal::Sigmas(Vector1(angular_velocity_noise_val));
            // graph.add(boost::make_shared<AngularVelocityLimitFactor>(first_x, second_x, angular_velocity_noise, dt, max_vel_theta));


            // Add obstacle factors
            graph.add(boost::make_shared<ObstacleFactor>(second_x, costmap_grid, obstacle_noise, obstacle_dist_threshold));
        }

        // Add inter robot factors
        if (step > 0 && step < horizon_steps)
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

                    if (dist < robot_dist_threshold)
                    {
                        Symbol first_x('x', step + 1e6 * metadata[robot].robot_id);
                        Symbol second_x('x', step + 1e6 * metadata[other_robot].robot_id);

                        graph.add(boost::make_shared<InterRobotFactor>(
                            first_x, second_x,
                            noiseModel::Gaussian::Covariance(inter_robot_noise->covariance() * (step * dt) * (step * dt)),
                            robot_dist_threshold));
                    }
                }
            }
        }
    }




    LevenbergMarquardtOptimizer optimizer(graph, estimates);
    Values result = optimizer.optimize();



#ifdef FGP_SAVE_GRAPH_DEBUG
    if(local_planning_count%50 == 0)
    {
        std::cout << "Saving graph " << local_planning_count << std::endl;
        Marginals graph_marginals(graph, result);
        save_graph(graph, result, graph_marginals, "./slam/local_planning/graphs/graph_" + std::to_string(local_planning_count) + ".txt");
        std::cout << "Graph saved" << std::endl;
    }
#endif


    for (std::string &robot : current_robots)
    {
        Symbol next_x('x', 1 + 1e6 * metadata[robot].robot_id);

        if (!result.exists(next_x))
        {
            RCLCPP_WARN(node->get_logger(), "No optimized state found for %s", robot.c_str());
            continue;
        }

        Vector4 next_state = result.at<Vector4>(next_x);
        Symbol curr_x('x', 1e6 * metadata[robot].robot_id);
        Vector4 curr_state = result.at<Vector4>(curr_x);

        // Compute displacement between timesteps
        double dx = next_state[0] - curr_state[0];
        double dy = next_state[1] - curr_state[1];

        // Forward velocity = displacement / dt
        double v = std::sqrt(dx * dx + dy * dy) / dt;
        v = std::clamp(v, min_vel_x, max_vel_x);

        // Heading at current pose (from velocity or direction of motion)
        double heading_curr = std::atan2(curr_state[3], curr_state[2]);

        // Heading from displacement
        double heading_next = std::atan2(dy, dx);

        // Angular velocity
        double dtheta = heading_next - heading_curr;
        dtheta = std::atan2(std::sin(dtheta), std::cos(dtheta));
        double r = dtheta / dt;
        r = std::clamp(r, -max_vel_theta, max_vel_theta);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = r;

        // Update only if no prior command
        if (robot_commands.find(robot) == robot_commands.end()) {
            last_cmd_vel[robot] = cmd;
            robot_commands[robot] = cmd;
        }
    }


    return robot_commands;
}

















void FGP::save_graph(NonlinearFactorGraph graph, Values estimates, Marginals marginals, const std::string &filename)
{
    std::ofstream graph_file(filename);

    if (!graph_file.is_open())
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Could not open file: " << filename);
        return;
    }

    for(std::string& robot : robots)
    {
        // STATE xn Pose2(x, y, vx, vy)
        for (int var = 0; var <= horizon_steps; var++)
        {
            Symbol x('x', 1e6 * metadata[robot].robot_id + var);

            if (estimates.exists(x))
            {
                Vector4 estimate = estimates.at<Vector4>(x);
                auto covariance = marginals.marginalCovariance(x);

                graph_file << "STATE " << x << " " << estimate[0] << " " << estimate[1] << " " << estimate[2] << " " << estimate[3];
                for (int i = 0; i < 4; i++)
                    for (int j = 0; j < 4; j++)
                        graph_file << " " << covariance(i, j);
                graph_file << "\n";
            }
        }
    }


    for (const auto &factor : graph)
    {
        // PRIORSTATE xn x y vx vy
        if (auto prior = boost::dynamic_pointer_cast<PriorFactor<Vector4>>(factor))
        {
            Symbol key = prior->keys().at(0);
            Vector4 prior_state = prior->prior();
            Matrix4 covariance = boost::dynamic_pointer_cast<noiseModel::Gaussian>(prior->noiseModel())->covariance();

            graph_file << "PRIORSTATE " << key << " " << prior_state[0] << " " << prior_state[1] << " " << prior_state[2] << " " << prior_state[3];
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    graph_file << " " << covariance(i, j);
            graph_file << "\n";
        }


        // DYNAMICFACTOR x1 x2
        if (auto dynamicFactor = boost::dynamic_pointer_cast<DynamicsFactor>(factor))
        {
            Symbol x1 = dynamicFactor->keys().at(0);
            Symbol x2 = dynamicFactor->keys().at(1);

            graph_file << "DYNAMICFACTOR " << x1 << " " << x2;
            graph_file << "\n";
        }

        // INTERROBOTFACTOR x1 x2
        if (auto interRobotFactor = boost::dynamic_pointer_cast<InterRobotFactor>(factor))
        {
            Symbol x1 = interRobotFactor->keys().at(0);
            Symbol x2 = interRobotFactor->keys().at(1);

            graph_file << "INTERROBOTFACTOR " << x1 << " " << x2;
            graph_file << "\n";
        }

    }
}

