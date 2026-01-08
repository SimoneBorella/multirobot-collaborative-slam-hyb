#include "decision_global_planning.h"

DecisionGlobalPlanning::DecisionGlobalPlanning(rclcpp::Node::SharedPtr node, std::shared_ptr<Mapping> mapping)
{
    this->node = node;
    this->mapping = mapping;

    robots = node->get_parameter("robots").as_string_array();

    std::vector<double> initial_poses_array = node->get_parameter("initial_poses").as_double_array();

    for (size_t i = 0; i < robots.size(); ++i)
    {
        std::string& robot = robots[i];
        initial_positions[robot] = std::make_pair(initial_poses_array[(i*3)], initial_poses_array[(i*3)+1]);
    }

    all_frame = node->get_parameter("all_frame").as_string();
    map_frame = node->get_parameter("map_frame").as_string();
    odom_frame = node->get_parameter("odom_frame").as_string();
    base_frame = node->get_parameter("base_frame").as_string();

    decision_global_planning_rate = node->get_parameter("decision_global_planning_rate").as_double();
    alpha_distance = node->get_parameter("alpha_distance").as_double();
    beta_dimension = node->get_parameter("beta_dimension").as_double();
    conflict_penalty = node->get_parameter("conflict_penalty").as_double();
    global_planner_type = node->get_parameter("global_planner_type").as_string();
    heuristic_type = node->get_parameter("heuristic_type").as_string();


    // Initialize tf buffer and listener
    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Initialize publishers
    for (const auto& robot : robots) {
        std::string robot_path_topic = "/" + robot + "/global_path";
        robot_path_publishers[robot] = node->create_publisher<nav_msgs::msg::Path>(robot_path_topic, 10);
    }

    // Initialize robot paths
    robot_paths_mutex = std::make_shared<std::shared_mutex>();
    robot_paths = std::make_shared<std::map<std::string, nav_msgs::msg::Path>>();

    // Initialize frontiers
    auto [frontiers_, frontiers_mutex_] = mapping->get_frontiers_with_mutex();
    frontiers = frontiers_;
    frontiers_mutex = frontiers_mutex_;

    // Initialize costmap grid
    auto [costmap_grid_, costmap_grid_mutex_] = mapping->get_costmap_grid_with_mutex();
    costmap_grid = costmap_grid_;
    costmap_grid_mutex = costmap_grid_mutex_;



    // Initialize path planner directions
    directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,-1},{1,-1},{-1,1}};
    directions_cost = {1.0, 1.0, 1.0, 1.0, std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)};


    // Initialize threads
    decision_global_planning_running = false;
}

DecisionGlobalPlanning::~DecisionGlobalPlanning()
{
    decision_global_planning_running = false;
    if (decision_global_planning_thread && decision_global_planning_thread->joinable()) {
        decision_global_planning_thread->join();
    }
}



void DecisionGlobalPlanning::startDecisionGlobalPlanning()
{
    decision_global_planning_running = true;
    decision_global_planning_thread = std::make_unique<std::thread>([this]() {
        rclcpp::Rate rate(decision_global_planning_rate);
        while (rclcpp::ok() && decision_global_planning_running.load()) {
            decision_global_planning();
            rate.sleep();
        }
    });
    RCLCPP_INFO_STREAM(node->get_logger(), "Frontier detection thread started.");
}




std::map<std::string, std::pair<double, double>> DecisionGlobalPlanning::decision_making_frontier_assignment(std::vector<std::pair<std::pair<double, double>, int>>& frontier_centroids, std::map<std::string, geometry_msgs::msg::TransformStamped>& robot_transforms)
{
    std::map<std::string, std::pair<double, double>> robot_frontier_assignments;

    // Params

    DiscreteFactorGraph graph;
    
    // Get robots poses
    std::vector<std::string> index_to_robot;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    for(const auto& [robot, transform] : robot_transforms)
    {
        index_to_robot.push_back(robot);
        transforms.push_back(transform);
    }

    if (transforms.empty() || frontier_centroids.empty()) {
        return robot_frontier_assignments;
    }

    size_t num_frontiers = frontier_centroids.size();
    size_t num_robots = transforms.size();

    // Create discrete keys for each robot
    std::vector<DiscreteKey> robot_keys;
    robot_keys.reserve(num_robots);

    for(size_t i = 0; i < num_robots; i++)
    {
        const std::string& robot = index_to_robot[i];
        std::string prefix = "robot_";
        int robot_id = std::stoi(robot.substr(prefix.length()));
        DiscreteKey robot_key(Symbol('x', robot_id), num_frontiers);
        robot_keys.push_back(robot_key);
    }


    // Add unary factors for each robot
    for (size_t i = 0; i < num_robots; i++)
    {
        std::pair<double, double> robot_pos = {transforms[i].transform.translation.x, transforms[i].transform.translation.y};

        std::vector<double> values(num_frontiers);
        for (size_t j = 0; j < num_frontiers; j++)
        {
            const std::pair<double, double> frontier_pos = frontier_centroids[j].first;
            const int dim = frontier_centroids[j].second;

            double dx = robot_pos.first - frontier_pos.first;
            double dy = robot_pos.second - frontier_pos.second;
            double dist = std::sqrt(dx * dx + dy * dy);

            values[j] = std::exp(std::min(-alpha_distance * dist + beta_dimension * dim, 0.0));
        }

        DecisionTreeFactor unary_factor(robot_keys[i], values);
        graph.add(unary_factor);
    }


    // Add pairwise factors between robots and frontiers
    for (size_t i = 0; i < num_robots; i++)
    {
        for (size_t j = i + 1; j < num_robots; j++)
        {
            std::vector<double> table(num_frontiers * num_frontiers, 1.0);
            for (size_t f = 0; f < num_frontiers; f++)
            {
                table[f * num_frontiers + f] = conflict_penalty; 
            }
            DecisionTreeFactor conflict_factor({robot_keys[i], robot_keys[j]}, table);
            graph.add(conflict_factor);
        }
    }

    // graph.print();

    // DiscreteMarginals marginals(graph);

    // for (size_t i = 0; i < transforms.size(); i++)
    // {
    //     const std::string& robot = index_to_robot[i];
    //     const DiscreteKey& robot_key = robot_keys[i];

    //     std::cout << "Robot: " << robot << " - Marginal: " << marginals.marginalProbabilities(robot_key) << std::endl;
    // }

    DiscreteValues result = graph.optimize();

    for (size_t i = 0; i < transforms.size(); i++)
    {
        const std::string& robot = index_to_robot[i];

        size_t assignment = result[robot_keys[i].first];
        robot_frontier_assignments[robot] = frontier_centroids[assignment].first;
    }

    return robot_frontier_assignments;
}






float DecisionGlobalPlanning::heuristic(int x1, int y1, int x2, int y2)
{
    if (heuristic_type == "manhattan")
        return std::abs(x1 - x2) + std::abs(y1 - y2);
    else if (heuristic_type == "euclidean")
        return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
    else
        throw std::runtime_error("Unknown heuristic type: " + heuristic_type);
}

template <typename T> 
nav_msgs::msg::Path DecisionGlobalPlanning::reconstructPath(std::shared_ptr<T> last)
{
    nav_msgs::msg::Path path;
    auto now = node->get_clock()->now();
    path.header.frame_id = all_frame;
    path.header.stamp = now;

    std::vector<geometry_msgs::msg::PoseStamped> temp_poses;

    std::shared_ptr<T> current = last;
    while (current) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = all_frame;
        pose.header.stamp = now;
        pose.pose.position.x = current->x * costmap_grid->info.resolution + costmap_grid->info.origin.position.x;
        pose.pose.position.y = current->y * costmap_grid->info.resolution + costmap_grid->info.origin.position.y;

        temp_poses.push_back(pose);
        current = current->parent;
    }

    if(global_planner_type == "a_star")
        std::reverse(temp_poses.begin(), temp_poses.end());

    // Compute orientation
    for(size_t i = 0; i < temp_poses.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose = temp_poses[i];

        if(i+1 < temp_poses.size()) {
            double dx = temp_poses[i+1].pose.position.x - temp_poses[i].pose.position.x;
            double dy = temp_poses[i+1].pose.position.y - temp_poses[i].pose.position.y;
            double yaw = std::atan2(dy, dx);

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation = tf2::toMsg(q);
        } else if(i > 0) {
            pose.pose.orientation = temp_poses[i-1].pose.orientation;
        } else {
            pose.pose.orientation.w = 1.0;
        }

        path.poses.push_back(pose);
    }

    return path;
}


nav_msgs::msg::Path DecisionGlobalPlanning::aStar(int sx, int sy, int gx, int gy)
{
    std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareAStarNode> open;
    std::vector<std::vector<bool>> closed(costmap_grid->info.height, std::vector<bool>(costmap_grid->info.width, false));

    std::shared_ptr<AStarNode> start = std::make_shared<AStarNode>(sx, sy, 0, heuristic(sx, sy, gx, gy));
    open.push(start);

    while (!open.empty()) {
        std::shared_ptr<AStarNode> current = open.top();
        open.pop();

        if (current->x == gx && current->y == gy) {
            return reconstructPath(current);
        }

        if (closed[current->y][current->x]) {
            continue;
        }

        closed[current->y][current->x] = true;

        
        for(size_t i=0; i<directions.size(); i++)
        {
            std::pair<int, int>& d = directions[i];

            int nx = current->x + d.first;
            int ny = current->y + d.second;
            
            if (nx >= 0 && nx < static_cast<int>(costmap_grid->info.width) && ny >= 0 && ny < static_cast<int>(costmap_grid->info.height) && !closed[ny][nx]) {
                int cost = costmap_grid->data[ny * costmap_grid->info.width + nx];
                if (cost < 254)
                {
                    float step_cost = directions_cost[i];
                    float g = current->g + step_cost + cost;
                    float h = heuristic(nx, ny, gx, gy);
                    std::shared_ptr<AStarNode> neighbor = std::make_shared<AStarNode>(nx, ny, g, h, current);
                    open.push(neighbor);
                }
            }
        }
    }

    return nav_msgs::msg::Path();
}




void DecisionGlobalPlanning::decision_global_planning()
{
    // Get robots poses
    std::map<std::string, geometry_msgs::msg::TransformStamped> transforms;
    
    for (const auto& robot : robots)
    {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer->lookupTransform(all_frame, robot + "/" + base_frame, tf2::TimePointZero);
            transforms[robot] = transform;
        } catch (const tf2::TransformException &ex) {
            // RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }
    


    
    
    // Decision making frontier assignment
#ifdef DECISION_MAKING_DEBUG
    auto start_decision_making = std::chrono::high_resolution_clock::now();
#endif


    std::vector<std::pair<std::pair<double, double>, int>> frontier_centroids;
    
    {
        // std::unique_lock lock(*frontiers_mutex);
        frontier_centroids = *frontiers;
    }

    std::map<std::string, std::pair<double, double>> robot_frontier_assignments;

    // If no frontiers or same frontier for a long time go to initial position
    if(frontier_centroids.size() == 0)
    {
        for(const auto& [robot, transform] : transforms)
        {
            robot_frontier_assignments[robot] = initial_positions[robot];
        }
    }
    else
    {
        robot_frontier_assignments = decision_making_frontier_assignment(frontier_centroids, transforms);
    }
    
#ifdef DECISION_MAKING_DEBUG
    auto end_decision_making = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_decision_making = end_decision_making - start_decision_making;
    std::cout << "Decision making time: " << (duration_decision_making.count() * 1000) << " ms" << std::endl;
    std::cout << std::endl;
#endif
    
#ifdef DECISION_MAKING_DEBUG
    for(const auto& [robot, frontier] : robot_frontier_assignments)
    {
        std::cout << std::fixed << std::setprecision(3) 
                    << "Robot " << robot << " (" << transforms[robot].transform.translation.x << ", " << transforms[robot].transform.translation.y 
                    << ") -> Frontier (" << frontier.first << ", " << frontier.second << ")" << std::endl;
    }
#endif





#ifdef GLOBAL_PLANNING_DEBUG
    auto start_global_planning = std::chrono::high_resolution_clock::now();
#endif

    // Global path planning

    std::map<std::string, nav_msgs::msg::Path> paths;
    {
        // std::shared_lock lock(*costmap_grid_mutex);

        for(const auto& [robot, frontier] : robot_frontier_assignments)
        {
            geometry_msgs::msg::TransformStamped transform = transforms[robot];

            int sx = static_cast<int>((transform.transform.translation.x - costmap_grid->info.origin.position.x) / costmap_grid->info.resolution);
            int sy = static_cast<int>((transform.transform.translation.y - costmap_grid->info.origin.position.y) / costmap_grid->info.resolution);
            int gx = static_cast<int>((frontier.first - costmap_grid->info.origin.position.x) / costmap_grid->info.resolution);
            int gy = static_cast<int>((frontier.second - costmap_grid->info.origin.position.y) / costmap_grid->info.resolution);

            nav_msgs::msg::Path path;
            if (global_planner_type == "a_star") {
                path = aStar(sx, sy, gx, gy);
            } else {
                throw std::runtime_error("Unknown global planner type: " + global_planner_type);
            }

            if (!path.poses.empty()) {
                paths[robot] = path;
            }
        }
    }

    {
        // std::unique_lock lock(*robot_paths_mutex);
        *robot_paths = paths;
    }

    for(const auto& [robot, path] : paths)
    {
        robot_path_publishers[robot]->publish(path);
    }

#ifdef GLOBAL_PLANNING_DEBUG
    auto end_global_planning = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_global_planning = end_global_planning - start_global_planning;
    std::cout << "Global planning time: " << (duration_global_planning.count() * 1000) << " ms" << std::endl;
    std::cout << std::endl;
#endif

}