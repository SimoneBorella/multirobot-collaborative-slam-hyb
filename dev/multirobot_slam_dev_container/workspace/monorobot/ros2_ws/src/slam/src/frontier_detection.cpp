#include "frontier_detection.h"

FrontierDetection::FrontierDetection(rclcpp::Node::SharedPtr node, std::shared_ptr<Mapping> mapping)
{
    this->node = node;
    this->mapping = mapping;

    map_frame = node->get_parameter("map_frame").as_string();
    odom_frame = node->get_parameter("odom_frame").as_string();
    base_frame = node->get_parameter("base_frame").as_string();

    frontier_detection_rate = node->get_parameter("frontier_detection_rate").as_double();
    epsilon = node->get_parameter("epsilon").as_double();
    min_points = node->get_parameter("min_points").as_int();
    min_frontier_size = node->get_parameter("min_frontier_size").as_int();


    // Initializa tf buffer and listener
    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Initialize nav2 action client
    nav2_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose");


    // Initialize publishers
    frontiers_marker_publisher = node->create_publisher<visualization_msgs::msg::Marker>("frontier_centroids", 10);

    // Initialize frontier grid
    auto [frontier_grid_, frontier_grid_mutex_] = mapping->get_frontier_grid_with_mutex();
    frontier_grid = frontier_grid_;
    frontier_grid_mutex = frontier_grid_mutex_;

    // Initialize threads
    frontier_detection_running = false;
}

FrontierDetection::~FrontierDetection()
{
    frontier_detection_running = false;
    if (frontier_detection_thread && frontier_detection_thread->joinable()) {
        frontier_detection_thread->join();
    }
}



void FrontierDetection::startFrontierDetection()
{
    frontier_detection_running = true;
    frontier_detection_thread = std::make_unique<std::thread>([this]() {
        rclcpp::Rate rate(frontier_detection_rate);
        while (rclcpp::ok() && frontier_detection_running.load()) {
            frontier_detection();
            rate.sleep();
        }
    });
    RCLCPP_INFO_STREAM(node->get_logger(), "Frontier detection thread started.");
}



std::vector<std::pair<int, int>> FrontierDetection::get_neighbors(int x, int y)
{
    std::vector<std::pair<int, int>> neighbors;

    int height_cells = frontier_grid->info.height;
    int width_cells = frontier_grid->info.width;
    double map_resolution = frontier_grid->info.resolution;
    int epsilon_cells = std::ceil(epsilon / map_resolution);

    for (int dx = -epsilon_cells; dx <= epsilon_cells; dx++)
    {
        for (int dy = -epsilon_cells; dy <= epsilon_cells; dy++)
        {
            if (dx == 0 && dy == 0)
                continue;

            if ((dx * dx + dy * dy) * map_resolution * map_resolution > epsilon * epsilon)
                continue;

            int nx = x + dx;
            int ny = y + dy;

            int nindex = ny * width_cells + nx;

            if (nx >= 0 && ny >= 0 && nx < width_cells && ny < height_cells && frontier_grid->data[nindex] == 100)
            {
                neighbors.emplace_back(nx, ny);
            }
        }
    }

    return neighbors;
}




std::map<int, std::vector<std::pair<int, int>>> FrontierDetection::dbscan()
{
    std::map<std::pair<int, int>, int> labels;
    std::map<int, std::vector<std::pair<int, int>>> clusters;

    int cluster_id = 0;
    int height_cells = frontier_grid->info.height;
    int width_cells = frontier_grid->info.width;

    for (int y = 0; y < height_cells; y++) {
        for (int x = 0; x < width_cells; x++) {
            int index = y * width_cells + x;
    
            if (frontier_grid->data[index] != 100)
                continue;
    
            if (labels.find({x, y}) != labels.end())
                continue;
    
            std::vector<std::pair<int, int>> neighbors = get_neighbors(x, y);
    
            if (neighbors.size() < static_cast<size_t>(min_points)) {
                labels[{x, y}] = -1;
                continue;
            }
    
            labels[{x, y}] = cluster_id;
            clusters[cluster_id].push_back({x, y});
    
            std::queue<std::pair<int, int>> queue;
    
            for (const auto& p : neighbors)
                queue.push(p);
    
            while (!queue.empty()) {
                auto p = queue.front(); queue.pop();
    
                if (labels.find(p) != labels.end()) {
                    if (labels[p] == -1) {
                        labels[p] = cluster_id;
                        clusters[cluster_id].push_back(p);
                    }
                    continue;
                }
    
                labels[p] = cluster_id;
                clusters[cluster_id].push_back(p);
    
                auto p_neighbors = get_neighbors(p.first, p.second);
                if (p_neighbors.size() >= static_cast<size_t>(min_points)) {
                    for (const auto& n : p_neighbors)
                        queue.push(n);
                }
            }
    
            cluster_id++;
        }
    }
    

    return clusters;
}


std::vector<std::pair<double, double>> FrontierDetection::compute_frontier_centroids(
    const std::map<int, std::vector<std::pair<int, int>>>& frontier_clusters)
{
    std::vector<std::pair<double, double>> centroids;

    double env_resolution = frontier_grid->info.resolution;
    double origin_x = frontier_grid->info.origin.position.x;
    double origin_y = frontier_grid->info.origin.position.y;

    for (const auto& [cluster_id, cluster] : frontier_clusters)
    {
        if (cluster.size() < static_cast<size_t>(min_frontier_size))
            continue;

        double sum_x = 0.0;
        double sum_y = 0.0;

        for (const auto& point : cluster)
        {
            sum_x += point.first;
            sum_y += point.second;
        }

        double centroid_x = sum_x / cluster.size();
        double centroid_y = sum_y / cluster.size();

        // Find the point in the cluster closest to the centroid
        std::pair<int, int> closest_point;
        double min_dist_sq = std::numeric_limits<double>::max();

        for (const auto& point : cluster)
        {
            double dx = point.first - centroid_x;
            double dy = point.second - centroid_y;
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                closest_point = point;
            }
        }

        // Convert closest grid cell to world coordinates
        double world_x = closest_point.first * env_resolution + origin_x + env_resolution / 2.0;
        double world_y = closest_point.second * env_resolution + origin_y + env_resolution / 2.0;

        centroids.emplace_back(world_x, world_y);
    }

    return centroids;
}




void FrontierDetection::publish_frontiers_marker(std::vector<std::pair<double, double>>& frontier_centroids)
{
    visualization_msgs::msg::Marker marker = visualization_msgs::msg::Marker();
    
    marker.header.frame_id = map_frame;
    marker.header.stamp = node->get_clock()->now();
    
    marker.ns = "frontiers";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.color.r = 0.1f;
    marker.color.g = 0.4f;
    marker.color.b = 0.1f;
    marker.color.a = 1.0f;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;

    for (const auto& frontier_centroid : frontier_centroids) {
        geometry_msgs::msg::Point p;
        p.x = frontier_centroid.first;
        p.y = frontier_centroid.second;
        p.z = 0.0;
        marker.points.push_back(p);
    }

    frontiers_marker_publisher->publish(marker);
}



void FrontierDetection::frontier_detection()
{
#ifdef FRONTIER_DETECTION_DEBUG
    auto start_frontier_detection = std::chrono::high_resolution_clock::now();
#endif

    std::map<int, std::vector<std::pair<int, int>>> frontier_clusters;
    {
        std::shared_lock lock(*frontier_grid_mutex);
        frontier_clusters = dbscan();
    }

    std::vector<std::pair<double, double>> frontier_centroids = compute_frontier_centroids(frontier_clusters);
    publish_frontiers_marker(frontier_centroids);


#ifdef FRONTIER_DETECTION_DEBUG
    auto end_frontier_detection = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_frontier_detection = end_frontier_detection - start_frontier_detection;
#endif

#ifdef FRONTIER_DETECTION_DEBUG
    std::cout << "Frontier detection time: " << (duration_frontier_detection.count() * 1000) << " ms" << std::endl;
    std::cout << std::endl;
#endif


    // Set closest frontier as goal
    {
        // Get robot pose
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            // RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }
    
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
    
        // Find closest centroid
        double min_dist = std::numeric_limits<double>::max();
        std::pair<double, double> closest_centroid;
    
        for (const auto& centroid : frontier_centroids) {
            double dx = centroid.first - robot_x;
            double dy = centroid.second - robot_y;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_centroid = centroid;
            }
        }
    
        if (!std::isfinite(min_dist)) {
            // RCLCPP_WARN(node->get_logger(), "No valid frontier found.");
            return;
        }
    
        // Send goal to Nav2
        if (!nav2_client->wait_for_action_server(std::chrono::seconds(2))) {
            // RCLCPP_ERROR(node->get_logger(), "Nav2 action server not available.");
            return;
        }
    
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = map_frame;
        goal_msg.pose.header.stamp = node->get_clock()->now();
        goal_msg.pose.pose.position.x = closest_centroid.first;
        goal_msg.pose.pose.position.y = closest_centroid.second;
        goal_msg.pose.pose.orientation.w = 1.0; // No rotation
    
        nav2_client->async_send_goal(goal_msg);
    }


    // Set closest frontier as goal with orientation
    // {
    //     // Get robot pose
    //     geometry_msgs::msg::TransformStamped transform;
    //     try {
    //         transform = tf_buffer->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
    //     } catch (const tf2::TransformException &ex) {
    //         // RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", ex.what());
    //         return;
    //     }

    //     double robot_x = transform.transform.translation.x;
    //     double robot_y = transform.transform.translation.y;

    //     // Get robot's orientation (quaternion)
    //     double robot_orientation_x = transform.transform.rotation.x;
    //     double robot_orientation_y = transform.transform.rotation.y;
    //     double robot_orientation_z = transform.transform.rotation.z;
    //     double robot_orientation_w = transform.transform.rotation.w;

    //     // Convert quaternion to yaw angle (heading)
    //     double pose_roll, pose_pitch, pose_yaw;
    //     tf2::Quaternion quat_tf(robot_orientation_x, robot_orientation_y, robot_orientation_z, robot_orientation_w);
    //     tf2::Matrix3x3(quat_tf).getRPY(pose_roll, pose_pitch, pose_yaw);


    //     // Calculate the robot's forward unit vector
    //     double robot_forward_x = std::cos(pose_yaw);
    //     double robot_forward_y = std::sin(pose_yaw);

    //     // Find closest centroid that is within 180 degrees ahead
    //     double min_dist = std::numeric_limits<double>::max();
    //     std::pair<double, double> closest_centroid;

    //     for (const auto& centroid : frontier_centroids) {
    //         // Calculate vector from robot to centroid
    //         double dx = centroid.first - robot_x;
    //         double dy = centroid.second - robot_y;

    //         // Calculate the distance
    //         double dist = std::hypot(dx, dy);

    //         // Calculate the angle between the robot's forward direction and the centroid direction
    //         double dot_product = (robot_forward_x * dx + robot_forward_y * dy) / dist;
    //         double angle = std::acos(std::clamp(dot_product, -1.0, 1.0));  // Angle between 0 and Pi

    //         // Check if the centroid is within the forward 180 degrees
    //         if (angle <= M_PI / 2) {  // 90 degrees in radians = Pi/2
    //             // If it's the closest valid centroid, update
    //             if (dist < min_dist) {
    //                 min_dist = dist;
    //                 closest_centroid = centroid;
    //             }
    //         }
    //     }

    //     if (!std::isfinite(min_dist)) {
    //         // RCLCPP_WARN(node->get_logger(), "No valid frontier found.");
    //         return;
    //     }

    //     // Send goal to Nav2
    //     if (!nav2_client->wait_for_action_server(std::chrono::seconds(2))) {
    //         // RCLCPP_ERROR(node->get_logger(), "Nav2 action server not available.");
    //         return;
    //     }

    //     auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    //     goal_msg.pose.header.frame_id = map_frame;
    //     goal_msg.pose.header.stamp = node->get_clock()->now();
    //     goal_msg.pose.pose.position.x = closest_centroid.first;
    //     goal_msg.pose.pose.position.y = closest_centroid.second;
    //     goal_msg.pose.pose.orientation.w = 1.0;  // No rotation (we don't care about orientation)

    //     nav2_client->async_send_goal(goal_msg);
    // }


}
