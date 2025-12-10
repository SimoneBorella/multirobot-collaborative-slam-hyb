#include "robot.h"

std::vector<std::shared_ptr<Robot>> Robot::robots;


Robot::Robot(rclcpp::Node::SharedPtr node, std::shared_ptr<Environment> environment, const std::string& robot_yaml_path)
{
    this->node = node;
    this->environment = environment;

    cmd.linear_vel = 0.0;
    cmd.angular_vel = 0.0;

    linear_bias = 0.0;
    angular_bias = 0.0;

    loadRobotConfiguration(robot_yaml_path);

    odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>(name + "/odom", 10);
    footprint_publisher = node->create_publisher<visualization_msgs::msg::Marker>(name + "/footprint", 10);

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    cmd_vel_subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        name + "/cmd_vel", 10, std::bind(&Robot::cmdVelCallback, this, std::placeholders::_1));

    cmd_vel_nav_subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        name + "/cmd_vel_nav", 10, std::bind(&Robot::cmdVelCallback, this, std::placeholders::_1));

    publishAllToMapTransform();
    // publishMapToOdomTransform();
    publishBaseLinkToBaseFootprintTransform();
    
    RCLCPP_INFO_STREAM(node->get_logger(), "Robot " << name << " initialized.");
}

Robot::~Robot()
{
    if (state_update_thread && state_update_thread->joinable()) {
        state_update_thread->join();
    }
}

void Robot::loadRobotConfiguration(const std::string& robot_yaml_path) {
    YAML::Node config = YAML::LoadFile(robot_yaml_path);

    name = config["name"].as<std::string>();

    random_spawn = config["random_spawn"].as<bool>();

    if(!random_spawn)
    {
        init_position.x = config["position"]["x"].as<double>();
        init_position.y = config["position"]["y"].as<double>();
        init_position.theta = config["position"]["theta"].as<double>() * (M_PI / 180);
        
        position = init_position;
        imu_position = init_position;
    }

    shape.type = config["shape"]["type"].as<std::string>();
    shape.diameter = config["shape"]["diameter"].as<double>();

    const YAML::Node& sensor_nodes = config["sensors"];
    for (const auto& sensor_node : sensor_nodes) {
        std::string type = sensor_node["type"].as<std::string>();

        if (type == "imu")
        {
            linear_bias_std_dev = sensor_node["linear_bias_std_dev"].as<double>();
            angular_bias_std_dev = sensor_node["angular_bias_std_dev"].as<double>();
            alpha = sensor_node["alpha"].as<double>();
            linear_scale_error = sensor_node["linear_scale_error"].as<double>();
            angular_scale_error = sensor_node["angular_scale_error"].as<double>();
        }
        else if (type == "lidar")
        {
            std::string lidar_name = sensor_node["name"].as<std::string>();
            Position lidar_position;
            lidar_position.x = sensor_node["position"]["x"].as<double>();
            lidar_position.y = sensor_node["position"]["y"].as<double>();
            lidar_position.theta = sensor_node["position"]["theta"].as<double>() * (M_PI / 180);
            double lidar_min_range = sensor_node["min_range"].as<double>();
            double lidar_max_range = sensor_node["max_range"].as<double>();
            double lidar_resolution = sensor_node["resolution"].as<double>() * (M_PI / 180);
            int lidar_points = sensor_node["points"].as<int>();
            double lidar_frequency = sensor_node["frequency"].as<double>();

            std::string scan_topic;
            if (sensor_node["scan_topic"]) {
                scan_topic = sensor_node["scan_topic"].as<std::string>();
            } else {
                scan_topic = name + "/scan";
            }

            std::string pcl_topic;
            if (sensor_node["pcl_topic"]) {
                pcl_topic = sensor_node["pcl_topic"].as<std::string>();
            } else {
                pcl_topic = name + "/scan/points";
            }

            std::shared_ptr<Lidar> lidar = std::make_shared<Lidar>(
                node, name, position, environment, lidar_name, lidar_position, lidar_frequency,
                lidar_min_range, lidar_max_range, lidar_resolution, lidar_points, scan_topic, pcl_topic
            );

            sensors.push_back(lidar);
        }
        else if (type == "camera")
        {
            std::string camera_name = sensor_node["name"].as<std::string>();
            Position camera_position;
            camera_position.x = sensor_node["position"]["x"].as<double>();
            camera_position.y = sensor_node["position"]["y"].as<double>();
            camera_position.theta = sensor_node["position"]["theta"].as<double>() * (M_PI / 180);
            double camera_max_range = sensor_node["max_range"].as<double>();
            double camera_field_of_view = sensor_node["field_of_view"].as<double>() * (M_PI / 180);
            int camera_frequency = sensor_node["frequency"].as<int>();
            double noise_std_dev = sensor_node["noise_std_dev"].as<double>();
            
            std::string topic;
            if (sensor_node["topic"]) {
                topic = sensor_node["topic"].as<std::string>();
            } else {
                topic = name + "/landmarks";
            }

            std::shared_ptr<Camera> camera = std::make_shared<Camera>(
                node, name, position, environment, camera_name, camera_position,
                camera_frequency, camera_max_range, camera_field_of_view, noise_std_dev, topic
            );
            
            sensors.push_back(camera);
        }
    }

    for(auto sensor : sensors)
        sensor->setRobotInitPosition(init_position);
}


void Robot::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd.linear_vel = msg->linear.x;
    cmd.angular_vel = msg->angular.z;
}


void Robot::spawnIfNotInitialized()
{
    if(!random_spawn)
        return;

    std::shared_ptr<std::vector<std::vector<int8_t>>> occupancy_map = environment->getOccupancyMap();
    int occupancy_map_width = environment->getWidth();
    int occupancy_map_height = environment->getHeight();
    std::vector<double> origin = environment->getOrigin();
    double env_resolution = environment->getResolution();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dis(origin[0], origin[0] + occupancy_map_width*env_resolution);
    std::uniform_real_distribution<> y_dis(origin[1], origin[1] + occupancy_map_height*env_resolution);
    std::uniform_real_distribution<> theta_dis(-M_PI, M_PI);

    bool valid_position = false;

    while(!valid_position)
    {
        init_position.x = x_dis(gen);
        init_position.y = y_dis(gen);
        init_position.theta = theta_dis(gen);

        valid_position = true;

        
        int circle_center_x = round((init_position.x - origin[0]) / env_resolution);
        int circle_center_y = round((init_position.y - origin[1]) / env_resolution);

        double radius_in_cells = 2 * (shape.diameter / (2 * env_resolution)); 

        for (int h = circle_center_y - round(radius_in_cells); h < circle_center_y + round(radius_in_cells); h++) {
            if (h>=0 && h<occupancy_map_height)
                for (int w = circle_center_x - round(radius_in_cells); w < circle_center_x + round(radius_in_cells); w++) {
                    if (w>=0 && w<occupancy_map_width)
                    {
                        double distance = sqrt(pow(h - circle_center_y, 2) + pow(w - circle_center_x, 2));

                        if (distance <= radius_in_cells)
                            if((*occupancy_map)[h][w] == 100)
                            {
                                valid_position = false;
                                break;
                            }
                    }
                }
        }
        

        for (auto robot : robots)
        {
            if(robot->name == name || robot->isNotSpawned())
                continue;

            double distance = std::sqrt((init_position.x - robot->init_position.x)*(init_position.x - robot->init_position.x) + (init_position.y - robot->init_position.y)*(init_position.y - robot->init_position.y));
            if(distance < 1.5 * (shape.diameter/2 + robot->getShape().diameter/2))
            {
                valid_position = false;
                break;
            }
        }
    }   

    position = init_position;
    imu_position = init_position;
    random_spawn = false;
}


void Robot::startUpdateLoop(double rate)
{
    startStateUpdateLoop(rate);

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    for(auto sensor : sensors)
        sensor->startUpdateLoop();
}

void Robot::startStateUpdateLoop(double rate)
{
    state_update_thread = std::make_unique<std::thread>([this, rate]() {
        rclcpp::Rate update_rate(rate);
        while (rclcpp::ok()) {
            stateUpdate(rate);
            update_rate.sleep();
        }
    });
    state_update_thread->detach();

    RCLCPP_INFO_STREAM(node->get_logger(), "Robot " << name << " thread started.");
}



void Robot::stateUpdate(double rate)
{
    updatePosition(rate);
    updateIMUPosition(rate);
    publishOdometry();

    publishOdomToBaseLinkTransform();
    
    std::vector<Position> other_robots_positions;
    std::vector<Shape> other_robots_shapes;

    for(auto robot : robots)
    {
        if(robot->name == name)
            continue;
        
        other_robots_positions.push_back(robot->position);
        other_robots_shapes.push_back(robot->shape);
    }

    for(auto sensor : sensors)
    {
        sensor->setRobotPosition(position);
        sensor->setOtherRobotsPositions(other_robots_positions);
        sensor->setOtherRobotsShapes(other_robots_shapes);
    }

    publishFootprintMarker();
}


void Robot::updatePosition(double rate)
{
    double dt = 1.0 / rate;

    double dx = cmd.linear_vel * cos(position.theta) * dt;
    double dy = cmd.linear_vel * sin(position.theta) * dt;

    position.x += dx;
    position.y += dy;
    position.theta += cmd.angular_vel * dt;

    position.theta = fmod(position.theta + M_PI, 2 * M_PI);
    if (position.theta < 0)
        position.theta += 2 * M_PI;
    position.theta -= M_PI;
}


void Robot::updateIMUPosition(double rate)
{
    double dt = 1.0 / rate;

    // Update biases using Gauss-Markov process
    std::normal_distribution<double> linear_bias_dist(0.0, linear_bias_std_dev);
    std::normal_distribution<double> angular_bias_dist(0.0, angular_bias_std_dev);
    linear_bias = alpha * linear_bias + linear_bias_dist(generator);
    angular_bias = alpha * angular_bias + angular_bias_dist(generator);

    // Apply scale error and bias to commands
    double linear_vel_noisy = (1 + linear_scale_error) * cmd.linear_vel + linear_bias;
    double angular_vel_noisy = (1 + angular_scale_error) * cmd.angular_vel + angular_bias;

    double dx = linear_vel_noisy * cos(imu_position.theta) * dt;
    double dy = linear_vel_noisy * sin(imu_position.theta) * dt;

    imu_position.x += dx;
    imu_position.y += dy;
    imu_position.theta += angular_vel_noisy * dt;

    // Normalize theta to [-pi, pi]
    imu_position.theta = fmod(imu_position.theta + M_PI, 2 * M_PI);
    if (imu_position.theta < 0)
        imu_position.theta += 2 * M_PI;
    imu_position.theta -= M_PI;
}



void Robot::publishOdometry() {
    nav_msgs::msg::Odometry odom_msg = nav_msgs::msg::Odometry();
    
    odom_msg.header.stamp = node->get_clock()->now();
    odom_msg.header.frame_id = "odom";

    odom_msg.pose.pose.position.x = position.x - init_position.x;
    odom_msg.pose.pose.position.y = position.y - init_position.y;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    double theta = position.theta - init_position.theta;

    theta = fmod(theta + M_PI, 2 * M_PI);
    if (theta < 0)
        theta += 2 * M_PI;
    theta -= M_PI;

    q.setRPY(0, 0, theta);

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = cmd.linear_vel;
    odom_msg.twist.twist.linear.y = 0.0;  // Assuming no lateral velocity
    odom_msg.twist.twist.linear.z = 0.0;  // Assuming no vertical movement
    odom_msg.twist.twist.angular.z = cmd.angular_vel;

    odom_publisher->publish(odom_msg);
}


void Robot::publishFootprintMarker()
{
    visualization_msgs::msg::Marker marker = visualization_msgs::msg::Marker();
    
    marker.header.frame_id = name + "/base_link";
    marker.header.stamp = node->get_clock()->now();
    
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.scale.x = 0.05;

    int num_points = 20;
    double radius = shape.diameter/2;

    for (int i = 0; i <= num_points; i++) {
        double angle = (2 * M_PI * i) / num_points;
        geometry_msgs::msg::Point p;
        p.x = radius * cos(angle);
        p.y = radius * sin(angle);
        p.z = 0.0;
        marker.points.push_back(p);
    }

    marker.points.push_back(marker.points.front());

    footprint_publisher->publish(marker);
}