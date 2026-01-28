#include "camera.h"

Camera::Camera(
    rclcpp::Node::SharedPtr node,
    const std::string& robot_name,
    Position& robot_position,
    std::shared_ptr<Environment> environment,
    const std::string& name,
    Position& position,
    double frequency,
    double max_range,
    double field_of_view,
    double noise_std_dev,
    std::string& topic)
    : Sensor(node, robot_name, robot_position, environment, name, position, frequency),
      max_range(max_range),
      field_of_view(field_of_view),
      noise_std_dev(noise_std_dev),
      max_range_sq_(max_range * max_range),
      noise_dist_(0.0, noise_std_dev)
{
    keypoints_publisher =
        node->create_publisher<interfaces::msg::KeyPointArray>(topic, 10);

    keypoints_marker_publisher =
        node->create_publisher<visualization_msgs::msg::Marker>(topic + "/plot", 10);

    keypoints_.reserve(64);
}


void Camera::sensorUpdate()
{
    auto& keypoints = getKeyPoints();
    publishKeyPoints(keypoints);
    publishKeyPointsMarker(keypoints);
}


bool Camera::bresenhamObstacleCheck(
    int x0, int y0,
    int x1, int y1,
    int w, int h,
    const std::vector<std::vector<int8_t>>& map)
{
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (x0 < 0 || y0 < 0 || x0 >= w || y0 >= h)
            return true;

        if (map[y0][x0] == 100)
            return true;

        if (x0 == x1 && y0 == y1)
            return false;

        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

inline void Camera::addDescriptorNoise(std::array<uint8_t, 32>& descriptor)
{
    // Assumes descriptor size is 32 bytes (256 bits)
    for (int i = 0; i < 2; ++i)
    {
        const int byte = byte_dist_(generator);
        const int bit  = bit_dist_(generator);
        descriptor[byte] ^= static_cast<uint8_t>(1u << bit);
    }
}


std::vector<KeyPoint>& Camera::getKeyPoints()
{
    keypoints_.clear();

    const auto& map = *environment->getOccupancyMap();
    int w = environment->getWidth();
    int h = environment->getHeight();
    auto origin = environment->getOrigin();
    double res = environment->getResolution();
    double inv_res = 1.0 / res;

    const auto& global_landmarks = environment->getLandmarks();

    double base_theta = robot_position.theta + position.theta;
    double cos_t = std::cos(base_theta);
    double sin_t = std::sin(base_theta);

    double cam_x = robot_position.x + position.x * std::cos(robot_position.theta)
                 - position.y * std::sin(robot_position.theta);
    double cam_y = robot_position.y + position.x * std::sin(robot_position.theta)
                 + position.y * std::cos(robot_position.theta);

    int cam_xg = static_cast<int>((cam_x - origin[0]) * inv_res);
    int cam_yg = static_cast<int>((cam_y - origin[1]) * inv_res);

    double half_fov = field_of_view * 0.5;

    for (const auto& lm : global_landmarks)
    {
        double dx = lm.point.x - cam_x;
        double dy = lm.point.y - cam_y;

        double dist_sq = dx*dx + dy*dy;
        if (dist_sq > max_range_sq_)
            continue;

        double angle = std::atan2(dy, dx);
        double dtheta = std::fabs(std::atan2(
            std::sin(angle - base_theta),
            std::cos(angle - base_theta)));

        if (dtheta > half_fov)
            continue;

        int lm_xg = static_cast<int>((lm.point.x - origin[0]) * inv_res);
        int lm_yg = static_cast<int>((lm.point.y - origin[1]) * inv_res);

        if (bresenhamObstacleCheck(cam_xg, cam_yg, lm_xg, lm_yg, w, h, map))
            continue;

        double rx =  dx * cos_t + dy * sin_t;
        double ry = -dx * sin_t + dy * cos_t;
        
        // Keypoint generated with noise
        KeyPoint kp(rx + noise_dist_(generator), ry + noise_dist_(generator));

        // Descriptor generated with noise
        kp.descriptor = lm.descriptor;
        // addDescriptorNoise(kp.descriptor);

        keypoints_.push_back(kp);
    }

    return keypoints_;
}


void Camera::publishKeyPointsMarker(std::vector<KeyPoint>& keypoints)
{
    visualization_msgs::msg::Marker marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = robot_name + "/" + name + "_link";
    marker.header.stamp = node->now();

    marker.ns = robot_name + "/" + name;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.color.r = 0.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0f;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;

    for (const auto& keypoint : keypoints) {
        geometry_msgs::msg::Point p;
        p.x = keypoint.point.x;
        p.y = keypoint.point.y;
        p.z = keypoint.point.z;
        marker.points.push_back(p);
    }

    keypoints_marker_publisher->publish(marker);
}


void Camera::publishKeyPoints(std::vector<KeyPoint>& keypoints)
{
    interfaces::msg::KeyPointArray keypoints_msg;

    keypoints_msg.header.frame_id = name + "_link";
    keypoints_msg.header.stamp = node->now();

    for (const auto& keypoint : keypoints) {
        interfaces::msg::KeyPoint kp;

        kp.point.x = keypoint.point.x;
        kp.point.y = keypoint.point.y;
        kp.point.z = keypoint.point.z;

        kp.descriptor = keypoint.descriptor;  // <-- now works

        keypoints_msg.keypoints.push_back(kp);
    }

    keypoints_publisher->publish(keypoints_msg);
}
