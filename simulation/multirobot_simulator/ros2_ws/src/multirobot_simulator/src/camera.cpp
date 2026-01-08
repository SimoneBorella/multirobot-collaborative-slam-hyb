#include "camera.h"

void Camera::sensorUpdate()
{
    std::vector<Point> landmarks_marker = getLandmarks();
    publishLandmarks(landmarks_marker);
    publishLandmarksMarker(landmarks_marker);
}


bool Camera::bresenhamObstacleCheck(
    int x0, int y0, int x1, int y1,
    int occupancy_map_width, int occupancy_map_height,
    std::shared_ptr<std::vector<std::vector<int8_t>>> occupancy_map)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    int x = x0, y = y0;

    while (true)
    {
        if (x < 0 || y < 0 || x >= occupancy_map_width || y >= occupancy_map_height)
            break;

        if ((*occupancy_map)[y][x] == 100)
            return true;

        if (x == x1 && y == y1)
            break;

        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }

    return false;
}


std::vector<Point> Camera::getLandmarks()
{
    std::vector<Point> landmarks;

    // Get environment data
    std::shared_ptr<std::vector<std::vector<int8_t>>> occupancy_map = environment->getOccupancyMap();
    int occupancy_map_width = environment->getWidth();
    int occupancy_map_height = environment->getHeight();
    std::vector<double> origin = environment->getOrigin();
    double env_resolution = environment->getResolution();
    std::vector<Point> global_landmarks = environment->getLandmarks();

    
    double camera_global_x = robot_position.x + position.x * cos(robot_position.theta) - position.y * sin(robot_position.theta);
    double camera_global_y = robot_position.y + position.x * sin(robot_position.theta) + position.y * cos(robot_position.theta);

    std::normal_distribution<double> noise_dist(0.0, noise_std_dev);

    for(Point landmark : global_landmarks)
    {
        // Check if there is in camera field of view
        double x_dist = landmark.x - camera_global_x;
        double y_dist = landmark.y - camera_global_y;

        double landmark_theta = std::atan2(y_dist, x_dist);

        double theta_dist = (robot_position.theta + position.theta) - landmark_theta;

        theta_dist = std::fmod(theta_dist + M_PI, 2 * M_PI);
        if (theta_dist < 0)
            theta_dist += 2 * M_PI;
        theta_dist -= M_PI;

        theta_dist = std::abs(theta_dist);

        double dist = std::sqrt(x_dist*x_dist + y_dist*y_dist);

        if (dist > max_range || theta_dist > field_of_view/2)
            continue;



        // Check if there is an obstacle in the middle
        int cam_x_grid = round((camera_global_x - origin[0]) / env_resolution);
        int cam_y_grid = round((camera_global_y - origin[1]) / env_resolution);
        int landmark_x_grid = round((landmark.x - origin[0]) / env_resolution);
        int landmark_y_grid = round((landmark.y - origin[1]) / env_resolution);

        bool obstacle = bresenhamObstacleCheck(cam_x_grid, cam_y_grid, landmark_x_grid, landmark_y_grid,
                                            occupancy_map_width, occupancy_map_height, occupancy_map);

        
        if (!obstacle)
        {
            double translated_x = landmark.x - camera_global_x;
            double translated_y = landmark.y - camera_global_y;

            double rototranslated_x = translated_x * cos(-(robot_position.theta + position.theta)) - translated_y * sin(-(robot_position.theta + position.theta));
            double rototranslated_y = translated_x * sin(-(robot_position.theta + position.theta)) + translated_y * cos(-(robot_position.theta + position.theta));

            // Add Gaussian noise
            double x = rototranslated_x + noise_dist(generator);
            double y = rototranslated_y + noise_dist(generator);

            landmarks.push_back(Point{x, y});
        }
    }

    return landmarks;
}






void Camera::publishLandmarksMarker(std::vector<Point>& landmarks)
{
    visualization_msgs::msg::Marker marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = robot_name + "/" + name + "_link";
    marker.header.stamp = node->get_clock()->now();

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

    for (const auto& landmark : landmarks) {
        geometry_msgs::msg::Point p;
        p.x = landmark.x;
        p.y = landmark.y;
        p.z = 0.0;
        marker.points.push_back(p);
    }

    landmarks_marker_publisher->publish(marker);
}



void Camera::publishLandmarks(std::vector<Point>& landmarks)
{
    interfaces::msg::PointArray landmarks_msg = interfaces::msg::PointArray();

    landmarks_msg.header.frame_id = name + "_link";
    landmarks_msg.header.stamp = node->get_clock()->now();

    for (const auto& landmark : landmarks) {
        geometry_msgs::msg::Point p;
        p.x = landmark.x;
        p.y = landmark.y;
        p.z = 0.0;

        landmarks_msg.points.push_back(p);
    }

    landmarks_publisher->publish(landmarks_msg);
}