#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>
#include <random>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include <array>

#include "types.h"

class Environment
{
public:
    Environment() = default;

    Environment(rclcpp::Node::SharedPtr node, const std::string& map_yaml_path, const std::string& map_pgm_path, const std::string& blind_spots_yaml_path, const std::string& obstacles_yaml_path, double landmarks_density)
    {
        this->node = node;
        map_subscription_count = 0;
        landmarks_subscription_count = 0;

        map_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map_ground_truth", 10);
        landmarks_marker_publisher = node->create_publisher<visualization_msgs::msg::Marker>("landmarks/plot", 10);

        RCLCPP_INFO_STREAM(node->get_logger(), "Initializing environment...");

        loadMap(map_yaml_path, map_pgm_path);
        RCLCPP_INFO_STREAM(node->get_logger(), "Map loaded.");


        YAML::Node blind_spots_config = YAML::LoadFile(blind_spots_yaml_path);

        std::vector<BlindSpot> blind_spots;
        try
        {
            YAML::Node blind_spots_config = YAML::LoadFile(blind_spots_yaml_path);
            if (blind_spots_config["blind_spots"])
            {
                for (const auto& spot_node : blind_spots_config["blind_spots"])
                {
                    BlindSpot spot;
                    spot.x = spot_node["x"].as<double>();
                    spot.y = spot_node["y"].as<double>();
                    spot.r = spot_node["r"].as<double>();
                    blind_spots.push_back(spot);
                }
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to load blind spots YAML: " << e.what());
        }


        std::vector<std::unique_ptr<Obstacle>> obstacles;

        try
        {
            YAML::Node obstacles_config = YAML::LoadFile(obstacles_yaml_path);
            if (obstacles_config["obstacles"])
            {
                for (const auto& obstacle_node : obstacles_config["obstacles"])
                {
                    std::string type = obstacle_node["type"].as<std::string>();

                    if (type == "circle")
                    {
                        auto obs = std::make_unique<ObstacleCircle>();
                        obs->x = obstacle_node["x"].as<double>();
                        obs->y = obstacle_node["y"].as<double>();
                        if (obstacle_node["theta"])
                            obs->theta = obstacle_node["theta"].as<double>();
                        obs->r = obstacle_node["r"].as<double>();
                        obstacles.push_back(std::move(obs));
                    }
                    else if (type == "rectangle")
                    {
                        auto obs = std::make_unique<ObstacleRectangle>();
                        obs->x = obstacle_node["x"].as<double>();
                        obs->y = obstacle_node["y"].as<double>();
                        obs->theta = obstacle_node["theta"].as<double>();
                        obs->w = obstacle_node["w"].as<double>();
                        obs->h = obstacle_node["h"].as<double>();
                        obstacles.push_back(std::move(obs));
                    }
                    else
                    {
                        std::cerr << "Unknown obstacle type: " << type << std::endl;
                    }
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to load obstacles YAML: " << e.what() << std::endl;
        }

        generateObstacles(obstacles);
        RCLCPP_INFO_STREAM(node->get_logger(), "Obstacles generated.");

        generateLandmarks(landmarks_density, blind_spots);
        RCLCPP_INFO_STREAM(node->get_logger(), "Landmarks generated.");

        RCLCPP_INFO_STREAM(node->get_logger(), "Environment initialized.");
    }

    void loadMap(const std::string& map_yaml_path, const std::string& map_pgm_path)
    {
        YAML::Node map_config = YAML::LoadFile(map_yaml_path);
        std::string map_filename = map_config["image"].as<std::string>();
        resolution = map_config["resolution"].as<double>();
        origin = map_config["origin"].as<std::vector<double>>();

        std::ifstream file(map_pgm_path, std::ios::binary);

        if (!file.is_open()) {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Error: Unable to open file " << map_pgm_path);
            return;
        }

        std::string format;
        int max_value;

        file >> format >> width >> height >> max_value;
        file.ignore(1);

        if (format != "P5" || max_value != 255) {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Error: Invalid PGM format or max value");
            return;
        }

        occupancy_map = std::make_shared<std::vector<std::vector<int8_t>>>(height, std::vector<int8_t>(width));

        for (int h = height - 1; h >= 0; h--)
        {
            for (size_t w = 0; w < width; w++)
            {
                uint8_t pixel = file.get();
                if (pixel == 0) {
                    (*occupancy_map)[h][w] = 100; // Occupied
                } else if (pixel == 255) {
                    (*occupancy_map)[h][w] = 0;   // Free
                } else {
                    (*occupancy_map)[h][w] = 100;  // Set as occupied
                }
            }
        }

        file.close();
    }

    bool worldToGrid(double x, double y, int& gx, int& gy)
    {
        gx = static_cast<int>(std::round((x - origin[0]) / resolution));
        gy = static_cast<int>(std::round((y - origin[1]) / resolution));

        return gx >= 0 && gx < static_cast<int>(width) &&
            gy >= 0 && gy < static_cast<int>(height);
    }


    void generateObstacles(const std::vector<std::unique_ptr<Obstacle>>& obstacles)
    {
        if (!occupancy_map)
            return;

        for (const auto& obs_ptr : obstacles)
        {
            // Cricle obstacle
            if (auto* c = dynamic_cast<ObstacleCircle*>(obs_ptr.get()))
            {
                int cx, cy;
                if (!worldToGrid(c->x, c->y, cx, cy))
                    continue;

                int r_cells = static_cast<int>(std::ceil(c->r / resolution));

                for (int dy = -r_cells; dy <= r_cells; ++dy)
                {
                    for (int dx = -r_cells; dx <= r_cells; ++dx)
                    {
                        int gx = cx + dx;
                        int gy = cy + dy;

                        if (gx < 0 || gx >= static_cast<int>(width) ||
                            gy < 0 || gy >= static_cast<int>(height))
                            continue;

                        double wx = origin[0] + gx * resolution;
                        double wy = origin[1] + gy * resolution;

                        double dist_sq =
                            (wx - c->x) * (wx - c->x) +
                            (wy - c->y) * (wy - c->y);

                        if (dist_sq <= c->r * c->r)
                            (*occupancy_map)[gy][gx] = 100;
                    }
                }
            }


            // Rectangle obstacle
            else if (auto* r = dynamic_cast<ObstacleRectangle*>(obs_ptr.get()))
            {
                double half_w = r->w * 0.5;
                double half_h = r->h * 0.5;

                double c = std::cos(r->theta);
                double s = std::sin(r->theta);

                // Bounding box in world
                double radius = std::hypot(half_w, half_h);

                int cx, cy;
                if (!worldToGrid(r->x, r->y, cx, cy))
                    continue;

                int range = static_cast<int>(std::ceil(radius / resolution));

                for (int dy = -range; dy <= range; ++dy)
                {
                    for (int dx = -range; dx <= range; ++dx)
                    {
                        int gx = cx + dx;
                        int gy = cy + dy;

                        if (gx < 0 || gx >= static_cast<int>(width) ||
                            gy < 0 || gy >= static_cast<int>(height))
                            continue;

                        double wx = origin[0] + gx * resolution;
                        double wy = origin[1] + gy * resolution;

                        // Transform world → obstacle frame
                        double lx =  c * (wx - r->x) + s * (wy - r->y);
                        double ly = -s * (wx - r->x) + c * (wy - r->y);

                        if (std::abs(lx) <= half_w && std::abs(ly) <= half_h)
                            (*occupancy_map)[gy][gx] = 100;
                    }
                }
            }
        }
    }

    std::vector<std::vector<int>> computeObstacleDistanceGrid()
    {
        const int INF = 1e9;
        std::vector<std::vector<int>> dist(height, std::vector<int>(width, INF));

        std::queue<std::pair<int,int>> q;

        for (int y = 0; y < (int)height; ++y)
        {
            for (int x = 0; x < (int)width; ++x)
            {
                if ((*occupancy_map)[y][x] > 0)   // obstacle
                {
                    dist[y][x] = 0;
                    q.emplace(x, y);
                }
            }
        }

        const int dx[8] = {1,-1,0,0,1,1,-1,-1};
        const int dy[8] = {0,0,1,-1,1,-1,1,-1};

        while (!q.empty())
        {
            auto [x, y] = q.front();
            q.pop();

            for (int k = 0; k < 8; ++k)
            {
                int nx = x + dx[k];
                int ny = y + dy[k];

                if (nx < 0 || ny < 0 || nx >= (int)width || ny >= (int)height)
                    continue;

                if (dist[ny][nx] > dist[y][x] + 1)
                {
                    dist[ny][nx] = dist[y][x] + 1;
                    q.emplace(nx, ny);
                }
            }
        }

        return dist;
    }


    



    void generateLandmarks(double landmarks_density,
                       const std::vector<BlindSpot>& blind_spots)
    {
        double width_dim  = width  * resolution;
        double height_dim = height * resolution;

        const size_t n_landmarks =
            std::round((width_dim * height_dim) * landmarks_density);

        if (n_landmarks == 0 || !occupancy_map)
            return;

        // distance transform
        auto dist_grid = computeObstacleDistanceGrid();

        const double r_min = 0.05;   // m
        const double r_max = 0.30;   // m

        const int r_min_cells = std::ceil(r_min / resolution);
        const int r_max_cells = std::ceil(r_max / resolution);

        // collect candidate cells 
        std::vector<std::pair<int,int>> candidates;
        candidates.reserve(width * height / 10);

        for (int y = 0; y < (int)height; ++y)
        {
            for (int x = 0; x < (int)width; ++x)
            {
                if ((*occupancy_map)[y][x] != 0)
                    continue;

                int d = dist_grid[y][x];
                if (d >= r_min_cells && d <= r_max_cells)
                    candidates.emplace_back(x, y);
            }
        }

        if (candidates.empty())
        {
            RCLCPP_WARN_STREAM(node->get_logger(),
                "No valid cells near obstacles for landmark generation");
            return;
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<size_t> cell_dist(0, candidates.size() - 1);
        std::uniform_real_distribution<> jitter(0.0, resolution);
        std::uniform_real_distribution<> val_z(0.0, 2.5);
        std::uniform_int_distribution<uint8_t> descriptor_dist(0, 255);

        landmarks.reserve(landmarks.size() + n_landmarks);

        // generate landmarks
        for (size_t i = 0; i < n_landmarks; ++i)
        {
            auto [cx, cy] = candidates[cell_dist(gen)];

            double x = origin[0] + cx * resolution + jitter(gen);
            double y = origin[1] + cy * resolution + jitter(gen);
            double z = val_z(gen);

            // blind spots check
            bool blind_hit = false;
            for (const auto& b : blind_spots)
            {
                double d2 = (x - b.x)*(x - b.x) + (y - b.y)*(y - b.y);
                if (d2 < b.r * b.r)
                {
                    blind_hit = true;
                    break;
                }
            }

            if (blind_hit)
            {
                --i;
                continue;
            }

            KeyPoint kp;
            kp.point.x = x;
            kp.point.y = y;
            kp.point.z = z;

            for (int k = 0; k < 32; ++k)
                kp.descriptor[k] = descriptor_dist(gen);

            landmarks.push_back(kp);
        }
    }



    void publishMapOccupancyGridIfNewSubscriber()
    {
        if (map_publisher->get_subscription_count() != map_subscription_count)
        {
            publishMapOccupancyGrid();
            map_subscription_count = map_publisher->get_subscription_count();
        }
    }

    void publishLandmarksMarkerIfNewSubscriber()
    {
        if (landmarks_marker_publisher->get_subscription_count() != landmarks_subscription_count)
        {
            publishLandmarksMarker();
            landmarks_subscription_count = landmarks_marker_publisher->get_subscription_count();
        }
    }

    void publishMapOccupancyGrid()
    {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.frame_id = "world";
        map_msg.header.stamp = node->get_clock()->now();
        map_msg.info.resolution = resolution;
        map_msg.info.width = width;
        map_msg.info.height = height;

        map_msg.info.origin.position.x = origin[0];
        map_msg.info.origin.position.y = origin[1];
        map_msg.info.origin.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, origin[2]);

        map_msg.info.origin.orientation.x = q.x();
        map_msg.info.origin.orientation.y = q.y();
        map_msg.info.origin.orientation.z = q.z();
        map_msg.info.origin.orientation.w = q.w();

        map_msg.data.resize(height * width);

        for (size_t h = 0; h < height; h++) {
            for (size_t w = 0; w < width; w++) {
                size_t index = h * width + w;
                map_msg.data[index] = (*occupancy_map)[h][w];
            }
        }

        map_publisher->publish(map_msg);
    }

    void publishLandmarksMarker()
    {
        visualization_msgs::msg::Marker marker = visualization_msgs::msg::Marker();

        marker.header.frame_id = "world";
        marker.header.stamp = node->get_clock()->now();

        marker.ns = "world";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.color.r = 0.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
        marker.color.a = 0.4f;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;

        for (const auto& landmark : landmarks) {
            geometry_msgs::msg::Point p;
            p.x = landmark.point.x;
            p.y = landmark.point.y;
            p.z = landmark.point.z;
            marker.points.push_back(p);
        }

        landmarks_marker_publisher->publish(marker);
    }

    double getResolution() { return resolution; }
    double getWidth() { return width; }
    double getHeight() { return height; }
    std::shared_ptr<std::vector<std::vector<int8_t>>> getOccupancyMap() { return occupancy_map; }
    std::vector<std::vector<int8_t>> getOccupancyMapCopy()
    {
        std::vector<std::vector<int8_t>> occupancy_map_copy = std::vector<std::vector<int8_t>>(height, std::vector<int8_t>(width));

        for (size_t h = 0; h < height; h++)
            for (size_t w = 0; w < width; w++)
                occupancy_map_copy[h][w] = (*occupancy_map)[h][w];

        return occupancy_map_copy;
    }
    std::vector<double>& getOrigin() { return origin; }
    std::vector<KeyPoint>& getLandmarks() { return landmarks; }

private:
    std::vector<KeyPoint> landmarks;

    size_t width, height;
    double resolution;
    std::vector<double> origin;
    std::shared_ptr<std::vector<std::vector<int8_t>>> occupancy_map;

    size_t map_subscription_count;
    size_t landmarks_subscription_count;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmarks_marker_publisher;
    rclcpp::Node::SharedPtr node;
};

#endif // ENVIRONMENT_H
