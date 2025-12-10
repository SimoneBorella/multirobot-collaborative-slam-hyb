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

#include "types.h"


class Environment
{
public:
    Environment() = default;

    Environment(rclcpp::Node::SharedPtr node, const std::string& map_yaml_path, const std::string& map_pgm_path, double landmarks_density)
    {
        this->node = node;
        map_subscription_count = 0;
        landmarks_subscription_count = 0;

        map_publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map_ground_truth", 10);
        landmarks_marker_publisher = node->create_publisher<visualization_msgs::msg::Marker>("landmarks/plot", 10);
    
        RCLCPP_INFO_STREAM(node->get_logger(), "Initializing environment...");

        loadMap(map_yaml_path, map_pgm_path);
        RCLCPP_INFO_STREAM(node->get_logger(), "Map loaded.");

        generateLandmarks(landmarks_density);
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

        for (size_t h = 0; h < height; h++) {
            for (size_t w = 0; w < width; w++) {
                uint8_t pixel = file.get();
                if (pixel == 0) {
                    (*occupancy_map)[h][w] = 100; // Occupied
                } else if (pixel == 255) {
                    (*occupancy_map)[h][w] = 0;   // Free
                } else {
                    (*occupancy_map)[h][w] = -1;  // Unknown
                }
            }
        }
    
        file.close();
    }

    void generateLandmarks(double landmarks_density)
    {
        double width_dim = width * resolution;
        double height_dim = height * resolution;

        size_t n_landmarks = round((width_dim*height_dim)*landmarks_density);


        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(origin[0], origin[0] + width_dim);
        std::uniform_real_distribution<> dis_y(origin[1], origin[1] + height_dim);

        
        for (size_t i = 0; i < n_landmarks; ++i) {
            double x = dis_x(gen);
            double y = dis_y(gen);

            size_t x_grid = std::clamp<size_t>(round((x - origin[0]) / resolution), 0, width - 1);
            size_t y_grid = std::clamp<size_t>(round((y - origin[1]) / resolution), 0, height - 1);
            
            if((*occupancy_map)[y_grid][x_grid] == 0)
            {
                Point landmark;
                landmark.x = x;
                landmark.y = y;
                landmarks.push_back(landmark);
            }
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
        map_msg.header.frame_id = "all";
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
        
        marker.header.frame_id = "all";
        marker.header.stamp = node->get_clock()->now();
        
        marker.ns = "all";
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
            p.x = landmark.x;
            p.y = landmark.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        landmarks_marker_publisher->publish(marker);
    }


    double getResolution()
    {
        return resolution;
    }

    double getWidth()
    {
        return width;
    }

    double getHeight()
    {
        return height;
    }

    std::shared_ptr<std::vector<std::vector<int8_t>>> getOccupancyMap()
    {
        return occupancy_map;
    }

    std::vector<std::vector<int8_t>> getOccupancyMapCopy()
    {
        std::vector<std::vector<int8_t>> occupancy_map_copy = std::vector<std::vector<int8_t>>(height, std::vector<int8_t>(width));

        for (size_t h = 0; h < height; h++)
            for (size_t w = 0; w < width; w++)
                occupancy_map_copy[h][w] = (*occupancy_map)[h][w];

        return occupancy_map_copy;
    }

    std::vector<double>& getOrigin()
    {
        return origin;
    }

    std::vector<Point>& getLandmarks()
    {
        return landmarks;
    }

    std::vector<Point> landmarks;

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
