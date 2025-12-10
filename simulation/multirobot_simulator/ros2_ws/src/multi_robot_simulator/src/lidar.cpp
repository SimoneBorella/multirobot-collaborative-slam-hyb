#include "lidar.h"

void Lidar::sensorUpdate()
{
    std::vector<Point> lidar_points = getLidarPoints();
    publishLidarScan(lidar_points);
    publishLidarPointCloud2(lidar_points);
}


bool Lidar::bresenhamRaytrace(int x0, int y0, int x1, int y1, int occupancy_map_width, int occupancy_map_height, double env_resolution, std::vector<double>& origin, double lidar_global_x, double lidar_global_y, std::vector<Point>& lidar_points)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    int x = x0, y = y0;
    bool hit = false;

    while (true) {
        if (x < 0 || y < 0 || x >= occupancy_map_width || y >= occupancy_map_height)
            break;

        if (occupancy_map_aux[y][x] == 100) {
            double hit_x = origin[0] + x * env_resolution;
            double hit_y = origin[1] + y * env_resolution;

            double translated_x = hit_x - lidar_global_x;
            double translated_y = hit_y - lidar_global_y;

            double angle = -(robot_position.theta + position.theta);
            double rototranslated_x = translated_x * cos(angle) - translated_y * sin(angle);
            double rototranslated_y = translated_x * sin(angle) + translated_y * cos(angle);

            lidar_points.push_back(Point{rototranslated_x, rototranslated_y});
            hit = true;
            break;
        }

        if (x == x1 && y == y1)
            break;

        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }

    return hit;
}

std::vector<Point> Lidar::getLidarPoints()
{
    std::vector<Point> lidar_points;

    // Get environment data
    int occupancy_map_width = environment->getWidth();
    int occupancy_map_height = environment->getHeight();
    std::vector<double> origin = environment->getOrigin();
    double env_resolution = environment->getResolution();

    // Add robots to occupancy_map_aux
    std::vector<Position> curr_other_robots_positions;
    std::vector<Shape> curr_other_robots_shapes;

    for(size_t i=0; i<other_robots_positions.size(); i++)
    {
        Position p = other_robots_positions[i];
        Shape s = other_robots_shapes[i];

        curr_other_robots_positions.push_back(p);
        curr_other_robots_shapes.push_back(s);
        
        int circle_center_x = round((p.x - origin[0]) / env_resolution);
        int circle_center_y = round((p.y - origin[1]) / env_resolution);

        double radius_in_cells = s.diameter / (2 * env_resolution); 

        for (int h = circle_center_y - round(radius_in_cells); h < circle_center_y + round(radius_in_cells); h++) {
            if (h>=0 && h<occupancy_map_height)
                for (int w = circle_center_x - round(radius_in_cells); w < circle_center_x + round(radius_in_cells); w++) {
                    if (w>=0 && w<occupancy_map_width)
                    {
                        double distance = sqrt(pow(h - circle_center_y, 2) + pow(w - circle_center_x, 2));

                        if (distance <= radius_in_cells) {
                            occupancy_map_aux[h][w] = 100;
                        }
                    }
                }
        }
    }



    // Scan

    // double lidar_global_x = robot_position.x + position.x * cos(robot_position.theta) - position.y * sin(robot_position.theta);
    // double lidar_global_y = robot_position.y + position.x * sin(robot_position.theta) + position.y * cos(robot_position.theta);

    // for (int i = 0; i < points; i++) {
    //     double ray_theta = fmod((robot_position.theta + position.theta) + i * resolution, 2 * M_PI);
    
    //     bool hit = false;
    //     for (double distance = min_range; distance < max_range; distance += env_resolution)
    //     {
    //         double curr_x = lidar_global_x + distance * cos(ray_theta);
    //         double curr_y = lidar_global_y + distance * sin(ray_theta);
    
    //         int curr_x_grid = round((curr_x - origin[0]) / env_resolution);
    //         int curr_y_grid = round((curr_y - origin[1]) / env_resolution);
    
    //         if (curr_y_grid >= 0 && curr_x_grid >= 0 && curr_y_grid < occupancy_map_height && curr_x_grid < occupancy_map_width)
    //         {
    //             if (occupancy_map_aux[curr_y_grid][curr_x_grid] == 100)
    //             {
    //                 double translated_x = curr_x - lidar_global_x;
    //                 double translated_y = curr_y - lidar_global_y;
    
    //                 double rototranslated_x = translated_x * cos(-(robot_position.theta + position.theta)) - translated_y * sin(-(robot_position.theta + position.theta));
    //                 double rototranslated_y = translated_x * sin(-(robot_position.theta + position.theta)) + translated_y * cos(-(robot_position.theta + position.theta));
    
    //                 lidar_points.push_back(Point{rototranslated_x, rototranslated_y});
    
    //                 hit = true;
    //                 break;
    //             }
    //         }
    //     }
    //     if (!hit)
    //     {
    //         lidar_points.push_back(Point{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()});
    //     }
    // }




    // Scan

    double lidar_global_x = robot_position.x + position.x * cos(robot_position.theta) - position.y * sin(robot_position.theta);
    double lidar_global_y = robot_position.y + position.x * sin(robot_position.theta) + position.y * cos(robot_position.theta);

    for (int i = 0; i < points; i++)
    {
        double ray_theta = fmod((robot_position.theta + position.theta) + i * resolution, 2 * M_PI);

        double end_x = lidar_global_x + max_range * cos(ray_theta);
        double end_y = lidar_global_y + max_range * sin(ray_theta);

        int start_x_grid = round((lidar_global_x - origin[0]) / env_resolution);
        int start_y_grid = round((lidar_global_y - origin[1]) / env_resolution);
        int end_x_grid   = round((end_x - origin[0]) / env_resolution);
        int end_y_grid   = round((end_y - origin[1]) / env_resolution);

        bool hit = bresenhamRaytrace(start_x_grid, start_y_grid, end_x_grid, end_y_grid, occupancy_map_width, occupancy_map_height, env_resolution, origin, lidar_global_x, lidar_global_y, lidar_points);
        
        if (!hit)
        {
            lidar_points.push_back(Point{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()});
        }
    }



    // Remove robots to occupancy_map_aux

    for(size_t i=0; i<curr_other_robots_positions.size(); i++)
    {
        Position p = curr_other_robots_positions[i];
        Shape s = curr_other_robots_shapes[i];
        
        int circle_center_x = round((p.x - origin[0]) / env_resolution);
        int circle_center_y = round((p.y - origin[1]) / env_resolution);

        double radius_in_cells = s.diameter / (2 * env_resolution); 

        for (int h = circle_center_y - round(radius_in_cells); h < circle_center_y + round(radius_in_cells); h++) {
            if (h>=0 && h<occupancy_map_height)
                for (int w = circle_center_x - round(radius_in_cells); w < circle_center_x + round(radius_in_cells); w++) {
                    if (w>=0 && w<occupancy_map_width)
                    {
                        double distance = sqrt(pow(h - circle_center_y, 2) + pow(w - circle_center_x, 2));

                        if (distance <= radius_in_cells) {
                            occupancy_map_aux[h][w] = 0;
                        }
                    }
                }
        }
    }

    return lidar_points;
}






void Lidar::publishLidarScan(std::vector<Point>& lidar_points)
{
    sensor_msgs::msg::LaserScan scan;

    scan.header.stamp = node->get_clock()->now();
    scan.header.frame_id = name + "_link";
    scan.angle_min = 0.0;
    scan.angle_max = 2 * M_PI;
    scan.angle_increment = resolution;
    scan.time_increment = 0.0;
    scan.scan_time = 0.0;
    scan.range_min = min_range;
    scan.range_max = max_range;

    // Set ranges and intensities
    scan.ranges.resize(points);
    scan.intensities.resize(points);

    for (size_t i = 0; i < lidar_points.size(); i++)
    {
        double distance = std::hypot(lidar_points[i].x, lidar_points[i].y);

        if (std::isinf(distance))
        {
            scan.ranges[i] = std::numeric_limits<float>::infinity();
            scan.intensities[i] = 0.0;
        }
        else
        {
            scan.ranges[i] = static_cast<float>(distance);
            scan.intensities[i] = 0.0;
        }
    }

    scan_publisher->publish(scan);

    scan.header.frame_id = robot_name + "/" + name + "_link";

    scan_plot_publisher->publish(scan);

}



void Lidar::publishLidarPointCloud2(std::vector<Point>& lidar_points)
{
    sensor_msgs::msg::PointCloud2 pcl;

    pcl.header.stamp = node->get_clock()->now();
    pcl.header.frame_id = name + "_link";
    pcl.height = 1;
    pcl.width = points;

    pcl.fields.resize(3);

    pcl.fields[0].name = "x"; pcl.fields[0].offset = 0; pcl.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; pcl.fields[0].count = 1;
    pcl.fields[1].name = "y"; pcl.fields[1].offset = 4; pcl.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; pcl.fields[1].count = 1;
    pcl.fields[2].name = "z"; pcl.fields[2].offset = 8; pcl.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; pcl.fields[2].count = 1;

    pcl.is_bigendian = false;
    pcl.point_step = 12;
    pcl.row_step = pcl.point_step * pcl.width;
    pcl.is_dense = true;
    pcl.data.resize(pcl.row_step * pcl.height);

    
    for (size_t i = 0; i < lidar_points.size(); i++)
    {
        size_t p_offset = i * pcl.point_step;
        
        float x = static_cast<float>(lidar_points[i].x);
        float y = static_cast<float>(lidar_points[i].y);
        float z = 0.0;

        std::memcpy(&pcl.data[p_offset + 0], &x, sizeof(float));
        std::memcpy(&pcl.data[p_offset + 4], &y, sizeof(float));
        std::memcpy(&pcl.data[p_offset + 8], &z, sizeof(float));
    }

    pcl_publisher->publish(pcl);

    pcl.header.frame_id = robot_name + "/" + name + "_link";

    pcl_plot_publisher->publish(pcl);
}