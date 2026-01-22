#include "lidar.h"


Lidar::Lidar(
    rclcpp::Node::SharedPtr node,
    const std::string& robot_name,
    Position& robot_position,
    std::shared_ptr<Environment> environment,
    const std::string& name,
    Position& position,
    double frequency,
    double min_range,
    double max_range,
    double resolution,
    int points,
    std::string& scan_topic,
    std::string& pcl_topic)
    : Sensor(node, robot_name, robot_position, environment, name, position, frequency),
      min_range(min_range),
      max_range(max_range),
      resolution(resolution),
      points(points)
{
    scan_publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);
    pcl_publisher  = node->create_publisher<sensor_msgs::msg::PointCloud2>(pcl_topic, 10);

    scan_plot_publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic + "/plot", 10);
    pcl_plot_publisher  = node->create_publisher<sensor_msgs::msg::PointCloud2>(pcl_topic + "/plot", 10);

    occupancy_map_aux = environment->getOccupancyMapCopy();

    inv_resolution_ = 1.0 / environment->getResolution();

    ray_cos_.resize(points);
    ray_sin_.resize(points);
    for (int i = 0; i < points; ++i) {
        double a = i * resolution;
        ray_cos_[i] = std::cos(a);
        ray_sin_[i] = std::sin(a);
    }

    lidar_points_.reserve(points);
}




void Lidar::sensorUpdate()
{
    auto& points = getLidarPoints();
    publishLidarScan(points);
}



bool Lidar::bresenhamRaytrace(
    int x0, int y0,
    int x1, int y1,
    int map_w, int map_h,
    const std::vector<double>& origin,
    double lidar_x, double lidar_y,
    double cos_base, double sin_base,
    std::vector<Point>& points)
{
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    int x = x0, y = y0;

    while (true) {
        if (x < 0 || y < 0 || x >= map_w || y >= map_h)
            break;

        double wx = origin[0] + x / inv_resolution_;
        double wy = origin[1] + y / inv_resolution_;

        if (occupancy_map_aux[y][x] == 100 || hitRobot(wx, wy)) {
            double tx = wx - lidar_x;
            double ty = wy - lidar_y;

            points.push_back({
                tx * cos_base + ty * sin_base,
               -tx * sin_base + ty * cos_base
            });
            return true;
        }

        if (x == x1 && y == y1)
            break;

        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
    return false;
}

inline bool Lidar::hitRobot(double wx, double wy) const
{
    for (size_t i = 0; i < other_robots_positions.size(); ++i) {
        double dx = wx - other_robots_positions[i].x;
        double dy = wy - other_robots_positions[i].y;
        double r  = other_robots_shapes[i].diameter * 0.5;
        if (dx * dx + dy * dy <= r * r)
            return true;
    }
    return false;
}


std::vector<Point>& Lidar::getLidarPoints()
{
    lidar_points_.clear();

    int w = environment->getWidth();
    int h = environment->getHeight();
    auto origin = environment->getOrigin();

    double lidar_x = robot_position.x
        + position.x * std::cos(robot_position.theta)
        - position.y * std::sin(robot_position.theta);

    double lidar_y = robot_position.y
        + position.x * std::sin(robot_position.theta)
        + position.y * std::cos(robot_position.theta);

    double base_angle = robot_position.theta + position.theta;
    double cos_base = std::cos(base_angle);
    double sin_base = std::sin(base_angle);

    int sx = static_cast<int>((lidar_x - origin[0]) * inv_resolution_);
    int sy = static_cast<int>((lidar_y - origin[1]) * inv_resolution_);

    for (int i = 0; i < points; ++i) {
        double dx = cos_base * ray_cos_[i] - sin_base * ray_sin_[i];
        double dy = sin_base * ray_cos_[i] + cos_base * ray_sin_[i];

        int ex = static_cast<int>((lidar_x + max_range * dx - origin[0]) * inv_resolution_);
        int ey = static_cast<int>((lidar_y + max_range * dy - origin[1]) * inv_resolution_);

        if (!bresenhamRaytrace(sx, sy, ex, ey, w, h, origin,
                               lidar_x, lidar_y,
                               cos_base, sin_base,
                               lidar_points_)) {
            lidar_points_.push_back({INFINITY, INFINITY});
        }
    }
    return lidar_points_;
}







void Lidar::publishLidarScan(std::vector<Point>& lidar_points)
{
    sensor_msgs::msg::LaserScan scan;

    scan.header.stamp = node->now();
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

    pcl.header.stamp = node->now();
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