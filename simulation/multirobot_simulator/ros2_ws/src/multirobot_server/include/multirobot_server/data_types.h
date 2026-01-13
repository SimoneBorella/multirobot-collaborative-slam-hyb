#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace multirobot_slam
{
    struct ImuData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d linear_acceleration;

        ImuData()
            : timestamp(0.0),
              orientation(Eigen::Quaterniond::Identity()),
              angular_velocity(Eigen::Vector3d::Zero()),
              linear_acceleration(Eigen::Vector3d::Zero()) {}

        ImuData(double timestamp,
                const Eigen::Quaterniond &orientation,
                const Eigen::Vector3d &angular_velocity,
                const Eigen::Vector3d &linear_acceleration)
            : timestamp(timestamp),
              orientation(orientation),
              angular_velocity(angular_velocity),
              linear_acceleration(linear_acceleration) {}
    };

    struct GNSSData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp;
        double latitude;
        double longitude;
        double altitude;
        Eigen::Matrix3d covariance;

        GNSSData()
            : timestamp(0.0),
              latitude(0.0),
              longitude(0.0),
              altitude(0.0),
              covariance(Eigen::Matrix3d::Identity() * 1e-2) {}

        GNSSData(double timestamp, double latitude, double longitude, double altitude)
            : timestamp(timestamp),
              latitude(latitude),
              longitude(longitude),
              altitude(altitude),
              covariance(Eigen::Matrix3d::Identity() * 1e-2) {}

        GNSSData(double timestamp, double latitude, double longitude, double altitude,
                 const Eigen::Matrix3d &covariance)
            : timestamp(timestamp),
              latitude(latitude),
              longitude(longitude),
              altitude(altitude),
              covariance(covariance) {}
    };

    struct OdomData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

        OdomData()
            : timestamp(0.0),
              position(Eigen::Vector3d::Zero()),
              orientation(Eigen::Quaterniond::Identity()) {}

        OdomData(double timestamp,
                 const Eigen::Vector3d &position,
                 const Eigen::Quaterniond &orientation)
            : timestamp(timestamp),
              position(position),
              orientation(orientation) {}
    };

    struct LandmarksData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp;
        std::vector<
            Eigen::Vector3d,
            Eigen::aligned_allocator<Eigen::Vector3d>>
            points;

        LandmarksData()
            : timestamp(0.0),
              points() {}

        LandmarksData(double timestamp,
                      const std::vector<
                          Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>> &points)
            : timestamp(timestamp),
              points(points) {}
    };

    struct State
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Quaterniond attitude;
        Eigen::Vector3d accelerometer_bias;
        Eigen::Vector3d gyroscope_bias;

        State()
            : position(Eigen::Vector3d::Zero()),
              velocity(Eigen::Vector3d::Zero()),
              attitude(Eigen::Quaterniond::Identity()),
              accelerometer_bias(Eigen::Vector3d::Zero()),
              gyroscope_bias(Eigen::Vector3d::Zero()) {}
    };

    struct Pose
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

        Pose()
            : position(),
              orientation() {}
    };

    struct Map
    {
        float resolution;
        int width;
        int height;
        Eigen::Vector3d origin_position;
        Eigen::Quaterniond origin_orientation;

        std::vector<int8_t> data;

        Map()
            : resolution(0.1f),
              width(0),
              height(0),
              origin_position(Eigen::Vector3d::Zero()),
              origin_orientation(Eigen::Quaterniond::Identity()),
              data() {}
    };

    struct MapLogOddsUpdate
    {
        float resolution;
        int width;
        int height;
        Eigen::Vector3d origin_position;
        Eigen::Quaterniond origin_orientation;

        std::vector<int> indicies;
        std::vector<float> delta_log_odds;

        MapLogOddsUpdate()
            : resolution(0.1f),
              width(0),
              height(0),
              origin_position(Eigen::Vector3d::Zero()),
              origin_orientation(Eigen::Quaterniond::Identity()),
              indicies(),
              delta_log_odds() {}
    };

    struct PosedScan
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

        float angle_min;
        float angle_max;
        float angle_increment;

        float time_increment;
        float scan_time;

        float range_min;
        float range_max;

        std::vector<float> ranges;
        std::vector<float> intensities;

        PosedScan()
            : timestamp(0.0),
              position(Eigen::Vector3d::Zero()),
              orientation(Eigen::Quaterniond::Identity()),
              angle_min(0.0f),
              angle_max(0.0f),
              angle_increment(0.0f),
              time_increment(0.0f),
              scan_time(0.0f),
              range_min(0.0f),
              range_max(0.0f),
              ranges(),
              intensities() {}

        PosedScan(double timestamp,
                  const Eigen::Vector3d &position,
                  const Eigen::Quaterniond &orientation,
                  float angle_min,
                  float angle_max,
                  float angle_increment,
                  float time_increment,
                  float scan_time,
                  float range_min,
                  float range_max,
                  const std::vector<float> &ranges,
                  const std::vector<float> &intensities)
            : timestamp(timestamp),
              position(position),
              orientation(orientation),
              angle_min(angle_min),
              angle_max(angle_max),
              angle_increment(angle_increment),
              time_increment(time_increment),
              scan_time(scan_time),
              range_min(range_min),
              range_max(range_max),
              ranges(ranges),
              intensities(intensities) {}
    };

    struct Frontier
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector2d centroid;
        double size;

        Frontier()
            : centroid(Eigen::Vector2d::Zero()),
              size(0.0) {}
    };

    struct Path
    {
        std::vector<Pose> poses;

        Path()
            : poses() {}
    };
}

#endif // DATA_TYPES_H