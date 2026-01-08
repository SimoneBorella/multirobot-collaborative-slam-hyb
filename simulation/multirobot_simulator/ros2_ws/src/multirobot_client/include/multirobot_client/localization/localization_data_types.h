#ifndef LOCALIZATION_DATA_TYPES_H
#define LOCALIZATION_DATA_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace localization
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
            Eigen::aligned_allocator<Eigen::Vector3d>
        > points;

        LandmarksData()
            : timestamp(0.0),
              points() {}

        LandmarksData(double timestamp,
                      const std::vector<
                          Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>
                      > &points)
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
}

#endif // LOCALIZATION_DATA_TYPES_H
