#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace multirobot_slam
{
    struct Pose
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;

        Pose() = default;

        Pose(Eigen::Vector3d position,
             Eigen::Quaterniond orientation)
            : position(position),
              orientation(orientation) {}
    };

    struct ImuData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d linear_acceleration;
    };

    struct GNSSData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp;
        double latitude;
        double longitude;
        double altitude;
        Eigen::Matrix3d covariance;
    };

    struct OdomData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
    };

    struct Keypoint
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d point;
        std::array<uint8_t, 32> descriptor;

        Keypoint() = default;

        Keypoint(Eigen::Vector3d point,
                 std::array<uint8_t, 32> descriptor)
            : point(point),
              descriptor(descriptor) {}
    };

    struct KeypointsData
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp;
        std::vector<Keypoint, Eigen::aligned_allocator<Keypoint>> keypoints;
        Pose pose;
    };

    struct KeyFrame
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp;
        int keyframe_id;
        std::pair<char, int> pose_symbol;
        Pose pose;
        Eigen::Matrix<double, 6, 6> covariance;
        std::vector<Keypoint, Eigen::aligned_allocator<Keypoint>> keypoints;
        int keypoints_number;
        bool is_active = false;
    };

    struct LoopClosureConstraint
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        int keyframe_i;
        int keyframe_j;
        Pose transform_pose;
        double score;
    };

    struct State
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        Eigen::Matrix<double, 6, 6> covariance;

        Eigen::Vector3d velocity;
        Eigen::Vector3d accelerometer_bias;
        Eigen::Vector3d gyroscope_bias;
    };


    struct Map
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        int keyframe_id;
        float resolution;
        int width;
        int height;
        Eigen::Vector3d origin_position;
        Eigen::Quaterniond origin_orientation;

        std::vector<int8_t> data;
    };

    struct MapLogOddsUpdate
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        float resolution;
        int width;
        int height;
        Eigen::Vector3d origin_position;
        Eigen::Quaterniond origin_orientation;

        std::vector<int> indices;
        std::vector<float> delta_log_odds;
    };

    struct FrontierMapUpdate
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        float resolution;
        int width;
        int height;
        Eigen::Vector3d origin_position;
        Eigen::Quaterniond origin_orientation;

        std::vector<int> frontier_indices;
        std::vector<int> explored_indices;
    };


    struct Submap
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        int keyframe_id;
        float resolution;
        int width;
        int height;
        Eigen::Vector3d origin_position;
        Eigen::Quaterniond origin_orientation;

        std::vector<float> log_odds;

        double timestamp_start;
        double timestamp_end;
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
    };

    struct Frontier
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector2d centroid;
        double size;
    };

    struct Task
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Pose pose;
        bool oriented;
    };

    struct Path
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::vector<Pose, Eigen::aligned_allocator<Pose>> poses;
        Eigen::Quaterniond final_orientation;
    };

    struct VelCmd
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d linear;
        Eigen::Vector3d angular;
    };
}

#endif // DATA_TYPES_H