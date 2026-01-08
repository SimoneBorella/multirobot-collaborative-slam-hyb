#ifndef MAPPING_DATA_TYPES_H
#define MAPPING_DATA_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace mapping
{

    struct Map
    {
        float resolution;
        int width;
        int height;
        Eigen::Vector3d origin_position;
        Eigen::Quaterniond origin_orientation;

        std::vector<int8_t> data; // occupancy data: 0-100 occupied, -1 unknown

        Map()
            : resolution(0.1f),
              width(0),
              height(0),
              origin_position(Eigen::Vector3d::Zero()),
              origin_orientation(Eigen::Quaterniond::Identity()),
              data() {}
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
}

#endif // MAPPING_DATA_TYPES_H