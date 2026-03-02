#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "interfaces/msg/key_point.hpp"
#include "interfaces/msg/key_point_array.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>




class RGBDFeatureExtraction: public rclcpp::Node
{
public:
    RGBDFeatureExtraction()
        : Node("rgbd_feature_extraction")
    {
        declare_parameter("camera_frame", "oak");

        declare_parameter("fx", 525.0);
        declare_parameter("fy", 525.0);
        declare_parameter("cx", 319.5);
        declare_parameter("cy", 239.5);

        declare_parameter("max_keypoints", 200);
        declare_parameter("max_keypoints_3d", 80);

        declare_parameter("median_filter_depth", false);

        declare_parameter("min_range", 1.0);
        declare_parameter("max_range", 5.0);
        declare_parameter("border_filter", 50);


        camera_frame = get_parameter("camera_frame").as_string();

        fx = get_parameter("fx").as_double();
        fy = get_parameter("fy").as_double();
        cx = get_parameter("cx").as_double();
        cy = get_parameter("cy").as_double();

        max_keypoints = get_parameter("max_keypoints").as_int();
        max_keypoints_3d = get_parameter("max_keypoints_3d").as_int();

        median_filter_depth = get_parameter("median_filter_depth").as_bool();

        min_range = get_parameter("min_range").as_double();
        max_range = get_parameter("max_range").as_double();
        border_filter = get_parameter("border_filter").as_int();
    }

    void setup()
    {
        image_transport::ImageTransport it(shared_from_this());

        rgb_subscription = it.subscribe(
            "oak/rgb/image_raw", 10,
            std::bind(&RGBDFeatureExtraction::rgb_callback, this, std::placeholders::_1));

        depth_subscription = it.subscribe(
            "oak/stereo/image_raw", 10,
            std::bind(&RGBDFeatureExtraction::depth_callback, this, std::placeholders::_1));

        rgb_landmark_publisher = it.advertise("oak/rgb_keypoints/image_raw", 10);
        keypoints_marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("keypoints_marker", 10);
        keypoints_publisher = this->create_publisher<interfaces::msg::KeyPointArray>("keypoints", 10);
        
        orb = cv::ORB::create();
    }

private:
    void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        try
        {
            rgb_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
            process_images();
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (RGB): %s", e.what());
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        try
        {
            depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            process_images();
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (Depth): %s", e.what());
        }
    }

    bool getValidDepth(int x, int y, uint16_t& depth_out, cv::Mat& depth_image) {
        int radius = 2;
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                int nx = x + dx, ny = y + dy;
                if (nx >= 0 && ny >= 0 && nx < depth_image.cols && ny < depth_image.rows) {
                    uint16_t d = depth_image.at<uint16_t>(ny, nx);
                    if (d > 0) {
                        depth_out = d;
                        return true;
                    }
                }
            }
        }
        return false;
    }
    

    void process_images()
    {
        if (rgb_image.empty() || depth_image.empty())
            return;

        // ORB
        std::vector<cv::KeyPoint> raw_keypoints;
        cv::Mat raw_descriptors;
        orb->detectAndCompute(rgb_image, cv::noArray(), raw_keypoints, raw_descriptors);

        if (raw_keypoints.empty())
            return;

        // Quality sorting and filter N top
        std::vector<int> indices(raw_keypoints.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(), [&](int a, int b) {
            return raw_keypoints[a].response > raw_keypoints[b].response;
        });

        size_t keep = std::min((size_t)max_keypoints, raw_keypoints.size());
        
        // Results container preparation
        struct ValidFeature {
            geometry_msgs::msg::Point point;
            std::vector<uint8_t> descriptor;
        };
        std::vector<ValidFeature> valid_features;

        if(median_filter_depth)
            cv::medianBlur(depth_image, depth_image, 5);

        
        // £D projection
        for (size_t i = 0; i < keep; ++i)
        {
            int idx = indices[i];
            const cv::KeyPoint &kp = raw_keypoints[idx];
            
            int x = static_cast<int>(kp.pt.x);
            int y = static_cast<int>(kp.pt.y);

            // Border filtering
            if (x < border_filter || y < border_filter || 
                x >= (depth_image.cols - border_filter) || y >= (depth_image.rows - border_filter))
                continue;

            uint16_t depth;
            if (!getValidDepth(x, y, depth, depth_image))
                continue;
                
            float feature_z = depth * 0.001f; // Meters convertion

            if (feature_z < min_range || feature_z > max_range)
                continue;

            // Pinhole projection model
            // Coordinate change starting from camera frame (u,v,d)
            ValidFeature vf;
            vf.point.x = feature_z;
            vf.point.y = -(x - cx) * feature_z / fx;
            vf.point.z = -(y - cy) * feature_z / fy;

            // Descriptor conversion cv::Mat -> std::vector<uint8_t>
            cv::Mat desc_row = raw_descriptors.row(idx);
            vf.descriptor.assign(desc_row.begin<uint8_t>(), desc_row.end<uint8_t>());

            valid_features.push_back(vf);

            if (valid_features.size() >= max_keypoints_3d)
                break;
        }

        // Image publishing
        cv::Mat img_with_keypoints;
        cv::drawKeypoints(rgb_image, raw_keypoints, img_with_keypoints, cv::Scalar(0, 255, 0));
        auto rgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_with_keypoints).toImageMsg();
        rgb_landmark_publisher.publish(rgb_msg);

        // Marker publishing
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = camera_frame;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "keypoints_3d";
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.03; marker.scale.y = 0.03;
        marker.color.r = 1.0; marker.color.a = 1.0;

        // Keypoints publishing
        interfaces::msg::KeyPointArray keypoints_msg;
        keypoints_msg.header.frame_id = camera_frame;
        keypoints_msg.header.stamp = marker.header.stamp;

        for (const auto &vf : valid_features)
        {
            marker.points.push_back(vf.point);

            interfaces::msg::KeyPoint kp_msg;
            kp_msg.point = vf.point;

            for(size_t i=0; i<vf.descriptor.size(); i++)
                kp_msg.descriptor[i] = vf.descriptor[i];
            
            keypoints_msg.keypoints.push_back(kp_msg);
        }

        keypoints_marker_publisher->publish(marker);
        keypoints_publisher->publish(keypoints_msg);
    }

    image_transport::Subscriber rgb_subscription;
    image_transport::Subscriber depth_subscription;

    image_transport::Publisher rgb_landmark_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr keypoints_marker_publisher;
    rclcpp::Publisher<interfaces::msg::KeyPointArray>::SharedPtr keypoints_publisher;

    cv::Mat rgb_image;
    cv::Mat depth_image;

    cv::Ptr<cv::Feature2D> orb;

    std::string camera_frame;

    float fx, fy, cx, cy;
    size_t max_keypoints;
    size_t max_keypoints_3d;
    bool median_filter_depth;
    double min_range;
    double max_range;
    int border_filter;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RGBDFeatureExtraction>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
