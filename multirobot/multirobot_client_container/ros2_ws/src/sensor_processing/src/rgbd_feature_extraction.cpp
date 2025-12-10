#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "interfaces/msg/point_array.hpp"


#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

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
        declare_parameter("max_landmarks", 100);

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
        max_landmarks = get_parameter("max_landmarks").as_int();

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

        rgb_landmark_publisher = it.advertise("oak/rgb_landmarks/image_raw", 10);
        landmarks_marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("landmarks_marker", 10);
        landmarks_publisher = this->create_publisher<interfaces::msg::PointArray>("landmarks", 10);
        
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

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb->detectAndCompute(rgb_image, cv::noArray(), keypoints, descriptors);

        std::sort(keypoints.begin(), keypoints.end(),
                  [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
                      return a.response > b.response;
                  });

        if (keypoints.size() > max_keypoints)
            keypoints.resize(max_keypoints);

        
        cv::Mat img_with_keypoints;
        cv::drawKeypoints(rgb_image, keypoints, img_with_keypoints, cv::Scalar(0, 255, 0));

        if(median_filter_depth)
            cv::medianBlur(depth_image, depth_image, 5);




        std::vector<cv::Point3f> landmarks;
        for (const auto &kp : keypoints)
        {
            int x = static_cast<int>(kp.pt.x);
            int y = static_cast<int>(kp.pt.y);

            if (x < border_filter || y < border_filter || x >= (depth_image.cols - border_filter) || y >= (depth_image.rows - border_filter))
                continue;


            uint16_t depth;
            if (!getValidDepth(x, y, depth, depth_image))
                continue;
                

            float feature_z = depth * 0.001;

            if (feature_z < min_range || feature_z > max_range)
                continue;

            float feature_x = (x - cx) * feature_z / fx;
            float feature_y = (y - cy) * feature_z / fy;

            landmarks.emplace_back(feature_x, feature_y, feature_z);
        }

        if (landmarks.size() > max_landmarks)
            landmarks.resize(max_landmarks);

        {
            sensor_msgs::msg::Image::SharedPtr rgb_landmarks_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_with_keypoints).toImageMsg();
            rgb_landmark_publisher.publish(rgb_landmarks_msg);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = camera_frame;
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "landmarks";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::POINTS;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

            marker.scale.x = 0.05;
            marker.scale.y = 0.05;

            for (const auto &pt : landmarks)
            {
                geometry_msgs::msg::Point p;
                p.x = pt.z;
                p.y = -pt.x;
                p.z = -pt.y;
                marker.points.push_back(p);
            }
            landmarks_marker_publisher->publish(marker);
        }

        {
            interfaces::msg::PointArray landmarks_msg = interfaces::msg::PointArray();
        
            landmarks_msg.header.frame_id = camera_frame;
            landmarks_msg.header.stamp = this->get_clock()->now();
            
            for (const auto &pt : landmarks)
            {
                geometry_msgs::msg::Point p;
                p.x = pt.z;
                p.y = -pt.x;
                p.z = -pt.y;
                landmarks_msg.points.push_back(p);
            }

            landmarks_publisher->publish(landmarks_msg);
        }

    }

    image_transport::Subscriber rgb_subscription;
    image_transport::Subscriber depth_subscription;

    image_transport::Publisher rgb_landmark_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmarks_marker_publisher;
    rclcpp::Publisher<interfaces::msg::PointArray>::SharedPtr landmarks_publisher;

    cv::Mat rgb_image;
    cv::Mat depth_image;

    cv::Ptr<cv::Feature2D> orb;

    std::string camera_frame;

    float fx, fy, cx, cy;
    size_t max_keypoints;
    size_t max_landmarks;
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
