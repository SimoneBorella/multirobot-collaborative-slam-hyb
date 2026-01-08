#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include <thread>
#include <chrono>

class CameraCalibration : public rclcpp::Node
{
public:
    CameraCalibration()
        : Node("camera_calibration"), is_calibrated(false)
    {
        declare_parameter("board_size", std::vector<long int>());
        declare_parameter("square_size", 0.35);
        declare_parameter("required_frames", 20);

        std::vector<long int> board_size_vec = get_parameter("board_size").as_integer_array();
        board_size = cv::Size(board_size_vec[0], board_size_vec[1]);
        square_size = get_parameter("square_size").as_double();
        required_frames = get_parameter("required_frames").as_int();

        RCLCPP_INFO_STREAM(this->get_logger(), "CameraCalibration node starting in 5 seconds...\nMove checkerboard in view...");
    }

    void setup()
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        image_transport::ImageTransport it(shared_from_this());
        image_subscription = it.subscribe("oak/rgb/image_raw", 10, std::bind(&CameraCalibration::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        if (is_calibrated)
            return;

        cv::Mat image;
        try
        {
            image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(image, board_size, corners,
                                               cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
                                               
                                               
        if (found)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Checkboard found!");

            cv::Mat gray;
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
            image_points.push_back(corners);
            object_points.push_back(create_3d_corners());

            RCLCPP_INFO_STREAM(this->get_logger(), "Captured frame " << image_points.size() << "/" << required_frames);

            if (image_points.size() >= required_frames)
            {
                calibrate(gray.size());
            }


            cv::drawChessboardCorners(image, board_size, corners, found);
            cv::imshow("Calibration", image);
            cv::waitKey(2000);
        }

    }

    std::vector<cv::Point3f> create_3d_corners()
    {
        std::vector<cv::Point3f> corners;
        for (int i = 0; i < board_size.height; ++i)
        {
            for (int j = 0; j < board_size.width; ++j)
            {
                corners.emplace_back(j * square_size, i * square_size, 0.0f);
            }
        }
        return corners;
    }

    void calibrate(cv::Size image_size)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Starting calibration...");

        cv::Mat camera_matrix, dist_coeffs, rvecs, tvecs;
        double rms = cv::calibrateCamera(object_points, image_points, image_size,
                                         camera_matrix, dist_coeffs, rvecs, tvecs);

        is_calibrated = true;

        RCLCPP_INFO_STREAM(this->get_logger(), "Calibration done. RMS error = " << rms);
        RCLCPP_INFO_STREAM(this->get_logger(), "Camera Matrix:\n" << cv::format(camera_matrix, cv::Formatter::FMT_DEFAULT));
        RCLCPP_INFO_STREAM(this->get_logger(), "Distortion Coefficients:\n" << cv::format(dist_coeffs, cv::Formatter::FMT_DEFAULT));

        cv::FileStorage fs("camera_calibration.yaml", cv::FileStorage::WRITE);
        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coefficients" << dist_coeffs;
        fs.release();

        cv::destroyAllWindows();
    }

    image_transport::Subscriber image_subscription;
    cv::Size board_size;
    float square_size;
    size_t required_frames;
    bool is_calibrated;

    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraCalibration>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
