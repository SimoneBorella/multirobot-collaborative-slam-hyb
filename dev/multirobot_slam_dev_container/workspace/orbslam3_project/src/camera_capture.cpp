#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Open a video capture object (camera 0 is usually the default webcam)
    cv::VideoCapture cap(0);

    // Check if the camera is opened correctly
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to access the camera." << std::endl;
        return -1;
    }

    cv::Mat frame;

    while (true) {
        // Capture a frame-by-frame
        cap >> frame;

        // If the frame is empty, break the loop
        if (frame.empty()) {
            std::cerr << "Error: Failed to capture frame." << std::endl;
            break;
        }

        // Display the resulting frame
        cv::imshow("Camera Feed", frame);

        // Press 'q' to exit the video display
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Release the capture object and close all OpenCV windows
    cap.release();
    cv::destroyAllWindows();

    return 0;
}
