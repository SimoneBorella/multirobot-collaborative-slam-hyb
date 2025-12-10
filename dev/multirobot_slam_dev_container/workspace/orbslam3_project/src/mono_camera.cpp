#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <filesystem>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <System.h>
#include <Tracking.h>


bool b_continue_session = true;

void exit_loop_handler(int s)
{
    cout << "Finishing session" << endl;
    b_continue_session = false;
}


int get_last_trajectory_id(const string& folder)
{
    int max_id = -1;

    try {
        bool found_trajectory = false;

        for (const auto& entry : std::filesystem::directory_iterator(folder)) {
            const auto& filename = entry.path().filename().string();
            
            if (filename.find("frame_trajectory_") == 0) {
                std::stringstream ss(filename.substr(strlen("frame_trajectory_"), filename.find_last_of(".") - strlen("frame_trajectory_")));
                int id;
                if (ss >> id) {
                    max_id = std::max(max_id, id);
                    found_trajectory = true;
                }
            }
        }

        if (!found_trajectory) {
            return -1;
        }

    } catch (const std::filesystem::filesystem_error& e) {
        cerr << "Filesystem error: " << e.what() << endl;
        return -1;
    }

    return max_id;
}


int main(int argc, char **argv)
{

    if (argc < 3 || argc > 4)
    {
        std::cerr << "Usage: ./mono_webcam path_to_vocabulary path_to_settings (trajectory_file_name)" << std::endl;
        return 1;
    }

    string file_name;
    bool bFileName = false;

    if (argc == 4)
    {
        file_name = string(argv[argc - 1]);
        bFileName = true;
    }



    int last_id = get_last_trajectory_id("./res");
    int id = last_id + 1;




    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cout << "Cannot open camera";
        return 1;
    }
    

    cout.precision(17);

    
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    cv::Mat imCV;

    double t_resize = 0.f;
    double t_track = 0.f;

    

    while (b_continue_session)
    {
        double timestamp_ms = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
        cap >> imCV;

        if (imageScale != 1.f)
        {
            int width = imCV.cols * imageScale;
            int height = imCV.rows * imageScale;
            cv::resize(imCV, imCV, cv::Size(width, height));
        }

        SLAM.TrackMonocular(imCV, timestamp_ms);

        if (SLAM.GetTrackingState() == ORB_SLAM3::Tracking::OK)
        {
            // Access the 3D map points
            std::vector<ORB_SLAM3::MapPoint*> mapPoints = SLAM.GetTrackedMapPoints();

            std::cout << "Number of 3D points: " << mapPoints.size() << std::endl;

            // Iterate through map points and print their 3D coordinates
            for (auto& mapPoint : mapPoints)
            {
                if (mapPoint)
                {
                    Eigen::Vector3f point3D = mapPoint->GetWorldPos();
                    std::cout << "Landmark point: " << point3D[0] << ", " << point3D[1] << ", " << point3D[2] << std::endl;
                }
            }

            // Save the trajectory
            SLAM.SaveTrajectoryEuRoC("./res/frame_trajectory_" + to_string(id) + ".txt");
        }
    }

    SLAM.Shutdown();

    return 0;
}
