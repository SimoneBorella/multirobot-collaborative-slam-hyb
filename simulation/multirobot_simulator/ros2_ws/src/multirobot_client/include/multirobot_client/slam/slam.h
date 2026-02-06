#ifndef SLAM_H
#define SLAM_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <mutex>

#include "data_types.h"

#include "backend.h"
#include "loop_closure_detector.h"
#include "submap_manager.h"


namespace multirobot_slam
{
    struct SLAMParams
    {
        BackendParams backend_params;
        LoopClosureDetectorParams loop_closure_detector_params;
        SubmapManagerParams submap_manager_params;
    };

    class SLAM
    {
    public:
        SLAM();
        SLAM(SLAMParams &params);
        ~SLAM();

        static SLAMParams params_from_yaml(std::string &params_path);

        void init(SLAMParams &params);
        void start();

        void add_odom(OdomData &odom_data);
        void add_imu(ImuData &imu_data);
        void add_keypoints(KeypointsData &keypoints_data);
        void add_posed_scan(PosedScan &posed_scan);
        State get_state();
        std::optional<State> get_state_if_updated();
        std::map<int, KeyFrame> get_keyframes();
        std::map<int, KeyFrame> get_keyframes_updates();
        std::map<int, Map> get_updated_submaps();
        
        
    private:
        void backend_loop();
        void loop_closure_detection_loop();
        void submap_mapping_loop();

        SLAMParams params_;

        Backend backend_;
        LoopClosureDetector loop_closure_detector_;
        SubmapManager submap_manager_;

        std::atomic<bool> slam_running_;

        std::thread backend_thread_;
        std::thread loop_closure_detection_thread_;
        std::thread submap_mapping_thread_;

        int keyframe_update_counter_;
        int keyframe_update_threshold_;
    };
}

#endif // SLAM_H
