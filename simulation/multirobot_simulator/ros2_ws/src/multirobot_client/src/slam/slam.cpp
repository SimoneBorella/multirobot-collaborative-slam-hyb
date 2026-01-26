#include "slam.h"

namespace multirobot_slam
{
    SLAM::SLAM()
        : slam_running_(false)
    {
    }

    SLAM::SLAM(SLAMParams &params)
        : params_(params), slam_running_(false)
    {
    }

    SLAM::~SLAM()
    {
        slam_running_.store(false);
        if (backend_thread_.joinable())
        {
            backend_thread_.join();
        }
        if (loop_closure_detection_thread_.joinable())
        {
            loop_closure_detection_thread_.join();
        }
        if (submap_mapping_thread_.joinable())
        {
            submap_mapping_thread_.join();
        }
    }

    SLAMParams SLAM::params_from_yaml(std::string &params_path)
    {
        SLAMParams p;

        p.backend_params = Backend::params_from_yaml(params_path);
        p.loop_closure_detector_params = LoopClosureDetector::params_from_yaml(params_path);
        p.submap_manager_params = SubmapManager::params_from_yaml(params_path);

        return p;
    }

    void SLAM::init(SLAMParams &params)
    {
        params_ = params;

        backend_.init(params_.backend_params);
        loop_closure_detector_.init(params_.loop_closure_detector_params);
        submap_manager_.init(params_.submap_manager_params);
    }

    void SLAM::start()
    {
        if (slam_running_)
            return;

        slam_running_.store(true);

        backend_thread_ = std::thread([this]()
                                           {
                auto period = std::chrono::milliseconds(
                    static_cast<int>(1000.0 / params_.backend_params.backend_rate));

                auto next_time = std::chrono::steady_clock::now() + period;

                while (slam_running_.load())
                {
                    backend_loop();

                    std::this_thread::sleep_until(next_time);
                    next_time += period;
                } });

        loop_closure_detection_thread_ = std::thread([this]()
                                           {
                auto period = std::chrono::milliseconds(
                    static_cast<int>(1000.0 / params_.loop_closure_detector_params.loop_closure_detection_rate));

                auto next_time = std::chrono::steady_clock::now() + period;

                while (slam_running_.load())
                {
                    loop_closure_detection_loop();

                    std::this_thread::sleep_until(next_time);
                    next_time += period;
                } });

        submap_mapping_thread_ = std::thread([this]()
                                           {
                auto period = std::chrono::milliseconds(
                    static_cast<int>(1000.0 / params_.submap_manager_params.submap_mapping_rate));

                auto next_time = std::chrono::steady_clock::now() + period;

                while (slam_running_.load())
                {
                    submap_mapping_loop();

                    std::this_thread::sleep_until(next_time);
                    next_time += period;
                } });
    }

    void SLAM::add_odom(OdomData &odom_data)
    {
        backend_.add_odom(odom_data);
    }

    void SLAM::add_imu(ImuData &imu_data)
    {
        backend_.add_imu(imu_data);
    }

    void SLAM::add_keypoints(KeypointsData &keypoints_data)
    {
        backend_.add_keypoints(keypoints_data);
    }

    State SLAM::get_state()
    {
        return backend_.get_state();
    }

    std::optional<State> SLAM::get_state_if_updated()
    {
        return backend_.get_state_if_updated();
    }




    void SLAM::backend_loop()
    {
        backend_.optimize();
    }

    void SLAM::loop_closure_detection_loop()
    {
        std::vector<KeyFrame> keyframes = backend_.get_keyframes();
        loop_closure_detector_.add_keyframes_to_db(keyframes);

        std::optional<LoopClosureConstraint> loop_closure_opt = loop_closure_detector_.detect(keyframes.back(), keyframes);

        if (loop_closure_opt.has_value())
        {
            backend_.add_loop_closure(loop_closure_opt.value());
            std::cout << "Loop closure detected between keyframes "
                      << loop_closure_opt->keyframe_i << " and "
                      << loop_closure_opt->keyframe_j
                      << " with score " << loop_closure_opt->score << std::endl;
        }
    }

    void SLAM::submap_mapping_loop()
    {

    }

}