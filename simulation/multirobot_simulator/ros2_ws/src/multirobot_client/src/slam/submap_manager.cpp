#include "submap_manager.h"

namespace multirobot_slam
{
    SubmapManager::SubmapManager()
    {
    }

    SubmapManager::SubmapManager(SubmapManagerParams &params)
        : params_(params)
    {
    }

    SubmapManagerParams SubmapManager::params_from_yaml(std::string &params_path)
    {
        SubmapManagerParams p;

        try
        {
            YAML::Node config = YAML::LoadFile(params_path);
            YAML::Node submap_manager_config = config["submap_manager"];

            if (submap_manager_config["submap_mapping_rate"])
                p.submap_mapping_rate = submap_manager_config["submap_mapping_rate"].as<double>();

            if (submap_manager_config["submap_resolution"])
                p.submap_resolution = submap_manager_config["submap_resolution"].as<double>();

            if (submap_manager_config["submap_width"])
                p.submap_width = submap_manager_config["submap_width"].as<double>();

            if (submap_manager_config["submap_height"])
                p.submap_height = submap_manager_config["submap_height"].as<double>();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
            std::cerr << "Using default parameters.\n";
        }


        return p;
    }

    void SubmapManager::init(SubmapManagerParams &params)
    {
        params_ = params;
    }

    void SubmapManager::update_keyframes(const std::vector<KeyFrame> &keyframes)
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        keyframes_ = keyframes;
    }

    void SubmapManager::add_posed_scan(const PosedScan &posed_scan)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        posed_scan_buffer_.push_back(posed_scan);
    }

    Submap SubmapManager::create_submap_from_keyframe(const KeyFrame& keyframe)
    {
        Submap submap;
        submap.keyframe_id = keyframe.keyframe_id;

        submap.resolution = params_.submap_resolution;
        submap.width = static_cast<int>(params_.submap_width / params_.submap_resolution);
        submap.height = static_cast<int>(params_.submap_height / params_.submap_resolution);

        submap.origin_position = keyframe.pose.position;
        submap.origin_orientation = keyframe.pose.orientation;

        submap.log_odds.resize(submap.width * submap.height, 0.0f);

        submap.timestamp_start = keyframe.timestamp;
        submap.timestamp_end = std::numeric_limits<double>::infinity();

        return submap;
    }

    int SubmapManager::idx(int x, int y, int width)
    {
        return y * width + x;
    }

    double SubmapManager::probability_to_log_odds(int8_t prob)
    {    
        double p = static_cast<double>(prob) / 100.0;
        
        p = std::clamp(p, 0.01, 0.99);
        
        return std::log(p / (1.0 - p));
    }

    int8_t SubmapManager::log_odds_to_probability(double log_odds)
    {
        double p = 1.0 / (1.0 + std::exp(-log_odds));
        int8_t prob = static_cast<int8_t>(std::round(p * 100.0));

        if (prob <= 0) prob = 0;
        if (prob >= 100) prob = 100;

        return prob;
    }

    void SubmapManager::bresenham_raytrace_submap(
        Submap& submap,
        int x0, int y0, int x1, int y1,
        bool hit_point,
        float free_logodds,
        float occ_logodds,
        float distance_factor,
        float noise_radius,
        float noise_std_dev)
    {
        int robot_x = x0;
        int robot_y = y0;

        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true)
        {
            if (x0 == x1 && y0 == y1)
                break;

            if (x0 >= 0 && x0 < submap.width && y0 >= 0 && y0 < submap.height)
            {
                int id = idx(x0, y0, submap.width);

                double cell_distance = std::hypot(x0 - robot_x, y0 - robot_y) * submap.resolution;
                double delta_log_odds = free_logodds * (1.0 - distance_factor * cell_distance);

                submap.log_odds[id] += delta_log_odds;
            }

            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }

        if (!hit_point)
            return;

        int radius = std::ceil(noise_radius / submap.resolution);
        float std_dev = noise_std_dev / submap.resolution;

        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dx = -radius; dx <= radius; ++dx)
            {
                if ((dx*dx + dy*dy) * submap.resolution * submap.resolution > noise_radius * noise_radius)
                    continue;

                int nx = x1 + dx;
                int ny = y1 + dy;

                if (nx >= 0 && nx < submap.width && ny >= 0 && ny < submap.height)
                {
                    float dist2 = dx*dx + dy*dy;
                    float weight = std::exp(-dist2 / (2.0f * std_dev * std_dev));

                    int id = idx(nx, ny, submap.width);
                    double cell_distance = std::hypot(nx - robot_x, ny - robot_y) * submap.resolution;

                    double delta_log_odds = (occ_logodds * weight) * (1.0 - distance_factor * cell_distance);
                    submap.log_odds[id] += delta_log_odds;
                }
            }
        }
    }



    void SubmapManager::submaps_mapping()
    {
        std::deque<PosedScan> posed_scan_buffer;

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            posed_scan_buffer = posed_scan_buffer_;

            posed_scan_buffer_.clear();
        }

        if(posed_scan_buffer.empty())
            return;


        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);

            if (keyframes_.size() > submaps_.size())
            {
                for (size_t i = submaps_.size(); i < keyframes_.size(); i++)
                {
                    const KeyFrame& keyframe = keyframes_[i];
                    Submap submap = create_submap_from_keyframe(keyframe);

                    if(!submaps_.empty())
                        submaps_.back().timestamp_end = keyframe.timestamp;

                    submaps_.push_back(submap);
                }
            }
        }

        for(PosedScan& posed_scan : posed_scan_buffer)
        {
            size_t submap_id = submaps_.size();

            for (size_t i = 0; i < submaps_.size(); i++)
            {
                if (posed_scan.timestamp >= submaps_[i].timestamp_start && posed_scan.timestamp <  submaps_[i].timestamp_end)
                {
                    submap_id = i;
                    break;
                }
            }

            if (submap_id == submaps_.size())
                continue;


            Submap& submap = submaps_[submap_id];

            // Transform robot pose in submap frame
            Eigen::Vector3d p = posed_scan.position - submap.origin_position;
            Eigen::Quaterniond q = submap.origin_orientation.inverse() * posed_scan.orientation;

            // HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE
            // Get robot position in submap map coordinates
            int robot_x = static_cast<int>(((p.x() + submap.width/2) / submap.resolution));
            int robot_y = static_cast<int>(((p.y() + submap.height/2) / submap.resolution));
            
            if (robot_x < 0 || robot_x >= submap.width || robot_y < 0 || robot_y >= submap.height)
            continue;
            
            double yaw = std::atan2(
                2.0 * (q.w() * q.z() + q.x() * q.y()),
                1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z())
            );


            // Iterate laser scan
            double angle = posed_scan.angle_min;

            for (size_t i = 0; i < posed_scan.ranges.size(); ++i)
            {
                bool hit_point = true;
                double range = posed_scan.ranges[i];

                if (range < posed_scan.range_min || std::isnan(range))
                {
                    angle += posed_scan.angle_increment;
                    continue;
                }

                if (range > posed_scan.range_max)
                {
                    range = posed_scan.range_max;
                    hit_point = false;
                }

                // local laser point in submap frame
                double laser_x = range * cos(angle);
                double laser_y = range * sin(angle);

                int laser_map_x = static_cast<int>((laser_x / submap.resolution) + submap.width / 2);
                int laser_map_y = static_cast<int>((laser_y / submap.resolution) + submap.height / 2);

                // Bresenham update
                bresenham_raytrace_submap(
                    submap,
                    robot_x, robot_y,
                    laser_map_x, laser_map_y,
                    hit_point,
                    /*free*/  -0.4f,
                    /*occ*/   +0.85f,
                    /*dist factor*/ 0.01f,
                    /*noise radius*/ 0.05f,
                    /*noise std*/    0.03f
                );

                angle += posed_scan.angle_increment;
            }
        }
    }

    Map SubmapManager::get_map()
    {
        Map map;
        // map.resolution = resolution;
        // map.width = width;
        // map.height = height;
        // map.origin_position = origin_position;
        // map.origin_orientation = origin_orientation;

        // map.data.resize(width * height);

        // for (size_t i = 0; i < log_odds.size(); ++i)
        // {
        //     float p = 1.0f - 1.0f / (1.0f + std::exp(log_odds[i]));
        //     map.data[i] = static_cast<int8_t>(std::round(p * 100.0f));
        // }

        return map;
    }

}