#include "mapping.h"


namespace mapping
{
    Mapping::Mapping()
        : mapping_thread_running(false)
    {
    }

    Mapping::Mapping(MappingParams &params)
        : params_(params), mapping_thread_running(false)
    {
    }

    Mapping::~Mapping()
    {
        mapping_thread_running.store(false);
        if (mapping_thread.joinable())
        {
            mapping_thread.join();
        }
    }


    MappingParams Mapping::params_from_yaml(std::string &params_path)
    {
        MappingParams p;

        try
        {
            YAML::Node config = YAML::LoadFile(params_path);

            if (config["mapping_rate"])
                p.mapping_rate = config["mapping_rate"].as<double>();

            if (config["map_resolution"])
                p.map_resolution = config["map_resolution"].as<double>();
            if (config["map_width"])
                p.map_width = config["map_width"].as<double>();
            if (config["map_height"])
                p.map_height = config["map_height"].as<double>();

            if (config["free_belief"])
                p.free_belief = config["free_belief"].as<double>();
            if (config["occupied_belief"])
                p.occupied_belief = config["occupied_belief"].as<double>();
            if (config["distance_belief_factor"])
                p.distance_belief_factor = config["distance_belief_factor"].as<double>();

            if (config["occupied_threshold"])
                p.occupied_threshold = config["occupied_threshold"].as<double>();
            if (config["free_threshold"])
                p.free_threshold = config["free_threshold"].as<double>();

            if (config["noise_model_radius"])
                p.noise_model_radius = config["noise_model_radius"].as<double>();
            if (config["noise_model_std_dev"])
                p.noise_model_std_dev = config["noise_model_std_dev"].as<double>();

        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
            std::cerr << "Using default parameters.\n";
        }

        return p;
    }


    void Mapping::init(MappingParams &params)
    {
        params_ = params;

        free_belief_log_odds = std::log(params_.free_belief / (1.0 - params_.free_belief));
        occupied_belief_log_odds = std::log(params_.occupied_belief / (1.0 - params_.occupied_belief));

        // Initialize map
        map_.resolution = static_cast<float>(params_.map_resolution);
        map_.width = static_cast<int>(params_.map_width / params_.map_resolution);
        map_.height = static_cast<int>(params_.map_height / params_.map_resolution);

        map_.origin_position = Eigen::Vector3d(-params_.map_width / 2.0,
                                               -params_.map_height / 2.0,
                                               0.0);

        map_.origin_orientation = Eigen::Quaterniond::Identity();

        map_.data.assign(map_.width * map_.height, -1);
        map_log_odds_data_.assign(map_.width * map_.height, 0.0);
    }

    void Mapping::start()
    {
        if (mapping_thread_running)
            return;

        mapping_thread_running.store(true);

        mapping_thread = std::thread([this]()
                                     {
                auto period = std::chrono::milliseconds(
                    static_cast<int>(1000.0 / params_.mapping_rate));

                auto next_time = std::chrono::steady_clock::now() + period;

                while (mapping_thread_running.load())
                {
                    mapping();

                    std::this_thread::sleep_until(next_time);
                    next_time += period;
                } });
    }

    void Mapping::add_posed_scan(const PosedScan &posed_scan)
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        posed_scan_buffer_.push_back(posed_scan);
    }

    Map Mapping::get_map()
    {
        return map_;
    }


    float Mapping::probability_to_log_odds(int8_t prob) {    
        float p = static_cast<float>(prob) / 100.0f;
        
        p = std::clamp(p, 0.001f, 0.999f);
        
        return std::log(p / (1.0f - p));
    }

    int8_t Mapping::log_odds_to_probability(float log_odds) {
        float p = 1.0f / (1.0f + std::exp(-log_odds));
        int8_t prob = static_cast<int8_t>(std::round(p * 100.0f));

        if (prob <= 0) prob = 0;
        if (prob >= 100) prob = 100;

        return prob;
    }



    void Mapping::bresenham_raytrace(int x0, int y0, int x1, int y1, bool hit_point)
    {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        double distance = std::sqrt((x1 - x0)*(x1 - x0) + (y1 - y0)*(y1 - y0)) * map_.resolution;

        while (true)
        {
            int i = y0 * map_.width + x0;
            if (x0 >= 0 && x0 < map_.width && y0 >= 0 && y0 < map_.height)
            {
                // Free cell
                if (map_.data[i] == -1)
                {
                    map_.data[i] = 50;
                    map_log_odds_data_[i] = probability_to_log_odds(50);
                }

                map_log_odds_data_[i] += free_belief_log_odds * (1.0 - params_.distance_belief_factor * distance);
                map_.data[i] = log_odds_to_probability(map_log_odds_data_[i]);
            }

            if (x0 == x1 && y0 == y1) break;

            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }

        // Occupied cell with noise model
        if (!hit_point)
        return;

        int radius = std::ceil(params_.noise_model_radius / map_.resolution);
        float std_dev = params_.noise_model_std_dev / map_.resolution;

        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dx = -radius; dx <= radius; ++dx)
            {
                if ((dx * dx + dy * dy) * map_.resolution * map_.resolution > params_.noise_model_radius * params_.noise_model_radius)
                    continue;

                int nx = x1 + dx;
                int ny = y1 + dy;

                if (nx >= 0 && nx < map_.width && ny >= 0 && ny < map_.height)
                {
                    float dist2 = dx * dx + dy * dy;
                    float weight = std::exp(-dist2 / (2.0f * std_dev * std_dev));

                    int idx = ny * map_.width + nx;

                    if (map_.data[idx] == -1)
                    {
                        map_.data[idx] = 50;
                        map_log_odds_data_[idx] = probability_to_log_odds(50);
                    }

                    map_log_odds_data_[idx] += (occupied_belief_log_odds * weight) * (1.0 - params_.distance_belief_factor * distance);
                    map_.data[idx] = log_odds_to_probability(map_log_odds_data_[idx]);
                }
            }
        }
    }


    void Mapping::mapping()
    {
        std::deque<PosedScan> posed_scan_buffer;

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            posed_scan_buffer = posed_scan_buffer_;

            posed_scan_buffer_.clear();
        }


        for(PosedScan& posed_scan : posed_scan_buffer)
        {
            int rx = static_cast<int>((posed_scan.position.x() - map_.origin_position.x()) / map_.resolution);
            int ry = static_cast<int>((posed_scan.position.y() - map_.origin_position.y()) / map_.resolution);

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
                
                // Local laser point
                double lx = range * cos(angle);
                double ly = range * sin(angle);

                // Transform to global
                double yaw = posed_scan.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];

                double gx = posed_scan.position.x() + lx * cos(yaw) - ly * sin(yaw);
                double gy = posed_scan.position.y() + lx * sin(yaw) + ly * cos(yaw);

                // Map indices of endpoint
                int mx = static_cast<int>((gx - map_.origin_position.x()) / map_.resolution);
                int my = static_cast<int>((gy - map_.origin_position.y()) / map_.resolution);

                bresenham_raytrace(rx, ry, mx, my, hit_point);

                angle += posed_scan.angle_increment;
            }
        }


        for (size_t i = 0; i < map_.data.size(); i++) {
            if (map_.data[i] == -1) continue;

            float val = map_.data[i] / 100.0f;
            if (val > params_.occupied_threshold) {
                map_.data[i] = 100;
            } else if (val < params_.free_threshold) {
                map_.data[i] = 0;
            } else {
                map_.data[i] = -1;
            }
        }

    }
}