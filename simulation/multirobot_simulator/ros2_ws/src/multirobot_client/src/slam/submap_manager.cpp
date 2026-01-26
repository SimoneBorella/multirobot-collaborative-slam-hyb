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
}