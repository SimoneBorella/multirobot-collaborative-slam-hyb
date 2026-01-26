#ifndef SUBMAP_MANAGER_H
#define SUBMAP_MANAGER_H

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

#include "data_types.h"

namespace multirobot_slam
{
    struct SubmapManagerParams
    {
        double submap_mapping_rate;

        SubmapManagerParams(
            double submap_mapping_rate = 1.0)
            : submap_mapping_rate(submap_mapping_rate) {}
    };

    class SubmapManager
    {
    public:
        SubmapManager();
        SubmapManager(SubmapManagerParams &params);

        static SubmapManagerParams params_from_yaml(std::string &params_path);
        void init(SubmapManagerParams &params);

    private:
        SubmapManagerParams params_;

    };
}

#endif // SUBMAP_MANAGER_H
