#include "global_planning.h"


namespace multirobot_slam
{
    GlobalPlanning::GlobalPlanning()
        : costmap_received_(false)
    {
    }

    GlobalPlanning::GlobalPlanning(GlobalPlanningParams &params)
        : params_(params), costmap_received_(false)
    {
    }

    GlobalPlanningParams GlobalPlanning::params_from_yaml(std::string &params_path)
    {
        GlobalPlanningParams p;

        try
        {
            YAML::Node config = YAML::LoadFile(params_path);

            if (config["heuristic"])
                p.heuristic = config["heuristic"].as<std::string>();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
            std::cerr << "Using default parameters.\n";
        }

        return p;
    }

    void GlobalPlanning::init(GlobalPlanningParams &params)
    {
        params_ = params;

        directions_ = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,-1},{1,-1},{-1,1}};
        directions_cost_ = {1.0, 1.0, 1.0, 1.0, std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)};
    }

    void GlobalPlanning::update_costmap(Map costmap)
    {
        costmap_ = costmap;
        costmap_received_ = true;
    }



    double GlobalPlanning::heuristic(int x1, int y1, int x2, int y2)
    {
        if (params_.heuristic == "manhattan")
            return std::abs(x1 - x2) + std::abs(y1 - y2);
        else if (params_.heuristic == "euclidean")
            return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
        else
            throw std::runtime_error("Unknown heuristic type: " + params_.heuristic);
    }


    // Path GlobalPlanning::reconstructPath(std::shared_ptr<AStarNode> last)
    // {
    //     Path path;

    //     std::vector<Pose, Eigen::aligned_allocator<Pose>> temp_poses;

    //     std::shared_ptr<AStarNode> current = last;

    //     while (current) {
    //         Pose pose;
    //         pose.position.x() = current->x * costmap_.resolution + costmap_.origin_position.x();
    //         pose.position.y() = current->y * costmap_.resolution + costmap_.origin_position.y();

    //         temp_poses.push_back(pose);
    //         current = current->parent;
    //     }

    //     std::reverse(temp_poses.begin(), temp_poses.end());

    //     // Compute orientation
    //     for(size_t i = 0; i < temp_poses.size(); ++i) {
    //         Pose pose = temp_poses[i];

    //         if(i+1 < temp_poses.size()) {
    //             double dx = temp_poses[i+1].position.x() - temp_poses[i].position.x();
    //             double dy = temp_poses[i+1].position.y() - temp_poses[i].position.y();
    //             double yaw = std::atan2(dy, dx);

    //             pose.orientation = Eigen::Quaterniond(
    //                 Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
    //             );

    //         } else if(i > 0) {
    //             pose.orientation = temp_poses[i-1].orientation;
    //         } else {
    //             pose.orientation.w() = 1.0;
    //         }

    //         path.poses.push_back(pose);
    //     }

    //     return path;
    // }

    Path GlobalPlanning::reconstructPath(std::shared_ptr<AStarNode> last)
    {
        Path path;
        std::vector<Pose, Eigen::aligned_allocator<Pose>> temp_poses;
        std::shared_ptr<AStarNode> current = last;

        while (current && current->parent) {
            int cell_cost = costmap_.data[current->y * costmap_.width + current->x];
            
            if (cell_cost >= 5) {
                current = current->parent;
            } else {
                break;
            }
        }

        while (current) {
            Pose pose;
            pose.position.x() = current->x * costmap_.resolution + costmap_.origin_position.x();
            pose.position.y() = current->y * costmap_.resolution + costmap_.origin_position.y();
            
            temp_poses.push_back(pose);
            current = current->parent;
        }

        std::reverse(temp_poses.begin(), temp_poses.end());

        for(size_t i = 0; i < temp_poses.size(); ++i) {
            Pose pose = temp_poses[i];
            if(i+1 < temp_poses.size()) {
                double dx = temp_poses[i+1].position.x() - temp_poses[i].position.x();
                double dy = temp_poses[i+1].position.y() - temp_poses[i].position.y();
                double yaw = std::atan2(dy, dx);
                pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
            } else if(i > 0) {
                pose.orientation = temp_poses[i-1].orientation;
            } else {
                pose.orientation.w() = 1.0;
            }
            path.poses.push_back(pose);
        }

        return path;
    }

    Path GlobalPlanning::aStar(int sx, int sy, int gx, int gy)
    {
        std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareAStarNode> open;
        std::vector<std::vector<bool>> closed(costmap_.height, std::vector<bool>(costmap_.width, false));

        std::shared_ptr<AStarNode> start = std::make_shared<AStarNode>(sx, sy, 0, heuristic(sx, sy, gx, gy));
        open.push(start);

        while (!open.empty()) {
            std::shared_ptr<AStarNode> current = open.top();
            open.pop();

            if (current->x == gx && current->y == gy)
                return reconstructPath(current);
            
            if (closed[current->y][current->x])
                continue;

            closed[current->y][current->x] = true;

            
            for(size_t i=0; i<directions_.size(); i++)
            {
                std::pair<int, int>& d = directions_[i];

                int nx = current->x + d.first;
                int ny = current->y + d.second;
                
                if (nx >= 0 && nx < static_cast<int>(costmap_.width) && ny >= 0 && ny < static_cast<int>(costmap_.height) && !closed[ny][nx]) {
                    int cost = costmap_.data[ny * costmap_.width + nx];
                    if (cost < 254)
                    {
                        float step_cost = directions_cost_[i];
                        float g = current->g + step_cost + cost;
                        float h = heuristic(nx, ny, gx, gy);
                        std::shared_ptr<AStarNode> neighbor = std::make_shared<AStarNode>(nx, ny, g, h, current);
                        open.push(neighbor);
                    }
                }
            }
        }

        return Path();
    }
    
    
    std::map<std::string, Path> GlobalPlanning::plan_global_path(std::map<std::string, Pose> robot_poses, std::map<std::string, Task> tasks)
    {
        // auto start = std::chrono::high_resolution_clock::now();

        std::map<std::string, Path> global_paths;

        if(!costmap_received_)
            return global_paths;
        
        for(const auto& [robot, task] : tasks)
        {
            Pose pose = robot_poses[robot];

            int sx = static_cast<int>((pose.position.x() - costmap_.origin_position.x()) / costmap_.resolution);
            int sy = static_cast<int>((pose.position.y() - costmap_.origin_position.y()) / costmap_.resolution);
            int gx = static_cast<int>((task.pose.position.x() - costmap_.origin_position.x()) / costmap_.resolution);
            int gy = static_cast<int>((task.pose.position.y() - costmap_.origin_position.y()) / costmap_.resolution);

            Path path = aStar(sx, sy, gx, gy);

            if(task.oriented)
                path.final_orientation = task.pose.orientation;
            else
                path.final_orientation = path.poses.back().orientation;

            if (!path.poses.empty())
                global_paths[robot] = path;
        }

        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> duration = end - start;

        // std::cout << "Global planning time: " << (duration.count() * 1000) << " ms (" << 1/duration.count() << " Hz)" << std::endl;
        
        return global_paths;
    }
}