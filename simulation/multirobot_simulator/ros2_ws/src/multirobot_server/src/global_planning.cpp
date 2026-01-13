#include "global_planning.h"


namespace multirobot_slam
{
    GlobalPlanning::GlobalPlanning()
    {
    }

    GlobalPlanning::GlobalPlanning(GlobalPlanningParams &params)
        : params_(params)
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
    }

    void GlobalPlanning::update_costmap(Map costmap)
    {
        costmap_ = costmap;
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


    Path GlobalPlanning::reconstructPath(std::shared_ptr<AStarNode> last)
    {
        Path path;

        // std::vector<geometry_msgs::msg::PoseStamped> temp_poses;

        // std::shared_ptr<AStarNode> current = last;
        // while (current) {
        //     geometry_msgs::msg::PoseStamped pose;
        //     pose.header.frame_id = all_frame;
        //     pose.header.stamp = now;
        //     pose.pose.position.x = current->x * costmap_->info.resolution + costmap_->info.origin.position.x;
        //     pose.pose.position.y = current->y * costmap_->info.resolution + costmap_->info.origin.position.y;

        //     temp_poses.push_back(pose);
        //     current = current->parent;
        // }

        // if(global_planner_type == "a_star")
        //     std::reverse(temp_poses.begin(), temp_poses.end());

        // // Compute orientation
        // for(size_t i = 0; i < temp_poses.size(); ++i) {
        //     geometry_msgs::msg::PoseStamped pose = temp_poses[i];

        //     if(i+1 < temp_poses.size()) {
        //         double dx = temp_poses[i+1].pose.position.x - temp_poses[i].pose.position.x;
        //         double dy = temp_poses[i+1].pose.position.y - temp_poses[i].pose.position.y;
        //         double yaw = std::atan2(dy, dx);

        //         tf2::Quaternion q;
        //         q.setRPY(0, 0, yaw);
        //         pose.pose.orientation = tf2::toMsg(q);
        //     } else if(i > 0) {
        //         pose.pose.orientation = temp_poses[i-1].pose.orientation;
        //     } else {
        //         pose.pose.orientation.w = 1.0;
        //     }

        //     path.poses.push_back(pose);
        // }

        return path;
    }


    Path GlobalPlanning::aStar(int sx, int sy, int gx, int gy)
    {
        // std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareAStarNode> open;
        // std::vector<std::vector<bool>> closed(costmap_->info.height, std::vector<bool>(costmap_->info.width, false));

        // std::shared_ptr<AStarNode> start = std::make_shared<AStarNode>(sx, sy, 0, heuristic(sx, sy, gx, gy));
        // open.push(start);

        // while (!open.empty()) {
        //     std::shared_ptr<AStarNode> current = open.top();
        //     open.pop();

        //     if (current->x == gx && current->y == gy) {
        //         return reconstructPath(current);
        //     }

        //     if (closed[current->y][current->x]) {
        //         continue;
        //     }

        //     closed[current->y][current->x] = true;

            
        //     for(size_t i=0; i<directions.size(); i++)
        //     {
        //         std::pair<int, int>& d = directions[i];

        //         int nx = current->x + d.first;
        //         int ny = current->y + d.second;
                
        //         if (nx >= 0 && nx < static_cast<int>(costmap_->info.width) && ny >= 0 && ny < static_cast<int>(costmap_->info.height) && !closed[ny][nx]) {
        //             int cost = costmap_->data[ny * costmap_->info.width + nx];
        //             if (cost < 254)
        //             {
        //                 float step_cost = directions_cost[i];
        //                 float g = current->g + step_cost + cost;
        //                 float h = heuristic(nx, ny, gx, gy);
        //                 std::shared_ptr<AStarNode> neighbor = std::make_shared<AStarNode>(nx, ny, g, h, current);
        //                 open.push(neighbor);
        //             }
        //         }
        //     }
        // }

        return Path();
    }
    
    
    std::map<std::string, Path> GlobalPlanning::plan_global_path(std::map<std::string, Pose> robot_poses, std::map<std::string, Frontier> tasks)
    {
        std::map<std::string, Path> global_paths;
        
        for(const auto& [robot, frontier] : tasks)
        {
            Pose pose = robot_poses[robot];

            int sx = static_cast<int>((pose.position.x() - costmap_.origin_position.x()) / costmap_.resolution);
            int sy = static_cast<int>((pose.position.y() - costmap_.origin_position.y()) / costmap_.resolution);
            int gx = static_cast<int>((frontier.centroid.x() - costmap_.origin_position.x()) / costmap_.resolution);
            int gy = static_cast<int>((frontier.centroid.y() - costmap_.origin_position.y()) / costmap_.resolution);

            Path path = aStar(sx, sy, gx, gy);

            if (!path.poses.empty()) {
                global_paths[robot] = path;
            }
        }
        
        return global_paths;
    }
}