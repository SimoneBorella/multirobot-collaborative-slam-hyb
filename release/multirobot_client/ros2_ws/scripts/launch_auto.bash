#!/bin/bash
clear

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=2
export CAMERA_MODEL=oakd

source /opt/ros/humble/setup.bash
source install/setup.bash

namespace=robot_13
launch_nav2=False
log_level=error

ros2 launch launch launch.py \
    namespace:=$namespace \
    launch_nav2:=$launch_nav2 \
    log_level:=$log_level
