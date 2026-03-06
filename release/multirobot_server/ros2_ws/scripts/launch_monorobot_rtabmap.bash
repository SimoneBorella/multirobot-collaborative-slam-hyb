#!/bin/bash
clear

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=19
export CAMERA_MODEL=oakd

source /opt/ros/humble/setup.bash
source install/setup.bash

# ros2 launch rtabmap_demos turtlebot3_scan.launch.py

ros2 run rtabmap_slam rtabmap \
    --ros-args \
    -p subscribe_scan:=true \
    -p qos_scan:=2 \
    -p frame_id:=base_footprint \
    -p subscribe_depth:=false \
    -p subscribe_rgb:=false \
    -p approx_sync:=true \
    -p use_action_for_goal:=true \
    -p Reg/Strategy:='"1"' \
    -p Reg/Force3DoF:='"true"' \
    -p RGBD/NeighborLinkRefining:='"true"' \
    -p Grid/RangeMin:='"0.2"' \
    -p Optimizer/GravitySigma:='"0"' \
    -r scan:=/scan \
    -r odom:=/odom