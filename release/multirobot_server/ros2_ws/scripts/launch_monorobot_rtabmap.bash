#!/bin/bash
clear

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=2
export CAMERA_MODEL=oakd

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rtabmap_demos turtlebot3_scan.launch.py