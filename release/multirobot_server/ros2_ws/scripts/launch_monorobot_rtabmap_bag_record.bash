#!/bin/bash
clear

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=2
export CAMERA_MODEL=oakd

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record /cmd_vel /grid_prob_map /imu /info /joint_states /map /odom /robot_description /scan /tf /tf_static