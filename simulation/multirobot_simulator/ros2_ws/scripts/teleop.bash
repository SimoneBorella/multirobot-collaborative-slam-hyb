#!/bin/bash
clear

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run teleop teleop_keyboard --ros-args -p namespace:=robot_0