#!/bin/bash
clear

source /opt/ros/humble/setup.bash
source install/setup.bash

namespace=robot_14

ros2 run teleop teleop_keyboard --ros-args -p namespace:=$namespace