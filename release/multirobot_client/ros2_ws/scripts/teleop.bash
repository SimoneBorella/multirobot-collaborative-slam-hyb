#!/bin/bash
clear

source /opt/ros/humble/setup.bash
source install/setup.bash

namespace=$ROBOT_NAME

ros2 run teleop teleop_keyboard --ros-args -p namespace:=$namespace