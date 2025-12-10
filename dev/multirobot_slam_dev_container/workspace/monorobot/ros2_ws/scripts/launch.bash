#!/bin/bash
clear

source /opt/ros/humble/setup.bash
source install/setup.bash

rviz_simulator=True
rviz_nav2=False
log_level=error

ros2 launch launch launch.py \
    rviz_simulator:=$rviz_simulator \
    rviz_nav2:=$rviz_nav2 \
    log_level:=$log_level
