#!/bin/bash
clear

source /opt/ros/humble/setup.bash
source install/setup.bash

bag_record=False
rviz=True
log_level=error

ros2 launch launch launch.py \
    bag_record:=$bag_record \
    rviz:=$rviz \
    log_level:=$log_level
