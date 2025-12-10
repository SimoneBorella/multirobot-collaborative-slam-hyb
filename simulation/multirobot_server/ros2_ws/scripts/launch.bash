#!/bin/bash
clear

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/humble/setup.bash
source install/setup.bash

bag_record=True
rviz=True
vicon=True
log_level=error

ros2 launch launch launch.py \
    bag_record:=$bag_record \
    rviz:=$rviz \
    vicon:=$vicon \
    log_level:=$log_level
