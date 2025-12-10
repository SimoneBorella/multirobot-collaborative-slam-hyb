#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

rviz=True
verbose=True

ros2 launch launch launch_vicon.py \
    rviz:=$rviz \
    verbose:=$verbose