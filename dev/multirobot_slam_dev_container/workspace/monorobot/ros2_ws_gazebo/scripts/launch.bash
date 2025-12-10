#!/bin/bash
clear

source install/setup.bash

rviz_gz=False
rviz_nav2=True
log_level=error


ros2 launch launch launch.py \
    rviz_gz:=$rviz_gz \
    rviz_nav2:=$rviz_nav2 \
    log_level:=$log_level
