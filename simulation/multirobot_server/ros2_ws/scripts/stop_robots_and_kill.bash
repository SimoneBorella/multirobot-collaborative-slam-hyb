#!/bin/bash
clear

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/humble/setup.bash
source install/setup.bash

robots=("robot_12" "robot_13" "robot_14")

# Loop through each robot
for robot in "${robots[@]}"; do
    ros2 topic pub --once \
        "/${robot}/cmd_vel" geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" &
done

sleep 3

ps -a -o pid= | while read pid; do
    kill -9 "$pid" 2>/dev/null
done

wait