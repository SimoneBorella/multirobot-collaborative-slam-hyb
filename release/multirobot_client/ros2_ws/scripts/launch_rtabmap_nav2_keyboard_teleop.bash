#!/bin/bash
clear

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=19
export CAMERA_MODEL=oakd

source /opt/ros/humble/setup.bash
source install/setup.bash


# Put launch process into its own process group
# This makes it easier to kill all children with a single kill command
# (On many systems, this is automatic for background jobs, but explicitly safer)
set -m  # enable job control
# Or you can start with 'setsid' or 'nohup' if needed

cleanup() {
    echo "Stopping processes..."

    # Kill the entire process group of LAUNCH_PID
    kill -- -"$LAUNCH_PID"

    wait "$LAUNCH_PID"
    exit 0
}
trap cleanup SIGINT


ros2 launch turtlebot3_bringup robot.launch.py >> /dev/null &
LAUNCH_PID=$!

ros2 launch nav2_bringup bringup_launch.py >> /dev/null &
LAUNCH_PID=$!

# ros2 launch rtabmap_demos turtlebot3_scan.launch.py >> /dev/null &
# LAUNCH_PID=$!

# ros2 bag record /cmd_vel /grid_prob_map /imu /info /joint_states /map /odom /robot_description /scan /tf /tf_static

ros2 run turtlebot3_teleop teleop_keyboard

cleanup