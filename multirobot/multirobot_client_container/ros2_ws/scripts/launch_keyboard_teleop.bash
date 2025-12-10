#!/bin/bash
clear

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
export ROS_DOMAIN_ID=2
export CAMERA_MODEL=oakd

source /opt/ros/humble/setup.bash
source install/setup.bash

namespace=robot_13
launch_nav2=True
log_level=error


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


ros2 launch launch launch.py \
    namespace:=$namespace \
    launch_nav2:=$launch_nav2 \
    log_level:=$log_level >> /dev/null &
LAUNCH_PID=$!



# Start teleop in foreground
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r __ns:=/${namespace}

# When teleop exits normally, cleanup
cleanup
