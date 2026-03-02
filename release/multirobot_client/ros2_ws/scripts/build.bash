#!/bin/bash
clear

source /opt/ros/jazzy/setup.bash

colcon build
source install/setup.bash
