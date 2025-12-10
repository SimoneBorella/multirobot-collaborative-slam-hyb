#!/bin/bash

# Camera capture
# ./build/bin/camera_capture

# Monocular euroc
# ./build/bin/mono_euroc ./libs/ORB_SLAM3/Vocabulary/ORBvoc.txt ./params/Monocular/EuRoC.yaml ./datasets/EuRoc/MH01 ./params/Monocular/EuRoC_TimeStamps/MH01.txt dataset-MH01_mono

# Monocular camera
./build/bin/mono_camera ./libs/ORB_SLAM3/Vocabulary/ORBvoc.txt ./params/Monocular/camera.yaml
