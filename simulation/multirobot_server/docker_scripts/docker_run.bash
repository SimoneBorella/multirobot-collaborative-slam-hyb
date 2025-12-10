#!/bin/bash

IMAGE="multirobot_slam_server"
TAG="latest"
DOCKERFILE_PATH="."

usage() {
    echo "Usage: $(basename "$0") [OPTIONS]"
    echo "Options:"
    echo "  -i, --image IMAGE  Image name"
    echo "  -t, --tag TAG      Tag name"
    echo "  -h, --help         Show this help message"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--image)
	        IMAGE=$2
            shift
            ;;
        -t|--tag)
            TAG=$2
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
    shift
done


# if [[ $BASH_SOURCE == "./"* ]]; then
#     DOCKERFILE_PATH=$BASH_SOURCE
# else
#     DOCKERFILE_PATH="./"$BASH_SOURCE
# fi
# DOCKERFILE_PATH="${DOCKERFILE_PATH%/*}/."


xhost local:docker

XAUTH=/tmp/.docker.xauth

docker run \
    --rm \
    --privileged \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env XAUTHORITY=$XAUTH \
    --net host \
    --ipc=host \
    --volume $DOCKERFILE_PATH/ros2_ws:/ros2_ws \
    --volume /dev/bus/usb:/dev/bus/usb \
    --name $IMAGE \
    --hostname $IMAGE \
    $IMAGE:$TAG \
    bash -c "/ros2_ws/scripts/launch_auto.bash"

    # --gpus all \