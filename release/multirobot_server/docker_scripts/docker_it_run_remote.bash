#!/bin/bash

IMAGE="multirobot_server"
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


# xhost local:docker

# XAUTH=/tmp/.docker.xauth


docker run -it \
    --rm \
    --privileged \
    --volume $DOCKERFILE_PATH/ros2_ws:/ros2_ws \
    -p 6981:6080 \
    --env VNC_RESOLUTION="3000x2000x16" \
    --name $IMAGE \
    --hostname $IMAGE \
    $IMAGE:$TAG \
    /usr/local/share/desktop-init.sh bash


    # --env VNC_PASSWORD="" \

    # --volume /dev/bus/usb:/dev/bus/usb \
    # --net host \
    # --ipc=host \
    
    # --env DISPLAY=$DISPLAY \
    # --env QT_X11_NO_MITSHM=1 \
    # --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    # --env XAUTHORITY=$XAUTH \

    # --gpus all \
