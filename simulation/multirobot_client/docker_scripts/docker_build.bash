#!/bin/bash

IMAGE="multirobot_slam_client"
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
            echo "Error: unknown argument $1"
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

docker build -t $IMAGE:$TAG $DOCKERFILE_PATH
docker image prune -f
