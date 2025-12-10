#!/bin/bash

IMAGE="multirobot_slam_server"
TAG="latest"

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


docker exec -it $IMAGE bash
