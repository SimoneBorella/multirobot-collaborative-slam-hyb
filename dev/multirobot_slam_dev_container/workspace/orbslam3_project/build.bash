#!/bin/bash

(
    cd libs/ORB_SLAM3
    sed -i 's/++11/++14/g' CMakeLists.txt
    ./build.sh
    # cp ./lib/libORB_SLAM3.so /usr/local/lib/
    # mkdir -p /usr/local/include/ORB_SLAM3
    # cp -r ./include/* /usr/local/include/ORB_SLAM3
    # ldconfig
)

if [ ! -d "build" ]; then
    mkdir build
    (cd build && cmake ..)
fi

(cd build && make -j4)
