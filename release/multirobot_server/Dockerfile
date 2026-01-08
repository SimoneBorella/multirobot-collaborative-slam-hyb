FROM osrf/ros:humble-desktop

WORKDIR /

# Basic packages
RUN apt -y update \
    && apt install -y usbutils \
    && apt install -y unzip \
    && apt install -y libcanberra-gtk3-module \
    && apt install -y at-spi2-core

# Python
RUN apt -y update \
    && apt install -y python3-pip \
    && pip install --no-cache-dir --upgrade pip

# Python packages
# RUN pip install setuptools==58.2.0 \
#     && pip install numpy==1.24.4 \
#     && pip install --ignore-installed sympy==1.10 \
#     && pip install --no-cache-dir torch torchvision torchaudio \
#     && pip install --no-cache-dir ultralytics \
#     && pip install --no-cache-dir dill \
#     && pip install --no-cache-dir scipy \
#     && pip install --no-cache-dir ros2-numpy \
#     && pip install --no-cache-dir squaternion \
#     && pip install --no-cache-dir quadprog==0.1.12


# Cpp tools and libraries
RUN apt -y update \
    && apt install -y vim \
    && apt install -y cmake \
    && apt install -y --no-install-recommends build-essential \
    && apt install -y libeigen3-dev libboost-all-dev libtbb-dev

    
# Gtsam library
COPY dep/gtsam-4.2.0-ros /dep/gtsam-4.2.0-ros
WORKDIR /dep/gtsam-4.2.0-ros
RUN mkdir build \
    && cd build \
    && cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON\
    && make install -j$(nproc) \
    && ldconfig

# BehaviorTree.CPP library
# COPY dep/BehaviorTree.CPP /dep/BehaviorTree.CPP
# WORKDIR /dep/BehaviorTree.CPP
# RUN mkdir build \
#     && cd build \
#     && cmake .. \
#     && make install -j$(nproc) \
#     && ldconfig


# Pangolin library
COPY dep/Pangolin /dep/Pangolin
WORKDIR /dep/Pangolin
RUN ./scripts/install_prerequisites.sh recommended \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make install -j$(nproc) \
    && ldconfig



# OpenCV3
RUN apt -y update \
    && apt install -y libgtk2.0-dev \
    && apt install -y pkg-config \
    && apt install -y libavcodec-dev \
    && apt install -y libavformat-dev \
    && apt install -y libswscale-dev \
    && apt install -y libtbb2 libtbb-dev \
    && apt install -y libjpeg-dev \
    && apt install -y libpng-dev \
    && apt install -y libtiff-dev

COPY dep/opencv /dep/opencv
COPY dep/opencv_contrib /dep/opencv_contrib

WORKDIR /dep/opencv
RUN mkdir build \
    && cd build \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local/opencv3 \
      -D INSTALL_C_EXAMPLES=OFF \
      -D INSTALL_PYTHON_EXAMPLES=OFF \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=OFF .. \
    && make install -j$(nproc) \
    && ldconfig
    


# ORB_SLAM2 library
# COPY dep/ORB_SLAM2 /dep/ORB_SLAM2
# WORKDIR /dep/ORB_SLAM2
# RUN sed -i 's/++11/++14/g' CMakeLists.txt \
#     && ./build.sh \
#     && cp ./lib/libORB_SLAM2.so /usr/local/lib/ \
#     && cp -r ./include/ORB_SLAM2 /usr/local/include/ \
#     && ldconfig


# ORB_SLAM3 library
# COPY dep/ORB_SLAM3 /dep/ORB_SLAM3
# WORKDIR /dep/ORB_SLAM3
# RUN sed -i 's/++11/++14/g' CMakeLists.txt \
#     && ./build.sh \
#     && cp ./lib/libORB_SLAM3.so /usr/local/lib/ \
#     && mkdir /usr/local/include/ORB_SLAM3 \
#     && cp -r ./include/* /usr/local/include/ORB_SLAM3 \
#     && ldconfig


# FLANN library
COPY dep/flann /dep/flann
WORKDIR /dep/flann
RUN mkdir build \
    && cd build \
    && cmake .. \
    && make install -j$(nproc) \
    && ldconfig


RUN apt -y update \
    && apt install -y libunwind-dev \
    && apt install -y libgoogle-glog-dev

# Gazebo Fortress
RUN apt -y update \
&& apt install -y ros-${ROS_DISTRO}-ros-gz

# Turtlebot3
RUN apt install -y ros-humble-turtlebot3*

# Depthai oakd pro driver
RUN apt install -y ros-humble-depthai-ros

# Navigation2
RUN apt -y update \
    && apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox\
    && apt install -y ros-humble-ros-ign-bridge

# Rtabmap
RUN apt -y update \
    && apt install -y ros-humble-rtabmap-ros

# Cyclone DDS
RUN apt install -y ros-humble-rmw-cyclonedds-cpp


COPY ros2_ws /ros2_ws

# Configuration .bashrc

RUN echo "# Set default XDG_RUNTIME_DIR" >> /root/.bashrc \
    && echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> /root/.bashrc

RUN echo "# Avoid graphic issues using DRI2 instead of DRI3 with Intel Iris" >> /root/.bashrc \
    && echo "export LIBGL_DRI3_DISABLE=1" >> /root/.bashrc

RUN echo "# Setup domain identifier to avoid wrong communications" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=2" >> /root/.bashrc

RUN echo "# Setup ros2 middleware implementation" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc
    # && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc

RUN echo "# Source ROS2 Humble environment" >> /root/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN echo "# Change dir to ros2_ws when container is started" >> /root/.bashrc \
    && echo "cd /ros2_ws/" >> /root/.bashrc
    
WORKDIR /ros2_ws
