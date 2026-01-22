FROM osrf/ros:jazzy-desktop

WORKDIR /

# Basic packages
RUN apt -y update \
    && apt install -y usbutils \
    && apt install -y unzip \
    && apt install -y libcanberra-gtk3-module \
    && apt install -y at-spi2-core

# Cpp tools and libraries
RUN apt -y update \
    && apt install -y vim \
    && apt install -y cmake \
    && apt install -y --no-install-recommends build-essential \
    && apt install -y libeigen3-dev libboost-all-dev libtbb-dev


# Cyclone DDS
RUN apt install -y ros-jazzy-rmw-cyclonedds-cpp

RUN apt install -y ros-jazzy-nav2-bringup \
    && apt install -y ros-jazzy-navigation2

# Gtsam
COPY dep/gtsam /dep/gtsam
WORKDIR /dep/gtsam
RUN mkdir build \
    && cd build \
    && cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON\
    && make install -j$(nproc)\
    && ldconfig




# OpenCV
# RUN apt -y update \
#     && apt install -y libgtk2.0-dev \
#     && apt install -y pkg-config \
#     && apt install -y libavcodec-dev \
#     && apt install -y libavformat-dev \
#     && apt install -y libswscale-dev \
#     && apt install -y libtbb2 libtbb-dev \
#     && apt install -y libjpeg-dev \
#     && apt install -y libpng-dev \
#     && apt install -y libtiff-dev

# COPY dep/opencv /dep/opencv
# COPY dep/opencv_contrib /dep/opencv_contrib

# WORKDIR /dep/opencv
# RUN mkdir build \
#     && cd build \
#     && cmake -D CMAKE_BUILD_TYPE=RELEASE \
#     -D CMAKE_INSTALL_PREFIX=/usr/local/opencv3 \
#     -D INSTALL_C_EXAMPLES=OFF \
#     -D INSTALL_PYTHON_EXAMPLES=OFF \
#     -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
#     -D BUILD_EXAMPLES=OFF .. \
#     && make install -j$(nproc) \
#     && ldconfig



# FLANN library
COPY dep/flann /dep/flann
WORKDIR /dep/flann
RUN mkdir build \
    && cd build \
    && cmake .. \
    && make install -j$(nproc) \
    && ldconfig


# RUN apt -y update \
#     && apt install -y libunwind-dev \
#     && apt install -y libgoogle-glog-dev



# Python
RUN apt -y update \
    && apt install -y python3-pip

# Python packages
# RUN pip install <package> --break-system-packages



# Remote desktop lite vnc novnc
COPY desktop-lite-install.sh /tmp/desktop-lite-install.sh

ENV USERNAME="root"
RUN chmod +x /tmp/desktop-lite-install.sh && \
    apt-get update && /tmp/desktop-lite-install.sh
    # "none" "vscode" "true" "5901" "6080"

ENV DBUS_SESSION_BUS_ADDRESS="autolaunch:" \
    VNC_RESOLUTION="1440x768x16" \
    VNC_DPI="96" \
    VNC_PORT="5901" \
    NOVNC_PORT="6080" \
    DISPLAY=":1" \
    LANG="en_US.UTF-8" \
    LANGUAGE="en_US.UTF-8"

# Visual improvements and customizations
RUN ln -s /etc/profile.d/vte-2.91.sh /etc/profile.d/vte.sh && \
    echo "if [ \$TILIX_ID ] || [ \$VTE_VERSION ]; then source /etc/profile.d/vte.sh; fi" >> /root/.bashrc && \
    sed -i 's|^session.styleFile:.*|session.styleFile: /usr/share/fluxbox/styles/ubuntu-dark|' /root/.fluxbox/init

ENTRYPOINT ["/usr/local/share/desktop-init.sh"]



COPY ros2_ws /ros2_ws

# Configuration .bashrc

RUN echo "# Set default XDG_RUNTIME_DIR" >> /root/.bashrc \
    && echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> /root/.bashrc

RUN echo "# Avoid graphic issues using DRI2 instead of DRI3 with Intel Iris" >> /root/.bashrc \
    && echo "export LIBGL_DRI3_DISABLE=1" >> /root/.bashrc

RUN echo "# Setup domain identifier to avoid wrong communications" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=17" >> /root/.bashrc

# RUN echo "# Setup ROS2 middleware implementation" >> /root/.bashrc \
#     && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc
#     && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc

RUN echo "# Source ROS2 Jazzy environment" >> /root/.bashrc \
    && echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

RUN echo "# Change dir to ros2_ws when container is started" >> /root/.bashrc \
    && echo "cd /ros2_ws/" >> /root/.bashrc

WORKDIR /ros2_ws
