cmake_minimum_required(VERSION 3.16)
project(orbslam3_project)

# Set build type
set(CMAKE_BUILD_TYPE Release)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Set max optimization for GNU compiler
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH  ${CMAKE_BINARY_DIR}/lib)



find_package(Eigen3 3.3.0 REQUIRED NO_MODULE)
find_package(OpenCV 4.2 REQUIRED)
find_package(Pangolin REQUIRED)

# Include directories
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

set(ORB_SLAM3_DIR ${CMAKE_SOURCE_DIR}/libs/ORB_SLAM3)
include_directories(
    ${ORB_SLAM3_DIR}
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
)
link_directories(${ORB_SLAM3_DIR}/lib)




# Executable mono_euroc.cpp
add_executable(
    mono_euroc
    src/mono_euroc.cpp
)
target_link_libraries(
    mono_euroc
    ORB_SLAM3
    ${EIGEN3_LIBS}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)


# Executable camera_capture.cpp
add_executable(
    camera_capture
    src/camera_capture.cpp
)
target_link_libraries(
    camera_capture
    ${OpenCV_LIBS}
)


# Executable mono_camera.cpp
add_executable(
    mono_camera
    src/mono_camera.cpp
)
target_link_libraries(
    mono_camera
    ORB_SLAM3
    ${EIGEN3_LIBS}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)