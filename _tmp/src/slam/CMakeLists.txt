cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(FLANN REQUIRED)
find_library(LZ4_LIBRARY NAMES lz4 REQUIRED)

find_package(GTSAM REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})


add_executable(slam src/slam.cpp src/localization.cpp src/mapping.cpp src/decision_global_planning.cpp src/local_planning.cpp)
ament_target_dependencies(slam rclcpp interfaces nav2_msgs geometry_msgs visualization_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(slam
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/slam/>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)


target_link_libraries(slam
  gtsam
  ${FLANN_LIBRARIES}
  ${LZ4_LIBRARY}
)

install(TARGETS
  slam
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
