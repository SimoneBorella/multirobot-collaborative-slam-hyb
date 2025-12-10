cmake_minimum_required(VERSION 3.8)
project(mocap4ros2_vicon_tf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(mocap4r2_msgs REQUIRED)
find_package(mocap4r2_control REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



add_executable(tf_publisher src/tf_publisher.cpp)
ament_target_dependencies(tf_publisher rclcpp tf2 tf2_ros tf2_geometry_msgs mocap4r2_msgs mocap4r2_control)
target_include_directories(tf_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mocap4ros2_vicon_tf/>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)


install(TARGETS
  tf_publisher
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
