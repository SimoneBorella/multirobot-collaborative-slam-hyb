cmake_minimum_required(VERSION 3.8)
project(multi_robot_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(multi_robot_simulator src/multi_robot_simulator.cpp src/robot.cpp src/lidar.cpp src/camera.cpp)
ament_target_dependencies(multi_robot_simulator rclcpp interfaces nav_msgs std_msgs sensor_msgs visualization_msgs yaml-cpp tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(multi_robot_simulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/multi_robot_simulator/>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(
  multi_robot_simulator ${YAML_CPP_LIBRARIES}
)

add_executable(tf_publisher src/tf_publisher.cpp)
ament_target_dependencies(tf_publisher rclcpp tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(tf_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/multi_robot_simulator/>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)

add_executable(visualization_publisher src/visualization_publisher.cpp)
ament_target_dependencies(visualization_publisher rclcpp nav_msgs)
target_include_directories(visualization_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/multi_robot_simulator/>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)

add_executable(clock_publisher src/clock_publisher.cpp)
ament_target_dependencies(clock_publisher rclcpp)
target_include_directories(clock_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/multi_robot_simulator/>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)

install(TARGETS
  multi_robot_simulator
  tf_publisher
  visualization_publisher
  clock_publisher
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  config
  maps
  robots
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
