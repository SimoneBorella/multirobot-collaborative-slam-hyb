cmake_minimum_required(VERSION 3.8)
project(multirobot_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
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

set(DEPENDENCIES
    rclcpp
    ament_index_cpp
    interfaces
    visualization_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    yaml-cpp
    tf2 
    tf2_ros
    tf2_msgs
    tf2_geometry_msgs
)

set(SRC_FILES
  src/multirobot_simulator.cpp
  src/robot.cpp
  src/lidar.cpp
  src/camera.cpp
)


add_executable(${PROJECT_NAME} ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME}
  yaml-cpp
)

add_executable(tf_publisher src/tf_publisher.cpp)
ament_target_dependencies(tf_publisher rclcpp tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(tf_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)

add_executable(visualization_publisher src/visualization_publisher.cpp)
ament_target_dependencies(visualization_publisher rclcpp nav_msgs)
target_include_directories(visualization_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)

install(TARGETS
  multirobot_simulator
  tf_publisher
  visualization_publisher
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  config
  maps
  robots
  DESTINATION share/${PROJECT_NAME})

ament_package()
