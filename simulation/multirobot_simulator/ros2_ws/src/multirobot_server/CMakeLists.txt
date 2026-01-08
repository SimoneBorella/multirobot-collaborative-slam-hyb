cmake_minimum_required(VERSION 3.8)
project(multirobot_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPENDENCIES
    ament_index_cpp
    rclcpp
    interfaces
    # nav_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    yaml-cpp
    tf2 
    tf2_ros
    tf2_msgs
    tf2_geometry_msgs
)

set(SRC_FILES
  src/multirobot_server.cpp
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


install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
