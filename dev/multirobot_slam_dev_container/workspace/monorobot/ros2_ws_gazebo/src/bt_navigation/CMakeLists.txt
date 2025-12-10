cmake_minimum_required(VERSION 3.8)
project(bt_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



add_executable(initial_pose_publisher src/initial_pose_publisher/initial_pose_publisher.cpp)
ament_target_dependencies(initial_pose_publisher rclcpp nav2_msgs tf2 tf2_geometry_msgs)
target_include_directories(initial_pose_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/bt_nav2/initial_pose_publisher>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)

add_executable(bt_navigation src/bt_navigation/bt_navigation.cpp src/bt_navigation/go_to_pose.cpp)
ament_target_dependencies(bt_navigation rclcpp behaviortree_cpp_v3 yaml-cpp rclcpp_action nav2_msgs tf2 tf2_geometry_msgs)
target_include_directories(bt_navigation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/bt_navigation/bt_navigation>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(
  bt_navigation ${YAML_CPP_LIBRARIES}
)


install(TARGETS
  initial_pose_publisher
  bt_navigation
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  bt_xml
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
