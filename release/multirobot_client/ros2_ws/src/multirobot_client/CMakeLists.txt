cmake_minimum_required(VERSION 3.8)
project(multirobot_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(GTSAM REQUIRED)

find_package(OpenCV REQUIRED)

find_package(DBoW3 REQUIRED)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

set(DEPENDENCIES
    rclcpp
    ament_index_cpp
    interfaces
    visualization_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    std_msgs
    tf2 
    tf2_ros
    tf2_msgs
    tf2_geometry_msgs
    OpenCV
)

set(SRC_FILES
  src/multirobot_client.cpp
  src/slam/slam.cpp
  src/slam/backend.cpp
  src/slam/loop_closure_detector.cpp
  src/slam/submap_manager.cpp
  src/mapping.cpp
)


add_executable(${PROJECT_NAME} ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/slam>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/mapping>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME}
  yaml-cpp
  gtsam
  DBoW3
) 

add_executable(tf_publisher src/tf_publisher.cpp)
ament_target_dependencies(tf_publisher rclcpp tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(tf_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)


install(TARGETS
  ${PROJECT_NAME}
  tf_publisher
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  config
  params
  DESTINATION share/${PROJECT_NAME})

ament_package()
