cmake_minimum_required(VERSION 3.8)
project(sensor_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(OpenCV 4.2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(rgbd_feature_extraction src/rgbd_feature_extraction.cpp)
ament_target_dependencies(rgbd_feature_extraction rclcpp interfaces geometry_msgs visualization_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge image_transport)
target_include_directories(rgbd_feature_extraction
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/sensor_processing/rgbd_feature_extraction>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(rgbd_feature_extraction
  ${OpenCV_LIBS}
)



add_executable(camera_calibration src/camera_calibration.cpp)
ament_target_dependencies(camera_calibration rclcpp visualization_msgs cv_bridge image_transport)
target_include_directories(camera_calibration
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/sensor_processing/camera_calibration>
    $<INSTALL_INTERFACE:include>
    ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(camera_calibration
  ${OpenCV_LIBS}
)

install(TARGETS
  rgbd_feature_extraction
  camera_calibration
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
