cmake_minimum_required(VERSION 3.5)

project(ros_gz_bringup)

find_package(ament_cmake REQUIRED)
find_package(ros_gz_description REQUIRED)
find_package(ros_gz_gazebo REQUIRED)

install(
  DIRECTORY
    launch
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
