cmake_minimum_required(VERSION 3.16)
project(bt_project)

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

# Include directories
include_directories(
    include/
    include/bt_demo_0/
    include/bt_demo_0/nodes/
    include/bt_demo_1/
    include/bt_demo_2/nodes/
    include/bt_demo_2/types/
)

# Include BehaviorTree.CPP
find_package(behaviortree_cpp_v3 REQUIRED)


# Executable bt_demo_0
add_executable(
    bt_demo_0
    src/bt_demo_0/main.cpp
    src/bt_demo_0/nodes/approach_object.cpp
    src/bt_demo_0/nodes/check_battery.cpp
    src/bt_demo_0/nodes/gripper_interface.cpp
)
target_link_libraries(
    bt_demo_0
    BT::behaviortree_cpp_v3
)



# Executable bt_demo_1
add_executable(
    bt_demo_1
    src/bt_demo_1/main.cpp
)
target_link_libraries(
    bt_demo_1
    BT::behaviortree_cpp_v3
)



# Executable bt_demo_2
add_executable(
    bt_demo_2
    src/bt_demo_2/main.cpp
    src/bt_demo_2/nodes/move.cpp
    src/bt_demo_2/nodes/check_battery.cpp
)
target_link_libraries(
    bt_demo_2
    BT::behaviortree_cpp_v3
)


