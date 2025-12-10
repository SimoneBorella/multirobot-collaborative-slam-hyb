cmake_minimum_required(VERSION 3.16)
project(gtsam_benchmark_project)

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
    ${YAML_CPP_INCLUDE_DIRS}
)


# Include GTSAM
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${GTSAM_INCLUDE_DIR})

# Executable main.cpp
add_executable(
    gtsam_benchmark
    src/main.cpp
)
target_link_libraries(
    gtsam_benchmark
    gtsam
    ${YAML_CPP_LIBRARIES}
)

# Executable isam_no_window.cpp
add_executable(
    gtsam_isam_no_window_benchmark
    src/isam_no_window.cpp
)
target_link_libraries(
    gtsam_isam_no_window_benchmark
    gtsam
    ${YAML_CPP_LIBRARIES}
)

# Executable isam.cpp
add_executable(
    gtsam_isam_benchmark
    src/isam.cpp
)
target_link_libraries(
    gtsam_isam_benchmark
    gtsam
    ${YAML_CPP_LIBRARIES}
)