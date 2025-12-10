cmake_minimum_required(VERSION 3.16)
project(gtsam_project)

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
# include_directories(
#     include/
# )


# Include Eigen3
# find_package(Eigen3 REQUIRED NO_MODULE)
# include_directories(${EIGEN3_INCLUDE_DIR})
# if(Eigen3_FOUND)  
#   message(STATUS "Found Eigen3 ${Eigen3_VERSION}")
# else()
#   message(STATUS "Eigen3 Not Found")
# endif()

# Include GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
if(GTSAM_FOUND)  
  message(STATUS "Found GTSAM ${GTSAM_VERSION}")
else()
  message(STATUS "GTSAM Not Found")
endif()


# Executable slam_2d.cpp
add_executable(
    slam_2d
    src/slam_2d.cpp
)
target_link_libraries(
    slam_2d
    gtsam
)


# Executable slam_3d.cpp
add_executable(
    slam_3d
    src/slam_3d.cpp
)
target_link_libraries(
    slam_3d
    gtsam
)

