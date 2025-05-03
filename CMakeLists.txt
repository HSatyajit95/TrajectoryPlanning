cmake_minimum_required(VERSION 3.10)
project(QuinticTrajectoryPlanner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR})

# Source files
set(SOURCES src/QuinticTrajectoryPlanner.cpp)

# Create library
add_library(quintic_trajectory_planner ${SOURCES})

# Create example executable
add_executable(example examples/example.cpp)
target_link_libraries(example quintic_trajectory_planner)

# Install library and headers
install(TARGETS quintic_trajectory_planner
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)