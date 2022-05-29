# Set(CMAKE_BUILD_TYPE "Debug")

find_package(Sophus REQUIRED)

INCLUDE_DIRECTORIES(
    ${SOPHUS_INCLUDE_DIR}
)
cmake_minimum_required(VERSION 2.8)
project(VisualSLAM)
cmake_policy(SET CMP0042 NEW)
add_library(slam_shared SHARED slam.cpp)
add_executable(main main.cpp)
target_link_libraries(main slam_shared)
# Change this for specific OS
# include_directories("eigen3/")
# include_directories("pangolin/")
add_executable(eigenmatrix eigenMatrix.cpp)
add_executable(eigenGeometry eigenGeometry.cpp)
add_executable(plotTrajectory plotTrajectory.cpp)
add_executable((optimization optimization.cpp))