# Set(CMAKE_BUILD_TYPE "Debug")

# find_package(Sophus REQUIRED)

# INCLUDE_DIRECTORIES(
#     ${SOPHUS_INCLUDE_DIR}
# )

set( CMAKE_CXX_FLAGS "-std=c++11")


cmake_minimum_required(VERSION 2.8)
project(VisualSLAM)
cmake_policy(SET CMP0042 NEW)
add_library(slam_shared SHARED slam.cpp)
add_executable(main main.cpp)
target_link_libraries(main slam_shared)
# Change this for specific OS, update accordingly
# include_directories("eigen3/")
# include_directories("pangolin/")
# include_directories("sophus/")
# find_package(OpenCV REQUIRED)
# include_directories("opencv/")
add_executable(eigenmatrix eigenMatrix.cpp)
add_executable(eigenGeometry eigenGeometry.cpp)
add_executable(plotTrajectory plotTrajectory.cpp)
add_executable(optimization optimization.cpp)
add_executable(trajectoryError trajectoryError.cpp)
add_executable(image image.cpp)
add_executable(undistortion undistortion.cpp)
add_executable(rgbdvision rgbdvision.cpp)
add_executable(gaussNewton gaussNewton.cpp)

#Update accordingly
# target_link_libraries(eigenmatrix eigen3)
# target_link_libraries(eigenGeometry eigen3)
# target_link_libraries(plotTrajectory eigen3)
# target_link_libraries(optimization eigen3)
# target_link_libraries(optimization sophus)
# target_link_libraries(trajectoryError pangolin)
# target_link_libraries(optimization pangolin)
# target_link_libraries(image ${OpenCV_LIBRARIES})
# target_link_libraries(undistortion ${OpenCV_LIBRARIES})
# target_link_libraries(rgbdvision ${OpenCV_LIBRARIES})
# target_link_libraries(rgbdvision ${pangolin_LIBRARIES})
# target_link_libraries(rgbdvision sophus)
# target_link_libraries(rgbdvision eigen3)
# target_link_libraries(gaussNewton eigen3)
# target_link_libraries(gaussNewton ${OpenCV_LIBRARIES})