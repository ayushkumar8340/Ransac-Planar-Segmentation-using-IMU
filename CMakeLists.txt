cmake_minimum_required(VERSION 3.5)

project(Planar_Segmentation)

find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)

set(FRAMES_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/frames/include)

add_subdirectory(realsense)
add_subdirectory(pre_processing)
add_subdirectory(detection)