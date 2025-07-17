cmake_minimum_required(VERSION 3.5)

project(tests)

add_executable(test_z_plane src/test_z_plane_seg.cpp)

target_link_libraries(test_z_plane PUBLIC planar_segmentation)
