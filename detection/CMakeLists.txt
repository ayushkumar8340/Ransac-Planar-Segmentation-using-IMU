cmake_minimum_required(VERSION 3.5)

project(ransac_plane)

add_library(plane_ransac SHARED src/ransac.cpp)

target_link_libraries(plane_ransac PUBLIC ${PCL_LIBRARY_DIRS} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

target_include_directories(plane_ransac PUBLIC ${FRAMES_INCLUDE_DIRS} include)

########################################################################################################################################

add_library(planar_segmentation SHARED src/segmentation_worker.cpp)

target_link_libraries(planar_segmentation PUBLIC plane_ransac)


