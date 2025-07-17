cmake_minimum_required(VERSION 3.5)

project(Realsense)

add_library(realsense SHARED src/realsense.cpp)

target_link_libraries(realsense PUBLIC realsense2 ${PCL_LIBRARY_DIRS} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

target_include_directories(realsense PUBLIC include ${FRAMES_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


#################################################################################################################################################
