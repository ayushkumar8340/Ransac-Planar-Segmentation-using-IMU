cmake_minimum_required(VERSION 3.5)

project(pre_processing)

add_library(pre_process SHARED src/pre_processing.cpp)

target_link_libraries(pre_process PUBLIC ${PCL_LIBRARY_DIRS} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

target_include_directories(pre_process PUBLIC include ${FRAMES_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#######################################################################################################################################

