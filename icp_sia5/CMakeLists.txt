cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl-interactive_icp)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (interactive_icp interactive_icp_origin.cpp)
add_executable (matrix_estimation matrix_estimation_test.cpp)

target_link_libraries (interactive_icp ${PCL_LIBRARIES})
target_link_libraries (matrix_estimation ${PCL_LIBRARIES})
