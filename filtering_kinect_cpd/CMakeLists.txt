cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl-filtering_kinect_pcd)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (filtering_kinect_pcd visualizer.cpp)
add_executable (filtering_kinect_pcd get_euler_angles.cpp)
#add_executable (rotate_axis_pcd tf_transformer.cpp)
add_executable (rotate_axis_pcd rotate_axis.cpp)


target_link_libraries (filtering_kinect_pcd ${PCL_LIBRARIES})
target_link_libraries (rotate_axis_pcd ${PCL_LIBRARIES})
