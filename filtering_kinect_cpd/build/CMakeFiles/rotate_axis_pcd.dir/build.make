# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yuji/workspace/PCL/filtering_kinect_cpd

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuji/workspace/PCL/filtering_kinect_cpd/build

# Include any dependencies generated for this target.
include CMakeFiles/rotate_axis_pcd.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rotate_axis_pcd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rotate_axis_pcd.dir/flags.make

CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o: CMakeFiles/rotate_axis_pcd.dir/flags.make
CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o: ../rotate_axis.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yuji/workspace/PCL/filtering_kinect_cpd/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o -c /home/yuji/workspace/PCL/filtering_kinect_cpd/rotate_axis.cpp

CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yuji/workspace/PCL/filtering_kinect_cpd/rotate_axis.cpp > CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.i

CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yuji/workspace/PCL/filtering_kinect_cpd/rotate_axis.cpp -o CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.s

CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o.requires:
.PHONY : CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o.requires

CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o.provides: CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o.requires
	$(MAKE) -f CMakeFiles/rotate_axis_pcd.dir/build.make CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o.provides.build
.PHONY : CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o.provides

CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o.provides.build: CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o

# Object files for target rotate_axis_pcd
rotate_axis_pcd_OBJECTS = \
"CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o"

# External object files for target rotate_axis_pcd
rotate_axis_pcd_EXTERNAL_OBJECTS =

rotate_axis_pcd: CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o
rotate_axis_pcd: CMakeFiles/rotate_axis_pcd.dir/build.make
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_system.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_thread.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libpthread.so
rotate_axis_pcd: /usr/lib/libpcl_common.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
rotate_axis_pcd: /usr/lib/libpcl_kdtree.so
rotate_axis_pcd: /usr/lib/libpcl_octree.so
rotate_axis_pcd: /usr/lib/libpcl_search.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libqhull.so
rotate_axis_pcd: /usr/lib/libpcl_surface.so
rotate_axis_pcd: /usr/lib/libpcl_sample_consensus.so
rotate_axis_pcd: /usr/lib/libOpenNI.so
rotate_axis_pcd: /usr/lib/libOpenNI2.so
rotate_axis_pcd: /usr/lib/libvtkCommon.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkFiltering.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkImaging.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkGraphics.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkGenericFiltering.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkIO.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkRendering.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkVolumeRendering.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkHybrid.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkWidgets.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkParallel.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkInfovis.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkGeovis.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkViews.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkCharts.so.5.8.0
rotate_axis_pcd: /usr/lib/libpcl_io.so
rotate_axis_pcd: /usr/lib/libpcl_filters.so
rotate_axis_pcd: /usr/lib/libpcl_features.so
rotate_axis_pcd: /usr/lib/libpcl_keypoints.so
rotate_axis_pcd: /usr/lib/libpcl_registration.so
rotate_axis_pcd: /usr/lib/libpcl_segmentation.so
rotate_axis_pcd: /usr/lib/libpcl_recognition.so
rotate_axis_pcd: /usr/lib/libpcl_visualization.so
rotate_axis_pcd: /usr/lib/libpcl_people.so
rotate_axis_pcd: /usr/lib/libpcl_outofcore.so
rotate_axis_pcd: /usr/lib/libpcl_tracking.so
rotate_axis_pcd: /usr/lib/libpcl_apps.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_system.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_thread.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libpthread.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libqhull.so
rotate_axis_pcd: /usr/lib/libOpenNI.so
rotate_axis_pcd: /usr/lib/libOpenNI2.so
rotate_axis_pcd: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
rotate_axis_pcd: /usr/lib/libvtkCommon.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkFiltering.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkImaging.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkGraphics.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkGenericFiltering.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkIO.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkRendering.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkVolumeRendering.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkHybrid.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkWidgets.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkParallel.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkInfovis.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkGeovis.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkViews.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkCharts.so.5.8.0
rotate_axis_pcd: /usr/lib/libpcl_common.so
rotate_axis_pcd: /usr/lib/libpcl_kdtree.so
rotate_axis_pcd: /usr/lib/libpcl_octree.so
rotate_axis_pcd: /usr/lib/libpcl_search.so
rotate_axis_pcd: /usr/lib/libpcl_surface.so
rotate_axis_pcd: /usr/lib/libpcl_sample_consensus.so
rotate_axis_pcd: /usr/lib/libpcl_io.so
rotate_axis_pcd: /usr/lib/libpcl_filters.so
rotate_axis_pcd: /usr/lib/libpcl_features.so
rotate_axis_pcd: /usr/lib/libpcl_keypoints.so
rotate_axis_pcd: /usr/lib/libpcl_registration.so
rotate_axis_pcd: /usr/lib/libpcl_segmentation.so
rotate_axis_pcd: /usr/lib/libpcl_recognition.so
rotate_axis_pcd: /usr/lib/libpcl_visualization.so
rotate_axis_pcd: /usr/lib/libpcl_people.so
rotate_axis_pcd: /usr/lib/libpcl_outofcore.so
rotate_axis_pcd: /usr/lib/libpcl_tracking.so
rotate_axis_pcd: /usr/lib/libpcl_apps.so
rotate_axis_pcd: /usr/lib/libvtkViews.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkInfovis.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkWidgets.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkVolumeRendering.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkHybrid.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkParallel.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkRendering.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkImaging.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkGraphics.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkIO.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkFiltering.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtkCommon.so.5.8.0
rotate_axis_pcd: /usr/lib/libvtksys.so.5.8.0
rotate_axis_pcd: CMakeFiles/rotate_axis_pcd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable rotate_axis_pcd"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rotate_axis_pcd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rotate_axis_pcd.dir/build: rotate_axis_pcd
.PHONY : CMakeFiles/rotate_axis_pcd.dir/build

CMakeFiles/rotate_axis_pcd.dir/requires: CMakeFiles/rotate_axis_pcd.dir/rotate_axis.cpp.o.requires
.PHONY : CMakeFiles/rotate_axis_pcd.dir/requires

CMakeFiles/rotate_axis_pcd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rotate_axis_pcd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rotate_axis_pcd.dir/clean

CMakeFiles/rotate_axis_pcd.dir/depend:
	cd /home/yuji/workspace/PCL/filtering_kinect_cpd/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuji/workspace/PCL/filtering_kinect_cpd /home/yuji/workspace/PCL/filtering_kinect_cpd /home/yuji/workspace/PCL/filtering_kinect_cpd/build /home/yuji/workspace/PCL/filtering_kinect_cpd/build /home/yuji/workspace/PCL/filtering_kinect_cpd/build/CMakeFiles/rotate_axis_pcd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rotate_axis_pcd.dir/depend

