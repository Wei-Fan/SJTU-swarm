# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_COMMAND = /home/wade/clion-2018.2.5/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/wade/clion-2018.2.5/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wade/SJTU-swarm/swarm_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug

# Include any dependencies generated for this target.
include swarm_center/CMakeFiles/coverage_commander.dir/depend.make

# Include the progress variables for this target.
include swarm_center/CMakeFiles/coverage_commander.dir/progress.make

# Include the compile flags for this target's objects.
include swarm_center/CMakeFiles/coverage_commander.dir/flags.make

swarm_center/CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.o: swarm_center/CMakeFiles/coverage_commander.dir/flags.make
swarm_center/CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.o: ../swarm_center/src/coverage_commander.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object swarm_center/CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.o"
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/swarm_center && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.o -c /home/wade/SJTU-swarm/swarm_ws/src/swarm_center/src/coverage_commander.cpp

swarm_center/CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.i"
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/swarm_center && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wade/SJTU-swarm/swarm_ws/src/swarm_center/src/coverage_commander.cpp > CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.i

swarm_center/CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.s"
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/swarm_center && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wade/SJTU-swarm/swarm_ws/src/swarm_center/src/coverage_commander.cpp -o CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.s

# Object files for target coverage_commander
coverage_commander_OBJECTS = \
"CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.o"

# External object files for target coverage_commander
coverage_commander_EXTERNAL_OBJECTS =

devel/lib/swarm_center/coverage_commander: swarm_center/CMakeFiles/coverage_commander.dir/src/coverage_commander.cpp.o
devel/lib/swarm_center/coverage_commander: swarm_center/CMakeFiles/coverage_commander.dir/build.make
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libcv_bridge.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libimage_transport.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libdynamic_reconfigure_config_init_mutex.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libserial.so
devel/lib/swarm_center/coverage_commander: /home/wade/catkin_ws/devel/lib/libmavros.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libGeographic.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libclass_loader.so
devel/lib/swarm_center/coverage_commander: /usr/lib/libPocoFoundation.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libroslib.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/librospack.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libpython2.7.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libtinyxml.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libtf2_ros.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libactionlib.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libmessage_filters.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libroscpp.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/librosconsole.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libxmlrpcpp.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libtf2.so
devel/lib/swarm_center/coverage_commander: /home/wade/catkin_ws/devel/lib/libmavconn.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libeigen_conversions.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libroscpp_serialization.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/librostime.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libcpp_common.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libtinyxml.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libtf2_ros.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libactionlib.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libmessage_filters.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libroscpp.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/librosconsole.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libxmlrpcpp.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libtf2.so
devel/lib/swarm_center/coverage_commander: /home/wade/catkin_ws/devel/lib/libmavconn.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libeigen_conversions.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libroscpp_serialization.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/librostime.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/libcpp_common.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/swarm_center/coverage_commander: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
devel/lib/swarm_center/coverage_commander: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
devel/lib/swarm_center/coverage_commander: swarm_center/CMakeFiles/coverage_commander.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../devel/lib/swarm_center/coverage_commander"
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/swarm_center && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/coverage_commander.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
swarm_center/CMakeFiles/coverage_commander.dir/build: devel/lib/swarm_center/coverage_commander

.PHONY : swarm_center/CMakeFiles/coverage_commander.dir/build

swarm_center/CMakeFiles/coverage_commander.dir/clean:
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/swarm_center && $(CMAKE_COMMAND) -P CMakeFiles/coverage_commander.dir/cmake_clean.cmake
.PHONY : swarm_center/CMakeFiles/coverage_commander.dir/clean

swarm_center/CMakeFiles/coverage_commander.dir/depend:
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wade/SJTU-swarm/swarm_ws/src /home/wade/SJTU-swarm/swarm_ws/src/swarm_center /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/swarm_center /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/swarm_center/CMakeFiles/coverage_commander.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : swarm_center/CMakeFiles/coverage_commander.dir/depend

