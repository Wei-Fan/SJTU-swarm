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
include viconros/CMakeFiles/viconros_swarm.dir/depend.make

# Include the progress variables for this target.
include viconros/CMakeFiles/viconros_swarm.dir/progress.make

# Include the compile flags for this target's objects.
include viconros/CMakeFiles/viconros_swarm.dir/flags.make

viconros/CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.o: viconros/CMakeFiles/viconros_swarm.dir/flags.make
viconros/CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.o: ../viconros/src/viconros_swarm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object viconros/CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.o"
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/viconros && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.o -c /home/wade/SJTU-swarm/swarm_ws/src/viconros/src/viconros_swarm.cpp

viconros/CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.i"
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/viconros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wade/SJTU-swarm/swarm_ws/src/viconros/src/viconros_swarm.cpp > CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.i

viconros/CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.s"
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/viconros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wade/SJTU-swarm/swarm_ws/src/viconros/src/viconros_swarm.cpp -o CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.s

viconros/CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.o: viconros/CMakeFiles/viconros_swarm.dir/flags.make
viconros/CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.o: ../viconros/src/CFetchViconData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object viconros/CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.o"
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/viconros && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.o -c /home/wade/SJTU-swarm/swarm_ws/src/viconros/src/CFetchViconData.cpp

viconros/CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.i"
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/viconros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wade/SJTU-swarm/swarm_ws/src/viconros/src/CFetchViconData.cpp > CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.i

viconros/CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.s"
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/viconros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wade/SJTU-swarm/swarm_ws/src/viconros/src/CFetchViconData.cpp -o CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.s

# Object files for target viconros_swarm
viconros_swarm_OBJECTS = \
"CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.o" \
"CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.o"

# External object files for target viconros_swarm
viconros_swarm_EXTERNAL_OBJECTS =

devel/lib/viconros/viconros_swarm: viconros/CMakeFiles/viconros_swarm.dir/src/viconros_swarm.cpp.o
devel/lib/viconros/viconros_swarm: viconros/CMakeFiles/viconros_swarm.dir/src/CFetchViconData.cpp.o
devel/lib/viconros/viconros_swarm: viconros/CMakeFiles/viconros_swarm.dir/build.make
devel/lib/viconros/viconros_swarm: /opt/ros/kinetic/lib/libroscpp.so
devel/lib/viconros/viconros_swarm: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/viconros/viconros_swarm: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/viconros/viconros_swarm: /opt/ros/kinetic/lib/librosconsole.so
devel/lib/viconros/viconros_swarm: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
devel/lib/viconros/viconros_swarm: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
devel/lib/viconros/viconros_swarm: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/viconros/viconros_swarm: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/viconros/viconros_swarm: /opt/ros/kinetic/lib/libxmlrpcpp.so
devel/lib/viconros/viconros_swarm: /opt/ros/kinetic/lib/libroscpp_serialization.so
devel/lib/viconros/viconros_swarm: /opt/ros/kinetic/lib/librostime.so
devel/lib/viconros/viconros_swarm: /opt/ros/kinetic/lib/libcpp_common.so
devel/lib/viconros/viconros_swarm: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/viconros/viconros_swarm: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/viconros/viconros_swarm: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/viconros/viconros_swarm: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/viconros/viconros_swarm: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/viconros/viconros_swarm: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/viconros/viconros_swarm: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/viconros/viconros_swarm: viconros/CMakeFiles/viconros_swarm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../devel/lib/viconros/viconros_swarm"
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/viconros && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/viconros_swarm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
viconros/CMakeFiles/viconros_swarm.dir/build: devel/lib/viconros/viconros_swarm

.PHONY : viconros/CMakeFiles/viconros_swarm.dir/build

viconros/CMakeFiles/viconros_swarm.dir/clean:
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/viconros && $(CMAKE_COMMAND) -P CMakeFiles/viconros_swarm.dir/cmake_clean.cmake
.PHONY : viconros/CMakeFiles/viconros_swarm.dir/clean

viconros/CMakeFiles/viconros_swarm.dir/depend:
	cd /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wade/SJTU-swarm/swarm_ws/src /home/wade/SJTU-swarm/swarm_ws/src/viconros /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/viconros /home/wade/SJTU-swarm/swarm_ws/src/cmake-build-debug/viconros/CMakeFiles/viconros_swarm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : viconros/CMakeFiles/viconros_swarm.dir/depend

