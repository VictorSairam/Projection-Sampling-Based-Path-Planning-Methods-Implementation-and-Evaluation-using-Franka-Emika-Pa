# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vinayaka/project_ws/build/ardrone_autonomy

# Include any dependencies generated for this target.
include CMakeFiles/ardrone_driver.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ardrone_driver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ardrone_driver.dir/flags.make

CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o: CMakeFiles/ardrone_driver.dir/flags.make
CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o: /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/ardrone_driver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o -c /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/ardrone_driver.cpp

CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/ardrone_driver.cpp > CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.i

CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/ardrone_driver.cpp -o CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.s

CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o.requires:

.PHONY : CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o.requires

CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o.provides: CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o.requires
	$(MAKE) -f CMakeFiles/ardrone_driver.dir/build.make CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o.provides.build
.PHONY : CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o.provides

CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o.provides.build: CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o


CMakeFiles/ardrone_driver.dir/src/video.cpp.o: CMakeFiles/ardrone_driver.dir/flags.make
CMakeFiles/ardrone_driver.dir/src/video.cpp.o: /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/video.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ardrone_driver.dir/src/video.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ardrone_driver.dir/src/video.cpp.o -c /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/video.cpp

CMakeFiles/ardrone_driver.dir/src/video.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ardrone_driver.dir/src/video.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/video.cpp > CMakeFiles/ardrone_driver.dir/src/video.cpp.i

CMakeFiles/ardrone_driver.dir/src/video.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ardrone_driver.dir/src/video.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/video.cpp -o CMakeFiles/ardrone_driver.dir/src/video.cpp.s

CMakeFiles/ardrone_driver.dir/src/video.cpp.o.requires:

.PHONY : CMakeFiles/ardrone_driver.dir/src/video.cpp.o.requires

CMakeFiles/ardrone_driver.dir/src/video.cpp.o.provides: CMakeFiles/ardrone_driver.dir/src/video.cpp.o.requires
	$(MAKE) -f CMakeFiles/ardrone_driver.dir/build.make CMakeFiles/ardrone_driver.dir/src/video.cpp.o.provides.build
.PHONY : CMakeFiles/ardrone_driver.dir/src/video.cpp.o.provides

CMakeFiles/ardrone_driver.dir/src/video.cpp.o.provides.build: CMakeFiles/ardrone_driver.dir/src/video.cpp.o


CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o: CMakeFiles/ardrone_driver.dir/flags.make
CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o: /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/ardrone_sdk.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o -c /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/ardrone_sdk.cpp

CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/ardrone_sdk.cpp > CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.i

CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/ardrone_sdk.cpp -o CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.s

CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o.requires:

.PHONY : CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o.requires

CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o.provides: CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o.requires
	$(MAKE) -f CMakeFiles/ardrone_driver.dir/build.make CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o.provides.build
.PHONY : CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o.provides

CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o.provides.build: CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o


CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o: CMakeFiles/ardrone_driver.dir/flags.make
CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o: /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/teleop_twist.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o -c /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/teleop_twist.cpp

CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/teleop_twist.cpp > CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.i

CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy/src/teleop_twist.cpp -o CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.s

CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o.requires:

.PHONY : CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o.requires

CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o.provides: CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o.requires
	$(MAKE) -f CMakeFiles/ardrone_driver.dir/build.make CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o.provides.build
.PHONY : CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o.provides

CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o.provides.build: CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o


# Object files for target ardrone_driver
ardrone_driver_OBJECTS = \
"CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o" \
"CMakeFiles/ardrone_driver.dir/src/video.cpp.o" \
"CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o" \
"CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o"

# External object files for target ardrone_driver
ardrone_driver_EXTERNAL_OBJECTS =

/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: CMakeFiles/ardrone_driver.dir/src/video.cpp.o
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: CMakeFiles/ardrone_driver.dir/build.make
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libimage_transport.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libclass_loader.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/libPocoFoundation.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libdl.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libroslib.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/librospack.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libtf.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libtf2_ros.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libactionlib.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libmessage_filters.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libtf2.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libcamera_info_manager.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libcamera_calibration_parsers.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libroscpp.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/librosconsole.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/librostime.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /opt/ros/kinetic/lib/libcpp_common.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver: CMakeFiles/ardrone_driver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ardrone_driver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ardrone_driver.dir/build: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone_autonomy/ardrone_driver

.PHONY : CMakeFiles/ardrone_driver.dir/build

CMakeFiles/ardrone_driver.dir/requires: CMakeFiles/ardrone_driver.dir/src/ardrone_driver.cpp.o.requires
CMakeFiles/ardrone_driver.dir/requires: CMakeFiles/ardrone_driver.dir/src/video.cpp.o.requires
CMakeFiles/ardrone_driver.dir/requires: CMakeFiles/ardrone_driver.dir/src/ardrone_sdk.cpp.o.requires
CMakeFiles/ardrone_driver.dir/requires: CMakeFiles/ardrone_driver.dir/src/teleop_twist.cpp.o.requires

.PHONY : CMakeFiles/ardrone_driver.dir/requires

CMakeFiles/ardrone_driver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ardrone_driver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ardrone_driver.dir/clean

CMakeFiles/ardrone_driver.dir/depend:
	cd /home/vinayaka/project_ws/build/ardrone_autonomy && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy /home/vinayaka/project_ws/build/ardrone_autonomy /home/vinayaka/project_ws/build/ardrone_autonomy /home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles/ardrone_driver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ardrone_driver.dir/depend
