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
CMAKE_SOURCE_DIR = /home/vinayaka/project_ws/src/octomap/octomap

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vinayaka/project_ws/build/octomap

# Include any dependencies generated for this target.
include src/CMakeFiles/binvox2bt.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/binvox2bt.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/binvox2bt.dir/flags.make

src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o: src/CMakeFiles/binvox2bt.dir/flags.make
src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o: /home/vinayaka/project_ws/src/octomap/octomap/src/binvox2bt.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vinayaka/project_ws/build/octomap/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o"
	cd /home/vinayaka/project_ws/build/octomap/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o -c /home/vinayaka/project_ws/src/octomap/octomap/src/binvox2bt.cpp

src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/binvox2bt.dir/binvox2bt.cpp.i"
	cd /home/vinayaka/project_ws/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vinayaka/project_ws/src/octomap/octomap/src/binvox2bt.cpp > CMakeFiles/binvox2bt.dir/binvox2bt.cpp.i

src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/binvox2bt.dir/binvox2bt.cpp.s"
	cd /home/vinayaka/project_ws/build/octomap/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vinayaka/project_ws/src/octomap/octomap/src/binvox2bt.cpp -o CMakeFiles/binvox2bt.dir/binvox2bt.cpp.s

src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o.requires:

.PHONY : src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o.requires

src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o.provides: src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/binvox2bt.dir/build.make src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o.provides.build
.PHONY : src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o.provides

src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o.provides.build: src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o


# Object files for target binvox2bt
binvox2bt_OBJECTS = \
"CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o"

# External object files for target binvox2bt
binvox2bt_EXTERNAL_OBJECTS =

/home/vinayaka/project_ws/src/octomap/octomap/bin/binvox2bt: src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o
/home/vinayaka/project_ws/src/octomap/octomap/bin/binvox2bt: src/CMakeFiles/binvox2bt.dir/build.make
/home/vinayaka/project_ws/src/octomap/octomap/bin/binvox2bt: /home/vinayaka/project_ws/src/octomap/octomap/lib/liboctomap.so.1.9.7
/home/vinayaka/project_ws/src/octomap/octomap/bin/binvox2bt: /home/vinayaka/project_ws/src/octomap/octomap/lib/liboctomath.so.1.9.7
/home/vinayaka/project_ws/src/octomap/octomap/bin/binvox2bt: src/CMakeFiles/binvox2bt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vinayaka/project_ws/build/octomap/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/vinayaka/project_ws/src/octomap/octomap/bin/binvox2bt"
	cd /home/vinayaka/project_ws/build/octomap/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/binvox2bt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/binvox2bt.dir/build: /home/vinayaka/project_ws/src/octomap/octomap/bin/binvox2bt

.PHONY : src/CMakeFiles/binvox2bt.dir/build

src/CMakeFiles/binvox2bt.dir/requires: src/CMakeFiles/binvox2bt.dir/binvox2bt.cpp.o.requires

.PHONY : src/CMakeFiles/binvox2bt.dir/requires

src/CMakeFiles/binvox2bt.dir/clean:
	cd /home/vinayaka/project_ws/build/octomap/src && $(CMAKE_COMMAND) -P CMakeFiles/binvox2bt.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/binvox2bt.dir/clean

src/CMakeFiles/binvox2bt.dir/depend:
	cd /home/vinayaka/project_ws/build/octomap && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vinayaka/project_ws/src/octomap/octomap /home/vinayaka/project_ws/src/octomap/octomap/src /home/vinayaka/project_ws/build/octomap /home/vinayaka/project_ws/build/octomap/src /home/vinayaka/project_ws/build/octomap/src/CMakeFiles/binvox2bt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/binvox2bt.dir/depend

