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

# Utility rule file for ardronelib.

# Include the progress variables for this target.
include CMakeFiles/ardronelib.dir/progress.make

CMakeFiles/ardronelib: CMakeFiles/ardronelib-complete


CMakeFiles/ardronelib-complete: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-install
CMakeFiles/ardronelib-complete: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-mkdir
CMakeFiles/ardronelib-complete: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-download
CMakeFiles/ardronelib-complete: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-update
CMakeFiles/ardronelib-complete: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-patch
CMakeFiles/ardronelib-complete: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-configure
CMakeFiles/ardronelib-complete: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-build
CMakeFiles/ardronelib-complete: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'ardronelib'"
	/usr/bin/cmake -E make_directory /home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles
	/usr/bin/cmake -E touch /home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles/ardronelib-complete
	/usr/bin/cmake -E touch /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-done

/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-install: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing install step for 'ardronelib'"
	cd /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib && make install INSTALL_PREFIX=/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/lib/ardrone
	cd /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib && /usr/bin/cmake -E touch /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-install

/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Creating directories for 'ardronelib'"
	/usr/bin/cmake -E make_directory /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib
	/usr/bin/cmake -E make_directory /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib
	/usr/bin/cmake -E make_directory /home/vinayaka/project_ws/devel/.private/ardrone_autonomy
	/usr/bin/cmake -E make_directory /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/tmp
	/usr/bin/cmake -E make_directory /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp
	/usr/bin/cmake -E make_directory /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src
	/usr/bin/cmake -E touch /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-mkdir

/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-download: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-gitinfo.txt
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-download: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'ardronelib'"
	cd /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src && /usr/bin/cmake -P /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/tmp/ardronelib-gitclone.cmake
	cd /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src && /usr/bin/cmake -E touch /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-download

/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-update: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Performing update step for 'ardronelib'"
	cd /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib && /usr/bin/cmake -P /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/tmp/ardronelib-gitupdate.cmake

/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-patch: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "No patch step for 'ardronelib'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-patch

/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-configure: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/tmp/ardronelib-cfgcmd.txt
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-configure: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-update
/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-configure: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Performing configure step for 'ardronelib'"
	cd /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib && echo "No configure"
	cd /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib && /usr/bin/cmake -E touch /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-configure

/home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-build: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing build step for 'ardronelib'"
	cd /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib && make
	cd /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib && /usr/bin/cmake -E touch /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-build

ardronelib: CMakeFiles/ardronelib
ardronelib: CMakeFiles/ardronelib-complete
ardronelib: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-install
ardronelib: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-mkdir
ardronelib: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-download
ardronelib: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-update
ardronelib: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-patch
ardronelib: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-configure
ardronelib: /home/vinayaka/project_ws/devel/.private/ardrone_autonomy/src/ardronelib-stamp/ardronelib-build
ardronelib: CMakeFiles/ardronelib.dir/build.make

.PHONY : ardronelib

# Rule to build all files generated by this target.
CMakeFiles/ardronelib.dir/build: ardronelib

.PHONY : CMakeFiles/ardronelib.dir/build

CMakeFiles/ardronelib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ardronelib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ardronelib.dir/clean

CMakeFiles/ardronelib.dir/depend:
	cd /home/vinayaka/project_ws/build/ardrone_autonomy && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy /home/vinayaka/project_ws/src/scoutrobot/ardrone_autonomy /home/vinayaka/project_ws/build/ardrone_autonomy /home/vinayaka/project_ws/build/ardrone_autonomy /home/vinayaka/project_ws/build/ardrone_autonomy/CMakeFiles/ardronelib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ardronelib.dir/depend

