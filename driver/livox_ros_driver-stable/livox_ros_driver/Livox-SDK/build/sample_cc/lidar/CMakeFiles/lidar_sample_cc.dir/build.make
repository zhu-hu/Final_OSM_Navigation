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
CMAKE_SOURCE_DIR = /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build

# Include any dependencies generated for this target.
include sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/depend.make

# Include the progress variables for this target.
include sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/progress.make

# Include the compile flags for this target's objects.
include sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/flags.make

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/flags.make
sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o: ../sample_cc/lidar/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o"
	cd /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/sample_cc/lidar && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lidar_sample_cc.dir/main.cpp.o -c /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/sample_cc/lidar/main.cpp

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lidar_sample_cc.dir/main.cpp.i"
	cd /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/sample_cc/lidar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/sample_cc/lidar/main.cpp > CMakeFiles/lidar_sample_cc.dir/main.cpp.i

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lidar_sample_cc.dir/main.cpp.s"
	cd /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/sample_cc/lidar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/sample_cc/lidar/main.cpp -o CMakeFiles/lidar_sample_cc.dir/main.cpp.s

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o.requires:

.PHONY : sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o.requires

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o.provides: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o.requires
	$(MAKE) -f sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/build.make sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o.provides.build
.PHONY : sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o.provides

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o.provides.build: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o


sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/flags.make
sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o: ../sample_cc/lidar/lds_lidar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o"
	cd /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/sample_cc/lidar && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o -c /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/sample_cc/lidar/lds_lidar.cpp

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.i"
	cd /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/sample_cc/lidar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/sample_cc/lidar/lds_lidar.cpp > CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.i

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.s"
	cd /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/sample_cc/lidar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/sample_cc/lidar/lds_lidar.cpp -o CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.s

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o.requires:

.PHONY : sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o.requires

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o.provides: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o.requires
	$(MAKE) -f sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/build.make sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o.provides.build
.PHONY : sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o.provides

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o.provides.build: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o


# Object files for target lidar_sample_cc
lidar_sample_cc_OBJECTS = \
"CMakeFiles/lidar_sample_cc.dir/main.cpp.o" \
"CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o"

# External object files for target lidar_sample_cc
lidar_sample_cc_EXTERNAL_OBJECTS =

sample_cc/lidar/lidar_sample_cc: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o
sample_cc/lidar/lidar_sample_cc: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o
sample_cc/lidar/lidar_sample_cc: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/build.make
sample_cc/lidar/lidar_sample_cc: sdk_core/liblivox_sdk_static.a
sample_cc/lidar/lidar_sample_cc: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable lidar_sample_cc"
	cd /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/sample_cc/lidar && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lidar_sample_cc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/build: sample_cc/lidar/lidar_sample_cc

.PHONY : sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/build

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/requires: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/main.cpp.o.requires
sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/requires: sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/lds_lidar.cpp.o.requires

.PHONY : sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/requires

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/clean:
	cd /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/sample_cc/lidar && $(CMAKE_COMMAND) -P CMakeFiles/lidar_sample_cc.dir/cmake_clean.cmake
.PHONY : sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/clean

sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/depend:
	cd /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/sample_cc/lidar /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/sample_cc/lidar /home/cyberc3/Projects/E100/src/driver/livox_ros_driver-stable/livox_ros_driver/Livox-SDK/build/sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sample_cc/lidar/CMakeFiles/lidar_sample_cc.dir/depend

