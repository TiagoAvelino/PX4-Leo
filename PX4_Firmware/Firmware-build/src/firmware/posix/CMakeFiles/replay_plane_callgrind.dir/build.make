# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/leonardo/src/PX4_Firmware/PX4_Firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leonardo/src/PX4_Firmware/Firmware-build

# Utility rule file for replay_plane_callgrind.

# Include the progress variables for this target.
include src/firmware/posix/CMakeFiles/replay_plane_callgrind.dir/progress.make

src/firmware/posix/CMakeFiles/replay_plane_callgrind:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/tmp && /home/leonardo/src/PX4_Firmware/PX4_Firmware/Tools/sitl_run.sh /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix/px4 posix-configs/SITL/init/ekf2 callgrind replay plane /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/Firmware-build

replay_plane_callgrind: src/firmware/posix/CMakeFiles/replay_plane_callgrind
replay_plane_callgrind: src/firmware/posix/CMakeFiles/replay_plane_callgrind.dir/build.make

.PHONY : replay_plane_callgrind

# Rule to build all files generated by this target.
src/firmware/posix/CMakeFiles/replay_plane_callgrind.dir/build: replay_plane_callgrind

.PHONY : src/firmware/posix/CMakeFiles/replay_plane_callgrind.dir/build

src/firmware/posix/CMakeFiles/replay_plane_callgrind.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix && $(CMAKE_COMMAND) -P CMakeFiles/replay_plane_callgrind.dir/cmake_clean.cmake
.PHONY : src/firmware/posix/CMakeFiles/replay_plane_callgrind.dir/clean

src/firmware/posix/CMakeFiles/replay_plane_callgrind.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/firmware/posix /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix/CMakeFiles/replay_plane_callgrind.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/firmware/posix/CMakeFiles/replay_plane_callgrind.dir/depend

