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

# Include any dependencies generated for this target.
include src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/depend.make

# Include the progress variables for this target.
include src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/progress.make

# Include the compile flags for this target's objects.
include src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/flags.make

src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o: src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/flags.make
src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/px4_mavlink_debug/px4_mavlink_debug.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/px4_mavlink_debug && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o   -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/px4_mavlink_debug/px4_mavlink_debug.c

src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/px4_mavlink_debug && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/px4_mavlink_debug/px4_mavlink_debug.c > CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.i

src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/px4_mavlink_debug && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/px4_mavlink_debug/px4_mavlink_debug.c -o CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.s

src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o.requires:

.PHONY : src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o.requires

src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o.provides: src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o.requires
	$(MAKE) -f src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/build.make src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o.provides.build
.PHONY : src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o.provides

src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o.provides.build: src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o


# Object files for target examples__px4_mavlink_debug
examples__px4_mavlink_debug_OBJECTS = \
"CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o"

# External object files for target examples__px4_mavlink_debug
examples__px4_mavlink_debug_EXTERNAL_OBJECTS =

src/examples/px4_mavlink_debug/libexamples__px4_mavlink_debug.a: src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o
src/examples/px4_mavlink_debug/libexamples__px4_mavlink_debug.a: src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/build.make
src/examples/px4_mavlink_debug/libexamples__px4_mavlink_debug.a: src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libexamples__px4_mavlink_debug.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/px4_mavlink_debug && $(CMAKE_COMMAND) -P CMakeFiles/examples__px4_mavlink_debug.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/px4_mavlink_debug && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/examples__px4_mavlink_debug.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/build: src/examples/px4_mavlink_debug/libexamples__px4_mavlink_debug.a

.PHONY : src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/build

src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/requires: src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/px4_mavlink_debug.c.o.requires

.PHONY : src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/requires

src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/px4_mavlink_debug && $(CMAKE_COMMAND) -P CMakeFiles/examples__px4_mavlink_debug.dir/cmake_clean.cmake
.PHONY : src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/clean

src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/px4_mavlink_debug /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/px4_mavlink_debug /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/examples/px4_mavlink_debug/CMakeFiles/examples__px4_mavlink_debug.dir/depend

