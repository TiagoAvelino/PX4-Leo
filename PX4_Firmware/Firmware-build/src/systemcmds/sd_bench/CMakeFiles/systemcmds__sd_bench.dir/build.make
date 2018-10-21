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
include src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/depend.make

# Include the progress variables for this target.
include src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/progress.make

# Include the compile flags for this target's objects.
include src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/flags.make

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o: src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/flags.make
src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/systemcmds/sd_bench/sd_bench.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/systemcmds/sd_bench && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o   -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/systemcmds/sd_bench/sd_bench.c

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/systemcmds/sd_bench && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/systemcmds/sd_bench/sd_bench.c > CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.i

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/systemcmds/sd_bench && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/systemcmds/sd_bench/sd_bench.c -o CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.s

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o.requires:

.PHONY : src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o.requires

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o.provides: src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o.requires
	$(MAKE) -f src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/build.make src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o.provides.build
.PHONY : src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o.provides

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o.provides.build: src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o


# Object files for target systemcmds__sd_bench
systemcmds__sd_bench_OBJECTS = \
"CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o"

# External object files for target systemcmds__sd_bench
systemcmds__sd_bench_EXTERNAL_OBJECTS =

src/systemcmds/sd_bench/libsystemcmds__sd_bench.a: src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o
src/systemcmds/sd_bench/libsystemcmds__sd_bench.a: src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/build.make
src/systemcmds/sd_bench/libsystemcmds__sd_bench.a: src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libsystemcmds__sd_bench.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/systemcmds/sd_bench && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__sd_bench.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/systemcmds/sd_bench && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/systemcmds__sd_bench.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/build: src/systemcmds/sd_bench/libsystemcmds__sd_bench.a

.PHONY : src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/build

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/requires: src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/sd_bench.c.o.requires

.PHONY : src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/requires

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/systemcmds/sd_bench && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__sd_bench.dir/cmake_clean.cmake
.PHONY : src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/clean

src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/systemcmds/sd_bench /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/systemcmds/sd_bench /home/leonardo/src/PX4_Firmware/Firmware-build/src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/systemcmds/sd_bench/CMakeFiles/systemcmds__sd_bench.dir/depend

