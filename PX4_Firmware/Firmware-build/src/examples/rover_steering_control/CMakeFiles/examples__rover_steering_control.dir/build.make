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
include src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/depend.make

# Include the progress variables for this target.
include src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/progress.make

# Include the compile flags for this target's objects.
include src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/flags.make

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/flags.make
src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/rover_steering_control/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/rover_steering_control && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/examples__rover_steering_control.dir/main.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/rover_steering_control/main.cpp

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/examples__rover_steering_control.dir/main.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/rover_steering_control && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/rover_steering_control/main.cpp > CMakeFiles/examples__rover_steering_control.dir/main.cpp.i

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/examples__rover_steering_control.dir/main.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/rover_steering_control && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/rover_steering_control/main.cpp -o CMakeFiles/examples__rover_steering_control.dir/main.cpp.s

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o.requires:

.PHONY : src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o.requires

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o.provides: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o.requires
	$(MAKE) -f src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/build.make src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o.provides.build
.PHONY : src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o.provides

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o.provides.build: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o


src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/flags.make
src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/rover_steering_control/params.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/rover_steering_control && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/examples__rover_steering_control.dir/params.c.o   -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/rover_steering_control/params.c

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/examples__rover_steering_control.dir/params.c.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/rover_steering_control && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/rover_steering_control/params.c > CMakeFiles/examples__rover_steering_control.dir/params.c.i

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/examples__rover_steering_control.dir/params.c.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/rover_steering_control && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/rover_steering_control/params.c -o CMakeFiles/examples__rover_steering_control.dir/params.c.s

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o.requires:

.PHONY : src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o.requires

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o.provides: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o.requires
	$(MAKE) -f src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/build.make src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o.provides.build
.PHONY : src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o.provides

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o.provides.build: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o


# Object files for target examples__rover_steering_control
examples__rover_steering_control_OBJECTS = \
"CMakeFiles/examples__rover_steering_control.dir/main.cpp.o" \
"CMakeFiles/examples__rover_steering_control.dir/params.c.o"

# External object files for target examples__rover_steering_control
examples__rover_steering_control_EXTERNAL_OBJECTS =

src/examples/rover_steering_control/libexamples__rover_steering_control.a: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o
src/examples/rover_steering_control/libexamples__rover_steering_control.a: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o
src/examples/rover_steering_control/libexamples__rover_steering_control.a: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/build.make
src/examples/rover_steering_control/libexamples__rover_steering_control.a: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libexamples__rover_steering_control.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/rover_steering_control && $(CMAKE_COMMAND) -P CMakeFiles/examples__rover_steering_control.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/rover_steering_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/examples__rover_steering_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/build: src/examples/rover_steering_control/libexamples__rover_steering_control.a

.PHONY : src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/build

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/requires: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/main.cpp.o.requires
src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/requires: src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/params.c.o.requires

.PHONY : src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/requires

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/rover_steering_control && $(CMAKE_COMMAND) -P CMakeFiles/examples__rover_steering_control.dir/cmake_clean.cmake
.PHONY : src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/clean

src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/rover_steering_control /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/rover_steering_control /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/examples/rover_steering_control/CMakeFiles/examples__rover_steering_control.dir/depend
