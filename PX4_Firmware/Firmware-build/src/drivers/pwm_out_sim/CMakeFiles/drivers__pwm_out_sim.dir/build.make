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
include src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/depend.make

# Include the progress variables for this target.
include src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/flags.make

src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o: src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/flags.make
src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/pwm_out_sim/pwm_out_sim.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/pwm_out_sim && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/pwm_out_sim/pwm_out_sim.cpp

src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/pwm_out_sim && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/pwm_out_sim/pwm_out_sim.cpp > CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.i

src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/pwm_out_sim && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/pwm_out_sim/pwm_out_sim.cpp -o CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.s

src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o.requires:

.PHONY : src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o.requires

src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o.provides: src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o.requires
	$(MAKE) -f src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/build.make src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o.provides.build
.PHONY : src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o.provides

src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o.provides.build: src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o


# Object files for target drivers__pwm_out_sim
drivers__pwm_out_sim_OBJECTS = \
"CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o"

# External object files for target drivers__pwm_out_sim
drivers__pwm_out_sim_EXTERNAL_OBJECTS =

src/drivers/pwm_out_sim/libdrivers__pwm_out_sim.a: src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o
src/drivers/pwm_out_sim/libdrivers__pwm_out_sim.a: src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/build.make
src/drivers/pwm_out_sim/libdrivers__pwm_out_sim.a: src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdrivers__pwm_out_sim.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/pwm_out_sim && $(CMAKE_COMMAND) -P CMakeFiles/drivers__pwm_out_sim.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/pwm_out_sim && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__pwm_out_sim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/build: src/drivers/pwm_out_sim/libdrivers__pwm_out_sim.a

.PHONY : src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/build

src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/requires: src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/pwm_out_sim.cpp.o.requires

.PHONY : src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/requires

src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/pwm_out_sim && $(CMAKE_COMMAND) -P CMakeFiles/drivers__pwm_out_sim.dir/cmake_clean.cmake
.PHONY : src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/clean

src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/pwm_out_sim /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/pwm_out_sim /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/pwm_out_sim/CMakeFiles/drivers__pwm_out_sim.dir/depend

