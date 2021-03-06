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
include src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/depend.make

# Include the progress variables for this target.
include src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/flags.make

src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o: src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/flags.make
src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/pid_att_control/pid_att_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/pid_att_control && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/pid_att_control/pid_att_control.cpp

src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/pid_att_control && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/pid_att_control/pid_att_control.cpp > CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.i

src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/pid_att_control && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/pid_att_control/pid_att_control.cpp -o CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.s

src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o.requires:

.PHONY : src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o.requires

src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o.provides: src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o.requires
	$(MAKE) -f src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/build.make src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o.provides.build
.PHONY : src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o.provides

src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o.provides.build: src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o


# Object files for target modules__pid_att_control
modules__pid_att_control_OBJECTS = \
"CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o"

# External object files for target modules__pid_att_control
modules__pid_att_control_EXTERNAL_OBJECTS =

src/modules/pid_att_control/libmodules__pid_att_control.a: src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o
src/modules/pid_att_control/libmodules__pid_att_control.a: src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/build.make
src/modules/pid_att_control/libmodules__pid_att_control.a: src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmodules__pid_att_control.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/pid_att_control && $(CMAKE_COMMAND) -P CMakeFiles/modules__pid_att_control.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/pid_att_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__pid_att_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/build: src/modules/pid_att_control/libmodules__pid_att_control.a

.PHONY : src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/build

src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/requires: src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/pid_att_control.cpp.o.requires

.PHONY : src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/requires

src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/pid_att_control && $(CMAKE_COMMAND) -P CMakeFiles/modules__pid_att_control.dir/cmake_clean.cmake
.PHONY : src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/clean

src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/pid_att_control /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/pid_att_control /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/pid_att_control/CMakeFiles/modules__pid_att_control.dir/depend

