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
include src/modules/load_mon/CMakeFiles/modules__load_mon.dir/depend.make

# Include the progress variables for this target.
include src/modules/load_mon/CMakeFiles/modules__load_mon.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/load_mon/CMakeFiles/modules__load_mon.dir/flags.make

src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o: src/modules/load_mon/CMakeFiles/modules__load_mon.dir/flags.make
src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/load_mon/load_mon.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/load_mon && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__load_mon.dir/load_mon.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/load_mon/load_mon.cpp

src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__load_mon.dir/load_mon.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/load_mon && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/load_mon/load_mon.cpp > CMakeFiles/modules__load_mon.dir/load_mon.cpp.i

src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__load_mon.dir/load_mon.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/load_mon && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/load_mon/load_mon.cpp -o CMakeFiles/modules__load_mon.dir/load_mon.cpp.s

src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o.requires:

.PHONY : src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o.requires

src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o.provides: src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o.requires
	$(MAKE) -f src/modules/load_mon/CMakeFiles/modules__load_mon.dir/build.make src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o.provides.build
.PHONY : src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o.provides

src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o.provides.build: src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o


# Object files for target modules__load_mon
modules__load_mon_OBJECTS = \
"CMakeFiles/modules__load_mon.dir/load_mon.cpp.o"

# External object files for target modules__load_mon
modules__load_mon_EXTERNAL_OBJECTS =

src/modules/load_mon/libmodules__load_mon.a: src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o
src/modules/load_mon/libmodules__load_mon.a: src/modules/load_mon/CMakeFiles/modules__load_mon.dir/build.make
src/modules/load_mon/libmodules__load_mon.a: src/modules/load_mon/CMakeFiles/modules__load_mon.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmodules__load_mon.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/load_mon && $(CMAKE_COMMAND) -P CMakeFiles/modules__load_mon.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/load_mon && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__load_mon.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/load_mon/CMakeFiles/modules__load_mon.dir/build: src/modules/load_mon/libmodules__load_mon.a

.PHONY : src/modules/load_mon/CMakeFiles/modules__load_mon.dir/build

src/modules/load_mon/CMakeFiles/modules__load_mon.dir/requires: src/modules/load_mon/CMakeFiles/modules__load_mon.dir/load_mon.cpp.o.requires

.PHONY : src/modules/load_mon/CMakeFiles/modules__load_mon.dir/requires

src/modules/load_mon/CMakeFiles/modules__load_mon.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/load_mon && $(CMAKE_COMMAND) -P CMakeFiles/modules__load_mon.dir/cmake_clean.cmake
.PHONY : src/modules/load_mon/CMakeFiles/modules__load_mon.dir/clean

src/modules/load_mon/CMakeFiles/modules__load_mon.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/load_mon /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/load_mon /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/load_mon/CMakeFiles/modules__load_mon.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/load_mon/CMakeFiles/modules__load_mon.dir/depend

