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
include src/lib/led/CMakeFiles/lib__led.dir/depend.make

# Include the progress variables for this target.
include src/lib/led/CMakeFiles/lib__led.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/led/CMakeFiles/lib__led.dir/flags.make

src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o: src/lib/led/CMakeFiles/lib__led.dir/flags.make
src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/led/led.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/led && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__led.dir/led.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/led/led.cpp

src/lib/led/CMakeFiles/lib__led.dir/led.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__led.dir/led.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/led && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/led/led.cpp > CMakeFiles/lib__led.dir/led.cpp.i

src/lib/led/CMakeFiles/lib__led.dir/led.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__led.dir/led.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/led && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/led/led.cpp -o CMakeFiles/lib__led.dir/led.cpp.s

src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o.requires:

.PHONY : src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o.requires

src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o.provides: src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o.requires
	$(MAKE) -f src/lib/led/CMakeFiles/lib__led.dir/build.make src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o.provides.build
.PHONY : src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o.provides

src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o.provides.build: src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o


# Object files for target lib__led
lib__led_OBJECTS = \
"CMakeFiles/lib__led.dir/led.cpp.o"

# External object files for target lib__led
lib__led_EXTERNAL_OBJECTS =

src/lib/led/liblib__led.a: src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o
src/lib/led/liblib__led.a: src/lib/led/CMakeFiles/lib__led.dir/build.make
src/lib/led/liblib__led.a: src/lib/led/CMakeFiles/lib__led.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library liblib__led.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/led && $(CMAKE_COMMAND) -P CMakeFiles/lib__led.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/led && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib__led.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/led/CMakeFiles/lib__led.dir/build: src/lib/led/liblib__led.a

.PHONY : src/lib/led/CMakeFiles/lib__led.dir/build

src/lib/led/CMakeFiles/lib__led.dir/requires: src/lib/led/CMakeFiles/lib__led.dir/led.cpp.o.requires

.PHONY : src/lib/led/CMakeFiles/lib__led.dir/requires

src/lib/led/CMakeFiles/lib__led.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/led && $(CMAKE_COMMAND) -P CMakeFiles/lib__led.dir/cmake_clean.cmake
.PHONY : src/lib/led/CMakeFiles/lib__led.dir/clean

src/lib/led/CMakeFiles/lib__led.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/led /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/led /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/led/CMakeFiles/lib__led.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/led/CMakeFiles/lib__led.dir/depend

