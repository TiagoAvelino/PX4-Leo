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
include src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/depend.make

# Include the progress variables for this target.
include src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/progress.make

# Include the compile flags for this target's objects.
include src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/flags.make

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/flags.make
src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/ekf_att_pos_estimator_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/ekf_att_pos_estimator_main.cpp

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/ekf_att_pos_estimator_main.cpp > CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.i

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/ekf_att_pos_estimator_main.cpp -o CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.s

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o.requires:

.PHONY : src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o.requires

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o.provides: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o.requires
	$(MAKE) -f src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/build.make src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o.provides.build
.PHONY : src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o.provides

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o.provides.build: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o


src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/flags.make
src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/estimator_22states.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/estimator_22states.cpp

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/estimator_22states.cpp > CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.i

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/estimator_22states.cpp -o CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.s

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o.requires:

.PHONY : src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o.requires

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o.provides: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o.requires
	$(MAKE) -f src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/build.make src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o.provides.build
.PHONY : src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o.provides

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o.provides.build: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o


src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/flags.make
src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/estimator_utilities.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/estimator_utilities.cpp

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/estimator_utilities.cpp > CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.i

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator/estimator_utilities.cpp -o CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.s

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o.requires:

.PHONY : src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o.requires

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o.provides: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o.requires
	$(MAKE) -f src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/build.make src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o.provides.build
.PHONY : src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o.provides

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o.provides.build: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o


# Object files for target examples__ekf_att_pos_estimator
examples__ekf_att_pos_estimator_OBJECTS = \
"CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o" \
"CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o" \
"CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o"

# External object files for target examples__ekf_att_pos_estimator
examples__ekf_att_pos_estimator_EXTERNAL_OBJECTS =

src/examples/ekf_att_pos_estimator/libexamples__ekf_att_pos_estimator.a: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o
src/examples/ekf_att_pos_estimator/libexamples__ekf_att_pos_estimator.a: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o
src/examples/ekf_att_pos_estimator/libexamples__ekf_att_pos_estimator.a: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o
src/examples/ekf_att_pos_estimator/libexamples__ekf_att_pos_estimator.a: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/build.make
src/examples/ekf_att_pos_estimator/libexamples__ekf_att_pos_estimator.a: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libexamples__ekf_att_pos_estimator.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && $(CMAKE_COMMAND) -P CMakeFiles/examples__ekf_att_pos_estimator.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/examples__ekf_att_pos_estimator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/build: src/examples/ekf_att_pos_estimator/libexamples__ekf_att_pos_estimator.a

.PHONY : src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/build

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/requires: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/ekf_att_pos_estimator_main.cpp.o.requires
src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/requires: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_22states.cpp.o.requires
src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/requires: src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/estimator_utilities.cpp.o.requires

.PHONY : src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/requires

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator && $(CMAKE_COMMAND) -P CMakeFiles/examples__ekf_att_pos_estimator.dir/cmake_clean.cmake
.PHONY : src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/clean

src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/examples/ekf_att_pos_estimator /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator /home/leonardo/src/PX4_Firmware/Firmware-build/src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/examples/ekf_att_pos_estimator/CMakeFiles/examples__ekf_att_pos_estimator.dir/depend

