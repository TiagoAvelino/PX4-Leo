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
include src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/depend.make

# Include the progress variables for this target.
include src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/flags.make

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/flags.make
src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/commander_tests/commander_tests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/commander_tests/commander_tests.cpp

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/commander_tests/commander_tests.cpp > CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.i

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/commander_tests/commander_tests.cpp -o CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.s

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o.requires:

.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o.requires

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o.provides: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o.requires
	$(MAKE) -f src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/build.make src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o.provides.build
.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o.provides

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o.provides.build: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o


src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/flags.make
src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/commander_tests/state_machine_helper_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/commander_tests/state_machine_helper_test.cpp

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/commander_tests/state_machine_helper_test.cpp > CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.i

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/commander_tests/state_machine_helper_test.cpp -o CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.s

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o.requires:

.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o.requires

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o.provides: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o.requires
	$(MAKE) -f src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/build.make src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o.provides.build
.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o.provides

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o.provides.build: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o


src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/flags.make
src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/state_machine_helper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/state_machine_helper.cpp

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/state_machine_helper.cpp > CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.i

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/state_machine_helper.cpp -o CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.s

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o.requires:

.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o.requires

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o.provides: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o.requires
	$(MAKE) -f src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/build.make src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o.provides.build
.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o.provides

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o.provides.build: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o


src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/flags.make
src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/PreflightCheck.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/PreflightCheck.cpp

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/PreflightCheck.cpp > CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.i

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/PreflightCheck.cpp -o CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.s

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o.requires:

.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o.requires

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o.provides: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o.requires
	$(MAKE) -f src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/build.make src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o.provides.build
.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o.provides

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o.provides.build: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o


# Object files for target modules__commander__commander_tests
modules__commander__commander_tests_OBJECTS = \
"CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o" \
"CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o" \
"CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o" \
"CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o"

# External object files for target modules__commander__commander_tests
modules__commander__commander_tests_EXTERNAL_OBJECTS =

src/modules/commander/commander_tests/libmodules__commander__commander_tests.a: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o
src/modules/commander/commander_tests/libmodules__commander__commander_tests.a: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o
src/modules/commander/commander_tests/libmodules__commander__commander_tests.a: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o
src/modules/commander/commander_tests/libmodules__commander__commander_tests.a: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o
src/modules/commander/commander_tests/libmodules__commander__commander_tests.a: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/build.make
src/modules/commander/commander_tests/libmodules__commander__commander_tests.a: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libmodules__commander__commander_tests.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && $(CMAKE_COMMAND) -P CMakeFiles/modules__commander__commander_tests.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__commander__commander_tests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/build: src/modules/commander/commander_tests/libmodules__commander__commander_tests.a

.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/build

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/requires: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/commander_tests.cpp.o.requires
src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/requires: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/state_machine_helper_test.cpp.o.requires
src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/requires: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/state_machine_helper.cpp.o.requires
src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/requires: src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/__/PreflightCheck.cpp.o.requires

.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/requires

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests && $(CMAKE_COMMAND) -P CMakeFiles/modules__commander__commander_tests.dir/cmake_clean.cmake
.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/clean

src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/commander/commander_tests /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/commander/commander_tests/CMakeFiles/modules__commander__commander_tests.dir/depend

