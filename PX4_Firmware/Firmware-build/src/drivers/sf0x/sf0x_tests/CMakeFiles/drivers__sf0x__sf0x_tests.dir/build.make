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
include src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/depend.make

# Include the progress variables for this target.
include src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/flags.make

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/flags.make
src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/sf0x/sf0x_tests/SF0XTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/sf0x/sf0x_tests && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/sf0x/sf0x_tests/SF0XTest.cpp

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/sf0x/sf0x_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/sf0x/sf0x_tests/SF0XTest.cpp > CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.i

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/sf0x/sf0x_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/sf0x/sf0x_tests/SF0XTest.cpp -o CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.s

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o.requires:

.PHONY : src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o.requires

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o.provides: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o.requires
	$(MAKE) -f src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/build.make src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o.provides.build
.PHONY : src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o.provides

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o.provides.build: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o


src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/flags.make
src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/sf0x/sf0x_parser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/sf0x/sf0x_tests && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/sf0x/sf0x_parser.cpp

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/sf0x/sf0x_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/sf0x/sf0x_parser.cpp > CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.i

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/sf0x/sf0x_tests && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/sf0x/sf0x_parser.cpp -o CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.s

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o.requires:

.PHONY : src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o.requires

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o.provides: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o.requires
	$(MAKE) -f src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/build.make src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o.provides.build
.PHONY : src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o.provides

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o.provides.build: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o


# Object files for target drivers__sf0x__sf0x_tests
drivers__sf0x__sf0x_tests_OBJECTS = \
"CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o" \
"CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o"

# External object files for target drivers__sf0x__sf0x_tests
drivers__sf0x__sf0x_tests_EXTERNAL_OBJECTS =

src/drivers/sf0x/sf0x_tests/libdrivers__sf0x__sf0x_tests.a: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o
src/drivers/sf0x/sf0x_tests/libdrivers__sf0x__sf0x_tests.a: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o
src/drivers/sf0x/sf0x_tests/libdrivers__sf0x__sf0x_tests.a: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/build.make
src/drivers/sf0x/sf0x_tests/libdrivers__sf0x__sf0x_tests.a: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libdrivers__sf0x__sf0x_tests.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/sf0x/sf0x_tests && $(CMAKE_COMMAND) -P CMakeFiles/drivers__sf0x__sf0x_tests.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/sf0x/sf0x_tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__sf0x__sf0x_tests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/build: src/drivers/sf0x/sf0x_tests/libdrivers__sf0x__sf0x_tests.a

.PHONY : src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/build

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/requires: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/SF0XTest.cpp.o.requires
src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/requires: src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/__/sf0x_parser.cpp.o.requires

.PHONY : src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/requires

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/sf0x/sf0x_tests && $(CMAKE_COMMAND) -P CMakeFiles/drivers__sf0x__sf0x_tests.dir/cmake_clean.cmake
.PHONY : src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/clean

src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/sf0x/sf0x_tests /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/sf0x/sf0x_tests /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/sf0x/sf0x_tests/CMakeFiles/drivers__sf0x__sf0x_tests.dir/depend

