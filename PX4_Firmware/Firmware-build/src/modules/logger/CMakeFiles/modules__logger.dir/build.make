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
include src/modules/logger/CMakeFiles/modules__logger.dir/depend.make

# Include the progress variables for this target.
include src/modules/logger/CMakeFiles/modules__logger.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/logger/CMakeFiles/modules__logger.dir/flags.make

src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o: src/modules/logger/CMakeFiles/modules__logger.dir/flags.make
src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/logger.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__logger.dir/logger.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/logger.cpp

src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__logger.dir/logger.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/logger.cpp > CMakeFiles/modules__logger.dir/logger.cpp.i

src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__logger.dir/logger.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/logger.cpp -o CMakeFiles/modules__logger.dir/logger.cpp.s

src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o.requires:

.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o.requires

src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o.provides: src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o.requires
	$(MAKE) -f src/modules/logger/CMakeFiles/modules__logger.dir/build.make src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o.provides.build
.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o.provides

src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o.provides.build: src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o


src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o: src/modules/logger/CMakeFiles/modules__logger.dir/flags.make
src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__logger.dir/log_writer.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer.cpp

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__logger.dir/log_writer.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer.cpp > CMakeFiles/modules__logger.dir/log_writer.cpp.i

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__logger.dir/log_writer.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer.cpp -o CMakeFiles/modules__logger.dir/log_writer.cpp.s

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o.requires:

.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o.requires

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o.provides: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o.requires
	$(MAKE) -f src/modules/logger/CMakeFiles/modules__logger.dir/build.make src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o.provides.build
.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o.provides

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o.provides.build: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o


src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o: src/modules/logger/CMakeFiles/modules__logger.dir/flags.make
src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer_file.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__logger.dir/log_writer_file.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer_file.cpp

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__logger.dir/log_writer_file.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer_file.cpp > CMakeFiles/modules__logger.dir/log_writer_file.cpp.i

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__logger.dir/log_writer_file.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer_file.cpp -o CMakeFiles/modules__logger.dir/log_writer_file.cpp.s

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o.requires:

.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o.requires

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o.provides: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o.requires
	$(MAKE) -f src/modules/logger/CMakeFiles/modules__logger.dir/build.make src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o.provides.build
.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o.provides

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o.provides.build: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o


src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o: src/modules/logger/CMakeFiles/modules__logger.dir/flags.make
src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer_mavlink.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer_mavlink.cpp

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer_mavlink.cpp > CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.i

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger/log_writer_mavlink.cpp -o CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.s

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o.requires:

.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o.requires

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o.provides: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o.requires
	$(MAKE) -f src/modules/logger/CMakeFiles/modules__logger.dir/build.make src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o.provides.build
.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o.provides

src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o.provides.build: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o


# Object files for target modules__logger
modules__logger_OBJECTS = \
"CMakeFiles/modules__logger.dir/logger.cpp.o" \
"CMakeFiles/modules__logger.dir/log_writer.cpp.o" \
"CMakeFiles/modules__logger.dir/log_writer_file.cpp.o" \
"CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o"

# External object files for target modules__logger
modules__logger_EXTERNAL_OBJECTS =

src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/build.make
src/modules/logger/libmodules__logger.a: src/modules/logger/CMakeFiles/modules__logger.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libmodules__logger.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && $(CMAKE_COMMAND) -P CMakeFiles/modules__logger.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__logger.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/logger/CMakeFiles/modules__logger.dir/build: src/modules/logger/libmodules__logger.a

.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/build

src/modules/logger/CMakeFiles/modules__logger.dir/requires: src/modules/logger/CMakeFiles/modules__logger.dir/logger.cpp.o.requires
src/modules/logger/CMakeFiles/modules__logger.dir/requires: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer.cpp.o.requires
src/modules/logger/CMakeFiles/modules__logger.dir/requires: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_file.cpp.o.requires
src/modules/logger/CMakeFiles/modules__logger.dir/requires: src/modules/logger/CMakeFiles/modules__logger.dir/log_writer_mavlink.cpp.o.requires

.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/requires

src/modules/logger/CMakeFiles/modules__logger.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger && $(CMAKE_COMMAND) -P CMakeFiles/modules__logger.dir/cmake_clean.cmake
.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/clean

src/modules/logger/CMakeFiles/modules__logger.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/modules/logger /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger /home/leonardo/src/PX4_Firmware/Firmware-build/src/modules/logger/CMakeFiles/modules__logger.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/logger/CMakeFiles/modules__logger.dir/depend

