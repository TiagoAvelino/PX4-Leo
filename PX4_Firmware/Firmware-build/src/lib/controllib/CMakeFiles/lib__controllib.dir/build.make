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
include src/lib/controllib/CMakeFiles/lib__controllib.dir/depend.make

# Include the progress variables for this target.
include src/lib/controllib/CMakeFiles/lib__controllib.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/controllib/CMakeFiles/lib__controllib.dir/flags.make

src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o: src/lib/controllib/CMakeFiles/lib__controllib.dir/flags.make
src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/blocks.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__controllib.dir/blocks.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/blocks.cpp

src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__controllib.dir/blocks.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/blocks.cpp > CMakeFiles/lib__controllib.dir/blocks.cpp.i

src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__controllib.dir/blocks.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/blocks.cpp -o CMakeFiles/lib__controllib.dir/blocks.cpp.s

src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o.requires:

.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o.requires

src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o.provides: src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o.requires
	$(MAKE) -f src/lib/controllib/CMakeFiles/lib__controllib.dir/build.make src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o.provides.build
.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o.provides

src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o.provides.build: src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o


src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o: src/lib/controllib/CMakeFiles/lib__controllib.dir/flags.make
src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/block/Block.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__controllib.dir/block/Block.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/block/Block.cpp

src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__controllib.dir/block/Block.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/block/Block.cpp > CMakeFiles/lib__controllib.dir/block/Block.cpp.i

src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__controllib.dir/block/Block.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/block/Block.cpp -o CMakeFiles/lib__controllib.dir/block/Block.cpp.s

src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o.requires:

.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o.requires

src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o.provides: src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o.requires
	$(MAKE) -f src/lib/controllib/CMakeFiles/lib__controllib.dir/build.make src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o.provides.build
.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o.provides

src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o.provides.build: src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o


src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o: src/lib/controllib/CMakeFiles/lib__controllib.dir/flags.make
src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/block/BlockParam.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/block/BlockParam.cpp

src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/block/BlockParam.cpp > CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.i

src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/block/BlockParam.cpp -o CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.s

src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o.requires:

.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o.requires

src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o.provides: src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o.requires
	$(MAKE) -f src/lib/controllib/CMakeFiles/lib__controllib.dir/build.make src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o.provides.build
.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o.provides

src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o.provides.build: src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o


src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o: src/lib/controllib/CMakeFiles/lib__controllib.dir/flags.make
src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/uorb/blocks.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/uorb/blocks.cpp

src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/uorb/blocks.cpp > CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.i

src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib/uorb/blocks.cpp -o CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.s

src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o.requires:

.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o.requires

src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o.provides: src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o.requires
	$(MAKE) -f src/lib/controllib/CMakeFiles/lib__controllib.dir/build.make src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o.provides.build
.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o.provides

src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o.provides.build: src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o


# Object files for target lib__controllib
lib__controllib_OBJECTS = \
"CMakeFiles/lib__controllib.dir/blocks.cpp.o" \
"CMakeFiles/lib__controllib.dir/block/Block.cpp.o" \
"CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o" \
"CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o"

# External object files for target lib__controllib
lib__controllib_EXTERNAL_OBJECTS =

src/lib/controllib/liblib__controllib.a: src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o
src/lib/controllib/liblib__controllib.a: src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o
src/lib/controllib/liblib__controllib.a: src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o
src/lib/controllib/liblib__controllib.a: src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o
src/lib/controllib/liblib__controllib.a: src/lib/controllib/CMakeFiles/lib__controllib.dir/build.make
src/lib/controllib/liblib__controllib.a: src/lib/controllib/CMakeFiles/lib__controllib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library liblib__controllib.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && $(CMAKE_COMMAND) -P CMakeFiles/lib__controllib.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib__controllib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/controllib/CMakeFiles/lib__controllib.dir/build: src/lib/controllib/liblib__controllib.a

.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/build

src/lib/controllib/CMakeFiles/lib__controllib.dir/requires: src/lib/controllib/CMakeFiles/lib__controllib.dir/blocks.cpp.o.requires
src/lib/controllib/CMakeFiles/lib__controllib.dir/requires: src/lib/controllib/CMakeFiles/lib__controllib.dir/block/Block.cpp.o.requires
src/lib/controllib/CMakeFiles/lib__controllib.dir/requires: src/lib/controllib/CMakeFiles/lib__controllib.dir/block/BlockParam.cpp.o.requires
src/lib/controllib/CMakeFiles/lib__controllib.dir/requires: src/lib/controllib/CMakeFiles/lib__controllib.dir/uorb/blocks.cpp.o.requires

.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/requires

src/lib/controllib/CMakeFiles/lib__controllib.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib && $(CMAKE_COMMAND) -P CMakeFiles/lib__controllib.dir/cmake_clean.cmake
.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/clean

src/lib/controllib/CMakeFiles/lib__controllib.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/controllib /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/controllib/CMakeFiles/lib__controllib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/controllib/CMakeFiles/lib__controllib.dir/depend

