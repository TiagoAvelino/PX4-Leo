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
include src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/depend.make

# Include the progress variables for this target.
include src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/flags.make

src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o: src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/flags.make
src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/DriverFramework/framework/px4/df_dummy.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/DriverFramework/framework && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/DriverFramework/framework/px4/df_dummy.cpp

src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/DriverFramework/framework && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/DriverFramework/framework/px4/df_dummy.cpp > CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.i

src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/DriverFramework/framework && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/DriverFramework/framework/px4/df_dummy.cpp -o CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.s

src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o.requires:

.PHONY : src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o.requires

src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o.provides: src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o.requires
	$(MAKE) -f src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/build.make src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o.provides.build
.PHONY : src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o.provides

src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o.provides.build: src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o


# Object files for target lib__DriverFramework__framework
lib__DriverFramework__framework_OBJECTS = \
"CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o"

# External object files for target lib__DriverFramework__framework
lib__DriverFramework__framework_EXTERNAL_OBJECTS =

src/lib/DriverFramework/framework/liblib__DriverFramework__framework.a: src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o
src/lib/DriverFramework/framework/liblib__DriverFramework__framework.a: src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/build.make
src/lib/DriverFramework/framework/liblib__DriverFramework__framework.a: src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library liblib__DriverFramework__framework.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/DriverFramework/framework && $(CMAKE_COMMAND) -P CMakeFiles/lib__DriverFramework__framework.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/DriverFramework/framework && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib__DriverFramework__framework.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/build: src/lib/DriverFramework/framework/liblib__DriverFramework__framework.a

.PHONY : src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/build

src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/requires: src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/px4/df_dummy.cpp.o.requires

.PHONY : src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/requires

src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/DriverFramework/framework && $(CMAKE_COMMAND) -P CMakeFiles/lib__DriverFramework__framework.dir/cmake_clean.cmake
.PHONY : src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/clean

src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/DriverFramework/framework /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/DriverFramework/framework /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/DriverFramework/framework/CMakeFiles/lib__DriverFramework__framework.dir/depend

