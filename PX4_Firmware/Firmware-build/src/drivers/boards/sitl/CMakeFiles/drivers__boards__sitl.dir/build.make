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
include src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/depend.make

# Include the progress variables for this target.
include src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/flags.make

src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o: src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/flags.make
src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/boards/sitl/sitl_led.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/boards/sitl && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o   -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/boards/sitl/sitl_led.c

src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/boards/sitl && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/boards/sitl/sitl_led.c > CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.i

src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/boards/sitl && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/boards/sitl/sitl_led.c -o CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.s

src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o.requires:

.PHONY : src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o.requires

src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o.provides: src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o.requires
	$(MAKE) -f src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/build.make src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o.provides.build
.PHONY : src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o.provides

src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o.provides.build: src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o


# Object files for target drivers__boards__sitl
drivers__boards__sitl_OBJECTS = \
"CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o"

# External object files for target drivers__boards__sitl
drivers__boards__sitl_EXTERNAL_OBJECTS =

src/drivers/boards/sitl/libdrivers__boards__sitl.a: src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o
src/drivers/boards/sitl/libdrivers__boards__sitl.a: src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/build.make
src/drivers/boards/sitl/libdrivers__boards__sitl.a: src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libdrivers__boards__sitl.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/boards/sitl && $(CMAKE_COMMAND) -P CMakeFiles/drivers__boards__sitl.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/boards/sitl && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__boards__sitl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/build: src/drivers/boards/sitl/libdrivers__boards__sitl.a

.PHONY : src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/build

src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/requires: src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/sitl_led.c.o.requires

.PHONY : src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/requires

src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/boards/sitl && $(CMAKE_COMMAND) -P CMakeFiles/drivers__boards__sitl.dir/cmake_clean.cmake
.PHONY : src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/clean

src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/drivers/boards/sitl /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/boards/sitl /home/leonardo/src/PX4_Firmware/Firmware-build/src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/boards/sitl/CMakeFiles/drivers__boards__sitl.dir/depend

