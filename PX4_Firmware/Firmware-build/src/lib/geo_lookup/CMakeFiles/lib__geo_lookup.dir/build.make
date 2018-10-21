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
include src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/depend.make

# Include the progress variables for this target.
include src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/flags.make

src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o: src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/flags.make
src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o: /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/geo_lookup/geo_mag_declination.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/geo_lookup && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o   -c /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/geo_lookup/geo_mag_declination.c

src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/geo_lookup && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/geo_lookup/geo_mag_declination.c > CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.i

src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/geo_lookup && /usr/bin/gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/geo_lookup/geo_mag_declination.c -o CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.s

src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o.requires:

.PHONY : src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o.requires

src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o.provides: src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o.requires
	$(MAKE) -f src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/build.make src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o.provides.build
.PHONY : src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o.provides

src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o.provides.build: src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o


# Object files for target lib__geo_lookup
lib__geo_lookup_OBJECTS = \
"CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o"

# External object files for target lib__geo_lookup
lib__geo_lookup_EXTERNAL_OBJECTS =

src/lib/geo_lookup/liblib__geo_lookup.a: src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o
src/lib/geo_lookup/liblib__geo_lookup.a: src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/build.make
src/lib/geo_lookup/liblib__geo_lookup.a: src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library liblib__geo_lookup.a"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/geo_lookup && $(CMAKE_COMMAND) -P CMakeFiles/lib__geo_lookup.dir/cmake_clean_target.cmake
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/geo_lookup && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lib__geo_lookup.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/build: src/lib/geo_lookup/liblib__geo_lookup.a

.PHONY : src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/build

src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/requires: src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/geo_mag_declination.c.o.requires

.PHONY : src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/requires

src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/geo_lookup && $(CMAKE_COMMAND) -P CMakeFiles/lib__geo_lookup.dir/cmake_clean.cmake
.PHONY : src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/clean

src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/lib/geo_lookup /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/geo_lookup /home/leonardo/src/PX4_Firmware/Firmware-build/src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/geo_lookup/CMakeFiles/lib__geo_lookup.dir/depend
