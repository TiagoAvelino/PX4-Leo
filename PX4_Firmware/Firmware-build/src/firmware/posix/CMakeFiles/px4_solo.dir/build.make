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
include src/firmware/posix/CMakeFiles/px4_solo.dir/depend.make

# Include the progress variables for this target.
include src/firmware/posix/CMakeFiles/px4_solo.dir/progress.make

# Include the compile flags for this target's objects.
include src/firmware/posix/CMakeFiles/px4_solo.dir/flags.make

src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o: src/firmware/posix/CMakeFiles/px4_solo.dir/flags.make
src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o: src/firmware/posix/sitl_runner_main_solo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o -c /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix/sitl_runner_main_solo.cpp

src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix/sitl_runner_main_solo.cpp > CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.i

src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix/sitl_runner_main_solo.cpp -o CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.s

src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o.requires:

.PHONY : src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o.requires

src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o.provides: src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o.requires
	$(MAKE) -f src/firmware/posix/CMakeFiles/px4_solo.dir/build.make src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o.provides.build
.PHONY : src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o.provides

src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o.provides.build: src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o


src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o: src/firmware/posix/CMakeFiles/px4_solo.dir/flags.make
src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o: src/firmware/posix/apps.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix && /usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/px4_solo.dir/apps.cpp.o -c /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix/apps.cpp

src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/px4_solo.dir/apps.cpp.i"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix/apps.cpp > CMakeFiles/px4_solo.dir/apps.cpp.i

src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/px4_solo.dir/apps.cpp.s"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix/apps.cpp -o CMakeFiles/px4_solo.dir/apps.cpp.s

src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o.requires:

.PHONY : src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o.requires

src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o.provides: src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o.requires
	$(MAKE) -f src/firmware/posix/CMakeFiles/px4_solo.dir/build.make src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o.provides.build
.PHONY : src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o.provides

src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o.provides.build: src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o


# Object files for target px4_solo
px4_solo_OBJECTS = \
"CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o" \
"CMakeFiles/px4_solo.dir/apps.cpp.o"

# External object files for target px4_solo
px4_solo_EXTERNAL_OBJECTS =

src/firmware/posix/px4_solo: src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o
src/firmware/posix/px4_solo: src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o
src/firmware/posix/px4_solo: src/firmware/posix/CMakeFiles/px4_solo.dir/build.make
src/firmware/posix/px4_solo: src/drivers/boards/sitl/libdrivers__boards__sitl.a
src/firmware/posix/px4_solo: src/drivers/camera_trigger/libdrivers__camera_trigger.a
src/firmware/posix/px4_solo: src/drivers/device/libdrivers__device.a
src/firmware/posix/px4_solo: src/drivers/gps/libdrivers__gps.a
src/firmware/posix/px4_solo: src/drivers/pwm_out_sim/libdrivers__pwm_out_sim.a
src/firmware/posix/px4_solo: src/drivers/vmount/libdrivers__vmount.a
src/firmware/posix/px4_solo: src/modules/sensors/libmodules__sensors.a
src/firmware/posix/px4_solo: src/platforms/posix/drivers/accelsim/libplatforms__posix__drivers__accelsim.a
src/firmware/posix/px4_solo: src/platforms/posix/drivers/adcsim/libplatforms__posix__drivers__adcsim.a
src/firmware/posix/px4_solo: src/platforms/posix/drivers/airspeedsim/libplatforms__posix__drivers__airspeedsim.a
src/firmware/posix/px4_solo: src/platforms/posix/drivers/barosim/libplatforms__posix__drivers__barosim.a
src/firmware/posix/px4_solo: src/platforms/posix/drivers/gpssim/libplatforms__posix__drivers__gpssim.a
src/firmware/posix/px4_solo: src/platforms/posix/drivers/gyrosim/libplatforms__posix__drivers__gyrosim.a
src/firmware/posix/px4_solo: src/platforms/posix/drivers/ledsim/libplatforms__posix__drivers__ledsim.a
src/firmware/posix/px4_solo: src/platforms/posix/drivers/tonealrmsim/libplatforms__posix__drivers__tonealrmsim.a
src/firmware/posix/px4_solo: src/systemcmds/esc_calib/libsystemcmds__esc_calib.a
src/firmware/posix/px4_solo: src/systemcmds/led_control/libsystemcmds__led_control.a
src/firmware/posix/px4_solo: src/systemcmds/mixer/libsystemcmds__mixer.a
src/firmware/posix/px4_solo: src/systemcmds/motor_ramp/libsystemcmds__motor_ramp.a
src/firmware/posix/px4_solo: src/systemcmds/param/libsystemcmds__param.a
src/firmware/posix/px4_solo: src/systemcmds/perf/libsystemcmds__perf.a
src/firmware/posix/px4_solo: src/systemcmds/pwm/libsystemcmds__pwm.a
src/firmware/posix/px4_solo: src/systemcmds/reboot/libsystemcmds__reboot.a
src/firmware/posix/px4_solo: src/systemcmds/sd_bench/libsystemcmds__sd_bench.a
src/firmware/posix/px4_solo: src/systemcmds/top/libsystemcmds__top.a
src/firmware/posix/px4_solo: src/systemcmds/topic_listener/libsystemcmds__topic_listener.a
src/firmware/posix/px4_solo: src/systemcmds/ver/libsystemcmds__ver.a
src/firmware/posix/px4_solo: src/drivers/sf0x/sf0x_tests/libdrivers__sf0x__sf0x_tests.a
src/firmware/posix/px4_solo: src/lib/rc/rc_tests/liblib__rc__rc_tests.a
src/firmware/posix/px4_solo: src/modules/commander/commander_tests/libmodules__commander__commander_tests.a
src/firmware/posix/px4_solo: src/modules/controllib_test/libmodules__controllib_test.a
src/firmware/posix/px4_solo: src/modules/mavlink/mavlink_tests/libmodules__mavlink__mavlink_tests.a
src/firmware/posix/px4_solo: src/modules/mc_pos_control/mc_pos_control_tests/libmodules__mc_pos_control__mc_pos_control_tests.a
src/firmware/posix/px4_solo: src/modules/unit_test/libmodules__unit_test.a
src/firmware/posix/px4_solo: src/modules/uORB/uORB_tests/libmodules__uORB__uORB_tests.a
src/firmware/posix/px4_solo: src/systemcmds/tests/libsystemcmds__tests.a
src/firmware/posix/px4_solo: src/modules/commander/libmodules__commander.a
src/firmware/posix/px4_solo: src/modules/events/libmodules__events.a
src/firmware/posix/px4_solo: src/modules/land_detector/libmodules__land_detector.a
src/firmware/posix/px4_solo: src/modules/load_mon/libmodules__load_mon.a
src/firmware/posix/px4_solo: src/modules/mavlink/libmodules__mavlink.a
src/firmware/posix/px4_solo: src/modules/navigator/libmodules__navigator.a
src/firmware/posix/px4_solo: src/modules/replay/libmodules__replay.a
src/firmware/posix/px4_solo: src/modules/simulator/libmodules__simulator.a
src/firmware/posix/px4_solo: src/modules/attitude_estimator_q/libmodules__attitude_estimator_q.a
src/firmware/posix/px4_solo: src/modules/ekf2/libmodules__ekf2.a
src/firmware/posix/px4_solo: src/modules/ekf2_replay/libmodules__ekf2_replay.a
src/firmware/posix/px4_solo: src/modules/local_position_estimator/libmodules__local_position_estimator.a
src/firmware/posix/px4_solo: src/modules/position_estimator_inav/libmodules__position_estimator_inav.a
src/firmware/posix/px4_solo: src/modules/fw_att_control/libmodules__fw_att_control.a
src/firmware/posix/px4_solo: src/modules/fw_pos_control_l1/libmodules__fw_pos_control_l1.a
src/firmware/posix/px4_solo: src/modules/mc_att_control/libmodules__mc_att_control.a
src/firmware/posix/px4_solo: src/modules/mc_pos_control/libmodules__mc_pos_control.a
src/firmware/posix/px4_solo: src/modules/vtol_att_control/libmodules__vtol_att_control.a
src/firmware/posix/px4_solo: src/modules/pid_att_control/libmodules__pid_att_control.a
src/firmware/posix/px4_solo: src/modules/lqr_att_control/libmodules__lqr_att_control.a
src/firmware/posix/px4_solo: src/modules/logger/libmodules__logger.a
src/firmware/posix/px4_solo: src/modules/sdlog2/libmodules__sdlog2.a
src/firmware/posix/px4_solo: src/modules/dataman/libmodules__dataman.a
src/firmware/posix/px4_solo: src/modules/param/libmodules__param.a
src/firmware/posix/px4_solo: src/modules/systemlib/libmodules__systemlib.a
src/firmware/posix/px4_solo: src/modules/systemlib/mixer/libmodules__systemlib__mixer.a
src/firmware/posix/px4_solo: src/modules/uORB/libmodules__uORB.a
src/firmware/posix/px4_solo: src/lib/controllib/liblib__controllib.a
src/firmware/posix/px4_solo: src/lib/conversion/liblib__conversion.a
src/firmware/posix/px4_solo: src/lib/DriverFramework/framework/liblib__DriverFramework__framework.a
src/firmware/posix/px4_solo: src/lib/ecl/liblib__ecl.a
src/firmware/posix/px4_solo: src/lib/external_lgpl/liblib__external_lgpl.a
src/firmware/posix/px4_solo: src/lib/geo/liblib__geo.a
src/firmware/posix/px4_solo: src/lib/geo_lookup/liblib__geo_lookup.a
src/firmware/posix/px4_solo: src/lib/launchdetection/liblib__launchdetection.a
src/firmware/posix/px4_solo: src/lib/led/liblib__led.a
src/firmware/posix/px4_solo: src/lib/mathlib/liblib__mathlib.a
src/firmware/posix/px4_solo: src/lib/mathlib/math/filter/liblib__mathlib__math__filter.a
src/firmware/posix/px4_solo: src/lib/rc/liblib__rc.a
src/firmware/posix/px4_solo: src/lib/runway_takeoff/liblib__runway_takeoff.a
src/firmware/posix/px4_solo: src/lib/tailsitter_recovery/liblib__tailsitter_recovery.a
src/firmware/posix/px4_solo: src/lib/terrain_estimation/liblib__terrain_estimation.a
src/firmware/posix/px4_solo: src/lib/version/liblib__version.a
src/firmware/posix/px4_solo: src/platforms/common/libplatforms__common.a
src/firmware/posix/px4_solo: src/platforms/posix/px4_layer/libplatforms__posix__px4_layer.a
src/firmware/posix/px4_solo: src/platforms/posix/work_queue/libplatforms__posix__work_queue.a
src/firmware/posix/px4_solo: src/modules/bottle_drop/libmodules__bottle_drop.a
src/firmware/posix/px4_solo: src/examples/rover_steering_control/libexamples__rover_steering_control.a
src/firmware/posix/px4_solo: src/examples/segway/libexamples__segway.a
src/firmware/posix/px4_solo: src/examples/px4_simple_app/libexamples__px4_simple_app.a
src/firmware/posix/px4_solo: src/examples/px4_daemon_app/libexamples__px4_daemon_app.a
src/firmware/posix/px4_solo: src/examples/px4_mavlink_debug/libexamples__px4_mavlink_debug.a
src/firmware/posix/px4_solo: src/examples/fixedwing_control/libexamples__fixedwing_control.a
src/firmware/posix/px4_solo: src/examples/ekf_att_pos_estimator/libexamples__ekf_att_pos_estimator.a
src/firmware/posix/px4_solo: libmsg_gen.a
src/firmware/posix/px4_solo: src/lib/DriverFramework/framework/src/libdf_driver_framework.a
src/firmware/posix/px4_solo: src/firmware/posix/CMakeFiles/px4_solo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leonardo/src/PX4_Firmware/Firmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable px4_solo"
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/px4_solo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/firmware/posix/CMakeFiles/px4_solo.dir/build: src/firmware/posix/px4_solo

.PHONY : src/firmware/posix/CMakeFiles/px4_solo.dir/build

src/firmware/posix/CMakeFiles/px4_solo.dir/requires: src/firmware/posix/CMakeFiles/px4_solo.dir/sitl_runner_main_solo.cpp.o.requires
src/firmware/posix/CMakeFiles/px4_solo.dir/requires: src/firmware/posix/CMakeFiles/px4_solo.dir/apps.cpp.o.requires

.PHONY : src/firmware/posix/CMakeFiles/px4_solo.dir/requires

src/firmware/posix/CMakeFiles/px4_solo.dir/clean:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix && $(CMAKE_COMMAND) -P CMakeFiles/px4_solo.dir/cmake_clean.cmake
.PHONY : src/firmware/posix/CMakeFiles/px4_solo.dir/clean

src/firmware/posix/CMakeFiles/px4_solo.dir/depend:
	cd /home/leonardo/src/PX4_Firmware/Firmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leonardo/src/PX4_Firmware/PX4_Firmware /home/leonardo/src/PX4_Firmware/PX4_Firmware/src/firmware/posix /home/leonardo/src/PX4_Firmware/Firmware-build /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix /home/leonardo/src/PX4_Firmware/Firmware-build/src/firmware/posix/CMakeFiles/px4_solo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/firmware/posix/CMakeFiles/px4_solo.dir/depend

