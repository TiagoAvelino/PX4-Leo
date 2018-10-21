/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file /home/leonardo/src/PX4_Firmware/PX4_Firmware/msg/vehicle_local_position.msg */


#include <px4_config.h>
#include <drivers/drv_orb_dev.h>
#include <uORB/topics/vehicle_local_position.h>


const char *__orb_vehicle_local_position_fields = "uint64_t timestamp;uint64_t ref_timestamp;double ref_lat;double ref_lon;uint64_t surface_bottom_timestamp;float x;float y;float z;float[2] delta_xy;float delta_z;float vx;float vy;float vz;float[2] delta_vxy;float delta_vz;float ax;float ay;float az;float yaw;float ref_alt;float dist_bottom;float dist_bottom_rate;float eph;float epv;uint8_t estimator_type;bool xy_valid;bool z_valid;bool v_xy_valid;bool v_z_valid;uint8_t xy_reset_counter;uint8_t z_reset_counter;uint8_t vxy_reset_counter;uint8_t vz_reset_counter;bool xy_global;bool z_global;bool dist_bottom_valid;";

ORB_DEFINE(vehicle_local_position, struct vehicle_local_position_s, 136,
    __orb_vehicle_local_position_fields);
ORB_DEFINE(vehicle_local_position_groundtruth, struct vehicle_local_position_s, 136,
    __orb_vehicle_local_position_fields);
ORB_DEFINE(vehicle_vision_position, struct vehicle_local_position_s, 136,
    __orb_vehicle_local_position_fields);

