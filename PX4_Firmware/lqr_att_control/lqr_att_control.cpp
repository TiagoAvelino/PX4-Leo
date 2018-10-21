/**
 * @file lqr_att_control.cpp
 *
 * LQR Attitude Control
 *
 *
 * Created by Leonardo Avelino
 *
 **/

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <lib/mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <systemlib/perf_counter.h>
#include <lib/geo/geo.h>
#include <pid_att_control/data.h>
#include "aux_func2.cpp"


/* Definitions ******************/

#define MAX_TORQUE              30.0f

#define PI                      3.1415f
#define DEG_TO_RAD(x)           (x * PI / 180.0f)

#define KP_ROLL                 3.5f                    /* 1/s       */ // 7.0 for tests
#define MAX_RATE_ROLL           DEG_TO_RAD(220.0f)      /* degree/s  */
#define KP_PITCH                3.5f                    /* 1/s       */ // 7.0 for tests
#define MAX_RATE_PITCH          DEG_TO_RAD(220.0f)      /* degree/s  */
#define KP_YAW                  2.8f                    /* 1/s       */
#define MAX_RATE_YAW            DEG_TO_RAD(90.0f)       /* degree/s  */ // increase for tests

#define KD_VEL_Z                0.8f

#define AXIS_INDEX_ROLL         0
#define AXIS_INDEX_PITCH        1
#define AXIS_INDEX_YAW          2

/********************************/

extern "C" __EXPORT int lqr_att_control_main(int argc, char *argv[]);

int loopcount = 0;
bool debug_status = false;

float _att_thrust, _thrust_sp;

class Lqr_Att_Control{
public:

    /**
     * Constructor
     */
    Lqr_Att_Control();

    /**
     * Destructor, also kills the main task
     */
    ~Lqr_Att_Control();

    /**
     * Start the Lqr Att control task
     * @return OK on success
     */
    int start();

private:

    bool _task_should_exit;                                         // if true, task_main() should exit;
    int _control_task;                                              // task handle
    float _dt;
    int _v_attitude_sp_sub;                                         // vehicle attitude setpoint subscription
    int _v_attitude_sub;                                            // vehicle attitude subscription
    int _motor_limits_sub;		                                    // motor limits subscription
    int _v_control_mode_sub;	                                    // vehicle control mode subscription
    int _armed_sub;                                                 // actuator armed subscription
    int _actuatorscontrol;
    int _v_rates_sp_sub;
    int _vehicle_position_sub;
    int _vehicle_position_sp_sub;
    int	_pos_sp_triplet_sub; //inicializar

    orb_advert_t	_actuators_0_pub;                               // attitude actuator controls publication
    orb_advert_t	_v_rates_sp_pub;                                // rate setpoint publication

    struct vehicle_attitude_setpoint_s	_v_attitude_sp;             // vehicle attitude setpoint
    struct vehicle_attitude_s _attitude;                            // vehicle attitude
    struct multirotor_motor_limits_s	_motor_limits;		        // motor limits
    struct vehicle_control_mode_s _v_control_mode;                  // vehicle control mode
    struct actuator_controls_s _actuators;			                // actuator controls
    struct actuator_armed_s _armed;                                 // arming status
    struct vehicle_rates_setpoint_s _v_rates_sp;
    struct vehicle_local_position_s _v_position;
    struct vehicle_local_position_setpoint_s _v_position_sp;
    struct v_angles _angles;                                        // current roll, pitch, yaw
    struct v_angles _angles_sp;                                     // current angles setpoint
    struct v_angles _att_control, _att_prev;                        // torques for control
    struct position_setpoint_triplet_s	_pos_sp_triplet;  //inicializar

    union {
        struct {
            uint16_t motor_pos	: 1; // 0 - true when any motor has saturated in the positive direction
            uint16_t motor_neg	: 1; // 1 - true when any motor has saturated in the negative direction
            uint16_t roll_pos	: 1; // 2 - true when a positive roll demand change will increase saturation
            uint16_t roll_neg	: 1; // 3 - true when a negative roll demand change will increase saturation
            uint16_t pitch_pos	: 1; // 4 - true when a positive pitch demand change will increase saturation
            uint16_t pitch_neg	: 1; // 5 - true when a negative pitch demand change will increase saturation
            uint16_t yaw_pos	: 1; // 6 - true when a positive yaw demand change will increase saturation
            uint16_t yaw_neg	: 1; // 7 - true when a negative yaw demand change will increase saturation
            uint16_t thrust_pos	: 1; // 8 - true when a positive thrust demand change will increase saturation
            uint16_t thrust_neg	: 1; // 9 - true when a negative thrust demand change will increase saturation
        } flags;
        uint16_t value;
    } _saturation_status;


    perf_counter_t	_loop_perf;			/** loop performance counter */

    math::Matrix <4, 8> Kn;
    math::Vector <8> x_s;
    math::Vector <8> y_r;
    /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     * Convert Attitude quaternion to euler angles
     */
    void att_quaternion_to_euler(const float *q, struct v_angles *angles);

    /**
     *  Check the arming status of the system
     */
    void arming_status_poll();

    /**
     * Poll angle setpoint from the position controller algorithm (or manual control)
     */
    void vehicle_attitude_setpoint_poll();

    /**
      * Poll vehicle attitude (roll, pitch, yaw)
      */
    void vehicle_attitude_poll();

    /**
      * Poll vehicle motor limits
      */
    void vehicle_motor_limits_poll();

    /**
      * Check for changes in vehicle control mode
      */
    void vehicle_control_mode_poll();

    /**
      * Poll vehicle rates setpoint
      */
    void vehicle_rates_setpoint_poll();

    /**
     * Linear mapping
     * Map torque value in fromLow - fromHigh range to toLow - toHigh range
     * ex: input -> value = 1, fromLow = 0, fromHigh = 10, toLow = 0, toHigh = 1
     *     output -> 0.1
     */
    float map(float value, float fromLow, float fromHigh, float toLow, float toHigh);

    /**
      * Poll actuator control topic in order to know what other apps are publishing as commands
      */
    void actuator_control_poll();

    /**
      * Publish to rate setpoint topic
      */
    void publish_rates_setpoint();

    /**
      * Build setpoint vector
      */
    void construct_matrix_yr();

    /**
      * Attitude controller;
      */
    void control_attitude(float dt);

    /**
      *
      */
    void vehicle_position_poll();

    /**
      *
      */
    void vehicle_position_setpoint_poll();

    /**
      *
      */
    void position_setpoint_triplet_poll();

    /**
     * Main attitude control task.
     */
    void task_main();
};

namespace lqr_att_control
{
    Lqr_Att_Control *g_control;
}

Lqr_Att_Control::Lqr_Att_Control() :

   _task_should_exit(false),
   _control_task(-1),
   _dt(0.0f),
   _v_attitude_sp_sub(-1),
   _v_attitude_sub(-1),
   _motor_limits_sub(-1),
   _v_control_mode_sub(-1),
   _armed_sub(-1),
   _actuatorscontrol(-1),
   _v_rates_sp_sub(-1),
   _vehicle_position_sub(-1),
   _vehicle_position_sp_sub(-1),
   _actuators_0_pub(nullptr),
   _v_rates_sp_pub(nullptr),
   _v_attitude_sp{},
   _attitude{},
   _motor_limits{},
   _v_control_mode{},
   _actuators{},
   _armed{},
   _v_rates_sp{},
   _v_position{},
   _v_position_sp{},
   _angles{},
   _angles_sp{},
   _att_control{},
   _att_prev{},
   _saturation_status{},
   _loop_perf(perf_alloc(PC_ELAPSED, "lqr_att_control"))

{
    /* build K matrix manually :( */
//    Kn(0,0) = 0.9973;
//    Kn(0,1) = 1.9967;
//    Kn(0,2) = 0.0;
//    Kn(0,3) = 0.0;
//    Kn(0,4) = 0.0;
//    Kn(0,5) = 0.0;
//    Kn(0,6) = 0.0;
//    Kn(0,7) = 0.0;
//    Kn(1,0) = 0.0;
//    Kn(1,1) = 0.0;
//    Kn(1,2) = 0.9206;
//    Kn(1,3) = 1.3263;
//    Kn(1,4) = 0.0;
//    Kn(1,5) = 0.0;
//    Kn(1,6) = 0.0;
//    Kn(1,7) = 0.0;
//    Kn(2,0) = 0.0;
//    Kn(2,1) = 0.0;
//    Kn(2,2) = 0.0;
//    Kn(2,3) = 0.0;
//    Kn(2,4) = 0.9390;
//    Kn(2,5) = 1.3599;
//    Kn(2,6) = 0.0;
//    Kn(2,7) = 0.0;
//    Kn(3,0) = 0.0;
//    Kn(3,1) = 0.0;
//    Kn(3,2) = 0.0;
//    Kn(3,3) = 0.0;
//    Kn(3,4) = 0.0;
//    Kn(3,5) = 0.0;
//    Kn(3,6) = 0.9701;
//    Kn(3,7) = 1.4394;
    mount_k_n(Kn);
}

Lqr_Att_Control::~Lqr_Att_Control()
{
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                px4_task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }

    lqr_att_control::g_control = nullptr;
}

float Lqr_Att_Control::map(float value, float fromLow, float fromHigh, float toLow, float toHigh)
{
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void Lqr_Att_Control::arming_status_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_armed_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
    }
}

void Lqr_Att_Control::vehicle_rates_setpoint_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_v_rates_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
    }
}

void Lqr_Att_Control::actuator_control_poll()
{
    bool updated;
    orb_check(_actuatorscontrol, &updated);

    if (updated) {
        orb_copy(ORB_ID(actuator_controls_0), _actuatorscontrol, &_actuators);
    }
}

void Lqr_Att_Control::vehicle_attitude_setpoint_poll()
{
    /* check if there is a new setpoint */
    bool updated;
    orb_check(_v_attitude_sp_sub, &updated);

    if(updated) {
        orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_attitude_sp_sub, &_v_attitude_sp);
    }        
    _angles_sp.roll = _v_attitude_sp.roll_body;
    _angles_sp.pitch = _v_attitude_sp.pitch_body;
    _angles_sp.yaw = _v_attitude_sp.yaw_body;
    _thrust_sp = _v_attitude_sp.thrust;
}

void Lqr_Att_Control::vehicle_attitude_poll()
{
    /* check if there is a new attitude */
    bool updated;
    orb_check(_v_attitude_sub, &updated);

    if(updated) {
        orb_copy(ORB_ID(vehicle_attitude), _v_attitude_sub, &_attitude);
    }

    att_quaternion_to_euler(_attitude.q, &_angles);
    x_s(2) = _angles.roll;
    x_s(3) = _attitude.rollspeed;
    x_s(4) = _angles.pitch;
    x_s(5) = _attitude.pitchspeed;
    x_s(6) = _angles.yaw;
    x_s(7) = _attitude.yawspeed;
}

void Lqr_Att_Control::vehicle_motor_limits_poll()
{
    /* check if there is a new message */
    bool updated;
    orb_check(_motor_limits_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
        _saturation_status.value = _motor_limits.saturation_status;
    }
}

void Lqr_Att_Control::vehicle_position_poll()
{
    /* check if there is a new message */
    bool updated;
    orb_check(_vehicle_position_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_local_position), _vehicle_position_sub, &_v_position);
    }
    x_s(0) = _v_position.z;
    x_s(1) = _v_position.vz;
}

void Lqr_Att_Control::vehicle_position_setpoint_poll()
{
    /* check if there is a new message */
    bool updated;
    orb_check(_vehicle_position_sp_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_local_position_setpoint), _vehicle_position_sp_sub, &_v_position_sp);
    }

    /*Adding derivative control to velocity position sp*/
    /*Delete if you don't need it*/
//    static float v_position_err_z_prev = 0.0f;
//    _v_position_sp.vz += KD_VEL_Z * (((_v_position_sp.z - _v_position.z) - v_position_err_z_prev) / _dt);
//    v_position_err_z_prev = _v_position_sp.z - _v_position.z;
    /*  End     */

    y_r(0) = _v_position_sp.z;
    y_r(1) = _v_position_sp.vz;
    
}

void Lqr_Att_Control::position_setpoint_triplet_poll()
{
    bool updated;
    orb_check(_pos_sp_triplet_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
        //Make sure that the position setpoint is valid
        // if (!PX4_ISFINITE(_pos_sp_triplet.current.lat) ||
        //     !PX4_ISFINITE(_pos_sp_triplet.current.lon) ||
        //     !PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
        //     _pos_sp_triplet.current.valid = false;
        // }
    }
}

void Lqr_Att_Control::vehicle_control_mode_poll()
{
    bool updated;

    /* Check if vehicle control mode has changed */
    orb_check(_v_control_mode_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
    }
}

void Lqr_Att_Control::publish_rates_setpoint()
{
    /* publish attitude rates setpoint */
    _v_rates_sp.roll = y_r(3);
    _v_rates_sp.pitch = y_r(5);
    _v_rates_sp.yaw = y_r(7);
    _v_rates_sp.timestamp = hrt_absolute_time();

    if (_v_rates_sp_pub != nullptr) {
        orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

    } else {
        _v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
    }
}

void Lqr_Att_Control::att_quaternion_to_euler(const float *q, struct v_angles *angles)
{
    matrix::Eulerf att_euler = matrix::Quatf(q);
    angles->roll = att_euler(0);
    angles->pitch = att_euler(1);
    angles->yaw = att_euler(2);
}

void Lqr_Att_Control::construct_matrix_yr()
{  
    /* Use rates sp generated by mc_att_control app */
//        y_r(0) = _angles_sp.roll;
//        y_r(1) = _v_rates_sp.roll;
//        y_r(2) = _angles_sp.pitch;
//        y_r(3) = _v_rates_sp.pitch;
//        y_r(4) = _angles_sp.yaw;
//        y_r(5) = _v_rates_sp.yaw;

    /* Build sp matrix */
    /* Rates setpoints will be produced by a P controller */
    y_r(2) = _angles_sp.roll;
    y_r(3) = math::constrain((_angles_sp.roll - _angles.roll) * KP_ROLL, -MAX_RATE_ROLL, MAX_RATE_ROLL);
    y_r(4) = _angles_sp.pitch;
    y_r(5) = math::constrain((_angles_sp.pitch - _angles.pitch) * KP_PITCH, -MAX_RATE_PITCH, MAX_RATE_PITCH);
    y_r(6) = _angles_sp.yaw;

    float error_yaw = _wrap_pi(_angles_sp.yaw - _angles.yaw); // make sure yaw error is: pi >= yaw error >= -pi

    y_r(7) = math::constrain(error_yaw * KP_YAW, -MAX_RATE_YAW, MAX_RATE_YAW);

}

void Lqr_Att_Control::control_attitude(float dt)
{
    /* poll atittude setpoint data */
    vehicle_attitude_setpoint_poll();
    /* poll rates setpoint data */
    vehicle_rates_setpoint_poll();
    /* check if there is an update in the attitude topic, and convert data from quaternion to euler */
    vehicle_attitude_poll();

    construct_matrix_yr();

    if (!_armed.armed) {
        _att_prev.roll = 0.0f;
        _att_prev.pitch = 0.0f;
        _att_prev.yaw = 0.0f;
    }

    math::Vector <8> Er = y_r-x_s;

    Er(6) = _wrap_pi(Er(6)); // make sure yaw error is: pi >= yaw error >= -pi


    /**/
    math::Vector <4> P_1 = Kn * Er;
    _att_control.roll = map((P_1(1) + _att_prev.roll), -MAX_TORQUE, MAX_TORQUE, -1.0f, 1.0f);
    _att_control.pitch = map((P_1(2) + _att_prev.pitch), -MAX_TORQUE, MAX_TORQUE, -1.0f, 1.0f);
    _att_control.yaw = map((P_1(3) + _att_prev.yaw), -MAX_TORQUE, MAX_TORQUE, -1.0f, 1.0f);


    float thrust_z = -1 * (P_1(0) - 0.5f);

//    if (_pos_sp_triplet.current.valid
//      && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
//      && _armed.armed)
//    {
//      _att_thrust = _v_attitude_sp.thrust;
//    }
//    else
//    {
        //PX4_INFO("thrust z %f",(double)thrust_z);

        if (fabsf(thrust_z) > 0.9f)
        {
            thrust_z = 0.9f;
        }

         _att_thrust = math::max(thrust_z,0.12f);
//    }

//    else{
//        if (-P_1(0) > 0.9f)
//        {
//            P_1(0) = -0.9f;
//        }
//        else if (-P_1(0) < 0.5f)
//        {
//            P_1(0) = -0.5f;
//        }
//        _att_thrust = -P_1(0);
//    }
//    PX4_INFO("X: %f",(double)_v_position.x);
//    PX4_INFO("X sp: %f",(double)_v_position_sp.x);
//    PX4_INFO("Y: %f",(double)_v_position.y);
//    PX4_INFO("Y sp: %f",(double)_v_position_sp.y);
    //_att_prev.roll = 0;//math::constrain(_att_control.roll, -0.3f,0.3f);
    //_att_prev.pitch = 0;//math::constrain(_att_control.pitch,-0.3f,0.3f);
    //_att_prev.yaw = 0;//math::constrain(_att_control.yaw,-0.3f,0.3f);

    
    /** DEBUG START */
    if (debug_status)
    {
        if (loopcount == 100)
        {
            PX4_INFO("-----ROLL------------------");
            PX4_INFO("Roll current angle: %f:",(double)(x_s(2)*180.0f/PI));
            PX4_INFO("Roll setpoint angle: %f:",(double)(y_r(2)*180.0f/PI));
            PX4_INFO("Roll angle ERROR: %f:",(double)((Er(2))*180.0f/PI));
            PX4_INFO("Roll current rate: %f:",(double)(x_s(3)*180.0f/PI));
            PX4_INFO("Roll setpoint rate: %f:",(double)(y_r(3)*180.0f/PI));
            PX4_INFO("Roll rate ERROR: %f:",(double)((Er(3))*180.0f/PI));
            PX4_INFO("Roll act control (torque): %f",(double)_att_control.roll);

            PX4_INFO("-----PITCH------------------");
            PX4_INFO("Pitch current angle: %f:",(double)(x_s(4)*180.0f/PI));
            PX4_INFO("Pitch setpoint angle: %f:",(double)(y_r(4)*180.0f/PI));
            PX4_INFO("Pitch angle ERROR: %f:",(double)((Er(4))*180.0f/PI));
            PX4_INFO("Pitch current rate: %f:",(double)(x_s(5)*180.0f/PI));
            PX4_INFO("Pitch setpoint rate: %f:",(double)(y_r(5)*180.0f/PI));
            PX4_INFO("Pitch rate ERROR: %f:",(double)((Er(5))*180.0f/PI));
            PX4_INFO("Pitch act control (torque): %f",(double)_att_control.pitch);

            PX4_INFO("-----YAW-------------------");
            PX4_INFO("Yaw current angle: %f:",(double)(x_s(6)*180.0f/PI));
            PX4_INFO("Yaw setpoint angle: %f:",(double)(y_r(6)*180.0f/PI));
            PX4_INFO("Yaw angle ERROR: %f:",(double)((Er(6))*180.0f/PI));
            PX4_INFO("Yaw current rate: %f:",(double)(x_s(7)*180.0f/PI));
            PX4_INFO("Yaw setpoint rate: %f:",(double)(y_r(7)*180.0f/PI));
            PX4_INFO("Yaw rate ERROR: %f:",(double)((Er(7))*180.0f/PI));
            PX4_INFO("Yaw act control (torque): %f",(double)_att_control.yaw);

            PX4_INFO("-----THRUST-------------------");
            PX4_INFO("Position Z: %f", (double)_v_position.z);
            PX4_INFO("SP Position Z: %f", (double)_v_position_sp.z);
            PX4_INFO("Error position Z: %f m", (double)Er(0));
            PX4_INFO("Velocity Z: %f", (double)_v_position.vz);
            PX4_INFO("SP Velocity Z: %f", (double)_v_position_sp.vz);
            PX4_INFO("Error velocity Z: %f m", (double)Er(1));
            PX4_INFO("Thrust before%f",(double)P_1(0));
            PX4_INFO("Thrust after %f",(double)_att_thrust);

            loopcount = 0;
        }
        else loopcount++;
    }
    /**    DEBUG END     */
}

int Lqr_Att_Control::start()
{
    ASSERT(_control_task == -1);
    /* start the task */
    _control_task = px4_task_spawn_cmd("lqr_att_control",
                                       SCHED_DEFAULT,
                                       SCHED_PRIORITY_MAX, // SCHED_PRIORITY_MAX-5
                                       2048,
                                       (px4_main_t)&task_main_trampoline,
                                       nullptr);
    if (_control_task < 0){
        warn("Task start failed");
        return -errno;
    }
    return OK;
}

void Lqr_Att_Control::task_main_trampoline(int argc, char *argv[])
{
   lqr_att_control::g_control->task_main();
}

void Lqr_Att_Control::task_main()
{
    /* Do subscriptions */
    _v_attitude_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    _v_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
    _v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    _armed_sub = orb_subscribe(ORB_ID(actuator_armed));
    _actuatorscontrol = orb_subscribe(ORB_ID(actuator_controls_0));
    _v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
    _vehicle_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _vehicle_position_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
    _pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

    px4_pollfd_struct_t poll_fds = {};
    poll_fds.events = POLLIN;

    while(!_task_should_exit)
    {
        poll_fds.fd = _v_attitude_sub;

        // wait for up to 50ms for data
        int pret = px4_poll(&poll_fds, 1, 100);

        // timed out - periodid check for _task_should_exit
        if (pret == 0) {
            continue;
        }

        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
            warn("lqr att ctrl: poll error %d, %d", pret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;
        }

        perf_begin(_loop_perf); //loop performance begin

        if (poll_fds.revents & POLLIN) {
            static uint64_t last_run = 0;
            float dt = (hrt_absolute_time() - last_run) / 1000000.0f;

            last_run = hrt_absolute_time();
            /* guard against too small (< 2ms) and too large (> 20ms) dt's */
            if (dt < 0.002f) {
                dt = 0.002f;
            }
            else if (dt > 0.02f) {
                dt = 0.02f;
            }
            _dt = dt; // saving dt as a global variable

            orb_copy(ORB_ID(vehicle_attitude), _v_attitude_sub, &_attitude);

            /* Poll important data */
            arming_status_poll();
            vehicle_motor_limits_poll();
            vehicle_control_mode_poll();
            vehicle_position_poll();
            vehicle_position_setpoint_poll();
            position_setpoint_triplet_poll();
            if ((_v_control_mode.flag_control_attitude_enabled) && (_v_control_mode.flag_control_rates_enabled))
            {
                control_attitude(dt);

                //actuator_control_poll(); /*DEBUG*/
                publish_rates_setpoint();

                float thrust_sp = _thrust_sp;
                //float thrust_sp = _att_thrust;

                _actuators.control[0] = (PX4_ISFINITE(_att_control.roll)) ? _att_control.roll : 0.0f;
                _actuators.control[1] = (PX4_ISFINITE(_att_control.pitch)) ? _att_control.pitch : 0.0f;
                _actuators.control[2] = (PX4_ISFINITE(_att_control.yaw)) ? _att_control.yaw : 0.0f;
                _actuators.control[3] = (PX4_ISFINITE(thrust_sp)) ? thrust_sp : 0.0f;

                if (_actuators_0_pub != nullptr) {

                    orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
                }
                else
                {
                    _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
                }
            }
        }
        perf_end(_loop_perf); //loop performance end
    } _control_task = -1;
}

int lqr_att_control_main(int argc, char* argv[])
{
    /* send a warn if no input argument available */
    if (argc < 2){
        warnx("usage: lqr_att_control {start|stop|status}");
        return 1;
    }
    /* start lqr_att_control manually */
    if (!strcmp(argv[1], "start")) {
        if (lqr_att_control::g_control != nullptr) {
            warnx("Already running!");
            return 1;
        }
        lqr_att_control::g_control = new Lqr_Att_Control;
        if (lqr_att_control::g_control == nullptr) {
            warnx("Allocation failed.");
            return 1;
        }
        if (OK != lqr_att_control::g_control->start()) {
            delete lqr_att_control::g_control;
            lqr_att_control::g_control = nullptr;
            warnx("Start failed.");
            return 1;
        }
        return 0;
    }
    /* stop lqr_att_control manually */
    if (!strcmp(argv[1], "stop")) {
        if (lqr_att_control::g_control == nullptr) {
            warnx("Not running.");
            return 1;
        }
        delete lqr_att_control::g_control;
        lqr_att_control::g_control = nullptr;
        return 0;
    }
    /* return running status of the application */
    if (!strcmp(argv[1], "status")) {
        if (lqr_att_control::g_control) {
            warnx("Running.");
            return 0;
        } else {
            warnx("Not running.");
            return 1;
        }
    }
    /*   DEBUG  messages  */
    if (!strcmp(argv[1], "debugon")) {
        if (lqr_att_control::g_control) {
            debug_status = true;
            return 0;
        } else {
            warnx("Lqr control not running.");
            return 1;
        }
    }

    if (!strcmp(argv[1], "debugoff")) {
        if (lqr_att_control::g_control) {
            debug_status = false;
            return 0;
        } else {
            warnx("Lqr control not running.");
            return 1;
        }
    }

    /* if argument is not in one of the if statement */
    warnx("Unrecognized command.");
    return 0;
}
