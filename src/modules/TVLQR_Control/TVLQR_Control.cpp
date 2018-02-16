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
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include "K_header_file_discrete_tvlqr.cpp"
/*
 * TVLQR_Control.cpp
 *
 *  Created on: 28Sep.,2016
 *      Author: Zihao
 */
extern "C" __EXPORT int TVLQR_control_main(int argc, char *argv[]);
class TVLQRControl
{
public:
    /**
     * Constructor
     */
    TVLQRControl();
    /**
     * Destructor, also kills the main task
     */
    ~TVLQRControl();
    /**
     * Start the flip state switch task
     *
     * @return OK on success
     */
    int start();
    float* qconjugate(float*);
    float* qmultiply(float q1, float q2);

    /**
     * This function handles the Mavlink command long messages
     * It will execute appropriate actions according to input
     */
    void handle_command(struct vehicle_command_s *cmd);
    /**
     * little function to print current flip state
     */
    void print_state();
    /**
     * check for changes in vehicle control mode
     */
    void vehicle_control_mode_poll();
private:
    bool         _task_should_exit;         /**< if true, main task should exit */
    int         _tvlqr_task;                /**< task handle */
    int         _time_step;
    enum TVLQR_STATE {
            TVLQR_STATE_DISABLED = 0,
            TVLQR_STATE_START = 1,
            TVLQR_STATE_FINISHED = 2
        }_tvlqr_state;                    /**< flip state */
    /* subscriptions */
    int         _command_sub;
    int         _vehicle_control_mode_sub;
    int         _vehicle_attitude_sub;
    int         _vehicle_global_position_sub;
    /* publications */
    orb_advert_t     _vehicle_control_mode_pub;
    orb_advert_t     _vehicle_rates_setpoint_pub;
    struct vehicle_command_s         _command;                /**< vehicle commands */
    struct vehicle_control_mode_s     _vehicle_control_mode;     /**< vehicle control mode */
    struct vehicle_attitude_s         _attitude;                /**< vehicle attitude */
    struct vehicle_global_position_s    _global_position;
    struct vehicle_rates_setpoint_s _vehicle_rates_setpoint;            /**< vehicle rate setpoint */
    /**
     * Shim for calling task_main from task_create
     */
    static void task_main_trampoline(int argc, char *argv[]);
    /**
     * Main attitude control task
     */
    void         task_main();
};
namespace TVLQR_Control
{
TVLQRControl *g_tvlqr;
}
TVLQRControl::TVLQRControl() :
        _task_should_exit(false),
        _tvlqr_task(-1),
        _time_step(0),
        _tvlqr_state(TVLQR_STATE_DISABLED),
        _command_sub(-1),
        _vehicle_control_mode_sub(-1),
        _vehicle_attitude_sub(-1),
        _vehicle_global_position_sub(-1),
        _vehicle_control_mode_pub(nullptr),
        _vehicle_rates_setpoint_pub(nullptr)
{
    memset(&_command, 0, sizeof(_command));
    memset(&_vehicle_control_mode, 0, sizeof(_vehicle_control_mode));
    memset(&_attitude, 0, sizeof(_attitude));
    memset(&_global_position, 0, sizeof(_global_position));
    memset(&_vehicle_rates_setpoint, 0, sizeof(_vehicle_rates_setpoint));
}
TVLQRControl::~TVLQRControl()
{
    _task_should_exit = true;
    TVLQR_Control::g_tvlqr = nullptr;
}
void TVLQRControl::print_state()
{
    warnx("Current tvlqr state is %d", _tvlqr_state);
}
void TVLQRControl::handle_command(struct vehicle_command_s *cmd)
{
     switch (cmd->command) {
     case vehicle_command_s::VEHICLE_CMD_TVLQR_START:
         warnx("TVLQR initiated");
         _tvlqr_state = TVLQR_STATE_START;
         break;
     case vehicle_command_s::VEHICLE_CMD_TVLQR_TERMINATE:
         warnx("TVLQR terminated");
         _tvlqr_state = TVLQR_STATE_FINISHED;
         break;
     }
}
void TVLQRControl::vehicle_control_mode_poll()
{
    bool updated;
    /* check if vehicle control mode has changed */
    orb_check(_vehicle_control_mode_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub, &_vehicle_control_mode);
    }
}
void TVLQRControl::task_main_trampoline(int argc, char *argv[])
{
    TVLQR_Control::g_tvlqr->task_main();
}
float* TVLQRControl::qconjugate(float q[4])
{
q[1] = -q[1];
q[2] = -q[2];
q[3] = -q[3];
return q;
}

float* TVLQRControl::qmultiply(float q1[4],  float q2[4])
{

 	float* qresult[4]; //= {q1[0]*q2[0]-q1[1]*q2[2]*q1[1]};  

 	qresult[0] = q1[0]*q2[0]- q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
qresult[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
qresult[2] = q1[0]*q2[2] + q1[2]*q2[0] - q1[1]*q2[3] + q1[3]*q2[1];
qresult[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];

	return qresult;
}
void TVLQRControl::task_main()
{
    /* make sure slip_state is disabled at initialization */
    _tvlqr_state = TVLQR_STATE_DISABLED;
    // inner loop sleep time
    const unsigned sleeptime_us = 50000;
    // first phase roll or pitch target
    //float rotate_target_45 = 45*3.14/180;
    // second phase roll or pitch target
//    float rotate_target_90 = 89*3.14/180;
    // rotate rate set point
    //float rotate_rate = 400*3.14/180;
    // use this to check if a topic is updated
    bool updated = false;
    int poll_interval = 100; // listen to the topic every x millisecond
    /* subscribe to vehicle command topic */
    _command_sub = orb_subscribe(ORB_ID(vehicle_command));
    /* subscribe to vehicle control mode topic */
    _vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    /* subscribe to vehicle attitude topic */
    _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    /* subscribe to the global position topic */
    _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    /* advertise control mode topic */
    _vehicle_control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &_vehicle_control_mode);
    /* advertise rate setpoint topic */
    _vehicle_rates_setpoint_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_vehicle_rates_setpoint);
    /*
     * declare file descriptor structure, # in the [] means the
     * # of topics, here is 1 since we are only
     * polling vehicle_command
     */
    px4_pollfd_struct_t fds[1];
    /*
     * initialize file descriptor to listen to vehicle_command
     */
    fds[0].fd = _command_sub;
    fds[0].events = POLLIN;
    /* start main slow loop */
    while (!_task_should_exit) {
        /* set the poll target, number of file descriptor, and poll interval */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), poll_interval);
        /*
         * this means no information is coming from the topic in the set interval
         * skip loop
         */
        if (pret == 0) {
            continue;
        }
        /*
         * this means some error happened, I don't know what to do
         * skip loop
         */
        if (pret < 0) {
            warn("poll error %d %d", pret, errno);
            continue;
        }
        /*
         * if everything goes well, copy the command into our variable
         * and handle command
         */
        if (fds[0].revents & POLLIN) {
            /*
             * copy command structure from the topic to our local structure
             */
            orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
            handle_command(&_command);
        }
        /*
         * check for updates in other topics
         */
        vehicle_control_mode_poll();
        /*
         * switch to faster update during the flip
         */
        while ((_tvlqr_state > TVLQR_STATE_DISABLED)){//&&(_vehicle_control_mode.flag_control_flip_enabled)){
            // update commands
            orb_check(_command_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
                handle_command(&_command);
            }
            bool topic_changed = false;
            // copy vehicle control mode topic if updated
            vehicle_control_mode_poll();
            // disable _v_control_mode.flag_control_manual_enabled
            if (_vehicle_control_mode.flag_control_manual_enabled) {
                _vehicle_control_mode.flag_control_manual_enabled = false;
                topic_changed = true;
            }
            // disable _v_control_mode.flag_conttrol_attitude_enabled
            if (_vehicle_control_mode.flag_control_attitude_enabled) {
                _vehicle_control_mode.flag_control_attitude_enabled = false;
                topic_changed = true;
            }
            // publish to vehicle control mode topic if topic is changed
            if (topic_changed) {
                orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
            }
            // update vehicle attitude
            orb_check(_vehicle_attitude_sub, &updated);
            if (updated) {
                orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_attitude);
            }
            // decide what to do based on current flip_state
             switch (_tvlqr_state) {
             case TVLQR_STATE_DISABLED:
            //     // shoudn't even enter this but just in case
            //     // do nothing
                 break;
             case TVLQR_STATE_START:
             {
                /*
                 * 400 degree/second roll to 45 degrees
            //      */

             float q[4] = {*_attitude.q};
             float q0[4] = {(float) x0[0][_time_step], (float) x0[1][_time_step], (float) x0[2][_time_step],(float) x0[3][_time_step]};

             float q0con[4] = {*qconjugate(q0)};


             qdiff = qmultiply(*q0con,*q);

                while(_attitude.timestamp > t[_time_step + 1])
                {
                    ++_time_step;
                }

            //     _vehicle_rates_setpoint.roll = rotate_rate;
            //     _vehicle_rates_setpoint.pitch = 0;
            //     _vehicle_rates_setpoint.yaw = 0;
            //     _vehicle_rates_setpoint.thrust = 1;
            //     orb_publish(ORB_ID(vehicle_rates_setpoint), _vehicle_rates_setpoint_pub, &_vehicle_rates_setpoint);
            //     // if ((_attitude.roll > 0.0f && _attitude.roll > rotate_target_45) || (_attitude.roll < 0.0f && _attitude.roll < -rotate_target_45)) {
            //     //     _tvlqr_state = FLIP_STATE_ROLL;
            //     // }
                 
             }
             break;
            // case FLIP_STATE_ROLL:
            //     /*
            //      * 400 degree/second roll to 90 degrees
            //      */
            // {
            //     _vehicle_rates_setpoint.roll = rotate_rate;
            //     _vehicle_rates_setpoint.pitch = 0;
            //     _vehicle_rates_setpoint.yaw = 0;
            //     _vehicle_rates_setpoint.thrust = 0.75;
            //     orb_publish(ORB_ID(vehicle_rates_setpoint), _vehicle_rates_setpoint_pub, &_vehicle_rates_setpoint);
            //     // if ((_attitude.roll > 0.0f && _attitude.roll < rotate_target_45) || (_attitude.roll < 0.0f && _attitude.roll > -rotate_target_45)) {
            //     //     _tvlqr_state = FLIP_STATE_RECOVER;
            //      }
            // }
            //     break;
            // case FLIP_STATE_RECOVER:
            //     /*
            //      * level the vehicle
            //      */
            //     _vehicle_control_mode.flag_control_attitude_enabled = true;
            //     orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
            //     _tvlqr_state = FLIP_STATE_FINISHED;
            //     break;
             case TVLQR_STATE_FINISHED:
                 /*
                  * go back to disabled state
                  */
            //     // enable manual control and attitude control
                 _vehicle_control_mode.flag_control_manual_enabled = true;
                 _vehicle_control_mode.flag_control_attitude_enabled = true;
                 orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
            //     // switch back to disabled flip state
                 _tvlqr_state = TVLQR_STATE_DISABLED;
                 break;
             }
            // run at roughly 100 hz
            usleep(sleeptime_us);
        }
    }
}
int TVLQRControl::start()
{
    ASSERT(_tvlqr_task == -1);
    /*start the task */
    _tvlqr_task = px4_task_spawn_cmd("TVLQR_Control",
                                    SCHED_DEFAULT,
                                    SCHED_PRIORITY_DEFAULT,
                                    2048,
                                    (px4_main_t)&TVLQRControl::task_main_trampoline,
                                    nullptr);
    if (_tvlqr_task < 0) {
        warn("task start failed");
        return -errno;
    }
    return OK;
}
int TVLQR_control_main(int argc, char *argv[])
{
    /* warn if no input argument */
    if (argc < 2) {
        warnx("usage: TVLQR_Control {start|stop|status|state}");
        return 1;
    }
    /* start TVLQR_Control manually */
    if (!strcmp(argv[1],"start")) {
        if (TVLQR_Control::g_tvlqr != nullptr) {
            warnx("already running");
            return 1;
        }
        TVLQR_Control::g_tvlqr = new TVLQRControl;
        if (TVLQR_Control::g_tvlqr == nullptr) {
            warnx("allocation failed");
            return 1;
        }
        if (OK != TVLQR_Control::g_tvlqr->start()) {
            delete TVLQR_Control::g_tvlqr;
            TVLQR_Control::g_tvlqr = nullptr;
            warnx("start failed");
            return 1;
        }
        return 0;
    }
    /* stop TVLQR_Control manually */
    if (!strcmp(argv[1], "stop")) {
        if (TVLQR_Control::g_tvlqr == nullptr) {
            warnx("not running");
            return 1;
        }
        delete TVLQR_Control::g_tvlqr;
        TVLQR_Control::g_tvlqr = nullptr;
        return 0;
    }
    /* return running status of the application */
    if (!strcmp(argv[1], "status")) {
        if (TVLQR_Control::g_tvlqr) {
            warnx("running");
            return 0;
        } else {
            warnx("not running");
            return 1;
        }
    }
    /* print current flip_state */
    if (!strcmp(argv[1], "state")) {
        TVLQR_Control::g_tvlqr->print_state();
        return 0;
    }
    /* if argument is not in one of the if statement */
    warnx("unrecognized command");
    return 0;
}
