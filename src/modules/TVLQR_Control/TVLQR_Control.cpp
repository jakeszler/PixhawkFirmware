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

#include <drivers/drv_pwm_output.h>
#include <uORB/topics/actuator_direct.h>
#include <drivers/drv_hrt.h>


#define _esc_pwm_min 400
#define _esc_pwm_max 2100
#define _max_channel 8

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
    float* qmultiply(float q1[], float q2[]);
    double* K_deltax(double dx[], double Ki[]);
    void _publish_actuators(double u_com[]);

    /**
     * This function handles the Mavlink command long messages
     * It will execute appropriate actions according to input
     */
    void handle_command(struct vehicle_command_s *cmd);
    /**
     * little function to print current flip state
     */
    void print_state();

    void print_data();
    /**
     * check for changes in vehicle control mode
     */
    void vehicle_control_mode_poll();
private:
    bool         _task_should_exit;         /**< if true, main task should exit */
    int         _tvlqr_task;                /**< task handle */
    int         _time_step;
    //int         _x_state;
    int         _x_cur1;
    int         _x_cur2;
    int         _x_cur3;
    int         _x_cur4;
    int         _x_cur5;
    int         _x_cur6;
    int         _x_cur7;
    int         _x_cur8;
    int         _x_cur9;
    int         _x_cur10;
    int         _x_cur11;
    int         _x_cur12;
    int         _x_cur13;
    //int         _u0_cur;
    int         _u_com1;
    int         _u_com2;
    int         _u_com3;
    int         _u_com4;
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
        _x_cur1(1),
        _x_cur2(2),
        _x_cur3(3),
        _x_cur4(4),
        _x_cur5(5),
        _x_cur6(6),
        _x_cur7(7),
        _x_cur8(8),
        _x_cur9(9),
        _x_cur10(10),
        _x_cur11(11),
        _x_cur12(12),
        _x_cur13(13),
        _u_com1(1),
        _u_com2(2),
        _u_com3(3),
        _u_com4(4),
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
void TVLQRControl::print_data()
{
    warnx("Current x is {%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d},\n u_com is {%d,%d,%d,%d}",
        _x_cur1,_x_cur2,_x_cur3,_x_cur4,_x_cur5,_x_cur6,_x_cur7,_x_cur8,_x_cur9,_x_cur10,_x_cur11,_x_cur12,_x_cur13,
        _u_com1,_u_com2,_u_com3,_u_com4);
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


void TVLQRControl::_publish_actuators(double u_com[4])
{
        //Temp ZZ
      //  uint8_t _period[4] = {0,0,0,0};
        void* _actuator_direct_pub = nullptr;

        struct actuator_direct_s actuators;
 
    if (_esc_pwm_min == 0 ||
        _esc_pwm_max == 0) {
        // not initialised yet
        return;
    }
 
        actuators.nvalues = _max_channel;
    if (actuators.nvalues > actuators.NUM_ACTUATORS_DIRECT) {
        actuators.nvalues = actuators.NUM_ACTUATORS_DIRECT;
    }
    // don't publish more than 8 actuators for now, as the uavcan ESC
    // driver refuses to update any motors if you try to publish more
    // than 8
    if (actuators.nvalues > 8) {
        actuators.nvalues = 8;
    }
   // bool armed = hal.util->get_soft_armed();
        actuators.timestamp = hrt_absolute_time();
    for (uint8_t i=0; i<actuators.nvalues; i++) {
       // if (!armed) {
         //   actuators.values[i] = 0;
        //} else {
           actuators.values[i] = u_com[i];
        //actuators.values[i] = (_period[i] - _esc_pwm_min) / (float)(_esc_pwm_max - _esc_pwm_min);
        //}
        // actuator values are from -1 to 1
        actuators.values[i] = actuators.values[i]*2 - 1;
    }
 
    if (_actuator_direct_pub == nullptr) {
        _actuator_direct_pub = orb_advertise(ORB_ID(actuator_direct), &actuators);
    } else {
        orb_publish(ORB_ID(actuator_direct), _actuator_direct_pub, &actuators);
    }
}
 


float* TVLQRControl::qmultiply(float q1[4],  float q2[4])
{

 	float *qresult[4] = {0,0,0,0}; //= {q1[0]*q2[0]-q1[1]*q2[2]*q1[1]};  

 	*qresult[0] = q1[0] * q2[0]- q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	*qresult[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	*qresult[2] = q1[0] * q2[2] + q1[2] * q2[0] - q1[1] * q2[3] + q1[3] * q2[1];
	*qresult[3] = q1[0] * q2[3] + q1[1]	* q2[2] - q1[2] * q2[1] + q1[3] * q2[0];

	return *qresult;
}

double* TVLQRControl::K_deltax(double dx[12],  double Ki[48])
{

        double *u_com[4] = {0,0,0,0}; //= {q1[0]*q2[0]-q1[1]*q2[2]*q1[1]};

        for (uint8_t i = 0; i<=3; i++) {

        *u_com[i] = dx[0]*Ki[i] + dx[1]*Ki[i+4] + dx[2]*Ki[i+8] + dx[3]*Ki[i+12] +
                    dx[4]*Ki[i+16] + dx[5]*Ki[i+20] + dx[6]*Ki[i+24] + dx[7]*Ki[i+28] +
                    dx[8]*Ki[i+32] + dx[9]*Ki[i+36] + dx[10]*Ki[i+40] + dx[11]*Ki[i+44];

        }

        return *u_com;
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
    _vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
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
    //fds[0].fd = _command_sub;
    //fds[0].events = POLLIN;
    fds[0].fd = _vehicle_attitude_sub;
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
            warnx("Im at 339 somehow");
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
   		float q[4] = {*_attitude.q};
		
        double delta_x[12];

        while ((_tvlqr_state > TVLQR_STATE_DISABLED)){//&&(_vehicle_control_mode.flag_control_flip_enabled)){
            warnx("376");
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
             warnx("disabled");
                 break;
             case TVLQR_STATE_START:
             {
                /*
                 * 400 degree/second roll to 45 degrees
            //      */
                warnx("running");
         		float q0[4] = {(float) x0[0][_time_step], (float) x0[1][_time_step], (float) x0[2][_time_step],(float) x0[3][_time_step]};
				float q0con[4] = {*qconjugate(q0)};
        		double qdiff[4] = {*qmultiply(q0con,q)};

                delta_x[0] = {_global_position.lat-(x0[0][_time_step])};
                delta_x[1] = {_global_position.lon-x0[1][_time_step]};
                delta_x[2] = {(double)_global_position.alt-x0[2][_time_step]};

                //delta_x[3] = {(double)qdiff[0]};
                delta_x[3] = {(double)qdiff[1]};
                delta_x[4] = {(double)qdiff[2]};
                delta_x[5] = {(double)qdiff[3]};

                delta_x[6] = {(double)_global_position.vel_n-x0[7][_time_step]};
                delta_x[7] = {(double)_global_position.vel_e-x0[8][_time_step]};
                delta_x[8] = {(double)_global_position.vel_d-x0[9][_time_step]};


                delta_x[9] = {(double)_attitude.rollspeed-x0[10][_time_step]};
                delta_x[10] = {(double)_attitude.pitchspeed-x0[11][_time_step]};
                delta_x[11] = {(double)_attitude.yawspeed-x0[12][_time_step]};

                double Ki[48];
                for(uint8_t i = 0; i < 48; ++i)
                {
                    Ki[i] = K[i][_time_step];
                }
                //u = -K*deltax+u0
                double u_com[4] = {*K_deltax(delta_x,  Ki)};
                u_com[0] = -1*u_com[0] + u0[0][_time_step];
                u_com[1] = -1*u_com[1] + u0[1][_time_step];
                u_com[2] = -1*u_com[2] + u0[2][_time_step];
                u_com[3] = -1*u_com[3] + u0[3][_time_step];
                _publish_actuators(u_com);

                //For printing and debugging
                _u_com1 = u_com[0];
                _u_com2 = u_com[1];
                _u_com3 = u_com[2];
                _u_com4 = u_com[3];

             //   double u[4];

             //	for(int i = 0; i <12; i++){
             //           
             //           float u_com[4] = {*K_deltax(float delta_x[12],  float Ki[48])};
             //		u[i] = -K[i][_time_step]*delta_x[i][_time_step] + u;
             //	}
             	


             	//printf("%d",(int)delta_x[1]);
             	//printf("%d",qdiff[0]);

             //matrix multiply
             


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
        warnx("usage: TVLQR_Control {start|stop|status|state|data}");
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
    if (!strcmp(argv[1], "data")) {
        TVLQR_Control::g_tvlqr->print_data();
        return 0;
    }
    /* if argument is not in one of the if statement */
    warnx("unrecognized command");
    return 0;
}
