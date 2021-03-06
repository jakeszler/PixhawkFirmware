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
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include "K_header_file_discrete_tvlqr.cpp"

#include <drivers/drv_pwm_output.h>
#include <uORB/topics/actuator_direct.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/actuator_controls.h>
//#include <uORB/topics/vehicle_rates_setpoint.h>
//#include <controllib/blocks.hpp>
//#include <controllib/block/blocks.hpp>
//#include <controllib/uorb/blocks.hpp>

//using control::BlockPI;
//using control::BlockP;
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
    void _publish_actuators(double u_com[], const struct vehicle_attitude_s *att, struct actuator_controls_s *actuators);

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


void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att, struct actuator_controls_s *actuators, double u_com[]);

    /**
     * check for changes in vehicle control mode
     */
    void vehicle_control_mode_poll();
private:
    bool         _task_should_exit;         /**< if true, main task should exit */
    int         _tvlqr_task;                /**< task handle */
    int         _time_step;
    double         _t_init; //Initial time when program is started
    //int         _x_state;
    double         _x_init;
    double         _y_init;
    double         _z_init;
    double         _x_cur1;
    double         _x_cur2;
    double         _x_cur3;
    double         _x_cur4;
    double         _x_cur5;
    double         _x_cur6;
    double         _x_cur7;
    double         _x_cur8;
    double         _x_cur9;
    double         _x_cur10;
    double         _x_cur11;
    double         _x_cur12;
    double         _x_cur13;
    //int         _u0_cur;
    double         _u_com1;
    double         _u_com2;
    double         _u_com3;
    double         _u_com4;
    enum TVLQR_STATE {
            TVLQR_STATE_DISABLED = 0,
            TVLQR_STATE_START = 1,
            TVLQR_STATE_FINISHED = 2
        }_tvlqr_state;                    /**< flip state */
    /* subscriptions */
    int         _command_sub;
    int         _vehicle_control_mode_sub;
    int         _vehicle_attitude_sub;
    int         _vehicle_local_position_sub;
    /* publications */
    orb_advert_t     _vehicle_control_mode_pub;
    orb_advert_t     _vehicle_rates_setpoint_pub;
    struct vehicle_command_s         _command;                /**< vehicle commands */
    struct vehicle_control_mode_s     _vehicle_control_mode;     /**< vehicle control mode */
    //struct vehicle_attitude_s         _attitude;                /**< vehicle attitude */
    struct vehicle_local_position_s    _local_position;
    struct vehicle_rates_setpoint_s _vehicle_rates_setpoint;            /**< vehicle rate setpoint */

    struct vehicle_attitude_setpoint_s _att_sp {};
    struct vehicle_rates_setpoint_s rates_sp {};
     orb_advert_t actuator_pub;
	orb_advert_t	_attitude_sp_pub{nullptr}; 
	orb_id_t _attitude_setpoint_id{nullptr};

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
        _t_init(0),
        _x_init(0),
        _y_init(0),
        _z_init(0),
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
        _vehicle_local_position_sub(-1),
        _vehicle_control_mode_pub(nullptr),
        _vehicle_rates_setpoint_pub(nullptr)
{
    memset(&_command, 0, sizeof(_command));
    memset(&_vehicle_control_mode, 0, sizeof(_vehicle_control_mode));
   // memset(&_attitude, 0, sizeof(_attitude));
    memset(&_local_position, 0, sizeof(_local_position));
    memset(&_vehicle_rates_setpoint, 0, sizeof(_vehicle_rates_setpoint));
    //memset(&actuators, 0, sizeof(actuators));
    memset(&rates_sp, 0, sizeof(rates_sp));


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
    warnx("Current x is {%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf},\n u_com is {%lf,%lf,%lf,%lf}\n t_init is %d",
        _x_cur1,_x_cur2,_x_cur3,_x_cur4,_x_cur5,_x_cur6,_x_cur7,_x_cur8,_x_cur9,_x_cur10,_x_cur11,_x_cur12,_x_cur13,
        _u_com1,_u_com2,_u_com3,_u_com4,_t_init);
}
void TVLQRControl::handle_command(struct vehicle_command_s *cmd)
{
     switch (cmd->command) {
     case vehicle_command_s::VEHICLE_CMD_TVLQR_START:
         warnx("TVLQR initiated");
         _tvlqr_state = TVLQR_STATE_START;
         break;
     case vehicle_command_s::VEHICLE_CMD_TVLQR_TERMINATE:
         //warnx("TVLQR terminated");
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


void TVLQRControl::control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,struct actuator_controls_s *actuators, double u_com[])
{
	/*
	 * The PX4 architecture provides a mixer outside of the controller.
	 * The mixer is fed with a default vector of actuator controls, representing
	 * moments applied to the vehicle frame. This vector
	 * is structured as:
	 *
	 * Control Group 0 (attitude):
	 *
	 *    0  -  roll   (-1..+1)
	 *    1  -  pitch  (-1..+1)
	 *    2  -  yaw    (-1..+1)
	 *    3  -  thrust ( 0..+1)
	 *    4  -  flaps  (-1..+1)
	 *    ...
	 *
	 * Control Group 1 (payloads / special):
	 *
	 *    ...
	 */

	/* set r/p zero */
	// actuators->control[0] = 0.0f;
	// actuators->control[1] = 0.0f;

	// /*
	//  * Calculate yaw error and apply P gain
	//  */
	// //float yaw_err = matrix::Eulerf(matrix::Quatf(att->q)).psi() - matrix::Eulerf(matrix::Quatf(att_sp->q_d)).psi();
	// actuators->control[2] = 0.0f;

	// /* copy throttle */
	// actuators->control[3] = att_sp->thrust;


     warnx("before: %f ", static_cast<double>(actuators->control[3]));
     warnx("shouldbe: %.6f ", ( u_com[0]));  //*2-1
	 for (int i=0; i<actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
	 	_att_sp.timestamp = hrt_absolute_time();
	 		_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);

	 		if (_attitude_sp_pub != nullptr) {
						/* publish the attitude setpoint */
						orb_publish(_attitude_setpoint_id, _attitude_sp_pub, &_att_sp);

					} else if (_attitude_setpoint_id != nullptr) {
						/* advertise and publish */
						_attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
}

			if(i == 0){
					// if(1 >= u_com[0] && u_com[0] >= -1)
	    //      			actuators->control[3]  = (float) u_com[0];
	    //      		else if (1 < u_com[0])
	    //      			actuators->control[3] = 1.0;
	    //      		else if (u_com[0] < -1)
	    //      			actuators->control[3] = -1.0;
				_att_sp.roll_body += (float)0.01;//_parameters.rollsp_offset_rad;
				_att_sp.pitch_body += (float)0.01;//_parameters.pitchsp_offset_rad;
				_att_sp.yaw_body += (float)0.01;
				_att_sp.thrust += (float)0.01;
				_attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				//actuators->control[3] =actuators->control[3] + (float) 0.01;

	         	}
         	else if(i == 1){
					// if(1 >= u_com[1] && u_com[1] >= -1)
	    //      			actuators->control[0]  = (float) u_com[1];
	    //      		else if (1 < u_com[1])
	    //      			actuators->control[0] = 1.0;
	    //      		else if (u_com[1] < -1)
	    //      			actuators->control[0] = -1.0;
         		actuators->control[0] = 0.99;//actuators->control[0] + (float)0.001;

	         		//actuators->control[0]  = (float) u_com[1]; warnx("writing"); 
	         	}
         	else if(i == 2){
         			// if(1 >= u_com[2] && u_com[2] >= -1)
	         		// 	actuators->control[1]  = (float) u_com[2];
	         		// else if (1 < u_com[2])
	         		// 	actuators->control[1] = 1.0;
	         		// else if (u_com[2] < -1)
	         		// 	actuators->control[1] = -1.0;
         			actuators->control[1] =  actuators->control[1] + (float)0.01;
	         	}
	         		//actuators->control[1]  = (float) u_com[2];
			else if(i == 3){
					// if(1 >= u_com[3] && u_com[3] >= -1)
	    //      			actuators->control[2]  = (float) u_com[3];
	    //      		else if (1 < u_com[3])
	    //      			actuators->control[2] = 1.0;
	    //      		else if (u_com[3] < -1)
	    //      			actuators->control[2] = -1.0;
				         actuators->control[2] = 0.0;

	         	}
	         		//actuators->control[2]  = (float) u_com[3];
         	else {
         		     actuators->control[1] = 0.0;

         		// if(1 >= u_com[i] && u_com[i] >= -1)
	         	// 		actuators->control[i]  = (float) u_com[i];
         		// else if (1 < u_com[i])
	         	// 		actuators->control[i] = 1.0;
         		// else if (u_com[i] < -1)
	         	// 		actuators->control[i] = -1.0;
	         	}
	         	//	actuators->control[i]  = (float) u_com[i];


        //actuators.values[i] = (_period[i] - _esc_pwm_min) / (float)(_esc_pwm_max - _esc_pwm_min);
        }

        actuators->timestamp = hrt_absolute_time();

       warnx("isafter:%.6f", static_cast<double>(actuators->control[3]));

        // actuator values are from -1 to 1
        //actuators.values[i] = actuators.values[i]*2 - 1;
    							//}
}

void TVLQRControl::_publish_actuators(double u_com[4],  const struct vehicle_attitude_s *att, struct actuator_controls_s *actuators)
{
        //Temp ZZ
      //  uint8_t _period[4] = {0,0,0,0};
	 // warnx("here6");

        //void* _actuator_direct_pub = nullptr;

        //struct actuator_direct_s actuators;
											    // if (_esc_pwm_min == 0 ||
											    //     _esc_pwm_max == 0) {
											    //     // not initialised yet
											    //     return;
											    // }
 
    //     actuators.nvalues = _max_channel;
    // if (actuators.nvalues > actuators.NUM_ACTUATORS_DIRECT) {
    //     actuators.nvalues = actuators.NUM_ACTUATORS_DIRECT;
    // }
    // // don't publish more than 8 actuators for now, as the uavcan ESC
    // // driver refuses to update any motors if you try to publish more
    // // than 8
    // if (actuators.nvalues > 8) {
    //     actuators.nvalues = 8;
    // }
   // bool armed = hal.util->get_soft_armed();
  
        //actuators->timestamp = hrt_absolute_time();
         
  							//  for (int i=0; i<actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
       // if (!armed) {
         //   actuators.values[i] = 0;
        //} else {
	//warnx("here8 %lf",  u_com[i]*2-1);



    	/* From actuator_controls.msg
    	uint8 INDEX_ROLL = 0
		uint8 INDEX_PITCH = 1
		uint8 INDEX_YAW = 2
		uint8 INDEX_THROTTLE = 3
		*/


									 // warnx("i: %d values: %.4f", i ,(double)u_com[i]*2-1);
									 // 		if(i == 0)
								  //          		actuators.control[3]  = (float) u_com[0]*2-1;
								  //          	else if(i == 1)
								  //          		actuators.control[0]  = (float) u_com[1]*2-1;
								  //          	else if(i == 2)
								  //          		actuators.control[1]  = (float) u_com[2]*2-1;
										// 	else if(i == 3)
								  //          		actuators.control[2]  = (float) u_com[3]*2-1;
								  //          	else
								  //          		actuators.control[i]  = (float) u_com[i]*2-1;


        //actuators.values[i] = (_period[i] - _esc_pwm_min) / (float)(_esc_pwm_max - _esc_pwm_min);
        //}
        // actuator values are from -1 to 1
        //actuators.values[i] = actuators.values[i]*2 - 1;
    							//}
    //warnx("here9");
   
// actuators.timestamp = hrt_absolute_time();
    //if (_actuator_direct_pub == nullptr) {
     //   _actuator_direct_pub = orb_advertise(ORB_ID(actuator_direct), &actuators);
    //} else {
    	//orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
       // orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
 	      // updatePublications();
      //  warnx("here8");
//orb_publish(ORB_ID(actuator_direct), _actuator_direct_pub, &actuators);
   // }
}
 


float* TVLQRControl::qmultiply(float q1[4],  float q2[4])
{

 	static float qresult[4] = {0,0,0,0}; //= {q1[0]*q2[0]-q1[1]*q2[2]*q1[1]};  
 	qresult[0] = q1[0] * q2[0]- q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	qresult[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	qresult[2] = q1[0] * q2[2] + q1[2] * q2[0] - q1[1] * q2[3] + q1[3] * q2[1];
	qresult[3] = q1[0] * q2[3] + q1[1]	* q2[2] - q1[2] * q2[1] + q1[3] * q2[0];

	return qresult;
}

double* TVLQRControl::K_deltax(double dx[12],  double Ki[48])
{

       static double u_com[4] = {0,0,0,0}; //= {q1[0]*q2[0]-q1[1]*q2[2]*q1[1]};

        for (uint8_t i = 0; i<=3; i++) {

        u_com[i] = dx[0]*Ki[i] + dx[1]*Ki[i+4] + dx[2]*Ki[i+8] + dx[3]*Ki[i+12] +
                    dx[4]*Ki[i+16] + dx[5]*Ki[i+20] + dx[6]*Ki[i+24] + dx[7]*Ki[i+28] +
                    dx[8]*Ki[i+32] + dx[9]*Ki[i+36] + dx[10]*Ki[i+40] + dx[11]*Ki[i+44];

        }

        return u_com;
}

void TVLQRControl::task_main()
{
    /* make sure slip_state is disabled at initialization */
    _tvlqr_state = TVLQR_STATE_DISABLED;
    //	memset(&actuators, 0, sizeof(actuators));
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
    _vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    /* advertise control mode topic */
    //_vehicle_control_mode_pub = orb_advertise(ORB_ID(vehicle_control_mode), &_vehicle_control_mode);
    /* advertise rate setpoint topic */
    //_vehicle_rates_setpoint_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_vehicle_rates_setpoint);
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

	//struct vehicle_attitude_setpoint_s _att_sp = {};
struct actuator_controls_s actuators;
memset(&actuators, 0, sizeof(actuators));
struct vehicle_attitude_s att;
memset(&att, 0, sizeof(att));


    fds[0].fd = _vehicle_attitude_sub;
    fds[0].events = POLLIN;
   // orb_advert_t rates_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

    for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}	
    actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

   // 	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

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






        //warnx("made it past 340");
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

        if(_tvlqr_state == 1){
        warnx("Current tvlqr state is %d", _tvlqr_state);
        _t_init = hrt_absolute_time(); // _att.timestamp;
        _x_init = _local_position.x+6;
        _y_init = _local_position.y;
        _z_init = (double)_local_position.z+1.5;

        }
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

           // orb_publish(ORB_ID(vehicle_rates_setpoint), rates_pub, &rates_sp);

            if (updated) {
                orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &att)	;
            }
              //      warnx("here5");

            // decide what to do based on current flip_state
             switch (_tvlqr_state) {
             case TVLQR_STATE_DISABLED:
            //     // shoudn't even enter this but just in case
            //     // do nothing
             warnx("disabled");
            // warnx("%lf", ((double)actuators.control[0]));
                 break;
             case TVLQR_STATE_START:
             {
                /*
                 * 400 degree/second roll to 45 degrees
            //      */
                float q[4] = {*att.q};
                warnx("%lf", ((double)actuators.control[0]));
        		 warnx("runnn");
                double delta_x[12];
                //warnx("attitude timestamp = %d \n t_init %d \n T_end %lf",_attitude.timestamp-_t_init,_t_init,t[(sizeof(t)/sizeof(t[0]))-1]*pow(10,6));
                if(att.timestamp-_t_init >= t[(sizeof(t)/sizeof(t[0]))-1]*pow(10,6)){
                     _tvlqr_state = TVLQR_STATE_FINISHED;
                     break;
                }
                while((att.timestamp-_t_init > t[_time_step + 1]*pow(10,6))&&(att.timestamp-_t_init < t[_time_step + 2]*pow(10,6)))
                {
                    ++_time_step;
                //warnx("Timestep %d",_time_step);
                }

         		float q0[4] = {(float) x0[3][_time_step], (float) x0[4][_time_step], (float) x0[5][_time_step],(float) x0[6][_time_step]};
                float q0con[4] = {*qconjugate(q0)};
        		double qdiff[4] = {*qmultiply(q0con,q)};

                delta_x[0] = {(double)_local_position.x-(x0[0][_time_step])-_x_init};
                //warnx("xd = %lf",-(x0[0][_time_step]));
                delta_x[1] = {(double)_local_position.y-x0[1][_time_step]-_y_init};
                delta_x[2] = {(double)_local_position.z-x0[2][_time_step]-_z_init};

                //delta_x[3] = {(double)qdiff[0]};
                delta_x[3] = {(double)qdiff[1]};
                delta_x[4] = {(double)qdiff[2]};
                delta_x[5] = {(double)qdiff[3]};

                delta_x[6] = {(double)_local_position.vx-x0[7][_time_step]};
                delta_x[7] = {(double)_local_position.vy-x0[8][_time_step]};
                delta_x[8] = {(double)_local_position.vz-x0[9][_time_step]};


                delta_x[9] = {(double)att.rollspeed-x0[10][_time_step]};
                delta_x[10] = {(double)att.pitchspeed-x0[11][_time_step]};
                delta_x[11] = {(double)att.yawspeed-x0[12][_time_step]};

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
                        

                //PX4_INFO("All the way at 530!");
              //  _publish_actuators(u_com);
                 	control_attitude(&_att_sp, &att, &actuators,  u_com);
                 	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);



                 	if (PX4_ISFINITE(actuators.control[0]) &&
				    PX4_ISFINITE(actuators.control[1]) &&
				    PX4_ISFINITE(actuators.control[2]) &&
				    PX4_ISFINITE(actuators.control[3])) {
					orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
						warnx("published");
					}



                //For printing and debugging
                _u_com1 = u_com[0];
                _u_com2 = u_com[1];
                _u_com3 = u_com[2];
                _u_com4 = u_com[3];

                             _x_cur1 = delta_x[0];
                             _x_cur2 = delta_x[1];
                             _x_cur3 = delta_x[2];
                             _x_cur4 = delta_x[3];
                             _x_cur5 = delta_x[4];
                             _x_cur6 = delta_x[5];
                             _x_cur7 = delta_x[6];
                             _x_cur8 = delta_x[7];
                             _x_cur9 = delta_x[8];
                             _x_cur10 = delta_x[9];
                             _x_cur11 = delta_x[10];	
                             _x_cur12 = delta_x[11];


             //   double u[4];

             //	for(int i = 0; i <12; i++){
             //           
             //           float u_com[4] = {*K_deltax(float delta_x[12],  float Ki[48])};
             //		u[i] = -K[i][_time_step]*delta_x[i][_time_step] + u;
             //	}
             	


             	//printf("%d",(int)delta_x[1]);
             	//printf("%d",qdiff[0]);

             //matrix multiply
             

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
            // warnx("I finished a loop");
            //     // enable manual control and attitude control
                 _vehicle_control_mode.flag_control_manual_enabled = true;
                 _vehicle_control_mode.flag_control_attitude_enabled = true;
                 orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
            //     // switch back to disabled flip state
                 _tvlqr_state = TVLQR_STATE_DISABLED;
                 _time_step = 0;
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
