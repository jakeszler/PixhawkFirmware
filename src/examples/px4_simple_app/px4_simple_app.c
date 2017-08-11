/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */


#define DEFAULT_UART "/dev/ttyS3"
#define MAX_MESSAGE_BUFFER_SIZE 45
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <systemlib/err.h>


#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include "comms.h"
#include "messages.h"

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>


#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>
#include <string.h>

#include <termios.h>
//#include <px4_log.h>
#include <px4_time.h>
#include <drivers/drv_pwm_output.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <mathlib/mathlib.h>
#include "sbp.h"
#include "navigation.h"
#include "tutorial_implementation.h"
#include "edc.h"

#define MAVLINK_MESSAGE_LENGTHS {9, 31, 12, 0, 14, 28, 3, 32, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 20, 2, 25, 23, 30, 101, 22, 26, 16, 14, 28, 32, 28, 28, 22, 22, 21, 6, 6, 37, 4, 4, 2, 2, 4, 2, 2, 3, 13, 12, 37, 4, 0, 0, 27, 25, 0, 0, 0, 0, 0, 72, 26, 181, 225, 42, 6, 4, 0, 11, 18, 0, 0, 37, 20, 35, 33, 3, 0, 0, 0, 22, 39, 37, 53, 51, 53, 51, 0, 28, 56, 42, 33, 81, 0, 0, 0, 0, 0, 0, 26, 32, 32, 20, 32, 62, 44, 64, 84, 9, 254, 16, 12, 36, 44, 64, 22, 6, 14, 12, 97, 2, 2, 113, 35, 6, 79, 35, 35, 22, 13, 255, 14, 18, 43, 8, 22, 14, 36, 43, 41, 32, 243, 14, 93, 0, 100, 36, 60, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 42, 40, 63, 182, 40, 0, 0, 0, 0, 0, 0, 32, 52, 53, 6, 2, 38, 19, 254, 36, 30, 18, 18, 51, 9, 0}
#define MAVLINK_MESSAGE_CRCS {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, 191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 163, 105, 151, 35, 150, 0, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0}

#include "mavlink_types.h"
//#include <modules/mavlink/mavlink_messages.h>

//#include <mavlink/include/mavlink/v1.0/mavlink_types.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>


#include "drivers/drv_pwm_output.h"
#include <drivers/drv_hrt.h>


#define MAX_LEN_DEV_PATH 32


__EXPORT int px4_simple_app_main(int argc, char *argv[]);



static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

volatile bool _task_should_exit = false; // flag indicating if snapdragon_rc_pwm task should exit
static char _device[MAX_LEN_DEV_PATH];
//static bool _is_running = false;         // flag indicating if snapdragon_rc_pwm app is running
//static px4_task_t _task_handle = -1;     // handle to the task main thread
static int _uart_fd = -1;
int _pwm_fd = -1;
static bool _flow_control_enabled = false;
int32_t _pwm_disarmed;

hrt_abstime _last_actuator_controls_received = 0;


// Print out the usage information
/*void usage();

void start();


void stop();

int initialise_uart();

int deinitialize_uart();

int enable_flow_control(bool enabled);

void send_rc_mavlink();

void handle_message(mavlink_message_t *msg);

void set_pwm_output(mavlink_actuator_control_target_t *actuator_controls);

*/
#define MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET 140

__EXPORT int initialise_uart(void);
int enable_flow_control(bool enabled);
int deinitialize_uart(void);

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

int piksi_uart(int argc, char *argv[]);
/*
 * State of the SBP message parser.
 * Must be statically allocated.
 */
sbp_state_t sbp_state;

/* SBP structs that messages from Piksi will feed. */
msg_pos_llh_t      pos_llh;
msg_baseline_ned_t baseline_ned;
msg_vel_ned_t      vel_ned;
msg_dops_t         dops;
msg_gps_time_t     gps_time;

/*
 * SBP callback nodes must be statically allocated. Each message ID / callback
 * pair must have a unique sbp_msg_callbacks_node_t associated with it.
 */
sbp_msg_callbacks_node_t pos_llh_node;
sbp_msg_callbacks_node_t baseline_ned_node;
sbp_msg_callbacks_node_t vel_ned_node;
sbp_msg_callbacks_node_t dops_node;
sbp_msg_callbacks_node_t gps_time_node;

/*
 * Callback functions to interpret SBP messages.
 * Every message ID has a callback associated with it to
 * receive and interpret the message payload.
 */

void sbp_pos_llh_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
void sbp_baseline_ned_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
void sbp_vel_ned_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
void sbp_dops_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
void sbp_gps_time_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
void sbp_setup(void);
int initialise_uart(void);

static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

void sbp_pos_llh_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
  pos_llh = *(msg_pos_llh_t *)msg;
}
void sbp_baseline_ned_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
  baseline_ned = *(msg_baseline_ned_t *)msg;
}
void sbp_vel_ned_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
  vel_ned = *(msg_vel_ned_t *)msg;
}
void sbp_dops_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
  dops = *(msg_dops_t *)msg;
}
void sbp_gps_time_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
  gps_time = *(msg_gps_time_t *)msg;
}

/*
 * Set up SwiftNav Binary Protocol (SBP) nodes; the sbp_process function will
 * search through these to find the callback for a particular message ID.
 *
 * Example: sbp_pos_llh_callback is registered with sbp_state, and is associated
 * with both a unique sbp_msg_callbacks_node_t and the message ID SBP_POS_LLH.
 * When a valid SBP message with the ID SBP_POS_LLH comes through the UART, written
 * to the FIFO, and then parsed by sbp_process, sbp_pos_llh_callback is called
 * with the data carried by that message.
 */
void sbp_setup(void)
{
  /* SBP parser state must be initialized before sbp_process is called. */
  sbp_state_init(&sbp_state);

  /* Register a node and callback, and associate them with a specific message ID. */
  sbp_register_callback(&sbp_state, SBP_MSG_GPS_TIME, &sbp_gps_time_callback,
                        NULL, &gps_time_node);
  sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_callback,
                        NULL, &pos_llh_node);
  sbp_register_callback(&sbp_state, SBP_MSG_BASELINE_NED, &sbp_baseline_ned_callback,
                        NULL, &baseline_ned_node);
  sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &sbp_vel_ned_callback,
                        NULL, &vel_ned_node);
  sbp_register_callback(&sbp_state, SBP_MSG_DOPS, &sbp_dops_callback,
                        NULL, &dops_node);
  //sbp_register_callback(&sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL,
    //                    &heartbeat_callback_node);

}

// void set_pwm_output(mavlink_actuator_control_target_t *actuator_controls)
// {
// 	if (actuator_controls == nullptr) {
// 		// Without valid argument, set all channels to PWM_DISARMED
// 		for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
// 			int ret = ::ioctl(_pwm_fd, PWM_SERVO_SET(i), _pwm_disarmed);

// 			if (ret != OK) {
// 				PX4_ERR("PWM_SERVO_SET(%d)", i);
// 			}
// 		}

// 	} else {
// 		for (unsigned i = 0; i < sizeof(actuator_controls->controls) / sizeof(actuator_controls->controls[0]); i++) {
// 			if (!isnan(actuator_controls->controls[i])) {
// 				long unsigned pwm = actuator_controls->controls[i];
// 				int ret = ::ioctl(_pwm_fd, PWM_SERVO_SET(i), pwm);

// 				if (ret != OK) {
// 					PX4_ERR("PWM_SERVO_SET(%d)", i);
// 				}
// 			}
// 		}
// 	}
// }

int piksi_uart(int argc, char *argv[])
{
		warnx("[daemon] starting\n");

	    thread_running = true;
		char serial_buf[1];
		//mavlink_status_t serial_status = {};
		struct vehicle_gps_position_s pos;
		memset(&pos, 0, sizeof(pos));
		orb_advert_t _gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &pos);

		//int gps_topic = orb_subscribe(ORB_ID(sensor_combined));


		initialise_uart();

		//_pwm_fd = open(PWM_OUTPUT0_DEVICE_PATH, 0);
		//if (_pwm_fd < 0) {
		//	PX4_ERR("can't open %s", PWM_OUTPUT0_DEVICE_PATH);
		//	return 0;
		//}
		PX4_INFO("here1");
		// we wait for uart actuator controls messages from snapdragon
		px4_pollfd_struct_t fds[1];
		fds[0].fd = _uart_fd;
		fds[0].events = POLLIN;
		PX4_INFO("here2");
		sbp_setup();
		param_get(param_find("PWM_DISARMED"), &_pwm_disarmed);
	    //vvvv for debugging
	    //char rj[30];
	 	//char str[1000];
	  	//int str_i;
	  	u16 crc;
	  	const u8 *test_data = (u8*)"123456789";
	  	crc = crc16_ccitt(test_data, 0, 22);
	  	PX4_INFO("crc %d",crc);
	  	PX4_INFO("herecrcrc");

		while (!thread_should_exit) 
		{

			// wait for up to 100ms for data
			int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

			// timed out
			if (pret == 0) {
				// let's run the loop anyway to send RC
			}

			if (pret < 0) {
				PX4_WARN("snapdragon_rc_pwm poll error");
				// sleep a bit before next try
				usleep(100000);
				continue;
			}
			//PX4_INFO("here3");
			
			//s8 ret = sbp_process(&sbp_state, &fifo_read); used to be here
			//if (ret < 0)
	      	//	PX4_INFO("sbp_process error: %d\n", (int)ret);
			if (fds[0].revents & POLLIN) {
				int len = read(_uart_fd, serial_buf, sizeof(serial_buf));
				//printf("%02x", *cp);
				//PX4_INFO("%d", len);
				if (len > 0) {
					//mavlink_message_t msg;

					for (int i = 0; i < len; ++i) {	
						 fifo_write(serial_buf[i]);
						  sbp_process(&sbp_state, &fifo_read);
						  	//PX4_INFO("sbp_process error: %d\n", (int)ret);
							//if (mavlink_parse_char(MAVLINK_COMM_0, serial_buf[i], &msg, &serial_status)) {
							// have a message, handle it
							//handle_message(&msg);
							//PX4_INFO("fifo %02X",sbp_msg_fifo[i]);
							//PX4_INFO("buffer %02X",serial_buf[i]);
							//PX4_INFO("here3");
					}
				}
			}

		 DO_EVERY(100,
	 
		  //str_i = 0;
	      //memset(str, 0, sizeof(str));
	      pos.lat = pos_llh.lat;
	      pos.lon = pos_llh.lon;
	      pos.alt = pos_llh.height;
	      pos.vel_n_m_s = vel_ned.n;
	      pos.vel_e_m_s = vel_ned.e;
		  pos.vel_d_m_s = vel_ned.d;
		  pos.hdop = (dops.hdop/100);
		  pos.vdop = (dops.vdop/100);
	      orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &pos);
	      // all this is for debugging
	      // str_i += sprintf(str + str_i, "\n\n\n\n");

	      // /* Print GPS time. */
	      // str_i += sprintf(str + str_i, "GPS Time:\n");
	      // str_i += sprintf(str + str_i, "\tWeek\t\t: %6d\n", (int)gps_time.wn);
	      // sprintf(rj, "%6d", (gps_time.tow));
	      // str_i += sprintf(str + str_i, "\tSeconds\t: %9s\n", rj);
	      // str_i += sprintf(str + str_i, "\n");

	      // /* Print absolute position. */
	      // str_i += sprintf(str + str_i, "Absolute Position:\n");
	      // sprintf(rj, "%4.10lf", pos_llh.lat);
	      // str_i += sprintf(str + str_i, "\tLatitude\t: %17s\n", rj);
	      // sprintf(rj, "%4.10lf", pos_llh.lon);
	      // str_i += sprintf(str + str_i, "\tLongitude\t: %17s\n", rj);
	      // sprintf(rj, "%4.10lf", pos_llh.height);
	      // str_i += sprintf(str + str_i, "\tHeight\t: %17s\n", rj);
	      // str_i += sprintf(str + str_i, "\tSatellites\t:     %02d\n", pos_llh.n_sats);
	      // str_i += sprintf(str + str_i, "\n");

	      // /* Print NED (North/East/Down) baseline (position vector from base to rover). */
	      // str_i += sprintf(str + str_i, "Baseline (mm):\n");
	      // str_i += sprintf(str + str_i, "\tNorth\t\t: %6d\n", (int)baseline_ned.n);
	      // str_i += sprintf(str + str_i, "\tEast\t\t: %6d\n", (int)baseline_ned.e);
	      // str_i += sprintf(str + str_i, "\tDown\t\t: %6d\n", (int)baseline_ned.d);
	      // str_i += sprintf(str + str_i, "\n");

	      // /* Print NED velocity. */
	      // str_i += sprintf(str + str_i, "Velocity (mm/s):\n");
	      // str_i += sprintf(str + str_i, "\tNorth\t\t: %6d\n", (int)vel_ned.n);
	      // str_i += sprintf(str + str_i, "\tEast\t\t: %6d\n", (int)vel_ned.e);
	      // str_i += sprintf(str + str_i, "\tDown\t\t: %6d\n", (int)vel_ned.d);
	      // str_i += sprintf(str + str_i, "\n");

	      // /* Print Dilution of Precision metrics. */
	      // str_i += sprintf(str + str_i, "Dilution of Precision:\n");
	      // sprintf(rj, "%4d", (dops.gdop/100));
	      // str_i += sprintf(str + str_i, "\tGDOP\t\t: %7s\n", rj);
	      // sprintf(rj, "%4d", (dops.hdop/100));
	      // str_i += sprintf(str + str_i, "\tHDOP\t\t: %7s\n", rj);
	      // sprintf(rj, "%4d", (dops.pdop/100));
	      // str_i += sprintf(str + str_i, "\tPDOP\t\t: %7s\n", rj);
	      // sprintf(rj, "%4d", (dops.tdop/100));
	      // str_i += sprintf(str + str_i, "\tTDOP\t\t: %7s\n", rj);
	      // sprintf(rj, "%4d", (dops.vdop/100));
	      // str_i += sprintf(str + str_i, "\tVDOP\t\t: %7s\n", rj);
	      // str_i += sprintf(str + str_i, "\n");

	      // PX4_INFO(str);
	      );
			// check if we have new rc data, if yes send it to snapdragon
			//bool rc_updated = false;
			//orb_check(_rc_sub, &rc_updated);

			//if (rc_updated) {
			//	orb_copy(ORB_ID(input_rc), _rc_sub, &_rc);
				// send mavlink message
				//send_rc_mavlink();
			//}
		}

		deinitialize_uart();
		close(_pwm_fd);

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
	
}

int initialise_uart()
{
	// open uart
	_uart_fd = px4_open("/dev/ttyS3", O_RDWR | O_NOCTTY);
	int termios_state = -1;

	if (_uart_fd < 0) {
		PX4_ERR("failed to open uart device!");
		return -1;
	}
PX4_INFO("here0");
	// set baud rate
	int speed = 115200;
	printf("%d\n", speed);
	struct termios uart_config;
	tcgetattr(_uart_fd, &uart_config);
	// clear ONLCR flag (which appends a CR for every LF)
	//uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		warnx("ERR SET BAUD %s: %d\n", _device, termios_state);
		//::close(_uart_fd);
		return -1;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("ERR SET CONF %s\n", _device);
		px4_close(_uart_fd);
		return -1;
	}

	/* setup output flow control */
	if (enable_flow_control(false)) {
		PX4_WARN("hardware flow disable failed");
	}

	return _uart_fd;
}

int enable_flow_control(bool enabled)
{
	struct termios uart_config;

	int ret = tcgetattr(_uart_fd, &uart_config);

	if (enabled) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;
	}

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	if (!ret) {
		_flow_control_enabled = enabled;
	}

	return ret;
}

int deinitialize_uart()
{
	return close(_uart_fd);
}



int px4_simple_app_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("daemon",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 piksi_uart,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		warnx("stopping");
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;

}

