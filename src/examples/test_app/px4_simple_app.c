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
 * PIKSI <-> Pixhawk interface
 *
 * @author Example User <mail@example.com>
 */
#include "px4_simple_app.h"

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
		//orb_advert_t _gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &pos);

		initialise_uart();
		//px4_pollfd_struct_t fds[1];
		//fds[0].fd = _uart_fd;
		//fds[0].events = POLLIN;
		sbp_setup();
	    //vvvv for debugging
	    	char rj[30];
	 		char str[1000];
	  		int str_i;
	  	u16 crc;
	  	const u8 *test_data = (u8*)"123456789"; //this is just checking the crc
	  	crc = crc16_ccitt(test_data, 0, 22);
	  	PX4_INFO("crc %d",crc);
	  	PX4_INFO("herecrcrc");

		while (!thread_should_exit) 
		{
			//PX4_INFO("inwhile");

			// wait for up to 100ms for data
			// int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

			// // timed out
			// if (pret == 0) {
			// 	// let's run the loop anyway to send RC
			// 	PX4_ERR("no data within 1 sec");
			// }

			// if (pret < 0) {
			// 	PX4_WARN(" poll error");
			// 	// sleep a bit before next try
			// 	usleep(100);
			// 	continue;
			// }			
			//s8 ret = sbp_process(&sbp_state, &fifo_read); used to be here
			//s8 ret = 
			sbp_process(&sbp_state, &fifo_read);
			// if (ret < 0){
	  //     		PX4_INFO("sbp_process error: %d\n", (int)ret);
	  //     		PX4_INFO("fifo:");
	  //     		for(int i = 0; i < 512; i++)
	  //     		{
	  //     				printf("%02X ",sbp_msg_fifo[i]);
						
						
	  //     		}
	  //     		PX4_INFO("here3");
	  //     		PX4_INFO("buffer %02X",serial_buf[0]);
			// }
			if (true){//fds[0].revents & POLLIN) {
				int len = read(_uart_fd, serial_buf, sizeof(serial_buf));
				//printf("%02x", *cp);
				//PX4_INFO("%d", len);
				if (len > 0) {
					//mavlink_message_t msg;
					for (int i = 0; i < len; i++) {	
						 int test = fifo_write(serial_buf[i]);
						 if(test == 0){
						 	PX4_INFO("fifo full: %d \n",test );
						 	}

						  //sbp_process(&sbp_state, &fifo_read);
						  	//PX4_INFO("sbp_process error: %d\n", (int)ret);
							//if (mavlink_parse_char(MAVLINK_COMM_0, serial_buf[i], &msg, &serial_status)) {
							// have a message, handle it
							//handle_message(&msg);
						
					}
				}
			}

		 DO_EVERY(100, // this was 100
	 
		  str_i = 0;
	      memset(str, 0, sizeof(str));
	      pos.lat = pos_llh.lat* 1E7;  //TODO this could potentially be an issue since we are casting from double to int
	      pos.lon = pos_llh.lon* 1E7;  //TODO also not sure if this is really * 1e7 (mavlink protocol says so but maybe px4 is diffrent)
  	      pos.alt = pos_llh.height* 1000; //aslo not sure of this conversionrate
	      pos.vel_n_m_s = vel_ned.n;
	      pos.vel_e_m_s = vel_ned.e;
		  pos.vel_d_m_s = vel_ned.d;
		  pos.hdop = (dops.hdop/100);
		  pos.vdop = (dops.vdop/100);
		  //PX4_INFO("%4.10lf", pos_llh.lat);
		  //PX4_INFO("%ld",pos.lat);
	      				//orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &pos);
	      // all this is for debugging
	      str_i += sprintf(str + str_i, "\n\n\n\n");

	      /* Print GPS time. */
	      str_i += sprintf(str + str_i, "GPS Time:\n");
	      str_i += sprintf(str + str_i, "\tWeek\t\t: %6d\n", (int)gps_time.wn);
	      sprintf(rj, "%6d", (gps_time.tow));
	      str_i += sprintf(str + str_i, "\tSeconds\t: %9s\n", rj);
	      str_i += sprintf(str + str_i, "\n");

	      /* Print absolute position. */
	      str_i += sprintf(str + str_i, "Absolute Position:\n");
	      sprintf(rj, "%4.10lf", pos_llh.lat);
	      str_i += sprintf(str + str_i, "\tLatitude\t: %17s\n", rj);
	      sprintf(rj, "%4.10lf", pos_llh.lon);
	      str_i += sprintf(str + str_i, "\tLongitude\t: %17s\n", rj);
	      sprintf(rj, "%4.10lf", pos_llh.height);
	      str_i += sprintf(str + str_i, "\tHeight\t: %17s\n", rj);
	      str_i += sprintf(str + str_i, "\tSatellites\t:     %02d\n", pos_llh.n_sats);
	      str_i += sprintf(str + str_i, "\n");

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

	      PX4_INFO(str);
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



int px4_simple_app2_main(int argc, char *argv[])
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

