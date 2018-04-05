/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file fw_att_control_main.c
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler 	<thomasgubler@gmail.com>
 * @author Roman Bapst		<bapstr@ethz.ch>
 *
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <drivers/drv_hrt.h>
#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_wheel_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>
#include <vtol_att_control/vtol_type.h>

#include <vector>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/testing_data.h>

#include "K_header_file_discrete_tvlqr.cpp"



using matrix::Eulerf;
using matrix::Quatf;

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[]);

class FixedwingAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingAttitudeControl();

	/**
	 * Destructor, also kills the main task.
	 */
	~FixedwingAttitudeControl();

	/**
	 * Start the main task.
	 *
	 * @return	PX4_OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool		_task_should_exit;		/**< if true, attitude control task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle */

	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_battery_status_sub;		/**< battery status subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_global_pos_sub;		/**< global position subscription */
	int     _local_pos_sub;
	int		_manual_sub;			/**< notification of manual control updates */
	int		_params_sub;			/**< notification of parameter updates */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_2_pub;		/**< actuator control group 1 setpoint (Airframe) */
	orb_advert_t	_testing_data_pub;


	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure
	orb_id_t _attitude_setpoint_id;
	orb_id_t _testing_data_id;

	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct battery_status_s				_battery_status;	/**< battery status */
	struct control_state_s				_ctrl_state;	/**< control state */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_control_mode_s			_vcontrol_mode;		/**< vehicle control mode */
	struct vehicle_global_position_s		_global_pos;		/**< global position */
	struct vehicle_land_detected_s			_vehicle_land_detected;	/**< vehicle land detected */
	struct vehicle_rates_setpoint_s			_rates_sp;	/* attitude rates setpoint */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct vehicle_local_position_s    _local_pos;
	struct testing_data_s 			   _testing_data;



	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;				/**< if set to true, print debug output */

	float _flaps_applied;
	float _flaperons_applied;


	int         _tvlqr_task;                /**< task handle */
    int         _time_step;
    double         _t_init; //Initial time when program is started
    //int         _x_state;
    double         _x_init;
    double         _y_init;
    double         _z_init;
    double	       _q_rot[4];
    double		   _heading_init;
    double			_c_head;
    double			_s_head;
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
    int         _vehicle_attitude_sub;
    int flag;

    double  u_com[4];
    double	q_rot[4];
    double	u_com_temp[4];

    enum TVLQR_STATE {
            TVLQR_STATE_DISABLED = 0,
            TVLQR_STATE_START = 1,
            TVLQR_STATE_FINISHED = 2
        }_tvlqr_state;                    /**< flip state */

	struct {
		float p_tc;
		float p_p;
		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;
		float r_tc;
		float r_p;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_ff;
		float y_integrator_max;
		float y_coordinated_min_speed;
		float roll_to_yaw_ff;
		int32_t y_coordinated_method;
		float y_rmax;

		bool w_en;
		float w_p;
		float w_i;
		float w_ff;
		float w_integrator_max;
		float w_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float trim_roll;
		float trim_pitch;
		float trim_yaw;
		float rollsp_offset_deg;		/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;		/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;		/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;		/**< Pitch Setpoint Offset in rad */
		float man_roll_max;				/**< Max Roll in rad */
		float man_pitch_max;			/**< Max Pitch in rad */
		float man_roll_scale;			/**< scale factor applied to roll actuator control in pure manual mode */
		float man_pitch_scale;			/**< scale factor applied to pitch actuator control in pure manual mode */
		float man_yaw_scale; 			/**< scale factor applied to yaw actuator control in pure manual mode */

		float acro_max_x_rate_rad;
		float acro_max_y_rate_rad;
		float acro_max_z_rate_rad;

		float flaps_scale;				/**< Scale factor for flaps */
		float flaperon_scale;			/**< Scale factor for flaperons */

		float rattitude_thres;

		int32_t vtol_type;					/**< VTOL type: 0 = tailsitter, 1 = tiltrotor */

		int32_t bat_scale_en;			/**< Battery scaling enabled */


		int32_t tvlqr;



	}		_parameters{};			/**< local copies of interesting parameters */

	struct {

		param_t p_tc;
		param_t p_p;
		param_t p_i;
		param_t p_ff;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;
		param_t r_tc;
		param_t r_p;
		param_t r_i;
		param_t r_ff;
		param_t r_integrator_max;
		param_t r_rmax;
		param_t y_p;
		param_t y_i;
		param_t y_ff;
		param_t y_integrator_max;
		param_t y_coordinated_min_speed;
		param_t roll_to_yaw_ff;
		param_t y_coordinated_method;
		param_t y_rmax;

		param_t w_en;
		param_t w_p;
		param_t w_i;
		param_t w_ff;
		param_t w_integrator_max;
		param_t w_rmax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_roll_scale;
		param_t man_pitch_scale;
		param_t man_yaw_scale;

		param_t acro_max_x_rate;
		param_t acro_max_y_rate;
		param_t acro_max_z_rate;

		param_t flaps_scale;
		param_t flaperon_scale;

		param_t rattitude_thres;

		param_t vtol_type;

		param_t bat_scale_en;

		param_t tvlqr;

	}		_parameter_handles{};		/**< handles for interesting parameters */

	// Rotation matrix and euler angles to extract from control state
	math::Matrix<3, 3> _R;
	float _roll;
	float _pitch;
	float _yaw;

	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;
	ECL_WheelController			_wheel_ctrl;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	void		local_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle land detected updates.
	 */
	void		vehicle_land_detected_poll();

	/**
	 * Check for battery status updates.
	 */
	void		battery_status_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void		task_main();

	void 		print_data();

	double* qconjugate(double*);
    double* qmultiply(double q1[], float q2[]);
    void qrotate(double q1[], float q2[]);
    void K_deltax(double dx[], double Ki[]);

};

namespace att_control
{
FixedwingAttitudeControl	*g_control = nullptr;
}

FixedwingAttitudeControl::FixedwingAttitudeControl() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

	/* subscriptions */
	_att_sp_sub(-1),
	_battery_status_sub(-1),
	_ctrl_state_sub(-1),
	_global_pos_sub(-1),
	_local_pos_sub(-1),
	_manual_sub(-1),
	_params_sub(-1),
	_vcontrol_mode_sub(-1),
	_vehicle_land_detected_sub(-1),
	_vehicle_status_sub(-1),

	/* publications */
	_rate_sp_pub(nullptr),
	_attitude_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_actuators_2_pub(nullptr),
	_testing_data_pub(nullptr),

	_rates_sp_id(nullptr),
	_actuators_id(nullptr),
	_attitude_setpoint_id(nullptr),
	_testing_data_id(nullptr),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fwa_dt")),
#if 0
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fwa_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "fwa_nano")),
#else
	_nonfinite_input_perf(nullptr),
	_nonfinite_output_perf(nullptr),
#endif
	/* states */
	_setpoint_valid(false),
	_debug(false),
	_flaps_applied(0),
	_flaperons_applied(0),
	_roll(0.0f),
	_pitch(0.0f),
	_yaw(0.0f)
{
	/* safely initialize structs */
	_actuators = {};
	_actuators_airframe = {};
	_att_sp = {};
	_battery_status = {};
	_ctrl_state = {};
	_global_pos = {};
	_local_pos = {};
	_manual = {};
	_rates_sp = {};
	_vcontrol_mode = {};
	_vehicle_land_detected = {};
	_vehicle_status = {};
	_testing_data = {};

	_parameter_handles.p_tc = param_find("FW_P_TC");
	_parameter_handles.p_p = param_find("FW_PR_P");
	_parameter_handles.p_i = param_find("FW_PR_I");
	_parameter_handles.p_ff = param_find("FW_PR_FF");
	_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
	_parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");

	_parameter_handles.r_tc = param_find("FW_R_TC");
	_parameter_handles.r_p = param_find("FW_RR_P");
	_parameter_handles.r_i = param_find("FW_RR_I");
	_parameter_handles.r_ff = param_find("FW_RR_FF");
	_parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_p = param_find("FW_YR_P");
	_parameter_handles.y_i = param_find("FW_YR_I");
	_parameter_handles.y_ff = param_find("FW_YR_FF");
	_parameter_handles.y_integrator_max = param_find("FW_YR_IMAX");
	_parameter_handles.y_rmax = param_find("FW_Y_RMAX");
	_parameter_handles.roll_to_yaw_ff = param_find("FW_RLL_TO_YAW_FF");

	_parameter_handles.w_en = param_find("FW_W_EN");
	_parameter_handles.w_p = param_find("FW_WR_P");
	_parameter_handles.w_i = param_find("FW_WR_I");
	_parameter_handles.w_ff = param_find("FW_WR_FF");
	_parameter_handles.w_integrator_max = param_find("FW_WR_IMAX");
	_parameter_handles.w_rmax = param_find("FW_W_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.y_coordinated_min_speed = param_find("FW_YCO_VMIN");
	_parameter_handles.y_coordinated_method = param_find("FW_YCO_METHOD");

	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.man_roll_max = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max = param_find("FW_MAN_P_MAX");
	_parameter_handles.man_roll_scale = param_find("FW_MAN_R_SC");
	_parameter_handles.man_pitch_scale = param_find("FW_MAN_P_SC");
	_parameter_handles.man_yaw_scale = param_find("FW_MAN_Y_SC");

	_parameter_handles.acro_max_x_rate = param_find("FW_ACRO_X_MAX");
	_parameter_handles.acro_max_y_rate = param_find("FW_ACRO_Y_MAX");
	_parameter_handles.acro_max_z_rate = param_find("FW_ACRO_Z_MAX");

	_parameter_handles.flaps_scale = param_find("FW_FLAPS_SCL");
	_parameter_handles.flaperon_scale = param_find("FW_FLAPERON_SCL");

	_parameter_handles.rattitude_thres = param_find("FW_RATT_TH");

	_parameter_handles.vtol_type = param_find("VT_TYPE");

	_parameter_handles.bat_scale_en = param_find("FW_BAT_SCALE_EN");

	_parameter_handles.tvlqr = param_find("NAV_FT_FS");


	/* fetch initial parameter values */
	parameters_update();
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
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

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	att_control::g_control = nullptr;
}

int
FixedwingAttitudeControl::parameters_update()
{

	param_get(_parameter_handles.p_tc, &(_parameters.p_tc));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));

	param_get(_parameter_handles.r_tc, &(_parameters.r_tc));
	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_coordinated_min_speed, &(_parameters.y_coordinated_min_speed));
	param_get(_parameter_handles.y_coordinated_method, &(_parameters.y_coordinated_method));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));
	param_get(_parameter_handles.roll_to_yaw_ff, &(_parameters.roll_to_yaw_ff));

	int32_t wheel_enabled = 0;
	param_get(_parameter_handles.w_en, &wheel_enabled);
	_parameters.w_en = (wheel_enabled == 1);

	param_get(_parameter_handles.w_p, &(_parameters.w_p));
	param_get(_parameter_handles.w_i, &(_parameters.w_i));
	param_get(_parameter_handles.w_ff, &(_parameters.w_ff));
	param_get(_parameter_handles.w_integrator_max, &(_parameters.w_integrator_max));
	param_get(_parameter_handles.w_rmax, &(_parameters.w_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.rollsp_offset_deg, &(_parameters.rollsp_offset_deg));
	param_get(_parameter_handles.pitchsp_offset_deg, &(_parameters.pitchsp_offset_deg));
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_deg);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_deg);
	param_get(_parameter_handles.man_roll_max, &(_parameters.man_roll_max));
	param_get(_parameter_handles.man_pitch_max, &(_parameters.man_pitch_max));
	_parameters.man_roll_max = math::radians(_parameters.man_roll_max);
	_parameters.man_pitch_max = math::radians(_parameters.man_pitch_max);
	param_get(_parameter_handles.man_roll_scale, &(_parameters.man_roll_scale));
	param_get(_parameter_handles.man_pitch_scale, &(_parameters.man_pitch_scale));
	param_get(_parameter_handles.man_yaw_scale, &(_parameters.man_yaw_scale));

	param_get(_parameter_handles.acro_max_x_rate, &(_parameters.acro_max_x_rate_rad));
	param_get(_parameter_handles.acro_max_y_rate, &(_parameters.acro_max_y_rate_rad));
	param_get(_parameter_handles.acro_max_z_rate, &(_parameters.acro_max_z_rate_rad));
	_parameters.acro_max_x_rate_rad = math::radians(_parameters.acro_max_x_rate_rad);
	_parameters.acro_max_y_rate_rad = math::radians(_parameters.acro_max_y_rate_rad);
	_parameters.acro_max_z_rate_rad = math::radians(_parameters.acro_max_z_rate_rad);

	param_get(_parameter_handles.flaps_scale, &_parameters.flaps_scale);
	param_get(_parameter_handles.flaperon_scale, &_parameters.flaperon_scale);

	param_get(_parameter_handles.rattitude_thres, &_parameters.rattitude_thres);

	param_get(_parameter_handles.vtol_type, &_parameters.vtol_type);

	param_get(_parameter_handles.bat_scale_en, &_parameters.bat_scale_en);

	param_get(_parameter_handles.tvlqr, &(_parameters.tvlqr));


	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.p_tc);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);
	_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
	_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.r_tc);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);
	_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);
	_yaw_ctrl.set_coordinated_min_speed(_parameters.y_coordinated_min_speed);
	_yaw_ctrl.set_coordinated_method(_parameters.y_coordinated_method);
	_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

	/* wheel control parameters */
	_wheel_ctrl.set_k_p(_parameters.w_p);
	_wheel_ctrl.set_k_i(_parameters.w_i);
	_wheel_ctrl.set_k_ff(_parameters.w_ff);
	_wheel_ctrl.set_integrator_max(_parameters.w_integrator_max);
	_wheel_ctrl.set_max_rate(math::radians(_parameters.w_rmax));

	return PX4_OK;
}

void
FixedwingAttitudeControl::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
FixedwingAttitudeControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = true;
	}
}

void
FixedwingAttitudeControl::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
FixedwingAttitudeControl::local_pos_poll()
{
	/* check if there is a new global position */
	bool local_pos_updated;
	orb_check(_local_pos_sub, &local_pos_updated);

	if (local_pos_updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
}

void
FixedwingAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		_testing_data_id = ORB_ID(testing_data);
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(fw_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_land_detected_poll()
{
	/* check if there is new status information */
	bool vehicle_land_detected_updated;
	orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

	if (vehicle_land_detected_updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}
}

void
FixedwingAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}



void
FixedwingAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	att_control::g_control->task_main();
}

double* FixedwingAttitudeControl::qconjugate(double q[4])
{
	static double q_conj[4] = {0,0,0,0};
q_conj[0] = q[0];	
q_conj[1] = -q[1];
q_conj[2] = -q[2];
q_conj[3] = -q[3];

return q_conj;
}

double* FixedwingAttitudeControl::qmultiply(double q1[4],  float q2[4])
{

 	static double qresult[4] = {0,0,0,0}; //= {q1[0]*q2[0]-q1[1]*q2[2]*q1[1]};
 	qresult[0] = (q1[0] * (double)q2[0]- q1[1] * (double)q2[1] - q1[2] * (double)q2[2] - q1[3] * (double)q2[3]);
	qresult[1] = (q1[0] * (double)q2[1] + q1[1] * (double)q2[0] + q1[2] * (double)q2[3] - q1[3] * (double)q2[2]);
	qresult[2] = (q1[0] * (double)q2[2] + q1[2] * (double)q2[0] - q1[1] * (double)q2[3] + q1[3] * (double)q2[1]);
	qresult[3] = (q1[0] * (double)q2[3] + q1[1]	* (double)q2[2] - q1[2] * (double)q2[1] + q1[3] * (double)q2[0]);

	//warnx("q1 is {%lf,%lf,%lf,%lf} q2 is {%lf,%lf,%lf,%lf} qres is {%lf,%lf,%lf,%lf}",
    //q1[0],q1[1],q1[2],q1[3],q2[0],q2[1],q2[2],q2[3],qresult[0],qresult[1],qresult[2],qresult[3]);

	return qresult;
}

void FixedwingAttitudeControl::qrotate(double q1[4],  float q2[4])
{

 	//static double qresult[4] = {0,0,0,0}; //= {q1[0]*q2[0]-q1[1]*q2[2]*q1[1]};
 	q_rot[0] = (q1[0] * (double)q2[0]- q1[1] * (double)q2[1] - q1[2] * (double)q2[2] - q1[3] * (double)q2[3]);
	q_rot[1] = (q1[0] * (double)q2[1] + q1[1] * (double)q2[0] + q1[2] * (double)q2[3] - q1[3] * (double)q2[2]);
	q_rot[2] = (q1[0] * (double)q2[2] + q1[2] * (double)q2[0] - q1[1] * (double)q2[3] + q1[3] * (double)q2[1]);
	q_rot[3] = (q1[0] * (double)q2[3] + q1[1]	* (double)q2[2] - q1[2] * (double)q2[1] + q1[3] * (double)q2[0]);

	//warnx("q1 is {%lf,%lf,%lf,%lf} q2 is {%lf,%lf,%lf,%lf} qres is {%lf,%lf,%lf,%lf}",
    //q1[0],q1[1],q1[2],q1[3],q2[0],q2[1],q2[2],q2[3],q_rot[0],q_rot[1],q_rot[2],q_rot[3]);

	//return qresult;
}

void FixedwingAttitudeControl::K_deltax(double dx[12],  double Ki[48])
{

       //static double u_com1[4] = {0,0,0,0}; //= {q1[0]*q2[0]-q1[1]*q2[2]*q1[1]};

        for (uint8_t i = 0; i<=3; i++) {

        u_com_temp[i] = dx[0]*Ki[i] + dx[1]*Ki[i+4] + dx[2]*Ki[i+8] + dx[3]*Ki[i+12] +
                    dx[4]*Ki[i+16] + dx[5]*Ki[i+20] + dx[6]*Ki[i+24] + dx[7]*Ki[i+28] +
                    dx[8]*Ki[i+32] + dx[9]*Ki[i+36] + dx[10]*Ki[i+40] + dx[11]*Ki[i+44];

        }
        //warnx("dx is {%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf}",
        //dx[0],dx[1],dx[2],dx[3],dx[4],dx[5],dx[6],dx[7],dx[8],dx[9],dx[10],dx[11]);
        //return u_com1;
}

void FixedwingAttitudeControl::print_data()
{
    warnx("Current x is {%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf},\n u_com is {%lf,%lf,%lf,%lf}\n t_init is %lf",
        _x_cur1,_x_cur2,_x_cur3,_x_cur4,_x_cur5,_x_cur6,_x_cur7,_x_cur8,_x_cur9,_x_cur10,_x_cur11,_x_cur12,_x_cur13,
        _u_com1,_u_com2,_u_com3,_u_com4,_t_init);
}

void
FixedwingAttitudeControl::task_main()
{
	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
    _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_setpoint_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	vehicle_status_poll();
	vehicle_land_detected_poll();
	battery_status_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[3];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _ctrl_state_sub;
	fds[1].events = POLLIN;
	 fds[2].fd = _vehicle_attitude_sub;
    fds[2].events = POLLIN;

	_task_running = true;
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	//struct vehicle_local_position_s    _local_position;

	//memset(&_local_position, 0, sizeof(_local_position));

	bool updated = false;
	const unsigned sleeptime_us = 50000;
	_tvlqr_state = TVLQR_STATE_DISABLED;
	 flag = 0;
	while (!_task_should_exit) {

		
		static int loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);


			/* get current rotation matrix and euler angles from control state quaternions */
			math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
			_R = q_att.to_dcm();

			math::Vector<3> euler_angles;
			euler_angles = _R.to_euler();
			_roll    = euler_angles(0);
			_pitch   = euler_angles(1);
			_yaw     = euler_angles(2);

			if (_vehicle_status.is_vtol && _parameters.vtol_type == vtol_type::TAILSITTER) {
				/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
				 *
				 * Since the VTOL airframe is initialized as a multicopter we need to
				 * modify the estimated attitude for the fixed wing operation.
				 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
				 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
				 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
				 * Additionally, in order to get the correct sign of the pitch, we need to multiply
				 * the new x axis of the rotation matrix with -1
				 *
				 * original:			modified:
				 *
				 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
				 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
				 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
				 * */
				math::Matrix<3, 3> R_adapted = _R;		//modified rotation matrix

				/* move z to x */
				R_adapted(0, 0) = _R(0, 2);
				R_adapted(1, 0) = _R(1, 2);
				R_adapted(2, 0) = _R(2, 2);

				/* move x to z */
				R_adapted(0, 2) = _R(0, 0);
				R_adapted(1, 2) = _R(1, 0);
				R_adapted(2, 2) = _R(2, 0);

				/* change direction of pitch (convert to right handed system) */
				R_adapted(0, 0) = -R_adapted(0, 0);
				R_adapted(1, 0) = -R_adapted(1, 0);
				R_adapted(2, 0) = -R_adapted(2, 0);
				euler_angles = R_adapted.to_euler();  //adapted euler angles for fixed wing operation

				/* fill in new attitude data */
				_R = R_adapted;
				_roll    = euler_angles(0);
				_pitch   = euler_angles(1);
				_yaw     = euler_angles(2);

				/* lastly, roll- and yawspeed have to be swaped */
				float helper = _ctrl_state.roll_rate;
				_ctrl_state.roll_rate = -_ctrl_state.yaw_rate;
				_ctrl_state.yaw_rate = helper;
			}

			vehicle_setpoint_poll();

			vehicle_control_mode_poll();

			vehicle_manual_poll();

			global_pos_poll();

			local_pos_poll();

			vehicle_status_poll();

			vehicle_land_detected_poll();

			battery_status_poll();

			// the position controller will not emit attitude setpoints in some modes
			// we need to make sure that this flag is reset
			_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;

			/* lock integrator until control is started */
			bool lock_integrator = !(_vcontrol_mode.flag_control_rates_enabled && !_vehicle_status.is_rotary_wing);

			/* Simple handling of failsafe: deploy parachute if failsafe is on */
			if (_vcontrol_mode.flag_control_termination_enabled) {
				_actuators_airframe.control[7] = 1.0f;
				//warnx("_actuators_airframe.control[1] = 1.0f;");

			} else {
				_actuators_airframe.control[7] = 0.0f;
				//warnx("_actuators_airframe.control[1] = -1.0f;");
			}

			/* if we are in rotary wing mode, do nothing */
			if (_vehicle_status.is_rotary_wing && !_vehicle_status.is_vtol) {
				continue;
			}

			/* default flaps to center */
			float flap_control = 0.0f;

			/* map flaps by default to manual if valid */
			if (PX4_ISFINITE(_manual.flaps) && _vcontrol_mode.flag_control_manual_enabled
			    && fabsf(_parameters.flaps_scale) > 0.01f) {
				flap_control = 0.5f * (_manual.flaps + 1.0f) * _parameters.flaps_scale;

			} else if (_vcontrol_mode.flag_control_auto_enabled
				   && fabsf(_parameters.flaps_scale) > 0.01f) {
				flap_control = _att_sp.apply_flaps ? 1.0f * _parameters.flaps_scale : 0.0f;
			}

			// move the actual control value continuous with time, full flap travel in 1sec
			if (fabsf(_flaps_applied - flap_control) > 0.01f) {
				_flaps_applied += (_flaps_applied - flap_control) < 0 ? deltaT : -deltaT;

			} else {
				_flaps_applied = flap_control;
			}

			/* default flaperon to center */
			float flaperon_control = 0.0f;

			/* map flaperons by default to manual if valid */
			if (PX4_ISFINITE(_manual.aux2) && _vcontrol_mode.flag_control_manual_enabled
			    && fabsf(_parameters.flaperon_scale) > 0.01f) {
				flaperon_control = 0.5f * (_manual.aux2 + 1.0f) * _parameters.flaperon_scale;

			} else if (_vcontrol_mode.flag_control_auto_enabled
				   && fabsf(_parameters.flaperon_scale) > 0.01f) {
				flaperon_control = _att_sp.apply_flaps ? 1.0f * _parameters.flaperon_scale : 0.0f;
			}

			// move the actual control value continuous with time, full flap travel in 1sec
			if (fabsf(_flaperons_applied - flaperon_control) > 0.01f) {
				_flaperons_applied += (_flaperons_applied - flaperon_control) < 0 ? deltaT : -deltaT;

			} else {
				_flaperons_applied = flaperon_control;
			}

			// Check if we are in rattitude mode and the pilot is above the threshold on pitch
			if (_vcontrol_mode.flag_control_rattitude_enabled) { //got rid of this temporarily using ratitude to trigger tvlqr
				if (fabsf(_manual.y) > _parameters.rattitude_thres ||
				    fabsf(_manual.x) > _parameters.rattitude_thres) {
					_vcontrol_mode.flag_control_attitude_enabled = false;
				}
			}


			 orb_check(_vehicle_attitude_sub, &updated);

           // orb_publish(ORB_ID(vehicle_rates_setpoint), rates_pub, &rates_sp);

            if (updated) {
                orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &att)	;
            }
            //warnx("here000");
            //warnx("%d control ", _vcontrol_mode.flag_control_rates_enabled);
			if(_parameters.tvlqr == 2)// _vcontrol_mode.flag_control_rattitude_enabled )//_parameters.tvlqr == 2) //_vcontrol_mode.flag_control_rates_enabled &&
			{
				
				
				//warnx("here111");
				switch (_tvlqr_state) {

		             case TVLQR_STATE_DISABLED:
		            //     // shoudn't even enter this but just in case
		            //     // do nothing
		            
		             warnx("disabled");
		             flag = 0;
		             _tvlqr_state = TVLQR_STATE_START;
		            // warnx("%lf", ((double)actuators.control[0]));
		             break;
		             case TVLQR_STATE_START:
		             {
		             	//warnx("here");
		             	if(flag == 0){
		             		 _t_init = (double) hrt_absolute_time();
		             		 warnx("%lf",_t_init);

		             		 //float test_heading_init = atan2(2*(att.q[0]*att.q[3]+att.q[1]*att.q[2]),1-2*(att.q[0]*att.q[0]+att.q[3]*att.q[3]));
		             		 //_heading_init = atan2(2*(-1*att.q[1]*att.q[2]-1*att.q[0]*att.q[3]),1-2*(att.q[3]*att.q[3]+att.q[2]*att.q[2]));
		             		 _heading_init = -1*_heading_init;
		             		 //float test_heading_init = atan2(2*.707*.707,1-2*(.707*.707));
		             		_heading_init = _local_pos.yaw; 
		             		_c_head = cos(-1*_heading_init);
		             		_s_head = sin(-1*_heading_init);
		             	    _x_init = (double) _local_pos.x;
		             		_y_init = (double) _local_pos.y;
		             		//_x_init = x0[0][_time_step]*_c_head+ -1*(double) _local_pos.x;
		             		//_y_init = x0[0][_time_step]*_s_head+ -1*(double) _local_pos.y;
		             		_z_init = (double) _local_pos.z;
		             		_q_rot[0] = cos(-.5*_heading_init);
		             		_q_rot[1] = 0;
		             		_q_rot[2] = 0;
		             		_q_rot[3] = sin(-.5*_heading_init);
		             		//vec_des[0] = 0;
		             		//vec_des[1] = 0;
		             		//warnx("Heading(Yaw) %lf, Heading(quatz) %lf",_heading_init,test_heading_init);
		             		//warnx("%lf %lf %lf %lf",_q_rot[0],_q_rot[1],_q_rot[2],_q_rot[3]);
	
		             		flag = 1;
		             	}
		                /*
		                 * 400 degree/second roll to 45 degrees
		            //      */
		             	att.timestamp = (double) hrt_absolute_time(); // _att.timestamp;
		             	//float q[4] = {-1*att.q[1],att.q[0],-1*att.q[3],1*att.q[2]};
		             	float q[4] = {-1*att.q[1],att.q[0],-1*att.q[3],att.q[2]};
		                //float q[4] = {-1*att.q[2],-1*att.q[3],-1*att.q[0],-1*att.q[1]};
		               // warnx("%lf", ((double)_actuators.control[0]));
		        		// warnx("runnn");
		                double delta_x[12];
		                //warnx("attitude timestamp = %d \n t_init %d \n T_end %lf",att.timestamp-_t_init,_t_init,t[(sizeof(t)/sizeof(t[0]))-1]*pow(10,6));
		                if(att.timestamp-_t_init >= t[(sizeof(t)/sizeof(t[0]))-1]*pow(10,6)){
		                     _tvlqr_state = TVLQR_STATE_FINISHED;
		                     break;
		                }
		                   //	warnx("Timestep %lf %lf %lf",att.timestamp-_t_init,t[_time_step + 1]*pow(10,6),t[_time_step + 2]*pow(10,6));
		                while((att.timestamp-_t_init > t[_time_step ]*pow(10,6))&&(att.timestamp-_t_init > t[_time_step + 1]*pow(10,6)))
		                {
		                    ++_time_step;
		                	//warnx("Timestep %d",_time_step);
		                }



		         		//float q0[4] = {-1*(float)x0[4][_time_step],(float) x0[3][_time_step], -1*(float)x0[6][_time_step],(float) x0[5][_time_step]};
		         		float q0[4] = {(float)x0[3][_time_step],(float) x0[4][_time_step], (float)x0[5][_time_step],(float) x0[6][_time_step]};
		         		//double *q0_rotated[4];
		         		//q0_rotated = qmultiply(_q_rot,q0,q0_rotated);
		         		qrotate(_q_rot,q0);
		         		//     warnx("x is {%lf %lf %lf} xd is {%lf %lf %lf} q is {%lf,%lf,%lf,%lf}  qd is {%lf,%lf,%lf,%lf}",
    					//	_local_pos.x+_x_init,_local_pos.y+_y_init,_local_pos.z+_z_init,
    					//	((x0[0][_time_step])*_c_head),((x0[0][_time_step])*_s_head),x0[2][_time_step],
    					//	 q[0],q[1],q[2],q[3],q_rot[0],q_rot[1],q_rot[2],q_rot[3]);


    					    // warnx("v is {%lf %lf %lf} vd is {%lf %lf %lf}",
    						//_local_pos.vx,_local_pos.vy,_local_pos.vz,
    						//((x0[7][_time_step])*_c_head),((x0[7][_time_step])*_s_head),x0[9][_time_step]);

    						//warnx("\n[%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf]",
    						//_local_pos.x+_x_init,_local_pos.y+_y_init,_local_pos.z+_z_init,
    						//q[0],q[1],q[2],q[3],
    						//((x0[0][_time_step])*_c_head),((x0[0][_time_step])*_s_head),x0[2][_time_step],
    						 //q_rot[0],q_rot[1],q_rot[2],q_rot[3]);
		                
		                double *q0con;
		                //q0con = qconjugate(q0_rotated);
		                q0con = qconjugate(q_rot);	            
		                //warnx("q0_rotated: %lf %lf %lf %lf",q_rot[0],q_rot[1],q_rot[2],q_rot[3]);
		                //qconjugate(q0);
		                //PASS BY REFERENCE INSTEAD!!! THIS IS SCREWED UP!!!
		        		double *qdiff;
		        		qdiff = qmultiply(q0con,q);
		        		//warnx("q0_rotated: %lf %lf %lf %lf",q_rot[0],q_rot[1],q_rot[2],q_rot[3]);
		        		//warnx("qdiff: %lf %lf %lf %lf",qdiff[0],qdiff[1],qdiff[2],qdiff[3]);


		        			//warnx("updated xd is %lf , %lf , %;f",((x0[0][_time_step])*_c_head),((x0[0][_time_step])*_s_head),x0[2][_time_step]);
		        		double x_des_interp = x0[0][_time_step]+
		        				(att.timestamp-_t_init-t[_time_step]*pow(10,6))*
		        				(x0[0][_time_step+1]-x0[0][_time_step])/((t[_time_step+1]*pow(10,6)-t[_time_step]*pow(10,6)));
		                
		        		//warnx("%lf",x_des_interp);		
		                //delta_x[0] = {(double)_local_pos.x-((x0[0][_time_step])*_c_head)+_x_init};
		                
		        	    double vec_des[2] = {(x_des_interp-x0[0][0])*_c_head,(x_des_interp-x0[0][0])*_s_head};
		        	    double vec_meas[2] = {-1*_x_init+(double)_local_pos.x,-1*_y_init+(double) _local_pos.y};
		        	    double norm_vecd = sqrt(vec_des[0]*vec_des[0]+vec_des[1]*vec_des[1]);
		        	    double norm_vecm = sqrt(vec_meas[0]*vec_meas[0]+vec_meas[1]*vec_meas[1]);
		        	    double unit_vecd[2] = {vec_des[0]/norm_vecd,vec_des[1]/norm_vecd};
		        	    //double unit_vecm[2] = {vec_meas[0]/norm_vecm,vec_meas[1]/norm_vecm};
		        	    double proj_meas_des = vec_meas[0]*(vec_des[0]/norm_vecd)+vec_meas[1]*(vec_des[1]/norm_vecd);
		        	    //double del_x =  proj_meas_des-(x_des_interp-x0[0][0]);		        	    
		        	    double del_x =  proj_meas_des-norm_vecd;
		        	    double del_y =  sqrt(norm_vecm*norm_vecm-proj_meas_des*proj_meas_des);
		        	    double del_y_vec[2] = {vec_meas[0]-proj_meas_des*unit_vecd[0],vec_meas[1]-proj_meas_des*unit_vecd[1]};
		        	    //double del_y = sqrt(del_y_vec)
		        	    del_y = -1*del_y;
		        	    if(unit_vecd[0] > 0) //+
		        	    {
		        	    	if(unit_vecd[1] > 0) //++
		        	    	{
		        	    		if(del_y_vec[0] < 0)
		        	    		{
		        	    			del_y = -1*del_y;
		        	    		}
		        	    	}
		        	    	else //+-
		        	    	{
		        	    		if(del_y_vec[0] > 0)
		        	    		{
		        	    			del_y = -1*del_y;
		        	    		}
		        	    	}
		        	    }
		        	    else //-
		        	    {
		        	    	if(unit_vecd[1] < 0) //--
		        	    	{
		        	    		if(del_y_vec[0] > 0)
		        	    		{
		        	    			del_y = -1*del_y;
		        	    		}
		        	    	}
		        	    	else //-+
		        	    	{
		        	    		if(del_y_vec[0] < 0)
		        	    		{
		        	    			del_y = -1*del_y;
		        	    		}
		        	    	}
		        	    }
		        	    //double theta_meas = atan2(2*(-1*att.q[1]*att.q[2]-1*att.q[0]*att.q[3]),1-2*(att.q[3]*att.q[3]+att.q[2]*att.q[2]));
		        	    //if (theta_meas > _heading_init) del_y = -1*del_y;


		        	    warnx("del_x {%lf} del_y{%lf}",del_x,del_y);


		                //delta_x[0] = {(double)_local_pos.x-(x_des_interp*_c_head)+_x_init};
		                //warnx("xd = %lf",-(x0[0][_time_step]));
		                //delta_x[1] = {(double)_local_pos.y-((x0[0][_time_step])*_s_head)+_y_init};
		                //delta_x[1] = {(double)_local_pos.y-(x_des_interp*_s_head)+_y_init};
		                delta_x[0] = del_x;
		                delta_x[1] = del_y;
		                delta_x[2] = {(double)_local_pos.z-x0[2][_time_step]+x0[2][0]-_z_init};


		                //delta_x[3] = {(double)qdiff[0]};
		                delta_x[3] = {(double)qdiff[1]};
		                delta_x[4] = {(double)qdiff[2]};
		                delta_x[5] = {(double)qdiff[3]};
		                //warnx("qdiff_after: %lf %lf %lf ",delta_x[3],delta_x[4],delta_x[5]);


		                delta_x[6] = {(double)_local_pos.vx-(x0[7][_time_step]*_c_head)};
		                delta_x[7] = {(double)_local_pos.vy-(x0[7][_time_step]*_s_head)};
		                delta_x[8] = {(double)_local_pos.vz-x0[9][_time_step]};

		                //warnx("sine is %lf cos is %lf",_s_head,_c_head);

		                delta_x[9] = {(double)att.rollspeed-x0[10][_time_step]};
		                delta_x[10] = {(double)att.pitchspeed-x0[11][_time_step]};
		                delta_x[11] = {(double)att.yawspeed-x0[12][_time_step]};


		                //delta_x[0] = 0;
		                //delta_x[1] = 0;
		                //delta_x[2] = 0;
		                //delta_x[3] = 0;
		                //delta_x[4] = 0;
		                //delta_x[5] = 0;
		               //delta_x[3] = -1*delta_x[3];
		                //delta_x[4] = -1*delta_x[4];;
		                //delta_x[5] = -1*delta_x[5];;
		                delta_x[6] = 0;
		                delta_x[7] = 0;
		                delta_x[8] = 0;
		                //delta_x[9] = 0;
		                //delta_x[10] = 0;
		                //delta_x[11] = 0;


				                double Ki[48];
				                for(uint8_t i = 0; i < 48; ++i)				               
				                {
				                   //Ki[i] = K0[i];
				                   Ki[i] = K[i][_time_step];
				                	//Ki[i] = K[i][100];
				                }
		                //u = -K*deltax+u0
			                //double *u_com_temp;
			                //u_com_temp = K_deltax(delta_x,  Ki);
				                K_deltax(delta_x,  Ki);


			                u_com[0] = -1*u_com_temp[0] + u0[0][_time_step];
			                u_com[1] = -1*u_com_temp[1] + u0[1][_time_step];
			                u_com[2] = -1*u_com_temp[2] + u0[2][_time_step];
			                u_com[3] = -1*u_com_temp[3] + u0[3][_time_step];		               

		                 //u_com[0] = (double) .6*sin(att.timestamp);  // throttle yaw
		                 //u_com[1] = (double) .6*sin(att.timestamp);  // roll
		                 //u_com[2] = (double) .6*sin(att.timestamp);  // pitch
		                 //u_com[3] = (double) .6*sin(att.timestamp); //yaw
		                 

		                //warnx("ucom 0: %lf ucom 1: %lf ucom 2: %lf ucom 3: %lf", u_com[0], u_com[1], u_com[2], u_com[3]);
			            //warnx("dx: %lf dy: %lf dz: %lf dq0: %lf dq1: %lf dq2: %lf dq3: %lf dxd: %lf dyd: %lf dzd: %lf dwx: %lf dwy: %lf dwz: %lf",
			            //delta_x[0],delta_x[1],delta_x[2],qdiff[0],delta_x[3],delta_x[4],delta_x[5],delta_x[6],delta_x[7],delta_x[8],delta_x[9],delta_x[10],delta_x[10]);
		                //warnx("q0: %lf q1: %lf q2: %lf q3: %lf \n qd0: %lf  qd1: %lf qd2: %lf qd3: %lf",q[0],q[1],q[2],q[3],q0_rotated[0],q0_rotated[1],q0_rotated[2],q0_rotated[3]);
		                //print_data();

						//_actuators.timestamp = hrt_absolute_time();
		                //PX4_INFO("All the way at 530!");
		              //  _publish_actuators(u_com);
		                 	//control_attitude(&_att_sp, &att, &actuators,  u_com);
		                 	


		     //             	if (PX4_ISFINITE(actuators.control[0]) &&
						 //    PX4_ISFINITE(actuators.control[1]) &&
						 //    PX4_ISFINITE(actuators.control[2]) &&
						 //    PX4_ISFINITE(actuators.control[3])) {
							// //orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
							// 	warnx("published");
							// }



					                //For printing and debugging
					                _u_com1 = u_com[0];
					                _u_com2 = u_com[1];
					                _u_com3 = u_com[2];
					                _u_com4 = u_com[3];

		                             _x_cur1 = _local_pos.x+_x_init;
		                             _x_cur2 = _local_pos.y+_y_init;
		                             _x_cur3 = _local_pos.z+_z_init;
		                             _x_cur4 = -1*att.q[1];
		                             _x_cur5 = att.q[0];
		                             _x_cur6 = -1*att.q[3];
		                             _x_cur7 = att.q[2];
		                             _x_cur8 = delta_x[7];
		                             _x_cur9 = delta_x[8];
		                             _x_cur10 = delta_x[9];
		                             _x_cur11 = delta_x[10];	
		                             _x_cur12 = delta_x[11];

		                             print_data();


		                             _testing_data.x_des[0] = vec_des[0];
		                             _testing_data.x_des[1] = vec_des[1];
		                             _testing_data.x_des[2] = x0[2][_time_step]-x0[2][0]+_z_init;
		                             _testing_data.x_des[3] = q_rot[0];
		                             _testing_data.x_des[4] = q_rot[1];
		                             _testing_data.x_des[5] = q_rot[2];
		                             _testing_data.x_des[6] = q_rot[3];
		                             _testing_data.x_des[7] = x0[7][_time_step]*_c_head;
		                             _testing_data.x_des[8] = x0[7][_time_step]*_s_head;
		                             _testing_data.x_des[9] = x0[9][_time_step];
		                             _testing_data.x_des[10] = x0[10][_time_step];
		                             _testing_data.x_des[11] = x0[11][_time_step];
		                             _testing_data.x_des[12] = x0[12][_time_step];

		                             _testing_data.x_meas[0] = vec_meas[0];
		                             _testing_data.x_meas[1] = vec_meas[1];
		                             _testing_data.x_meas[2] = _local_pos.z;
		                             _testing_data.x_meas[3] = -1*att.q[1];
		                             _testing_data.x_meas[4] = att.q[0];
		                             _testing_data.x_meas[5] = -1*att.q[3];
		                             _testing_data.x_meas[6] = att.q[2];
		                             _testing_data.x_meas[7] = _local_pos.vx;
		                             _testing_data.x_meas[8] = _local_pos.vy;
		                             _testing_data.x_meas[9] = _local_pos.vz;
		                             _testing_data.x_meas[10] = att.rollspeed;
		                             _testing_data.x_meas[11] = att.pitchspeed;
		                             _testing_data.x_meas[12] = att.yawspeed;

		                             _testing_data.u_comm[0] = u_com[0];
		                             _testing_data.u_comm[1] = u_com[1];
		                             _testing_data.u_comm[2] = u_com[2];
		                             _testing_data.u_comm[3] = u_com[3];


		                             _testing_data.del_x[0] = del_x;
		                             _testing_data.del_x[1] = del_y;
		                             _testing_data.del_x[2] = (double)_local_pos.z-x0[2][_time_step]+x0[2][0]-_z_init;
		                             _testing_data.del_x[3] = delta_x[3];
		                             _testing_data.del_x[4] = delta_x[4];
		                             _testing_data.del_x[5] = delta_x[5];
		                             _testing_data.del_x[6] = delta_x[6];
		                             _testing_data.del_x[7] = delta_x[7];
		                             _testing_data.del_x[8] = delta_x[8];
		                             _testing_data.del_x[9] = delta_x[9];
		                             _testing_data.del_x[10] = delta_x[10];
		                             _testing_data.del_x[11] = delta_x[11];

		                             _testing_data.K_del_x[0] = u_com_temp[0];
		                             _testing_data.K_del_x[1] = u_com_temp[1];
		                             _testing_data.K_del_x[2] = u_com_temp[2];
		                             _testing_data.K_del_x[3] = u_com_temp[3];

		                             _testing_data.x_init[0] = _x_init;
		                             _testing_data.x_init[1] = _y_init;
		                             _testing_data.x_init[2] = _z_init;
		                             _testing_data.x_init[3] = _heading_init;

		                             _testing_data.times = att.timestamp;


		                             if (_testing_data_pub != nullptr) {
					
		
										orb_publish(_testing_data_id, _testing_data_pub, &_testing_data);

									} else if (_testing_data_id) {
										_testing_data_pub = orb_advertise(_testing_data_id, &_testing_data);
									}


		                             //orb_publish(ORB_ID(testing_data), _testing_data_pub, &_testing_data);


		                              //print_data();

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
		             case TVLQR_STATE_FINISHED:
		                 /*
		                  * go back to disabled state
		                  */
		            // warnx("I finished a loop");
		            //     // enable manual control and attitude control
		                 //_vehicle_control_mode.flag_control_manual_enabled = true;
		                // _vehicle_control_mode.flag_control_attitude_enabled = true;
		                // orb_publish(ORB_ID(vehicle_control_mode), _vehicle_control_mode_pub, &_vehicle_control_mode);
		            //     // switch back to disabled flip state
		                 _tvlqr_state = TVLQR_STATE_DISABLED;
		                 _time_step = 0;
		                 flag = 0;
		             break;
		             }
		            // run at roughly 100 hz
		            usleep(sleeptime_us);

			}
			/* decide if in stabilized or full manual control */
			if (_vcontrol_mode.flag_control_rates_enabled || _parameters.tvlqr != 2) {//_parameters.tvlqr != 2) {
				/* scale around tuning airspeed */
				float airspeed;
				//	warnx("tvlqr : %d ", _parameters.tvlqr);
				bool nonfinite = !PX4_ISFINITE(_ctrl_state.airspeed);

				//warnx("here4, _parameters.tvlqr: %d", _parameters.tvlqr);

				/* if airspeed is non-finite or not valid or if we are asked not to control it, we assume the normal average speed */
				if (nonfinite || !_ctrl_state.airspeed_valid) {
					airspeed = _parameters.airspeed_trim;

					if (nonfinite) {
						perf_count(_nonfinite_input_perf);
					}

				} else {
					/* prevent numerical drama by requiring 0.5 m/s minimal speed */
					airspeed = math::max(0.5f, _ctrl_state.airspeed);
				}

				/*
				 * For scaling our actuators using anything less than the min (close to stall)
				 * speed doesn't make any sense - its the strongest reasonable deflection we
				 * want to do in flight and its the baseline a human pilot would choose.
				 *
				 * Forcing the scaling to this value allows reasonable handheld tests.
				 */
				float airspeed_scaling = _parameters.airspeed_trim / ((airspeed < _parameters.airspeed_min) ? _parameters.airspeed_min :
							 airspeed);

				/* Use min airspeed to calculate ground speed scaling region.
				 * Don't scale below gspd_scaling_trim
				 */
				float groundspeed = sqrtf(_global_pos.vel_n * _global_pos.vel_n +
							  _global_pos.vel_e * _global_pos.vel_e);
				float gspd_scaling_trim = (_parameters.airspeed_min * 0.6f);
				float groundspeed_scaler = gspd_scaling_trim / ((groundspeed < gspd_scaling_trim) ? gspd_scaling_trim : groundspeed);

				// in STABILIZED mode we need to generate the attitude setpoint
				// from manual user inputs
				if (!_vcontrol_mode.flag_control_climb_rate_enabled && !_vcontrol_mode.flag_control_offboard_enabled && _parameters.tvlqr != 2){ // _parameters.tvlqr != 2) {

					//warnx("here5");
					_att_sp.roll_body = _manual.y * _parameters.man_roll_max + _parameters.rollsp_offset_rad;
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, -_parameters.man_roll_max, _parameters.man_roll_max);
					_att_sp.pitch_body = -_manual.x * _parameters.man_pitch_max + _parameters.pitchsp_offset_rad;
					_att_sp.pitch_body = math::constrain(_att_sp.pitch_body, -_parameters.man_pitch_max, _parameters.man_pitch_max);
					_att_sp.yaw_body = 0.0f;
					_att_sp.thrust = _manual.z;

					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);
					_att_sp.q_d_valid = true;

					int instance;
					orb_publish_auto(_attitude_setpoint_id, &_attitude_sp_pub, &_att_sp, &instance, ORB_PRIO_DEFAULT);
				}

				float roll_sp = _att_sp.roll_body;
				float pitch_sp = _att_sp.pitch_body;
				float yaw_sp = _att_sp.yaw_body;
				float throttle_sp = _att_sp.thrust;
				float yaw_manual = 0.0f;

				/* allow manual yaw in manual modes */
				if (_vcontrol_mode.flag_control_manual_enabled) {
					yaw_manual = _manual.r;
				}

				/* reset integrals where needed */
				if (_att_sp.roll_reset_integral) {
					_roll_ctrl.reset_integrator();
				}

				if (_att_sp.pitch_reset_integral) {
					_pitch_ctrl.reset_integrator();
				}

				if (_att_sp.yaw_reset_integral) {
					_yaw_ctrl.reset_integrator();
					_wheel_ctrl.reset_integrator();
				}

				/* Reset integrators if the aircraft is on ground
				 * or a multicopter (but not transitioning VTOL)
				 */
				if (_vehicle_land_detected.landed
				    || (_vehicle_status.is_rotary_wing && !_vehicle_status.in_transition_mode)) {

					_roll_ctrl.reset_integrator();
					_pitch_ctrl.reset_integrator();
					_yaw_ctrl.reset_integrator();
					_wheel_ctrl.reset_integrator();
				}

				/* Prepare data for attitude controllers */
				struct ECL_ControlData control_input = {};
				control_input.roll = _roll;
				control_input.pitch = _pitch;
				control_input.yaw = _yaw;
				control_input.body_x_rate = _ctrl_state.roll_rate;
				control_input.body_y_rate = _ctrl_state.pitch_rate;
				control_input.body_z_rate = _ctrl_state.yaw_rate;
				control_input.roll_setpoint = roll_sp;
				control_input.pitch_setpoint = pitch_sp;
				control_input.yaw_setpoint = yaw_sp;
				control_input.airspeed_min = _parameters.airspeed_min;
				control_input.airspeed_max = _parameters.airspeed_max;
				control_input.airspeed = airspeed;
				control_input.scaler = airspeed_scaling;
				control_input.lock_integrator = lock_integrator;
				control_input.groundspeed = groundspeed;
				control_input.groundspeed_scaler = groundspeed_scaler;

				_yaw_ctrl.set_coordinated_method(_parameters.y_coordinated_method);

				/* Run attitude controllers */
				if (_vcontrol_mode.flag_control_attitude_enabled ||  (_parameters.tvlqr == 2)){//_parameters.tvlqr == 2) {
					if (PX4_ISFINITE(roll_sp) && PX4_ISFINITE(pitch_sp)) {
						//warnx("here11");
						_roll_ctrl.control_attitude(control_input);
						_pitch_ctrl.control_attitude(control_input);
						_yaw_ctrl.control_attitude(control_input); //runs last, because is depending on output of roll and pitch attitude
						_wheel_ctrl.control_attitude(control_input);

						/* Update input data for rate controllers */
						control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
						control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
						control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

						/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
						float roll_u = _roll_ctrl.control_euler_rate(control_input);
						if( _parameters.tvlqr == 2){//_parameters.tvlqr == 2){
							
		                _actuators.control[actuator_controls_s::INDEX_ROLL] = (float) u_com[1];//-1.0;//(PX4_ISFINITE(roll_u)) ? roll_u + _parameters.trim_roll :_parameters.trim_roll;
				
						}
						else{
						_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + _parameters.trim_roll :
								_parameters.trim_roll;
							}

						if (!PX4_ISFINITE(roll_u)) {
							_roll_ctrl.reset_integrator();
							perf_count(_nonfinite_output_perf);

							if (_debug && loop_counter % 10 == 0) {
								warnx("roll_u %.4f", (double)roll_u);
							}
						}

						float pitch_u = _pitch_ctrl.control_euler_rate(control_input);

						if(_parameters.tvlqr ==2){//_parameters.tvlqr ==2){
							_actuators.control[actuator_controls_s::INDEX_PITCH] = (float) u_com[2];//1.0;//(PX4_ISFINITE(pitch_u)) ? pitch_u + _parameters.trim_pitch : _parameters.trim_pitch;

					
						}
						else{
						_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + _parameters.trim_pitch :
								_parameters.trim_pitch;
							}

						if (!PX4_ISFINITE(pitch_u)) {
							_pitch_ctrl.reset_integrator();
							perf_count(_nonfinite_output_perf);

							if (_debug && loop_counter % 10 == 0) {
								warnx("pitch_u %.4f, _yaw_ctrl.get_desired_rate() %.4f,"
								      " airspeed %.4f, airspeed_scaling %.4f,"
								      " roll_sp %.4f, pitch_sp %.4f,"
								      " _roll_ctrl.get_desired_rate() %.4f,"
								      " _pitch_ctrl.get_desired_rate() %.4f"
								      " att_sp.roll_body %.4f",
								      (double)pitch_u, (double)_yaw_ctrl.get_desired_rate(),
								      (double)airspeed, (double)airspeed_scaling,
								      (double)roll_sp, (double)pitch_sp,
								      (double)_roll_ctrl.get_desired_rate(),
								      (double)_pitch_ctrl.get_desired_rate(),
								      (double)_att_sp.roll_body);
							}
						}

						float yaw_u = 0.0f;

						if (_parameters.w_en && _att_sp.fw_control_yaw) {
							yaw_u = _wheel_ctrl.control_bodyrate(control_input);

						} else {
							yaw_u = _yaw_ctrl.control_euler_rate(control_input);
						}

						if( _parameters.tvlqr ==2)//_parameters.tvlqr ==2)
						{
						_actuators.control[actuator_controls_s::INDEX_YAW] =  (float) u_com[3];//(float)0.01;//(PX4_ISFINITE(yaw_u)) ? yaw_u + _parameters.trim_yaw :_parameters.trim_yaw;
						
						/* add in manual rudder control */
						//_actuators.control[actuator_controls_s::INDEX_YAW] += yaw_manual;

						}
						else{
						_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + _parameters.trim_yaw :
								_parameters.trim_yaw;

						/* add in manual rudder control */
						_actuators.control[actuator_controls_s::INDEX_YAW] += yaw_manual;
						}

						if (!PX4_ISFINITE(yaw_u)) {
							_yaw_ctrl.reset_integrator();
							_wheel_ctrl.reset_integrator();
							perf_count(_nonfinite_output_perf);

							if (_debug && loop_counter % 10 == 0) {
								warnx("yaw_u %.4f", (double)yaw_u);
							}
						}

						/* throttle passed through if it is finite and if no engine failure was detected */

						if( _parameters.tvlqr == 2 ){//_parameters.tvlqr == 2){

							/* throttle passed through if it is finite and if no engine failure was detected */
						//warnx("actuator_controls_s::INDEX_THROTTLE : %d", actuator_controls_s::INDEX_THROTTLE);
						_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (float) u_com[0];//0.0f;
						//warnx("_actuators.control[actuator_controls_s::INDEX_THROTTLE] = %lf", (double)_actuators.control[actuator_controls_s::INDEX_THROTTLE]);

						}
						else{
						_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(throttle_sp) &&
								!(_vehicle_status.engine_failure ||
								  _vehicle_status.engine_failure_cmd)) ?
								throttle_sp : 0.0f;
							}
						//warnx("_actuators.control[actuator_controls_s::INDEX_THROTTLE] = %lf", (double)_actuators.control[actuator_controls_s::INDEX_THROTTLE]);
						/* scale effort by battery status */
						if (_parameters.bat_scale_en && _battery_status.scale > 0.0f &&
						    _actuators.control[actuator_controls_s::INDEX_THROTTLE] > 0.1f) {
							_actuators.control[actuator_controls_s::INDEX_THROTTLE] *= _battery_status.scale;
						}


						if (!PX4_ISFINITE(throttle_sp)) {
							if (_debug && loop_counter % 10 == 0) {
								warnx("throttle_sp %.4f", (double)throttle_sp);
							}
						}

					} else {
						perf_count(_nonfinite_input_perf);

						if (_debug && loop_counter % 10 == 0) {
							warnx("Non-finite setpoint roll_sp: %.4f, pitch_sp %.4f", (double)roll_sp, (double)pitch_sp);
						}
					}

				} else {
					// pure rate control
					//warnx("rate control");
					_roll_ctrl.set_bodyrate_setpoint(_manual.y * _parameters.acro_max_x_rate_rad);
					_pitch_ctrl.set_bodyrate_setpoint(-_manual.x * _parameters.acro_max_y_rate_rad);
					_yaw_ctrl.set_bodyrate_setpoint(_manual.r * _parameters.acro_max_z_rate_rad);

					float roll_u = _roll_ctrl.control_bodyrate(control_input);
					_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + _parameters.trim_roll :
							_parameters.trim_roll;

					float pitch_u = _pitch_ctrl.control_bodyrate(control_input);
					_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + _parameters.trim_pitch :
							_parameters.trim_pitch;

					float yaw_u = _yaw_ctrl.control_bodyrate(control_input);
					_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + _parameters.trim_yaw :
							_parameters.trim_yaw;

					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(throttle_sp) &&
							//!(_vehicle_status.engine_failure ||
							!_vehicle_status.engine_failure_cmd) ?
							throttle_sp : 0.0f;
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.roll = _roll_ctrl.get_desired_bodyrate();
				_rates_sp.pitch = _pitch_ctrl.get_desired_bodyrate();
				_rates_sp.yaw = _yaw_ctrl.get_desired_bodyrate();

				_rates_sp.timestamp = hrt_absolute_time();

				if (_rate_sp_pub != nullptr) {
					/* publish the attitude rates setpoint */
					orb_publish(_rates_sp_id, _rate_sp_pub, &_rates_sp);

				} else if (_rates_sp_id) {
					/* advertise the attitude rates setpoint */
					warnx("rate_sp_id pub");
					_rate_sp_pub = orb_advertise(_rates_sp_id, &_rates_sp);
				}

			} else {
				//	warnx("here5");
				/* manual/direct control */
				_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y * _parameters.man_roll_scale + _parameters.trim_roll;
				_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x * _parameters.man_pitch_scale +
						_parameters.trim_pitch;
				_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _parameters.man_yaw_scale + _parameters.trim_yaw;
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
			}

			// Add feed-forward from roll control output to yaw control output
			// This can be used to counteract the adverse yaw effect when rolling the plane
			//warnx("here6");
			_actuators.control[actuator_controls_s::INDEX_YAW] += _parameters.roll_to_yaw_ff * math::constrain(
						_actuators.control[actuator_controls_s::INDEX_ROLL], -1.0f, 1.0f);

			_actuators.control[actuator_controls_s::INDEX_FLAPS] = _flaps_applied;
			_actuators.control[5] = _manual.aux1;
			_actuators.control[actuator_controls_s::INDEX_AIRBRAKES] = _flaperons_applied;
			// FIXME: this should use _vcontrol_mode.landing_gear_pos in the future
			_actuators.control[7] = _manual.aux3;

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _ctrl_state.timestamp;
			_actuators_airframe.timestamp = hrt_absolute_time();
			_actuators_airframe.timestamp_sample = _ctrl_state.timestamp;

			/* Only publish if any of the proper modes are enabled */
			if (_vcontrol_mode.flag_control_rates_enabled ||
			    _vcontrol_mode.flag_control_attitude_enabled ||
			    _vcontrol_mode.flag_control_manual_enabled) {


				
				/* publish the actuator controls */
				if (_actuators_0_pub != nullptr) {
					
					if(flag ==1)
					{
						//warnx("here7");
					}
					orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

				} else if (_actuators_id) {
					_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
				}

				if (_actuators_2_pub != nullptr) {
					/* publish the actuator controls*/
					orb_publish(ORB_ID(actuator_controls_2), _actuators_2_pub, &_actuators_airframe);

				} else {
					/* advertise and publish */
					_actuators_2_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators_airframe);
				}
			}
		}

		loop_counter++;
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
}

int
FixedwingAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("fw_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&FixedwingAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return PX4_OK;
}

int fw_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: fw_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		att_control::g_control = new FixedwingAttitudeControl;

		if (att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (PX4_OK != att_control::g_control->start()) {
			delete att_control::g_control;
			att_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (att_control::g_control == nullptr || !att_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (att_control::g_control == nullptr || !att_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete att_control::g_control;
		att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
