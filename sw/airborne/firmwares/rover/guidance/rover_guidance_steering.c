/*
 * Copyright (C) 2021 Jesús Bautista <jesusbautistavillar@gmail.com> 
 *                    Hector García  <noeth3r@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#define AUTOPILOT_CORE_GUIDANCE_C

/** Mandatory dependencies header **/
#include "firmwares/rover/guidance/rover_guidance_steering.h"

#include "generated/airframe.h"

#include "modules/actuators/actuators_default.h"
#include "modules/radio_control/radio_control.h"
#include "autopilot.h"
#include "navigation.h"
#include "state.h"

#include "filters/pid.h" // Used for p+i speed controller

// Needed for get_sys_time_ms()
#include "modules/datalink/telemetry.h"
#include <math.h>
#include <stdio.h>

#include "modules/guidance/gvf_common.h"

// Moving Average filter. Number of samples
#ifndef MOV_AVG_M
#define MOV_AVG_M 10
#endif

PRINT_CONFIG_VAR(MOV_AVG_M)

// Speed moving average filter parameters
static int ptr_avg = 0;
static float speed_avg = 0;
static float mvg_avg[MOV_AVG_M] = {0};


// Guidance control main variables
rover_ctrl guidance_control;

static struct PID_f rover_pid;
static float time_step;
static float last_speed_cmd;
static uint8_t last_ap_mode;

// Here the time in order to stop
static uint32_t rover_time = 0;
static int reset_time = 0;

/** INIT function **/
void rover_guidance_steering_init(void)
{
  guidance_control.cmd.delta = 0.0;
  guidance_control.cmd.speed = 0.0;
  guidance_control.throttle  = 0.0;

  last_speed_cmd = 0.0;
  last_ap_mode   = AP_MODE_KILL;

  guidance_control.speed_error = 0.0;
  guidance_control.kf = SR_MEASURED_KF;
  guidance_control.kp = 10;
  guidance_control.ki = 100;

	// Mov avg init speed 
	float speed = stateGetHorizontalSpeedNorm_f();
	for(int k = 0; k < MOV_AVG_M; k++)
		mvg_avg[k] = speed;
	speed_avg = speed;
	
  init_pid_f(&rover_pid, guidance_control.kp, 0.f, guidance_control.ki, MAX_PPRZ*0.2);

  // Based on autopilot state machine frequency
  time_step = 1.f/PERIODIC_FREQUENCY;
}

/** CTRL functions **/
// Steering control (GVF)
void rover_guidance_steering_heading_ctrl(float omega) //GVF give us this omega
{
  float delta = 0.0;

  // Speed is bounded to avoid GPS noise while driving at small velocity
  float speed = BoundSpeed(stateGetHorizontalSpeedNorm_f()); 

  if (fabs(omega)>0.0) {
      delta = DegOfRad(-atanf(omega*DRIVE_SHAFT_DISTANCE/speed));
    }

  guidance_control.cmd.delta = BoundDelta(delta);
}

// Speed control (feed feed forward + propotional + integral controler) (PID)
void rover_guidance_steering_speed_ctrl(void) 
{
  // - Looking for setting update
  if (guidance_control.kp != rover_pid.g[0] || guidance_control.ki != rover_pid.g[2]) {
    set_gains_pid_f(&rover_pid, guidance_control.kp, 0.f, guidance_control.ki);
  }
  if (guidance_control.cmd.speed != last_speed_cmd) {
    last_speed_cmd = guidance_control.cmd.speed;
  }
  
  rover_guidance_steering_obtain_setpoint();
  
  // Mov avg speed
	speed_avg = speed_avg - mvg_avg[ptr_avg]/MOV_AVG_M;
	mvg_avg[ptr_avg] = stateGetHorizontalSpeedNorm_f();
	speed_avg = speed_avg + mvg_avg[ptr_avg]/MOV_AVG_M;
	ptr_avg = (ptr_avg + 1) % MOV_AVG_M;
	
  // Updating PID using the average speed
  guidance_control.speed_error = (guidance_control.cmd.speed - speed_avg);
  update_pid_f(&rover_pid, guidance_control.speed_error, time_step);
  guidance_control.throttle = BoundThrottle(guidance_control.kf*guidance_control.cmd.speed + get_pid_f(&rover_pid));
  
}

// Obtains setpoint 
// Here is where the rover stops
void rover_guidance_steering_obtain_setpoint(void)
{

	//float kappa   = gvf_c_info.kappa;
	//float ori_err = gvf_c_info.ori_err;

	// Setpoint to zero if rover must stay still
	if((gvf_c_stopwp.stay_still) && (!reset_time))
	{
		guidance_control.cmd.speed = 0;
		rover_time = get_sys_time_msec();
		reset_time = 1;
	}
	else if((gvf_c_stopwp.stay_still) && (reset_time))
	{
		if( (get_sys_time_msec() - rover_time) >= 1000*gvf_c_stopwp.wait_time){
			reset_time = 0;
			gvf_c_stopwp.stay_still = 0;
		}	
	}

}

/** PID RESET function**/
void rover_guidance_steering_pid_reset(void)
{
    // Reset speed PID
    if (rover_pid.sum != 0) {
      reset_pid_f(&rover_pid);
    }
}

void rover_guidance_steering_kill(void)
{
  guidance_control.cmd.delta = 0.0;
  guidance_control.cmd.speed = 0.0;
}
