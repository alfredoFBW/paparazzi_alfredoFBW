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

#include <math.h>
#include <stdio.h>

// Guidance control main variables
rover_ctrl guidance_control;


#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
#endif

// Moving Average filter. Number of samples
#ifndef MOV_AVG_M
#define MOV_AVG_M 10
#endif

PRINT_CONFIG_VAR(MOV_AVG_M)

static int ptr_avg = 0;
static float speed_avg = 0;
static float mvg_avg[MOV_AVG_M] = {0};


static struct PID_f rover_pid;
static float time_step;
static float last_speed_cmd;
static uint8_t last_ap_mode;

// Integral and prop actions (telemetry)
static float i_action;
static float p_action;

#if PERIODIC_TELEMETRY
static void send_rover_ctrl(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_ROVER_CTRL(trans, dev, AC_ID,
                    	    &guidance_control.cmd.speed,
                    	    &guidance_control.speed_error,
                    	    &guidance_control.throttle,
                    	    &guidance_control.cmd.delta,
                    	    &i_action,				// Integral action
                    	    &p_action,                        // Prop action 
                    	    &speed_avg);                      // Avg speed measured 
}
#endif


/** INIT function **/
void rover_guidance_steering_init(void)
{

  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ROVER_CTRL, send_rover_ctrl);
  #endif

  guidance_control.cmd.delta = 0.0;
  guidance_control.cmd.speed = 0.0;
  guidance_control.throttle  = 0.0;

  last_speed_cmd = 0.0;
  last_ap_mode   = AP_MODE_KILL;

  guidance_control.speed_error = 0.0;
  guidance_control.kf = SR_MEASURED_KF;
  guidance_control.kp = 10;
  guidance_control.ki = 100;

  init_pid_f(&rover_pid, guidance_control.kp, 0.f, guidance_control.ki, MAX_PPRZ*0.2);

  // Mov avg init
  float speed = stateGetHorizontalSpeedNorm_f();
  for(int k = 0; k < MOV_AVG_M; k++)
  	mvg_avg[k] = speed;
  speed_avg = speed;
  
  // Based on autopilot state machine frequency
  time_step = 1.f/PERIODIC_FREQUENCY;
}

/** CTRL functions **/
// Steering control (GVF)
void rover_guidance_steering_heading_ctrl(float omega) //GVF give us this omega
{
  float delta = 0.0;

  // Speed is bounded to avoid GPS noise while driving at small velocity
  //float speed = BoundSpeed(stateGetHorizontalSpeedNorm_f()); 
  float speed = speed_avg; // Use avg, avoid noise
  
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
    //reset_pid_f(&rover_pid);
  }

  // Mov avg speed
  speed_avg = speed_avg - mvg_avg[ptr_avg]/MOV_AVG_M;
  mvg_avg[ptr_avg] = stateGetHorizontalSpeedNorm_f();
  speed_avg = speed_avg + mvg_avg[ptr_avg]/MOV_AVG_M;
  ptr_avg = (ptr_avg + 1) % MOV_AVG_M;

  // - Updating PID
  //guidance_control.speed_error = (guidance_control.cmd.speed - stateGetHorizontalSpeedNorm_f());
  guidance_control.speed_error = guidance_control.cmd.speed - speed_avg;
  update_pid_f(&rover_pid, guidance_control.speed_error, time_step);

  guidance_control.throttle = BoundThrottle(guidance_control.kf*guidance_control.cmd.speed + get_pid_f(&rover_pid));
  
  // Telemetry
  i_action = get_i_action(&rover_pid, guidance_control.speed_error, time_step);
  p_action = get_p_action(&rover_pid, guidance_control.speed_error);
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
