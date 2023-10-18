/*
 * Copyright (C) 2021 Jesús Bautista <jesusbautistavillar@gmail.com> 
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
#include "firmwares/rover/guidance/boat_guidance.h"

#include "generated/airframe.h"
#include "generated/radio.h"

#include "modules/actuators/actuators_default.h"
#include "modules/radio_control/radio_control.h"
#include "navigation.h"
#include "autopilot.h"
#include "state.h"

#include "modules/guidance/gvf/gvf.h"
#include "filters/pid.h"

#include "modules/guidance/gvf_common.h"

#include "modules/datalink/telemetry.h"

// Moving Average filter. Number of samples
#ifndef MOV_AVG_M
#define MOV_AVG_M 20
#endif
PRINT_CONFIG_VAR(MOV_AVG_M)

// Debugging telemetry. Must be included for sys_time
#ifdef BOAT_DEBUG
#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static uint8_t dummy = 0;
static void send_telemetry(struct transport_tx *trans, struct link_device *dev){
  pprz_msg_send_INFO_MSG(trans, dev, AC_ID, &dummy ,&guidance_control.bearing, &guidance_control.throttle, 
  					     &guidance_control.rc_throttle, &guidance_control.rc_bearing, &commands[COMMAND_MLEFT], &commands[COMMAND_MRIGHT]);
  }					
#endif
#endif

// Guidance control main variables
ctrl_t guidance_control;

static struct PID_f boat_pid;
static float time_step;
static float last_speed_cmd;
uint32_t rover_time = 0;
uint8_t reset_time = 0;

// Speed moving average filter parameters
static int ptr_avg = 0;
static float speed_avg = 0;
static float mvg_avg[MOV_AVG_M] = {0};


/** INIT function **/
void boat_guidance_init(void)
{
  guidance_control.cmd.speed = 0.0;
  guidance_control.cmd.omega = 0.0;
  guidance_control.throttle  = 0.0;
  guidance_control.bearing   = 0.0;
  guidance_control.rc_throttle = 0;
  guidance_control.rc_bearing  = 0;

  last_speed_cmd = 0.0;
  
  guidance_control.kf_bearing = BOAT_BEARING_KF;
  guidance_control.kf_speed   = BOAT_SPEED_KF;
	guidance_control.kf_bearing_static = BOAT_BEARING_KF;
	guidance_control.kf_speed_static = BOAT_SPEED_KF;
	guidance_control.use_dynamic_pos = 1;
  guidance_control.speed_error = 0.0;
  guidance_control.kp = 10;
  guidance_control.ki = 100;

  init_pid_f(&boat_pid, guidance_control.kp, 0.f, guidance_control.ki, MAX_PPRZ*0.4); // Increased integral bounds

	// Mov avg init Speed and distance
	float speed = stateGetHorizontalSpeedNorm_f();
	tfmini_event();
	for(int k = 0; k < MOV_AVG_M; k++){
		mvg_avg[k] = speed;
	}
	speed_avg = speed;
	
	// Reset time
	reset_time = 0;
	
  // Based on autopilot state machine frequency
  time_step = 1.f/PERIODIC_FREQUENCY;
  
  #ifdef BOAT_DEBUG
  #if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_INFO_MSG, send_telemetry);
  #endif
  #endif
}

void boat_bound_cmds(void)
{
 //Protejemos de la saturación, pero solo positiva... 
  if (commands[COMMAND_MRIGHT] > MAX_PPRZ){
   commands[COMMAND_MRIGHT] = MAX_PPRZ;
   commands[COMMAND_MLEFT]  = MAX_PPRZ*(guidance_control.throttle - guidance_control.bearing)/(guidance_control.throttle + guidance_control.bearing);
  }
  
  if (commands[COMMAND_MLEFT] > MAX_PPRZ){
   commands[COMMAND_MLEFT] = MAX_PPRZ;
   commands[COMMAND_MRIGHT]  = MAX_PPRZ*(guidance_control.throttle + guidance_control.bearing)/(guidance_control.throttle - guidance_control.bearing);
  }
}

/** RC guidance function **/
void boat_guidance_read_rc(void){

  guidance_control.rc_throttle = (int32_t)radio_control.values[RADIO_THROTTLE];
  guidance_control.rc_bearing  = (int32_t)radio_control.values[RADIO_ROLL];
  
  commands[COMMAND_MLEFT]  = (guidance_control.rc_throttle - guidance_control.rc_bearing)/2;
  commands[COMMAND_MRIGHT] = (guidance_control.rc_throttle + guidance_control.rc_bearing)/2;
  
  boat_bound_cmds();
}


/** Navigation guidance function **/
void boat_guidance_read_NAV(void)
{
	// If we must stay still obtain throttle and bearing from dynamic positioning
	if((gvf_c_stopwp.stay_still) && (!reset_time) && (guidance_control.use_dynamic_pos)){
		boat_guidance_bearing_static_ctrl();
	}
	else{
		boat_guidance_bearing_GVF_ctrl();
		boat_guidance_speed_ctrl();
	}
  
  //Definimos las ordones suponiendo que no hay saturacion
  commands[COMMAND_MLEFT]  = (guidance_control.throttle - guidance_control.bearing)/2;
  commands[COMMAND_MRIGHT] = (guidance_control.throttle + guidance_control.bearing)/2;
  
  boat_bound_cmds();
}


/** CTRL functions **/
void boat_guidance_bearing_GVF_ctrl(void)
{
	// omega from gvf_common.h
  guidance_control.cmd.omega = gvf_c_omega.omega; //GVF give us this omega
  guidance_control.bearing = BoundCmd(guidance_control.kf_bearing * guidance_control.cmd.omega);
}

/* Static ctrl. Only works for GVF line array TODO: Improvements to work in any point */
void boat_guidance_bearing_static_ctrl(void)
{ 
	// Current position of the boat
	struct EnuCoor_f *p = stateGetPositionEnu_f();
  float px = p->x;
  float py = p->y;
 
  // Desired position for the boat
  float pd[2]; 
  
  // psi = angle between the direction of the vehicle and the origin of coordinates
  float psi = stateGetNedToBodyEulers_f()->psi;
  float u[2];
  
  // s = p - pd
  float s[2];
  
 	pd[0] = gvf_c_stopwp.pxd;
 	pd[1] = gvf_c_stopwp.pyd;
 	
 	u[0] = cosf(psi); u[1] = sinf(psi);
 	s[0] = (px - pd[0]); s[1] = (py - pd[1]);
 
  // normalized using euclidean norm
  float ns = sqrtf(s[0] * s[0] + s[1] * s[1]);
  
	s[0] /= ns; s[1] /= ns;
  
  float sTu = u[0]*s[0] + u[1]*s[1];  // cos(beta), beta = angle(u,s)
  float sTEu = s[0]*u[1] - s[1]*u[0]; // sin(beta)
  
  float tau = guidance_control.kf_bearing_static * sTu * sTEu;
  float f = guidance_control.kf_speed_static * sTu * ns;
  
  guidance_control.bearing = BoundCmd(tau);
  guidance_control.throttle = BoundCmd(f);
}

void boat_guidance_speed_ctrl(void) // Feed forward + Integral controler + Propotional (PID)
{ 
  // - Looking for setting update
  if (guidance_control.kp != boat_pid.g[0] || guidance_control.ki != boat_pid.g[2]) {
    set_gains_pid_f(&boat_pid, guidance_control.kp, 0.f, guidance_control.ki);
  }
  if (guidance_control.cmd.speed != last_speed_cmd) {
    last_speed_cmd = guidance_control.cmd.speed;
  }

	boat_guidance_steering_obtain_setpoint();
	
	// Mov avg speed
	speed_avg = speed_avg - mvg_avg[ptr_avg]/MOV_AVG_M;
	mvg_avg[ptr_avg] = stateGetHorizontalSpeedNorm_f();
	speed_avg = speed_avg + mvg_avg[ptr_avg]/MOV_AVG_M;
	ptr_avg = (ptr_avg + 1) % MOV_AVG_M;
	
  // - Updating PID
  //guidance_control.speed_error = (guidance_control.cmd.speed - stateGetHorizontalSpeedNorm_f());
  
  // using moving average
  guidance_control.speed_error = guidance_control.cmd.speed - speed_avg;
  update_pid_f(&boat_pid, guidance_control.speed_error, time_step);
  
  // - Set throttle
  guidance_control.throttle = BoundCmd(guidance_control.kf_speed * guidance_control.cmd.speed + get_pid_f(&boat_pid));
}

// Obtain setpoin
void boat_guidance_steering_obtain_setpoint(void)
{
	
	// Setpoint to zero if rover must stay still
	if((gvf_c_stopwp.stay_still) && (!reset_time)){
		guidance_control.cmd.speed = 0;
		rover_time = get_sys_time_msec();
		reset_time = 1;
	}
	else if((gvf_c_stopwp.stay_still) && (reset_time)){
		if( (get_sys_time_msec() - rover_time) >= 1000*gvf_c_stopwp.wait_time){
			reset_time = 0;
			gvf_c_stopwp.stay_still = 0;
		}	
	}
}


/** PID RESET function**/
void boat_guidance_pid_reset(void)
{
    // Reset speed PID
    if (boat_pid.sum != 0) {
      reset_pid_f(&boat_pid);
    }
}


/** KILL function **/
void boat_guidance_kill(void)
{
  guidance_control.cmd.speed = 0.0;
  commands[COMMAND_MLEFT]  = 0;
  commands[COMMAND_MRIGHT] = 0;
}
