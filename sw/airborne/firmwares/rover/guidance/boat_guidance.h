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

#ifndef BOAT_GUIDANCE_H
#define BOAT_GUIDANCE_H

#include "std.h"
#include <math.h>
#include <stdio.h>

/** Generated airframe.h from airframe.xml
 * - fun: SetActuatorsFromCommands
 * - var: commands
 * - var: hardware and construction parameters
 **/
 
#include "generated/airframe.h"

// Check critical global definitiones
#ifndef SERVO_MOTOR_LEFT
#error "Boat guidance requires the servo MOTOR_LEFT"
#endif

#ifndef SERVO_MOTOR_RIGHT
#error "Boat guidance requires the servo MOTOR_RIGHT"
#endif

#ifndef COMMAND_MLEFT
#error "Boat guidance requires the command COMMAND_MLEFT"
#endif

#ifndef COMMAND_MRIGHT
#error "Boat guidance requires the command COMMAND_MRIGHT"
#endif


// Controller gains
#ifndef BOAT_SPEED_KF // Lineal feed forward speed control constant (have to be measured with new servos)
#define BOAT_SPEED_KF 10
#warning "Construction constant BOAT_MEASURED_KF for boat speed ctrl not defined"
#endif

#ifndef BOAT_BEARING_KF
#define BOAT_BEARING_KF 10
#warning "Construction constant BOAT_BEARING_KF for boat bearing ctrl not defined"
#endif

// Check controller gains values (error if they are negative)
#if (BOAT_KF_BEARING < 0) ||                   \
    (BOAT_KF_SPEED   < 0)
#error "ALL control gains must be positive!!!"
#endif


/** Steering rover guidance STRUCTURES **/
// High level commands
typedef struct {
  float speed;
  float omega;
} guidance_cmd_t;

// Main structure
typedef struct {
  guidance_cmd_t cmd;

  float throttle; //  Td + Ti
  float bearing;  // |Td - Ti|

  int32_t rc_throttle;
  int32_t rc_bearing;

  float kf_bearing;
  float kf_speed;

	float kf_speed_static;
	float kf_bearing_static;
	
	int use_dynamic_pos;
  float speed_error;
  float kp;
  float ki;
} ctrl_t;

extern ctrl_t guidance_control;

/** Boat guidance EXT FUNCTIONS **/
extern void boat_guidance_init(void);
extern void boat_guidance_read_rc(void);
extern void boat_guidance_bearing_GVF_ctrl(void);
extern bool boat_guidance_bearing_static_ctrl(void);
extern void boat_guidance_speed_ctrl(void);

extern void boat_guidance_read_NAV(void); //JJC

extern void boat_guidance_steering_obtain_setpoint(void);
extern void boat_guidance_pid_reset(void);
extern void boat_guidance_kill(void);


/** MACROS **/
// Bound commands
#define BoundCmd(cmd) 2*TRIM_PPRZ((int)cmd/2);

// Set AP throttle value
#define SetAPThrottleFromCommands(void) { \
autopilot.throttle = (commands[COMMAND_MLEFT] + commands[COMMAND_MRIGHT])/2; }

#endif // BOAT_GUIDANCE_H
