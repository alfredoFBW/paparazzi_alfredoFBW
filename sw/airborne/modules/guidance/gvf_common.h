/*
 * Copyright (C) 2023 Alfredo Gonzalez Calvin <alfredgo@ucm.es>
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
 
 #ifndef GVF_COMMON_H
 #define GVF_COMMON_H
 
 // uint32_t
#include "std.h"

 /** @typedef gvf_common_omega
* @brief Horizontal control signal for both gvf
* @param omega is the horizontal control signal
*/

 typedef struct{
 	
 	float omega;
 } gvf_common_omega;
 
extern gvf_common_omega gvf_c_omega;

 /** @typedef gvf_common_params
* @brief Different parameters obtained from gvfs. dot means d/dt
* @param kappa is the curve's curvature
* @param ori_err is the orientation error
*/
typedef struct{	
	float kappa;
	float kappa_dot;
	float ori_err;
	float ori_err_dot;
} gvf_common_params;

/** @typdef gvf_common_stop_at_wp (only for rovers)
* @brief Parameters in order to allow stoping at certain waypoints
* @param stay_still determines if vehicle must stay still if stop_at_wp = 1
* @param stop_at_wp determines if vehicle must stop at wp
* @param wait_time tells how much time it must wait
* @param distance_stop: Tells in which radius is must stop
* @param pxd, pyd are the desired waypoints in which to stop
*/
typedef struct{
	int   stay_still;
	int   stop_at_wp;
	uint32_t wait_time;
	float distance_stop;
	float pxd;
	float pyd;
} gvf_common_stop_at_wp;
 
extern gvf_common_params gvf_c_info;
extern gvf_common_stop_at_wp gvf_c_stopwp;
 
 #endif // GVF_COMMON_H

