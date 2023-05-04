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
 
 /** @typedef gvf_common_omega
* @brief Horizontal control signal for both gvf
* @param omega is the horizontal control signal
*/

 typedef struct{
 	
 	float omega;
 } gvf_common_omega;
 
extern gvf_common_omega gvf_c_omega;
 
 #endif // GVF_COMMON_H

