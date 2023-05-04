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

/**
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_2D_splines.h
 * @brief Parameters, structs and functions defined for natural cubic splines
 */
#ifndef GVF_PARAMETRIC_2D_SPLINES_H
#define GVF_PARAMETRIC_2D_SPLINES_H

// Number of segments
#ifndef GVF_PARAMETRIC_2D_SPLINES_N_SEG
#define GVF_PARAMETRIC_2D_SPLINES_N_SEG 8
#endif


#ifdef __cplusplus
extern "C" {
#endif



/** @typedef gvf_par_2d_spl_par
* @brief Parameters for the GVF parametric 2D natural cubic splines
* @param kx Gain defining how agressive is the vector field in x coordinate
* @param ky Gain defining how agressive is the vector field in y coordinate
*/
typedef struct {
	float kx;
	float ky;
}gvf_par_2d_spl_par;

/** @typedef gvf_par_2d_spl_par
* @brief Natural cubic spline polynomial
* a(t-t0)^3 + b(t1-t)^3 + c(t-t0) + d(t-t1) \in [t0,t1]
*/
typedef struct{
	float a;
	float b;
	float c;
	float d;
	float t0;
	float t1;
}spline_t;


extern gvf_par_2d_spl_par gvf_parametric_2d_splines_par;


extern void float_mat_invert_2(float o[GVF_PARAMETRIC_2D_SPLINES_N_SEG-1][GVF_PARAMETRIC_2D_SPLINES_N_SEG-1], float mat[GVF_PARAMETRIC_2D_SPLINES_N_SEG-1][GVF_PARAMETRIC_2D_SPLINES_N_SEG-1], int n);
extern void get_splines_from_ctrl_points(spline_t *spline, float *t, float *points);
extern float spline_eval(spline_t *spline, int d_order, float t);
extern void gvf_parametric_2d_splines_info(spline_t *splines_x, spline_t *splines_y, float *f1, float *f2, float *f1d, float *f2d, float *f1dd, float *f2dd);

#ifdef __cplusplus
}
#endif


#endif // SPLINES
