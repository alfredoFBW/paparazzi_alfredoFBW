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
 * @file modules/guidance/gvf_parametric/trajectories/gvf_parametric_2D_splines.c
 * @brief Natural cubic splines 
 * a(t-t0)^3 + b(t1-t)^3 + c(t-t0) + d(t-t1) \in [t0,t1]
 * Note: The splines ctrl points must be in meters wrt home
 */ 

#include "modules/nav/common_nav.h"
#include "modules/guidance/gvf_parametric/gvf_parametric.h"
#include "modules/guidance/gvf_parametric/trajectories/gvf_parametric_2d_splines.h"

/*! Default gain kx for the 2d splines trajectory */
#ifndef GVF_PARAMETRIC_2D_SPLINES_KX
#define GVF_PARAMETRIC_2D_SPLINES_KX 0.02
#endif

/*! Default gain ky for the 2d splines trajectory */
#ifndef GVF_PARAMETRIC_2D_SPLINES_KY
#define GVF_PARAMETRIC_2D_SPLINES_KY 0.02
#endif

gvf_par_2d_spl_par gvf_parametric_2d_splines_par = {GVF_PARAMETRIC_2D_SPLINES_KX, GVF_PARAMETRIC_2D_SPLINES_KY};


// Copied from paparazzi. Argument changed
void float_mat_invert_2(float o[GVF_PARAMETRIC_2D_SPLINES_N_SEG-1][GVF_PARAMETRIC_2D_SPLINES_N_SEG-1], float mat[GVF_PARAMETRIC_2D_SPLINES_N_SEG-1][GVF_PARAMETRIC_2D_SPLINES_N_SEG-1], int n)
{
  int i, j, k;
  float t;
  float a[n][2 * n];

  // Append an identity matrix on the right of the original matrix
  for (i = 0; i < n; i++) {
    for (j = 0; j < 2 * n; j++) {
      if (j < n) {
        a[i][j] = mat[i][j];
      } else if ((j >= n) && (j == i + n)) {
        a[i][j] = 1.0;
      } else {
        a[i][j] = 0.0;
      }
    }
  }

  // Do the inversion
  for (i = 0; i < n; i++) {
    t = a[i][i]; // Store diagonal variable (temp)

    for (j = i; j < 2 * n; j++) {
      a[i][j] = a[i][j] / t; // Divide by the diagonal value
    }

    for (j = 0; j < n; j++) {
      if (i != j) {
        t = a[j][i];
        for (k = 0; k < 2 * n; k++) {
          a[j][k] = a[j][k] - t * a[i][k];
        }
      }
    }
  }

  // Cut out the identity, which has now moved to the left side
  for (i = 0 ; i < n ; i++) {
    for (j = n; j < 2 * n; j++) {
      o[i][j - n] = a[i][j];
    }
  }
}


// Obtain spline ctrl points. Nf segments -> Nf-1 ctrl points
void get_splines_from_ctrl_points(spline_t *spline, float *t, float *points)
{
	// t0, t1, t2, ... tN N+1	
	float h[GVF_PARAMETRIC_2D_SPLINES_N_SEG];		// 0, 1, 2, .., GVF_PARAMETRIC_2D_SPLINES_N_SEG - 1
	float b[GVF_PARAMETRIC_2D_SPLINES_N_SEG];		// 0, 1, 2, .., GVF_PARAMETRIC_2D_SPLINES_N_SEG - 1
	float v[GVF_PARAMETRIC_2D_SPLINES_N_SEG];		// 0, 1, 2, .., GVF_PARAMETRIC_2D_SPLINES_N_SEG - 1
	float u[GVF_PARAMETRIC_2D_SPLINES_N_SEG];		// 0, 1, 2, .., GVF_PARAMETRIC_2D_SPLINES_N_SEG - 1
	float z[GVF_PARAMETRIC_2D_SPLINES_N_SEG+1];		// 0, 1, 2, .., GVF_PARAMETRIC_2D_SPLINES_N_SEG

	int i,k;

	// Prepare data
	for(i = 0; i < GVF_PARAMETRIC_2D_SPLINES_N_SEG; i++)
	{
		h[i] = t[i+1] - t[i];	
		b[i] = 1/h[i]*(points[i+1]- points[i]); 	
		z[i] = 0; 
	}

	// Don't care about 0 position
	for(i = 1; i < GVF_PARAMETRIC_2D_SPLINES_N_SEG; i++)
	{
		v[i] = 2*(h[i-1] + h[i]);	
		u[i] = 6*(b[i] - b[i-1]);     
	}

	/* Prepare matrix. Size: (GVF_PARAMETRIC_2D_SPLINES_N_SEG-1)x(GVF_PARAMETRIC_2D_SPLINES_N_SEG-1) 
	 * Index range: (0,1,...,GVF_PARAMETRIC_2D_SPLINES_N_SEG-2)x(0,1,...,GVF_PARAMETRIC_2D_SPLINES_N_SEG-2)
	 */

	float M[GVF_PARAMETRIC_2D_SPLINES_N_SEG-1][GVF_PARAMETRIC_2D_SPLINES_N_SEG-1]; 
	float Minv[GVF_PARAMETRIC_2D_SPLINES_N_SEG-1][GVF_PARAMETRIC_2D_SPLINES_N_SEG-1];
	
	for(k = 0; k < GVF_PARAMETRIC_2D_SPLINES_N_SEG-1; k++){
		for(i = 0; i < GVF_PARAMETRIC_2D_SPLINES_N_SEG-1; i++)
			M[k][i] = 0.0;
	}

	// Matrix
	M[0][0] = v[1]; M[0][1] = h[1];
	M[1][0] = h[1]; M[1][1] = v[2]; M[1][2] = h[2];
	for(k = 2; k < GVF_PARAMETRIC_2D_SPLINES_N_SEG-2; k++)
	{
		M[k][k-1] = h[k];
		M[k][k]   = v[k+1];
		M[k][k+1] = h[k+1];
	}
	M[GVF_PARAMETRIC_2D_SPLINES_N_SEG-2][GVF_PARAMETRIC_2D_SPLINES_N_SEG-3] = h[GVF_PARAMETRIC_2D_SPLINES_N_SEG-2];
	M[GVF_PARAMETRIC_2D_SPLINES_N_SEG-2][GVF_PARAMETRIC_2D_SPLINES_N_SEG-2] = v[GVF_PARAMETRIC_2D_SPLINES_N_SEG-1];


	float_mat_invert_2(Minv, M, GVF_PARAMETRIC_2D_SPLINES_N_SEG-1);

	for(k = 1; k < GVF_PARAMETRIC_2D_SPLINES_N_SEG; k++)          
	{
		for(i = 1; i < GVF_PARAMETRIC_2D_SPLINES_N_SEG; i++) 
			z[k] += Minv[k-1][i-1]*u[i];
	}
	z[0] = 0; z[GVF_PARAMETRIC_2D_SPLINES_N_SEG] = 0;

	// Create coefs (cubic splines: 4 coefs)
	float coefs[GVF_PARAMETRIC_2D_SPLINES_N_SEG][4];
	for(i = 0; i < GVF_PARAMETRIC_2D_SPLINES_N_SEG; i++)
	{
		coefs[i][0] = z[i+1] / (6.0 * h[i]);
		coefs[i][1] = z[i] / (6.0 * h[i]);
		coefs[i][2] = (points[i+1] / h[i] - z[i+1] * h[i] / 6.0);
		coefs[i][3] = (points[i] / h[i] -  z[i]*h[i] / 6.0);
	}

	// Create the splines
	for(k = 0; k < GVF_PARAMETRIC_2D_SPLINES_N_SEG; k++){
		spline[k].a  = coefs[k][0];
		spline[k].b  = coefs[k][1];
		spline[k].c  = coefs[k][2];
		spline[k].d  = coefs[k][3];
		spline[k].t0 = t[k];
		spline[k].t1 = t[k+1]; 
	}
}


// Evaluate the spline at a time t. d_order is the derivative order 
float spline_eval(spline_t *spline, int d_order, float t)
{
	/* d_order = 0, no derivative, d_order = 1, first derivative .. */
	int k = 0; 
	int found = 0; 
	int index = -1;

	// Check which spline must be evaluated
	while( (k < GVF_PARAMETRIC_2D_SPLINES_N_SEG) && (!found) )
	{
		if( (spline[k].t0 <= t) && (spline[k].t1 >= t) )
		{
			index = k;
			found = 1;
		}
		k++;
	}
	//if(index == -1){
	//	return -10000; 
	//}
	float a,b,c,d,t0,t1;
	a  = spline[index].a;   b = spline[index].b;
	c  = spline[index].c;   d = spline[index].d;
	t0 = spline[index].t0; t1 = spline[index].t1;

	if(d_order == 0)	   
		return a*powf(t-t0,3) + b*powf(t1-t,3) + c*(t-t0) + d*(t1-t);
	else if(d_order == 1)  // First derivative
		return 3*a*powf(t-t0,2) - 3*b*powf(t1-t,2) + c - d;
	else if(d_order == 2)  // Second derivative
		return 6*a*(t-t0) + 6*b*(t1-t);
	else if(d_order == 3)  // Third derivative
		return 6*a - 6*b;
	else 				   // Higher order derivatives
		return 0;
}


void gvf_parametric_2d_splines_info(spline_t *splines_x, spline_t *splines_y, float *f1, float *f2, float *f1d, float *f2d, float *f1dd, float *f2dd)
{
  	float w = gvf_parametric_control.w;
	*f1   = spline_eval(splines_x, 0, w);
	*f2   = spline_eval(splines_y, 0, w);
	*f1d  = spline_eval(splines_x, 1, w);
	*f2d  = spline_eval(splines_y, 1, w);
	*f1dd = spline_eval(splines_x, 2, w);
	*f2dd = spline_eval(splines_y, 2, w);
}

