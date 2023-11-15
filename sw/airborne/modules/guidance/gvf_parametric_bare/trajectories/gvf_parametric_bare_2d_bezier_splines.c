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

#include "modules/nav/common_nav.h"
#include "modules/guidance/gvf_parametric_bare/gvf_parametric_bare.h"
#include "modules/guidance/gvf_parametric_bare/trajectories/gvf_parametric_bare_2d_bezier_splines.h"

#ifndef GVF_PARAMETRIC_BARE_2D_BEZIER_SPLINES_KX
#define GVF_PARAMETRIC_BARE_2D_BEZIER_SPLINES_KX 0.5
#endif 

#ifndef GVF_PARAMETRIC_BARE_2D_BEZIER_SPLINES_KY
#define GVF_PARAMETRIC_BARE_2D_BEZIER_SPLINES_KY 0.5
#endif 

gvf_bare_par_2d_bezier_par gvf_parametric_bare_2d_bezier_par = {GVF_PARAMETRIC_BARE_2D_BEZIER_SPLINES_KX, 
							GVF_PARAMETRIC_BARE_2D_BEZIER_SPLINES_KY};

// Bezier is just an array
void bare_create_bezier_spline(bare_bezier_t *bezier, int order, float *px, float *py)
{

	int k, j;
	j = 0;
	
	// C^0 continuity third order Bézier Curves
	if(order == 3){
	  for(k = 0; k < GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG; k++)
	  {
		  bezier[k].p0[0] = px[j];
		  bezier[k].p0[1] = py[j];
		  bezier[k].p1[0] = px[j+1];
		  bezier[k].p1[1] = py[j+1];
		  bezier[k].p2[0] = px[j+2];
		  bezier[k].p2[1] = py[j+2];
		  bezier[k].p3[0] = px[j+3];
		  bezier[k].p3[1] = py[j+3];
		  
		  // This allows for C^0 continuity (last point is init point)
		  j += 3;
	  }
	}
	else if(order == 5){ // C^2 continuity Bézier curves
	  
	  // Init first segment
	  k = 0; j = 0;
	  bezier[k].p0[0] = px[0]; bezier[k].p0[1] = py[0];
	  bezier[k].p1[0] = px[1]; bezier[k].p1[1] = py[1];
	  bezier[k].p2[0] = px[2]; bezier[k].p2[1] = py[2];
	  bezier[k].p3[0] = px[3]; bezier[k].p3[1] = py[3];
	  bezier[k].p4[0] = px[4]; bezier[k].p4[1] = py[4];
	  bezier[k].p5[0] = px[5]; bezier[k].p5[1] = py[5];
	  
	  // Init the rest of segments
	  j = 5;
	  for(k = 1; k < GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG; k++){
	    bezier[k].p0[0] = px[j];
	    bezier[k].p0[1] = py[j];
	    // p1 and p2 are fixed by continuity conditions
	    bezier[k].p3[0] = px[j+1];
	    bezier[k].p3[1] = py[j+1];
	    bezier[k].p4[0] = px[j+2];
	    bezier[k].p4[1] = py[j+2];
	    bezier[k].p5[0] = px[j+3];
	    bezier[k].p5[1] = py[j+3];
	    j += 3;
	  }
	  
	  // Continuity conditions
	  for(k = 1; k < GVF_PARAMETRIC_BARE_2D_BEZIER_N_SEG; k++){
	    bezier[k].p1[0] = 2*bezier[k-1].p5[0] - bezier[k-1].p4[0];
	    bezier[k].p1[1] = 2*bezier[k-1].p5[1] - bezier[k-1].p4[1];
	    bezier[k].p2[0] = 4*bezier[k-1].p5[0] - 4*bezier[k-1].p4[0] + bezier[k-1].p3[0];
	    bezier[k].p2[1] = 4*bezier[k-1].p5[1] - 4*bezier[k-1].p4[1] + bezier[k-1].p3[1];
	  }
	
	}
}

void gvf_parametric_bare_2d_bezier_splines_info(bare_bezier_t *bezier, int order, float *f1, float *f2, float *f1d, float *f2d, float *f1dd, float *f2dd)
{
	// How can we select in which bezier curve are we? Check w. spline zero: 0 <= t <= 1, spline ones: 1 <= t <= 2;
	float t = gvf_parametric_bare_control.w;
	int n_seg = floorl(t);
	float tt = t - n_seg;
	if(n_seg < 0){
		n_seg = 0;	// w could be < 0 in that case go to first point of first segment
		tt = 0;
	}
	// Evalute the corresponding bezier curve
	/*
	float p0x = bezier[n_seg].p0[0]; float p0y = bezier[n_seg].p0[1];
	float p1x = bezier[n_seg].p1[0]; float p1y = bezier[n_seg].p1[1];
	float p2x = bezier[n_seg].p2[0]; float p2y = bezier[n_seg].p2[1];
	float p3x = bezier[n_seg].p3[0]; float p3y = bezier[n_seg].p3[1];	
	float p4x = bezier[n_seg].p4[0]; float p5y = bezier[n_seg].p4[1];
	float p5x = bezier[n_seg].p5[0]; float p5y = bezier[n_seg].p5[1];	
  */
  
  // 0->5 (are 6)
  float ps[6][2];
  float fs[3][2];
  ps[0][0] = bezier[n_seg].p0[0]; ps[0][1] = bezier[n_seg].p0[1];
  ps[1][0] = bezier[n_seg].p1[0]; ps[1][1] = bezier[n_seg].p1[1];
  ps[2][0] = bezier[n_seg].p2[0]; ps[2][1] = bezier[n_seg].p2[1];
  ps[3][0] = bezier[n_seg].p3[0]; ps[3][1] = bezier[n_seg].p3[1];
  ps[4][0] = bezier[n_seg].p4[0]; ps[4][1] = bezier[n_seg].p4[1];
  ps[5][0] = bezier[n_seg].p5[0]; ps[5][1] = bezier[n_seg].p5[1];
  
  for(int j = 0; j < 3; j++){
    for(int l = 0; l < 2; l++){
      fs[j][l] = 0.0;
    }
  }
  int k; int p; int nu; int l;
  int n = order;
  float berst;

  // p = 0 curve. p = 1 first deriv. p = 2 second derivative.
  for(p = 0; p < 3; p++){
    for(k = 0; k <= n - p; k++){
      for(nu = 0; nu <= p; nu++){
        berst = berstein_poly(tt, k, n-p);
        fs[p][0] += berst * powf((-1), nu) * binom(nu, p) * ps[k+p-nu][0];
        fs[p][1] += berst * powf((-1), nu) * binom(nu, p) * ps[k+p-nu][1];
      }
    }
    for(l = 0; l <= p-1; l++){
      fs[p][0] *= (n-l);
      fs[p][1] *= (n-l);
    }
  }	  
  *f1 = fs[0][0];   *f2 = fs[0][1];
  *f1d = fs[1][0];  *f2d = fs[1][1];
  *f1dd = fs[2][0]; *f2dd = fs[2][1];
}

// Compute the berstein polynomial
//b_{mu,n}(x) = (mu n) * x^mu * (1-x)^(n-mu)
float berstein_poly(float t, int mu, int n){
  return binom(mu,n) * powf(t,mu) * powf(1-t, n - mu);
}

// Compute the binomial coefficient
float binom(float mu, float n){
  int i;
  float fact_n = 1;
  float fact_mu = 1;
  float fact_mun = 1;
  float binom;
  for(i = 0; i < n; i++)
    fact_n *= (n-i);
  for(i = 0; i < mu; i++)
    fact_mu *= (mu - i);
  for(i = 0; i < (n-mu); i++)
    fact_mun *= (n-mu-i); 
  return fact_n/(fact_mu * fact_mun);
}


