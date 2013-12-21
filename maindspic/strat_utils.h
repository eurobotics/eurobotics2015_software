/*  
 *  Copyright Droids Corporation, Microb Technology (2009)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id: strat_utils.h,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  strat_utils.h,v 1.4 2009/05/27 20:04:07 zer0 Exp.
 */

#ifndef _STRAT_UTILS_H_
#define _STRAT_UTILS_H_

struct xy_point {
	int16_t x;
	int16_t y;
};

/* wait traj end flag or cond. return 0 if cond become true, else
 * return the traj flag */
#define WAIT_COND_OR_TRAJ_END(cond, mask)				\
	({								\
		uint8_t __err = 0;					\
		while ( (! (cond)) && (__err == 0)) {			\
			__err = test_traj_end(mask);	\
		}							\
		__err;							\
	})								\

int16_t distance_between(int16_t x1, int16_t y1, int16_t x2, int16_t y2);
int16_t distance_from_robot(int16_t x, int16_t y);
int16_t distance_from_robot_signed(int16_t x, int16_t y);

int16_t simple_modulo_360(int16_t a);
double simple_modulo_2pi(double a);

int16_t angle_abs_to_rel(int16_t a_abs);

void rel_da_to_abs_xy(double d_rel, double a_rel_rad, double *x_abs, double *y_abs);

double norm(double x, double y);

void rel_xy_to_abs_xy(double x_rel, double y_rel, double *x_abs, double *y_abs);
void abs_xy_to_rel_da(double x_abs, double y_abs, double *d_rel, double *a_rel_rad);

void rotate(double *x, double *y, double rot);

uint8_t is_in_area(int16_t x, int16_t y, int16_t margin);
uint8_t point_is_in_area(int16_t px, int16_t py,
								 int16_t x_up, int16_t y_up,
								 int16_t x_down, int16_t y_down);
uint8_t robot_is_in_area(int16_t margin);

uint8_t y_is_more_than(int16_t y);
uint8_t x_is_more_than(int16_t x);

uint8_t opp_x_is_more_than(int16_t x);
uint8_t opp2_x_is_more_than(int16_t x);
uint8_t robot_2nd_x_is_more_than(int16_t x);
uint8_t opp_y_is_more_than(int16_t y);
uint8_t opp2_y_is_more_than(int16_t y);
uint8_t robot_2nd_y_is_more_than(int16_t y);

int16_t fast_sin(int16_t deg);
int16_t fast_cos(int16_t deg);

uint8_t get_color(void);
uint8_t get_opponent_color(void);

int8_t get_opponent_xy(int16_t *x, int16_t *y);
int8_t get_opponent2_xy(int16_t *x, int16_t *y);
int8_t get_robot_2nd_xy(int16_t *x, int16_t *y);
int8_t get_opponent_da(int16_t *d, int16_t *a);
int8_t get_opponent2_da(int16_t *d, int16_t *a);
int8_t get_robot_2nd_da(int16_t *d, int16_t *a);
int8_t get_opponent_xyda(int16_t *x, int16_t *y, int16_t *d, int16_t *a);
int8_t get_opponent2_xyda(int16_t *x, int16_t *y, int16_t *d, int16_t *a);
int8_t get_robot_2nd_xyda(int16_t *x, int16_t *y, int16_t *d, int16_t *a);

uint8_t opponent_is_behind(void);
uint8_t opponent_is_infront(void);
uint8_t robots_behind(void);
uint8_t robots_infront(void);
/*
uint8_t opponent_is_behind_side(uint8_t side);
uint8_t opponent_is_infront_side(uint8_t side);
*/

uint8_t opponent_is_in_area(int16_t x_up, int16_t y_up,
									 int16_t x_down, int16_t y_down);

void strat_auto_position(void);
#endif

