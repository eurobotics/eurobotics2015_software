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
 *  Revision : $Id: strat_utils.c,v 1.6 2009/05/27 20:04:07 zer0 Exp $
 *
 */


/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  strat_utils.c,v 1.6 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"

#include "main.h"
#include "strat_utils.h"
#include "strat.h"
#include "strat_base.h"
#include "sensor.h"
#include "i2c_protocol.h"

/* return the distance between two points */
int16_t distance_between(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	int32_t x,y;
	x = (x2-x1);
	x = x*x;
	y = (y2-y1);
	y = y*y;
	return sqrt(x+y);
}

/* return the distance to a point in the area */
int16_t distance_from_robot(int16_t x, int16_t y)
{
	return distance_between(position_get_x_s16(&mainboard.pos),
				position_get_y_s16(&mainboard.pos), x, y);
}

/** do a modulo 360 -> [-180,+180], knowing that 'a' is in [-3*180,+3*180] */  
int16_t simple_modulo_360(int16_t a)
{
	if (a < -180) {
		a += 360;
	}
	else if (a > 180) {
		a -= 360;
	}
	return a;
}

/** do a modulo 2.pi -> [-Pi,+Pi], knowing that 'a' is in [-3Pi,+3Pi] */  
double simple_modulo_2pi(double a)
{
	if (a < -M_PI) {
		a += M_2PI;
	}
	else if (a > M_PI) {
		a -= M_2PI;
	}
	return a;
}

/* return the distance to a point in the area */
int16_t angle_abs_to_rel(int16_t a_abs)
{
	return simple_modulo_360(a_abs - position_get_a_deg_s16(&mainboard.pos));
}

void rel_da_to_abs_xy(double d_rel, double a_rel_rad, 
		      double *x_abs, double *y_abs)
{
	double x = position_get_x_double(&mainboard.pos); 
	double y = position_get_y_double(&mainboard.pos);
	double a = position_get_a_rad_double(&mainboard.pos);

	*x_abs = x + d_rel*cos(a+a_rel_rad);
	*y_abs = y + d_rel*sin(a+a_rel_rad);
}

double norm(double x, double y)
{
	return sqrt(x*x + y*y);
}

void rel_xy_to_abs_xy(double x_rel, double y_rel, 
		      double *x_abs, double *y_abs)
{
	double d_rel, a_rel;
	d_rel = norm(x_rel, y_rel);
	a_rel = atan2(y_rel, x_rel);
	rel_da_to_abs_xy(d_rel, a_rel, x_abs, y_abs);
}

/* return an angle between -pi and pi */
void abs_xy_to_rel_da(double x_abs, double y_abs, 
		      double *d_rel, double *a_rel_rad)
{
	double x = position_get_x_double(&mainboard.pos); 
	double y = position_get_y_double(&mainboard.pos);
	double a = position_get_a_rad_double(&mainboard.pos);
	
	*a_rel_rad = atan2(y_abs - y, x_abs - x) - a;
	if (*a_rel_rad < -M_PI) {
		*a_rel_rad += M_2PI;
	}
	else if (*a_rel_rad > M_PI) {
		*a_rel_rad -= M_2PI;
	}
	*d_rel = norm(x_abs-x, y_abs-y);
}

void rotate(double *x, double *y, double rot)
{
	double l, a;
	
	l = norm(*x, *y);
	a = atan2(*y, *x);

	a += rot;
	*x = l * cos(a);
	*y = l * sin(a);
}

/* return true if the point is in area */
uint8_t is_in_area(int16_t x, int16_t y, int16_t margin)
{
	if (x < margin)
		return 0;
	if (x > (AREA_X - margin))
		return 0;
	if (y < margin)
		return 0;
	if (y > (AREA_Y - margin))
		return 0;
	return 1;
}

uint8_t point_is_in_area(int16_t px, int16_t py,
								 int16_t x_up, int16_t y_up,
								 int16_t x_down, int16_t y_down)
{
	if (px < x_up)
		return 0;
	if (px > x_down)
		return 0;
	if (py > y_up)
		return 0;
	if (py < y_down)
		return 0;

	return 1;
}

/* return true if the point is in area */
uint8_t robot_is_in_area(int16_t margin)
{
	return is_in_area(position_get_x_s16(&mainboard.pos),
			  position_get_y_s16(&mainboard.pos),
			  margin);
}


/* return 1 or 0 depending on which side of a line (y=cste) is the
 * robot. works in red or green color. */
uint8_t y_is_more_than(int16_t y)
{
	int16_t posy;
	
	posy = position_get_y_s16(&mainboard.pos);
	if (posy > y)
		return 1;
	else
		return 0;

}



/* return 1 or 0 depending on which side of a line (x=cste) is the
 * robot. works in red or green color. */
uint8_t x_is_more_than(int16_t x)
{
	int16_t posx;
	
	posx = position_get_x_s16(&mainboard.pos);
	if (mainboard.our_color == I2C_COLOR_BLUE) {
		if (posx > x)
			return 1;
		else
			return 0;
	}
	else {
		if (posx < (AREA_X-x))
			return 1;
		else
			return 0;
	}
}


/* return 1 if x > x_opp or opponent not there */
uint8_t opp_x_is_more_than(int16_t x)
{
	int16_t x_opp, y_opp;
	
	if(get_opponent_xy(&x_opp, &y_opp) == -1)
		return 1;

	if (mainboard.our_color == I2C_COLOR_BLUE) {
		if (x_opp > x)
			return 1;
		else
			return 0;
	}
	else {
		if (x_opp < (AREA_X-x))
			return 1;
		else
			return 0;
	}
}

/* return 1 if y > y_opp or opponent not there */
uint8_t opp_y_is_more_than(int16_t y)
{
	int16_t x_opp, y_opp;
	
	if(get_opponent_xy(&x_opp, &y_opp) == -1)
		return 1;

	if (y_opp > y)
		return 1;
	else
		return 0;

}

int16_t sin_table[] = {
	0,
	3211,
	6392,
	9512,
	12539,
	15446,
	18204,
	20787,
	23170,
	25330,
	27245,
	28898,
	30273,
	31357,
	32138,
	32610,
	32767,
};

int16_t fast_sin(int16_t deg)
{
	deg %= 360;
	
	if (deg < 0)
		deg += 360;

	if (deg < 90) 
		return sin_table[(deg*16)/90];
	else if (deg < 180) 
		return sin_table[((180-deg)*16)/90];
	else if (deg < 270) 
		return -sin_table[((deg-180)*16)/90];
	else
		return -sin_table[((360-deg)*16)/90];
}

int16_t fast_cos(int16_t deg)
{
	return fast_sin(90+deg);
}


/* get the color of our robot */
uint8_t get_color(void)
{
	return mainboard.our_color;
}

/* get the color of the opponent robot */
uint8_t get_opponent_color(void)
{
	if (mainboard.our_color == I2C_COLOR_RED)
		return I2C_COLOR_BLUE;
	else
		return I2C_COLOR_RED;
}

/* get the xy pos of the opponent robot */
int8_t get_opponent_xy(int16_t *x, int16_t *y)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	*x = beaconboard.opponent_x;
	*y = beaconboard.opponent_y;
	IRQ_UNLOCK(flags);
	if (*x == I2C_OPPONENT_NOT_THERE)
		return -1;
	return 0;
}

/* get the da pos of the opponent robot */
int8_t get_opponent_da(int16_t *d, int16_t *a)
{
	uint8_t flags;
	int16_t x_tmp;
	IRQ_LOCK(flags);
	x_tmp = beaconboard.opponent_x;
	*d = beaconboard.opponent_d;
	*a = beaconboard.opponent_a;
	IRQ_UNLOCK(flags);
	if (x_tmp == I2C_OPPONENT_NOT_THERE)
		return -1;
	return 0;
}

/* get the da pos of the opponent robot */
int8_t get_opponent_xyda(int16_t *x, int16_t *y, int16_t *d, int16_t *a)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	*x = beaconboard.opponent_x;
	*y = beaconboard.opponent_y;
	*d = beaconboard.opponent_d;
	*a = beaconboard.opponent_a;
	IRQ_UNLOCK(flags);

	if (*x == I2C_OPPONENT_NOT_THERE)
		return -1;
	return 0;
}

uint8_t opponent_is_behind(void)
{
	int8_t opp_there;
	int16_t opp_d, opp_a;

	opp_there = get_opponent_da(&opp_d, &opp_a);

	if(opp_there == -1)
		return 0;

	if ((opp_a < 215 && opp_a > 145) && opp_d < 500)
		return 1;

	return 0;
}

uint8_t opponent_is_infront(void)
{
	int8_t opp_there;
	int16_t opp_d, opp_a;

	opp_there = get_opponent_da(&opp_d, &opp_a);
	
	if(opp_there == -1)
		return 0;

	if ((opp_a > 325 || opp_a < 35) && opp_d < 500)
		return 1;

	return 0;
}


/* return 1 if opp is in area, COLOR is take in acount!! */
uint8_t opponent_is_in_area(int16_t x_up, int16_t y_up,
									 int16_t x_down, int16_t y_down)
{
	int8_t opp_there;
	int16_t opp_x, opp_y;

	opp_there = get_opponent_xy(&opp_x, &opp_y);

	if(opp_there == -1)
		return 0;

	if (mainboard.our_color == I2C_COLOR_BLUE) {
		if ((opp_x > x_up && opp_x < x_down)
			&& (opp_y < y_up && opp_y > y_down) )
			return 1;
	}
	else {
		if ((opp_x < x_up && opp_x > x_down)
			 && (opp_y < y_up && opp_y > y_down) )
			return 1;
	}
	return 0;
}


/* return 1 if opponent is in slot (i,j) */
uint8_t opponent_is_in_slot(int8_t i, int8_t j)
{
	int8_t opp_there, opp_i, opp_j;
	int16_t opp_x, opp_y;

	opp_there = get_opponent_xy(&opp_x, &opp_y);

	if(opp_there == -1)
		return 0;

	get_slot_index(opp_x, opp_y, &opp_i, &opp_j);
	
	if(i == opp_i && j == opp_j)
		return 1;
	else
		return 0;
}

uint8_t opponent_is_near_to_slot(int8_t i, int8_t j)
{
	int8_t opp_there, opp_i, opp_j;
	int16_t opp_x, opp_y;

	opp_there = get_opponent_xy(&opp_x, &opp_y);

	if(opp_there == -1)
		return 0;

	get_slot_index(opp_x, opp_y, &opp_i, &opp_j);
	
	if( (ABS(i-opp_i) < 2)
		&& (ABS(j-opp_j) < 2) ) {
		return 1;
	}

	return 0;
}


uint8_t opponent_is_near_to_target_slot(int8_t i, int8_t j)
{
	int8_t opp_there, opp_i, opp_j;
	int16_t opp_x, opp_y;

	opp_there = get_opponent_xy(&opp_x, &opp_y);

	if(opp_there == -1)
		return 0;

	get_slot_index(opp_x, opp_y, &opp_i, &opp_j);
	
	if( (ABS(i-opp_i) < 2) && (ABS(j-opp_j) < 2)
		&& (i == opp_i || j == opp_j) ) {
		return 1;
	}

	return 0;
}

uint8_t opponent_is_behind_side(uint8_t side)
{
	if(side == SIDE_FRONT)
		return opponent_is_behind();
	else
		return opponent_is_infront();
}

uint8_t opponent_is_infront_side(uint8_t side)
{
	if(side == SIDE_REAR)
		return opponent_is_behind();
	else
		return opponent_is_infront();
}


uint8_t opponent_is_in_near_slots(void)
{
	int8_t opp_there;
	int16_t opp_d, opp_a;

	opp_there = get_opponent_da(&opp_d, &opp_a);

	if(opp_there == -1)
		return 0;

	if (opp_d < 400)
		return 1;

	return 0;
}

uint8_t token_catched(uint8_t side)
{
	return slavedspic.ts[side].token_catched;
}


/* return the score of a token on side */ 
uint8_t token_side_score(uint8_t side)
{
	if(side == SIDE_FRONT) {
		if(sensor_get(S_TOKEN_FRONT_TOWER2H))
			return TOWER2H_SCORE;
		else if(sensor_get(S_TOKEN_FRONT_TOWER1H))
			return TOWER1H_SCORE;
		else if(sensor_get(S_TOKEN_FRONT_FIGURE))
			return FIGURE_SCORE;
		else if(slavedspic.ts[SIDE_FRONT].token_catched)
			return PION_SCORE;
	}
	else if(side == SIDE_REAR) {
		if(sensor_get(S_TOKEN_REAR_TOWER2H))
			return TOWER2H_SCORE;
		else if(sensor_get(S_TOKEN_REAR_TOWER1H))
			return TOWER1H_SCORE;
		else if(sensor_get(S_TOKEN_REAR_FIGURE))
			return FIGURE_SCORE;
		else if(slavedspic.ts[SIDE_REAR].token_catched)
			return PION_SCORE;			
	}

	return 0;
}

/* return 1 if there is a token on side and has the lower priority */
uint8_t token_side_is_lower_score(uint8_t side)
{
	/* no token catched */
	if(!token_catched(side))
		return 0;

	/* check if is the lower or equal */
	if(side == SIDE_FRONT) {
		if(token_side_score(SIDE_FRONT) <= token_side_score(SIDE_REAR))
			return 1;
	}
	else if(side == SIDE_REAR) {
		if(token_side_score(SIDE_REAR) <= token_side_score(SIDE_FRONT))
			return 1;
	}

	/* isn't the lowest */
	return 0;
}

uint8_t belts_blocked(uint8_t side)
{
	return slavedspic.ts[side].belts_blocked;
}

uint8_t token_inside(uint8_t side)
{
	return (slavedspic.ts[side].state == 0);
}


/* turn to pickup token, return side used to pickup */
/* suppose that there is at least one side empty */
uint8_t strat_turnto_pickup_token(struct trajectory*traj, double x_abs_mm, double y_abs_mm)
{
	double d_rel;
	double a_rel_rad;

	/* get angle to token xy */
	abs_xy_to_rel_da(x_abs_mm, y_abs_mm, &d_rel, &a_rel_rad);

	if(ABS(a_rel_rad) < (M_PI/2)) {
		if(!token_catched(SIDE_FRONT)) {
			trajectory_turnto_xy(traj, x_abs_mm, y_abs_mm);
			return SIDE_FRONT;
		}
		else if(!token_catched(SIDE_REAR)){
			trajectory_turnto_xy_behind(traj, x_abs_mm, y_abs_mm);
			return SIDE_REAR;
		}
		/* XXX never should be reached */
		return SIDE_FRONT;
	}	
	else {
		if(!token_catched(SIDE_REAR)) {
			trajectory_turnto_xy_behind(traj, x_abs_mm, y_abs_mm);
			return SIDE_REAR;
		}
		else if(!token_catched(SIDE_FRONT)) {
			trajectory_turnto_xy(traj, x_abs_mm, y_abs_mm);
			return SIDE_FRONT;
		}
		/* XXX never should be reached */
		return SIDE_FRONT;
	}
}

/* turn to place token automaticaly, return side used to place */
/* suppose that there is at least one token catched */
uint8_t strat_turnto_place_token(struct trajectory*traj, double x_abs_mm, double y_abs_mm, uint8_t go)
{
	double d_rel;
	double a_rel_rad;

	/* get angle to token xy */
	abs_xy_to_rel_da(x_abs_mm, y_abs_mm, &d_rel, &a_rel_rad);

	if(go == GO_FORWARD) {
		if(ABS(a_rel_rad) < (M_PI/2)) {
			if(token_catched(SIDE_FRONT)) {
				trajectory_turnto_xy(traj, x_abs_mm, y_abs_mm);
				return SIDE_FRONT;
			}
			else if(token_catched(SIDE_REAR)) {
				trajectory_turnto_xy_behind(traj, x_abs_mm, y_abs_mm);
				return SIDE_REAR;
			}
			/* XXX never should be reached */
			return SIDE_FRONT;
		}	
		else {
			if(token_catched(SIDE_REAR)) {
				trajectory_turnto_xy_behind(traj, x_abs_mm, y_abs_mm);
				return SIDE_REAR;
			}
			else if(token_catched(SIDE_FRONT)) {
				trajectory_turnto_xy(traj, x_abs_mm, y_abs_mm);
				return SIDE_FRONT;
			}
			/* XXX never should be reached */
			return SIDE_REAR;
		}
	}
	else {
		if(ABS(a_rel_rad) < (M_PI/2)) {
			if(token_catched(SIDE_FRONT)) {
				trajectory_turnto_xy_behind(traj, x_abs_mm, y_abs_mm);
				return SIDE_FRONT;
			}
			else if(token_catched(SIDE_REAR)) {
				trajectory_turnto_xy(traj, x_abs_mm, y_abs_mm);
				return SIDE_REAR;
			}
			/* XXX never should be reached */
			return SIDE_FRONT;
		}	
		else {
			if(token_catched(SIDE_REAR)) {
				trajectory_turnto_xy(traj, x_abs_mm, y_abs_mm);
				return SIDE_REAR;
			}
			else if(token_catched(SIDE_FRONT)) {
				trajectory_turnto_xy_behind(traj, x_abs_mm, y_abs_mm);
				return SIDE_FRONT;
			}
			/* XXX never should be reached */
			return SIDE_REAR;
		}
	}
}

/* go straight forward with no side dependence (d is in mm) */
void strat_d_rel_side(struct trajectory*traj, double d_mm, uint8_t side)
{
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, d_mm);
	else
		trajectory_d_rel(&mainboard.traj, -d_mm);
}

/* return 1 if the opponent is near */
void wait_until_opponent_is_far(void)
{
//#ifdef HOMOLOGATION
	int16_t opp_x, opp_y, opp_d, opp_a;

	if (get_opponent_xyda(&opp_x, &opp_y, &opp_d, &opp_a) == -1)
		return;

	if(opp_d < 600 ) {
		DEBUG(E_USER_STRAT, "waiting opponent far");

		do {
			if (get_opponent_xyda(&opp_x, &opp_y,
					      &opp_d, &opp_a) == -1)
				return;
	
		} while(opp_d < 600);
	}
//#endif
}

/* apply flags to slot */
void strat_set_slot_flags(int16_t x, int16_t y, uint16_t flags)
{
	int8_t i, j;

	/* slot index */
	i = (int8_t)(x/SLOT_SIZE);
	j = (int8_t)(y/SLOT_SIZE);

	/* saturators */
	if(i >= NB_SLOT_X)
		i = (NB_SLOT_X-1);
	if(j >= NB_SLOT_Y)
		j = (NB_SLOT_Y-1);

	/* apply flags */
	strat_infos.slot[i][j].flags |= flags;

}

/* apply flags to slot */
void strat_clear_slot_flags(int16_t x, int16_t y, uint16_t flags)
{
	int8_t i, j;

	/* slot index */
	i = (int8_t)(x/SLOT_SIZE);
	j = (int8_t)(y/SLOT_SIZE);

	/* saturators */
	if(i >= NB_SLOT_X)
		i = (NB_SLOT_X-1);
	if(j >= NB_SLOT_Y)
		j = (NB_SLOT_Y-1);

	/* apply flags */
	strat_infos.slot[i][j].flags &= ~(flags);
}



/* get index (i,j) of slot from (x,y) coordinates */
void get_slot_index(int16_t x, int16_t y, int8_t *i, int8_t *j)
{
	*i = (int8_t)(x/SLOT_SIZE);
	*j = (int8_t)(y/SLOT_SIZE);

	/* saturators */
	if(*i >= NB_SLOT_X)
		*i = (NB_SLOT_X-1);
	if(*j >= NB_SLOT_Y)
		*j = (NB_SLOT_Y-1);
}
