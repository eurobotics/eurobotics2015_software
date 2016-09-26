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
 *  Revision : $Id$
 *
 */

/*
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org> and Javier Rodriguez Puigvert
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  strat_avoid.c,v 1.4 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#ifndef HOST_VERSION_OA_TEST
#include <uart.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <clock_time.h>

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

#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_utils.h"
#include "sensor.h"

#else
#warning HOST_VERSION_OA_TEST

#define M_2PI (M_PI*2)
#define DEG(x) ((x) * (180.0 / M_PI))
#define RAD(x) ((x) * (M_PI / 180.0))

#define E_USER_STRAT 200

#define END_TRAJ       1 /* traj successful */
#define END_BLOCKING   2 /* blocking during traj */
#define END_NEAR       4 /* we are near destination */
#define END_OBSTACLE   8 /* There is an obstacle in front of us */
#define END_ERROR     16 /* Cannot do the command */
#define END_INTR      32 /* interrupted by user */
#define END_TIMER     64 /* we don't a lot of time */
#define END_RESERVED 128 /* reserved */

#endif

#if defined(HOMOLOGATION)
#warning HOMOLOGATION compilation

/* /!\ half size */

#define O_HOME_WIDTH 200

#define O_HOME_HEIGHT 222

#define O_STAIRS_WIDTH 533
#define O_STAIRS_HEIGHT 265

#define O_WIDTH  400
#define O_LENGTH 550
#else
/* /!\ half size */
#define O_WIDTH  330 //360
#define O_LENGTH 500
#define O_HOME_WIDTH 200
#define O_HOME_HEIGHT 222
#define O_STAIRS_WIDTH 533
#define O_STAIRS_HEIGHT 300 //265
#define O_PLATFORM_WIDTH 300
#define O_PLATFORM_HEIGHT 50

#endif

#ifdef IM_SECONDARY_ROBOT
#warning IM SENCONDARY ROBOT

/* /!\ half size */
#define ROBOT_2ND_WIDTH  ((330/2)+OBS_CLERANCE)
#define ROBOT_2ND_LENGTH ((282/2)+OBS_CLERANCE)
#else
#warning IM MAIN ROBOT
/* /!\ half size */
#define ROBOT_2ND_WIDTH  ((230/2)+OBS_CLERANCE)
#define ROBOT_2ND_LENGTH ((150/2)+OBS_CLERANCE)
#endif

#define CENTER_X 1500
#define CENTER_Y 1000

#define STAIRS_X 1500
#define STAIRS_Y 1710

#define HOME_GREEN_X 2800
#define HOME_GREEN_Y 1000

#define HOME_YELLOW_X 200
#define HOME_YELLOW_Y 1000
#define PLATFORM_X 1500

#define PLATFORM_Y 50


/* don't care about polygons further than this distance for escape */
#define ESCAPE_POLY_THRES 500

/* XXX don't reduce opp if opp is too far */
#define REDUCE_POLY_THRES 3500

/* has to be longer than any poly */
#define ESCAPE_VECT_LEN 3000


#ifdef HOST_VERSION_OA_TEST
int16_t g_robot_x;
int16_t g_robot_y;
double  g_robot_a;

int16_t g_opp1_x;
int16_t g_opp1_y;
int16_t g_opp2_x;
int16_t g_opp2_y;
int16_t g_robot_2nd_x;
int16_t g_robot_2nd_y;
#endif

#ifdef HOST_VERSION_OA_TEST
#warning HOST_VERSION_OA_TEST compilation
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
#endif


/* normalize vector from origin */
#if HOST_VERSION_OA_TEST
#warning HOST_VERSION_OA_TEST compilation
double norm(double x, double y)
{
	return sqrt(x*x + y*y);
}
#endif

static int16_t obs_clerance = OBS_CLERANCE;

uint8_t  clerance_minimum_enabled = 0;

void clerance_minimum_enable (void)
{
    clerance_minimum_enabled = 1;
}

void clerance_minimum_disable (void)
{
    clerance_minimum_enabled = 0;
}

/* rotate point */
#ifdef HOST_VERSION_OA_TEST
#warning HOST_VERSION_OA_TEST compilation
void rotate(double *x, double *y, double rot)
{
	double l, a;

	l = norm(*x, *y);
	a = atan2(*y, *x);

	a += rot;
	*x = l * cos(a);
	*y = l * sin(a);
}
#endif

/* set rotated poly relative to robot coordinates */
void set_rotated_poly(poly_t *pol, const point_t *robot_pt,
		      int16_t w, int16_t l, int16_t x, int16_t y)

{
	double tmp_x, tmp_y;
	double a_rad = 0.0;

	/* calcule relative angle to robot */
	a_rad = atan2((double)(y - robot_pt->y), (double)(x - robot_pt->x));

	DEBUG(E_USER_STRAT, "%s() x,y=%d,%d a_rad=%2.2f",
	      __FUNCTION__, x, y, a_rad);

	/* point 1 */
	tmp_x = w;
	tmp_y = l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 0);

	/* point 2 */
	tmp_x = -w;
	tmp_y = l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 1);

	/* point 3 */
	tmp_x = -w;
	tmp_y = -l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 2);

	/* point 4 */
	tmp_x = w;
	tmp_y = -l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 3);
}
/* set rotated poly relative to robot coordinates */
void set_rotated_poly_abs(poly_t *pol, int16_t a_abs,
		      int16_t w, int16_t l, int16_t x, int16_t y)

{
	double tmp_x, tmp_y;
	double a_rad = 0.0;

	/* calcule absolute of poly */
	a_rad = RAD(a_abs);

	DEBUG(E_USER_STRAT, "%s() x,y=%d,%d a_rad=%2.2f",
	      __FUNCTION__, x, y, a_rad);

	/* point 1 */
	tmp_x = w;
	tmp_y = l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 0);

	/* point 2 */
	tmp_x = -w;
	tmp_y = l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 1);

	/* point 3 */
	tmp_x = -w;
	tmp_y = -l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 2);

	/* point 4 */
	tmp_x = w;
	tmp_y = -l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 3);
}

/* set poly*/
void set_home_yellow_poly_abs(poly_t *pol,
		      int16_t w, int16_t l, int16_t x, int16_t y)

{
	double tmp_x, tmp_y;

	/* point 1 */
	tmp_x = w;
	tmp_y = l;
	tmp_x += x;
	tmp_y += y;

	oa_poly_set_point(pol, tmp_x, tmp_y, 0);

	/* point 2 */
	tmp_x = -w;
	tmp_y = l;
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 1);

	/* point 3 */
	tmp_x = -w;
	tmp_y = -l;
	tmp_x += x;
	tmp_y += y;

	oa_poly_set_point(pol, tmp_x, tmp_y, 2);

	/* point 4 */
	tmp_x = w;
	tmp_y = -l;
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 3);
	/* point 5 */

#ifdef IM_SECONDARY_ROBOT
		tmp_x = 650+OBS_CLERANCE;
#else
		tmp_x = 650+OBS_CLERANCE/2;
#endif
	tmp_y = 1000;

	oa_poly_set_point(pol, tmp_x, tmp_y, 4);
}
/* set poly*/
void set_home_green_poly_abs(poly_t *pol,
		      int16_t w, int16_t l, int16_t x, int16_t y)

{
	double tmp_x, tmp_y;

	/* point 1 */
	tmp_x = w;
	tmp_y = l;
	tmp_x += x;
	tmp_y += y;

	oa_poly_set_point(pol, tmp_x, tmp_y, 0);

	/* point 2 */
	tmp_x = -w;
	tmp_y = l;
	tmp_x += x;
	tmp_y += y;

	oa_poly_set_point(pol, tmp_x, tmp_y, 1);

	/* point 2 */
#ifdef IM_SECONDARY_ROBOT
		tmp_x = 2350-OBS_CLERANCE;
#else
		tmp_x = 2350-OBS_CLERANCE/2;
#endif

	tmp_y = 1000;

	oa_poly_set_point(pol, tmp_x, tmp_y, 2);
	/* point 3 */
	tmp_x = -w;
	tmp_y = -l;
	tmp_x += x;
	tmp_y += y;

	oa_poly_set_point(pol, tmp_x, tmp_y, 3);

	/* point 4 */
	tmp_x = w;
	tmp_y = -l;
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 4);
}
/* set poly*/
void set_poly_abs(poly_t *pol,
		      int16_t w, int16_t l, int16_t x, int16_t y)

{
	double tmp_x, tmp_y;

	/* point 1 */
	tmp_x = w;
	tmp_y = l;
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 0);

	/* point 2 */
	tmp_x = -w;
	tmp_y = l;
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 1);

	/* point 3 */
	tmp_x = -w;
	tmp_y = -l;
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 2);

	/* point 4 */
	tmp_x = w;
	tmp_y = -l;
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 3);
}


#if 0
#define EDGE_NUMBER 5
void set_rotated_pentagon_manual(poly_t *pol, const point_t *robot_pt,
			  int16_t radius, int16_t x, int16_t y, double a_rad)
{

	double c_a, s_a;
	uint8_t i;
	double px1, py1, px2, py2;
	//double a_rad;

	//a_rad = atan2(y - robot_pt->y, x - robot_pt->x);

	/* generate pentagon  */
	c_a = cos(-2*M_PI/EDGE_NUMBER);
	s_a = sin(-2*M_PI/EDGE_NUMBER);

	px1 = radius * cos(a_rad + 2*M_PI/(2*EDGE_NUMBER));
	py1 = radius * sin(a_rad + 2*M_PI/(2*EDGE_NUMBER));


	for (i = 0; i < EDGE_NUMBER; i++){
		oa_poly_set_point(pol, x + px1, y + py1, i);

		px2 = px1*c_a + py1*s_a;
		py2 = -px1*s_a + py1*c_a;

		px1 = px2;
		py1 = py2;
	}
}
#endif

#define EDGE_NUMBER 5
void set_rotated_pentagon(poly_t *pol, const point_t *robot_pt,
			  int16_t radius, int16_t x, int16_t y)
{

	double c_a, s_a;
	uint8_t i;
	double px1, py1, px2, py2;
	double a_rad;

	a_rad = atan2(y - robot_pt->y, x - robot_pt->x);

	/* generate pentagon  */
	c_a = cos(-2*M_PI/EDGE_NUMBER);
	s_a = sin(-2*M_PI/EDGE_NUMBER);

	/*
	px1 = radius;
	py1 = 0;
	*/
	px1 = radius * cos(a_rad + 2*M_PI/(2*EDGE_NUMBER));
	py1 = radius * sin(a_rad + 2*M_PI/(2*EDGE_NUMBER));


	for (i = 0; i < EDGE_NUMBER; i++){
		oa_poly_set_point(pol, x + px1, y + py1, i);

		px2 = px1*c_a + py1*s_a;
		py2 = -px1*s_a + py1*c_a;

		px1 = px2;
  	py1 = py2;
	}
}

/*WWW set totem islands polygon
void set_heartfire_poly(poly_t *pol, point_t *robot_pt, int16_t rad)
{
  set_rotated_pentagon(pol, robot_pt,
	  rad, HEARTFIRE_X, HEARTFIRE_Y);
}
*/
/* set poly that represent the opponent */
void set_opponent_poly(uint8_t type, poly_t *pol, const point_t *robot_pt, int16_t w, int16_t l)
{
#define OPP1      0
#define OPP2      1
#define ROBOT2ND  2

	int16_t x=0, y=0, a_abs=0;
	int8_t *name = NULL;
	int8_t opp1[] = "opponent 1";
	int8_t opp2[] = "opponent 2";
	int8_t robot_2nd[] = "robot 2nd";

#ifndef HOST_VERSION_OA_TEST
   if(type == OPP1) {
	   get_opponent1_xy(&x, &y);
	   name = opp1;

	}
	else if(type == OPP2) {
	   get_opponent2_xy(&x, &y);
	   name = opp2;

	}
	else if(type == ROBOT2ND) {
		 get_robot_2nd_xy(&x, &y);
     get_robot_2nd_a_abs(&a_abs);
	   name = robot_2nd;
	}
#else
#warning HOST_VERSION_OA_TEST compilation
   if(type == OPP1) {
	   x = g_opp1_x;
	   y = g_opp1_y;
	   name = opp1;
	}
	else if(type == OPP2) {
	   x = g_opp2_x;
	   y = g_opp2_y;
	   name = opp2;
	}
	else if(type == ROBOT2ND) {
     /* TODO abs angle */
	   x = g_robot_2nd_x;
	   y = g_robot_2nd_y;
	   name = robot_2nd;
	}
#endif
	else
	   ERROR(E_USER_STRAT, "ERROR at %s", __FUNCTION__);
	DEBUG(E_USER_STRAT, "%s at: %d %d", name, x, y);

	/* place poly even if invalid, because it's -1000 */
  if(type == ROBOT2ND){
		DEBUG(E_USER_STRAT, "x1:%d, y1: %d, w1: %d,l1: %d",x,y, w,l );
 	//set_rotated_poly_abs(pol, a_abs, w, l, x, y);
	set_rotated_poly_abs(pol, 90, w, l, x, y);
	}

  else{
		DEBUG(E_USER_STRAT, "x2:%d, y2: %d, w2: %d,l2: %d",x,y, w,l );
		set_rotated_poly(pol, robot_pt, w, l, x, y);
	}

}



/*
 * Go in playground, loop until out of poly. The argument robot_pt is
 * the pointer to the current position of the robot.
 * Return 0 if there was nothing to do.
 * Return 1 if we had to move. In this case, update the theorical
 * position of the robot in robot_pt.
 */
static int8_t go_in_area(point_t *robot_pt)
{
	point_t  dst_pt;

	/* Go in playground */
	if (!is_in_boundingbox(robot_pt)){
		NOTICE(E_USER_STRAT, "not in playground %"PRId32", %"PRId32"",
		       (int32_t)robot_pt->x, (int32_t)robot_pt->y);
		NOTICE(E_USER_STRAT, "not in playground %"PRId32", %"PRId32"",
		       (int32_t)robot_pt->x, (int32_t)robot_pt->y);

		//The robot will calculate the intersection in bbox.

		if(robot_pt->y > LIMIT_BBOX_Y_UP){
			dst_pt.y = LIMIT_BBOX_Y_UP;
		}else if(robot_pt->y < LIMIT_BBOX_Y_DOWN){
			dst_pt.y = LIMIT_BBOX_Y_DOWN;
		}else{
			dst_pt.y= robot_pt->y;
		}

		if(robot_pt->x > LIMIT_BBOX_X_UP){
			dst_pt.x = LIMIT_BBOX_X_UP;
		}else if(robot_pt->x< LIMIT_BBOX_X_DOWN){
			dst_pt.x = LIMIT_BBOX_X_DOWN;
		}else{
			dst_pt.x= robot_pt->x;
		}
		NOTICE(E_USER_STRAT, "pt dst %"PRId32", %"PRId32"", (int32_t)dst_pt.x, (int32_t)dst_pt.y);

		NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
		       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

		/* scape from poly */
#ifndef HOST_VERSION_OA_TEST
		strat_goto_xy_force(dst_pt.x, dst_pt.y);
#endif
		robot_pt->x = dst_pt.x;
		robot_pt->y = dst_pt.y;


		return 1;
	}

	return 0;
}


/*
 * Escape from polygons if needed.
 * robot_pt is the current position of the robot, it will be
 * updated.
 */
static int8_t escape_from_poly(point_t *robot_pt, int16_t robot_2nd_x, int16_t robot_2nd_y,
										int16_t opp1_x, int16_t opp1_y,
										int16_t opp2_x, int16_t opp2_y,
										poly_t *pol_opp1, poly_t *pol_opp2,
										poly_t *pol_robot_2nd,poly_t *pol_home_green,
										poly_t *pol_home_yellow,poly_t *pol_stairs,poly_t *pol_platform)
{

	uint8_t in_opp1 = 0, in_opp2 = 0, in_robot_2nd = 0;
	double escape_dx = 0, escape_dy = 0;
	double opp1_dx = 0, opp1_dy = 0;
	double opp2_dx = 0, opp2_dy = 0;
	double robot_2nd_dx = 0, robot_2nd_dy = 0;
	double stairs_dx = 0, stairs_dy = 0;
	double home_green_dx = 0, home_green_dy=0;
	double home_yellow_dx = 0, home_yellow_dy=0;
	double platform_dx = 0, platform_dy=0;


	double len;

    uint8_t in_home_yellow = 0,in_home_green=0,in_stairs = 0,in_platform=0;

	point_t dst_pt;
	point_t intersect_opp1_pt, intersect_opp2_pt,  intersect_robot_2nd_pt,
	 intersect_home_yellow_pt,intersect_home_green_pt,intersect_stairs_pt,intersect_platform_pt;


	/* check if we are in any poly */
	if (is_in_poly(robot_pt, pol_opp1) == 1)
		in_opp1 = 1;

	if (is_in_poly(robot_pt, pol_opp2) == 1)
		in_opp2 = 1;

 	if (is_in_poly(robot_pt, pol_robot_2nd) == 1)
		in_robot_2nd = 1;

	if(is_in_poly(robot_pt,pol_home_green)) {
		in_home_green = 1;
	}
 	if(is_in_poly(robot_pt,pol_home_yellow)) {
		in_home_yellow = 1;
	}
	if(is_in_poly(robot_pt,pol_platform)) {
		in_platform = 1;
	}
	if(is_in_poly(robot_pt,pol_stairs)) {
		in_stairs = 1;
	}

	if (in_opp1 == 0 && in_opp2 == 0 && in_robot_2nd == 0 && in_home_green == 0 && in_home_yellow == 0 && in_stairs == 0) {
		NOTICE(E_USER_STRAT, "no need to escape");
		return 0;
	}
	NOTICE(E_USER_STRAT, "in_opp1=%d", in_opp1);
	NOTICE(E_USER_STRAT, "in_opp2=%d", in_opp2);
	NOTICE(E_USER_STRAT, "in_robot_2nd=%d", in_robot_2nd);
	NOTICE(E_USER_STRAT, "in_stairs=%d", in_stairs);
	NOTICE(E_USER_STRAT, "in_home_green=%d", in_home_green);

	NOTICE(E_USER_STRAT, "in_platform=%d", in_platform);

	NOTICE(E_USER_STRAT, "in_home_yellow=%d", in_home_yellow);

	/* process escape vectors */
	if (in_opp1 && distance_between(robot_pt->x, robot_pt->y, opp1_x, opp1_y) < ESCAPE_POLY_THRES) {
		opp1_dx = robot_pt->x - opp1_x;
		opp1_dy = robot_pt->y - opp1_y;
		NOTICE(E_USER_STRAT, " robot is near opp1: vect=%2.2f,%2.2f",
		       opp1_dx, opp1_dy);
		len = norm(opp1_dx, opp1_dy);
		if (len != 0) {
			opp1_dx /= len;
			opp1_dy /= len;
		}
		else {
			opp1_dx = 1.0; /* XXX why?, is posible? */
			opp1_dy = 0.0;
		}
		escape_dx += opp1_dx;
		escape_dy += opp1_dy;
	}

	if (in_opp2 && distance_between(robot_pt->x, robot_pt->y, opp2_x, opp2_y) < ESCAPE_POLY_THRES) {
		opp2_dx = robot_pt->x - opp2_x;
		opp2_dy = robot_pt->y - opp2_y;
		NOTICE(E_USER_STRAT, " robot is near opp2: vect=%2.2f,%2.2f",
		       opp2_dx, opp2_dy);
		len = norm(opp2_dx, opp2_dy);
		if (len != 0) {
			opp2_dx /= len;
			opp2_dy /= len;
		}
		else {
			opp2_dx = 1.0;
			opp2_dy = 0.0;
		}
		escape_dx += opp2_dx;
		escape_dy += opp2_dy;
	}

	if (in_robot_2nd && distance_between(robot_pt->x, robot_pt->y, robot_2nd_x, robot_2nd_y) < ESCAPE_POLY_THRES) {
		robot_2nd_dx = robot_pt->x - robot_2nd_x;
		robot_2nd_dy = robot_pt->y - robot_2nd_y;
		NOTICE(E_USER_STRAT, " robot is near robot_2nd: vect=%2.2f,%2.2f",
		       robot_2nd_dx, robot_2nd_dy);
		len = norm(robot_2nd_dx, robot_2nd_dy);
		if (len != 0) {
			robot_2nd_dx /= len;
			robot_2nd_dy /= len;
		}
		else {
			robot_2nd_dx = 1.0;
			robot_2nd_dy = 0.0;
		}
		escape_dx += robot_2nd_dx;
		escape_dy += robot_2nd_dy;
	}
	//&& distance_between(robot_pt->x, robot_pt->y, STAIRS_X, STAIRS_Y) <1500
	if (in_stairs ) {
		stairs_dx = robot_pt->x - STAIRS_X;
		stairs_dy = robot_pt->y - STAIRS_Y;
		NOTICE(E_USER_STRAT, " robot is near stairs: vect=%2.2f,%2.2f",
		       stairs_dx, stairs_dy);
		len = norm(stairs_dx, stairs_dy);
		if (len != 0) {
			stairs_dx/= len;
			stairs_dy /= len;
		}
		else {
			stairs_dx = 1.0;
			stairs_dy = 0.0;
		}
		escape_dx += stairs_dx;
		escape_dy += stairs_dy;
	}
	//
	if (in_home_green) {
		home_green_dx = robot_pt->x - HOME_GREEN_X;
		home_green_dy = robot_pt->y - HOME_GREEN_Y;
		NOTICE(E_USER_STRAT, " robot is near home_green: vect=%2.2f,%2.2f",
		       home_green_dx, home_green_dy);
		len = norm(home_green_dx, home_green_dy);
		if (len != 0) {
			home_green_dx/= len;
			home_green_dy /= len;
		}
		else {
			home_green_dx = 1.0;
			home_green_dy = 0.0;
		}
		escape_dx += home_green_dx;
		escape_dy += home_green_dy;
	}
	if (in_home_yellow) {
		home_yellow_dx = robot_pt->x - HOME_YELLOW_X;
		home_yellow_dy = robot_pt->y - HOME_YELLOW_Y;
		NOTICE(E_USER_STRAT, " robot is near home_yellow: vect=%2.2f,%2.2f",
		       home_yellow_dx, home_yellow_dy);
		len = norm(home_yellow_dx, home_yellow_dy);
		if (len != 0) {
			home_yellow_dx/= len;
			home_yellow_dy /= len;
		}
		else {
			home_yellow_dx = 1.0;
			home_yellow_dy = 0.0;
		}
		escape_dx += home_yellow_dx;
		escape_dy += home_yellow_dy;
	}
	if (in_platform) {
		platform_dx = robot_pt->x - PLATFORM_X;
		platform_dy = robot_pt->y - PLATFORM_Y;
		NOTICE(E_USER_STRAT, " robot is near platform: vect=%2.2f,%2.2f",
		       platform_dx, platform_dy);
		len = norm(platform_dx, platform_dy);
		if (len != 0) {
			platform_dx/= len;
			platform_dy /= len;
		}
		else {
			platform_dx = 1.0;
			platform_dy = 0.0;
		}
		escape_dx += platform_dx;
		escape_dy += platform_dy;
	}

	/* normalize escape vector */
	len = norm(escape_dx, escape_dy);
	if (len != 0) {
		escape_dx /= len;
		escape_dy /= len;
	}
	else {
      if (pol_opp1 != NULL) {
			/* rotate 90� */
			escape_dx = opp1_dy;
			escape_dy = opp1_dx;
		}
      else if (pol_opp2 != NULL) {
			/* rotate 90� */
			escape_dx = opp2_dy;
			escape_dy = opp2_dx;
		}
		else if (pol_robot_2nd != NULL) {
			/* rotate 90� */
			escape_dx = robot_2nd_dy;
			escape_dy = robot_2nd_dx;
		}
		else if (pol_stairs != NULL) {
			/* rotate 90� */
			escape_dx = stairs_dy;
			escape_dy = stairs_dx;
		}
		else if (pol_home_green != NULL) {
			/* rotate 90� */
			escape_dx = home_green_dy;
			escape_dy = home_green_dx;
		}
		else if (pol_home_yellow != NULL) {
			/* rotate 90� */
			escape_dx = home_yellow_dy;
			escape_dy = home_yellow_dx;
		}
		else { /* should not happend */
			escape_dx = 1.0;
			escape_dy = 0.0;
		}
	}

	NOTICE(E_USER_STRAT, " escape vect = %2.2f,%2.2f",
	       escape_dx, escape_dy);

	/* process the correct len of escape vector */
	dst_pt.x = robot_pt->x + escape_dx * ESCAPE_VECT_LEN;
	dst_pt.y = robot_pt->y + escape_dy * ESCAPE_VECT_LEN;

	NOTICE(E_USER_STRAT, "robot pt %"PRId32" %"PRId32,
	       (int32_t)robot_pt->x, (int32_t)robot_pt->y);
	NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
	       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

	if (in_opp1) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_opp1_pt,
				     pol_opp1) == 1) {

			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_opp1_pt.x + escape_dx * 20;
			dst_pt.y = intersect_opp1_pt.y + escape_dy * 20;

			NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
			       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

         /* XXX check that destination point is not in an other poly */
			if (is_point_in_poly(pol_opp2, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_robot_2nd, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_home_green, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_home_yellow, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_stairs, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_platform, dst_pt.x, dst_pt.y) != 1){

            /* check if destination point is in playground */
				if (!is_in_boundingbox(&dst_pt))
					return -1;

				NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
				       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

				/* XXX comment for virtual scape from poly */
#ifndef HOST_VERSION_OA_TEST
				strat_goto_xy_force(dst_pt.x, dst_pt.y);
#endif
				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;

				return 0;
			}
		}
	}

	if (in_opp2) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_opp2_pt,
				     pol_opp2) == 1) {

			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_opp2_pt.x + escape_dx * 20;
			dst_pt.y = intersect_opp2_pt.y + escape_dy * 20;

			NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
			       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

         /* XXX check that destination point is not in an other poly */
			if (is_point_in_poly(pol_opp1, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_robot_2nd, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_home_green, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_home_yellow, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_stairs, dst_pt.x, dst_pt.y) != 1&&
			is_point_in_poly(pol_platform, dst_pt.x, dst_pt.y) != 1){

            /* check if destination point is in playground */
				if (!is_in_boundingbox(&dst_pt))
					return -1;

				NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
				       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

				/* XXX comment for virtual scape from poly */
#ifndef HOST_VERSION_OA_TEST
				strat_goto_xy_force(dst_pt.x, dst_pt.y);
#endif
				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;

				return 0;
			}
		}
	}


	if (in_robot_2nd) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_robot_2nd_pt,
				     pol_robot_2nd) == 1) {

			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_robot_2nd_pt.x + escape_dx * 20;
			dst_pt.y = intersect_robot_2nd_pt.y + escape_dy * 20;

			NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
			       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

         /* XXX check that destination point is not in an other poly */
			if (is_point_in_poly(pol_opp1, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_opp2, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_home_green, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_home_yellow, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_stairs, dst_pt.x, dst_pt.y) != 1 &&
			is_point_in_poly(pol_platform, dst_pt.x, dst_pt.y) != 1){

            /* check if destination point is in playground */
				if (!is_in_boundingbox(&dst_pt))
					return -1;

				NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
				       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

				/* XXX comment for virtual scape from poly */
#ifndef HOST_VERSION_OA_TEST
				strat_goto_xy_force(dst_pt.x, dst_pt.y);
#endif
				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;

				return 0;
			}
		}
	}

	if (in_stairs) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_stairs_pt,
				     pol_stairs) == 1) {

			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_stairs_pt.x + escape_dx * 20;
			dst_pt.y = intersect_stairs_pt.y + escape_dy * 20;

			NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
			       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

         /* XXX check that destination point is not in an other poly */
			if (is_point_in_poly(pol_opp1, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_opp2, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_robot_2nd, dst_pt.x, dst_pt.y) != 1){

            /* check if destination point is in playground */
				if (!is_in_boundingbox(&dst_pt))
					return -1;

				NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
				       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

				/* XXX comment for virtual scape from poly */
#ifndef HOST_VERSION_OA_TEST
				strat_goto_xy_force(dst_pt.x, dst_pt.y);
#endif
				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;

				return 0;
			}
		}
	}
	if (in_home_green) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_home_green_pt,
				     pol_home_green) == 1) {

			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_home_green_pt.x + escape_dx * 20;
			dst_pt.y = intersect_home_green_pt.y + escape_dy * 20;

			NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
			       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

         /* XXX check that destination point is not in an other poly */
			if (is_point_in_poly(pol_opp1, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_opp2, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_robot_2nd, dst_pt.x, dst_pt.y) != 1 ){

            /* check if destination point is in playground */
				if (!is_in_boundingbox(&dst_pt))
					return -1;

				NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
				       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

				/* XXX comment for virtual scape from poly */
#ifndef HOST_VERSION_OA_TEST
				strat_goto_xy_force(dst_pt.x, dst_pt.y);
#endif
				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;

				return 0;
			}
		}
	}
	if (in_platform) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_platform_pt,
					pol_platform) == 1) {

			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_platform_pt.x + escape_dx * 20;
			dst_pt.y = intersect_platform_pt.y + escape_dy * 20;

			NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
				  (int32_t)dst_pt.x, (int32_t)dst_pt.y);

	    /* XXX check that destination point is not in an other poly */
			if (is_point_in_poly(pol_opp1, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_opp2, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_robot_2nd, dst_pt.x, dst_pt.y) != 1 ){

		  /* check if destination point is in playground */
				if (!is_in_boundingbox(&dst_pt))
					return -1;

				NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
					  (int32_t)dst_pt.x, (int32_t)dst_pt.y);

				/* XXX comment for virtual scape from poly */
	#ifndef HOST_VERSION_OA_TEST
				strat_goto_xy_force(dst_pt.x, dst_pt.y);
	#endif
				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;

				return 0;
			}
		}
	}
	if (in_home_yellow) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_home_yellow_pt,
				     pol_home_yellow) == 1) {

			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_home_yellow_pt.x + escape_dx * 20;
			dst_pt.y = intersect_home_yellow_pt.y + escape_dy * 20;

			NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
			       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

         /* XXX check that destination point is not in an other poly */
			if (is_point_in_poly(pol_opp1, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_opp2, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_robot_2nd, dst_pt.x, dst_pt.y) != 1 ){

            /* check if destination point is in playground */
				if (!is_in_boundingbox(&dst_pt))
					return -1;

				NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
				       (int32_t)dst_pt.x, (int32_t)dst_pt.y);

				/* XXX comment for virtual scape from poly */
#ifndef HOST_VERSION_OA_TEST
				strat_goto_xy_force(dst_pt.x, dst_pt.y);
#endif
				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;

				return 0;
			}
		}
	}
	/* should not happen */
	return -1;
}


#define GO_AVOID_AUTO		0
#define GO_AVOID_FORWARD	1
#define GO_AVOID_BACKWARD	2

#ifndef HOST_VERSION_OA_TEST
static int8_t __goto_and_avoid(int16_t x, int16_t y,
			       uint8_t flags_intermediate,
			       uint8_t flags_final, uint8_t direction)
#else
#warning HOST_VERSION_OA_TEST compilation
int8_t goto_and_avoid(int16_t x, int16_t y,
					   	int16_t robot_x, int16_t robot_y, double robot_a,
					   	int16_t robot_2nd_x, int16_t robot_2nd_y,
					   	int16_t opp1_x, int16_t opp1_y,
					   	int16_t opp2_x, int16_t opp2_y)

#endif
{
	int8_t len = -1;
	int8_t i;

	point_t *p;
	poly_t *pol_opp1, *pol_opp2, *pol_robot_2nd;
  	poly_t *pol_stairs,*pol_home_yellow,*pol_home_green,*pol_platform;

	int8_t ret;

	int16_t opp1_w, opp1_l;
	int16_t opp2_w, opp2_l;


#ifndef HOST_VERSION_OA_TEST
	int16_t opp1_x, opp1_y;
	int16_t opp2_x, opp2_y;
	int16_t robot_2nd_x, robot_2nd_y;
#endif

	point_t p_dst, robot_pt;

	void * p_retry;
	p_retry = &&retry;


#ifndef HOST_VERSION_OA_TEST
	DEBUG(E_USER_STRAT, "%s(%d,%d) flags_i=%x flags_f=%x direct=%d",
	      __FUNCTION__, x, y, flags_intermediate, flags_final, direction);
#else
#warning HOST_VERSION_OA_TEST compilation
	g_robot_x = robot_x;
	g_robot_y = robot_y;
	g_robot_a = robot_a;
	g_opp1_x = opp1_x;
	g_opp1_y = opp1_y;
	g_opp2_x = opp2_x;
	g_opp2_y = opp2_y;
	g_robot_2nd_x = robot_2nd_x;
	g_robot_2nd_y = robot_2nd_y;
#endif

retry:

	/* opponent info */
#ifndef HOST_VERSION_OA_TEST
  get_opponent1_xy(&opp1_x, &opp1_y);
  get_opponent2_xy(&opp2_x, &opp2_y);

	/* get second robot */
	//robot_2nd_x = I2C_OPPONENT_NOT_THERE;
	//robot_2nd_y = 0;
  get_robot_2nd_xy(&robot_2nd_x, &robot_2nd_y);

#endif

#ifndef IM_SECONDARY_ROBOT
	opp1_w = O_WIDTH;

    /* XXX */
    if (clerance_minimum_enabled)
	    opp1_l = O_WIDTH;
    else
    	opp1_l = O_LENGTH;

	opp2_w = O_WIDTH;

    /* XXX */
    if (clerance_minimum_enabled)
    	opp2_l = O_WIDTH;
    else
	    opp2_l = O_LENGTH;

    /* XXX */
    if (clerance_minimum_enabled)
        obs_clerance = OBS_CLERANCE_SIDE;
    else
        obs_clerance = OBS_CLERANCE;
#else
	opp1_w = O_WIDTH;
   	opp1_l = O_LENGTH;

	opp2_w = O_WIDTH;
    opp2_l = O_LENGTH;

    obs_clerance = OBS_CLERANCE;
#endif

	/* robot info */
#ifndef HOST_VERSION_OA_TEST
	robot_pt.x = position_get_x_s16(&mainboard.pos);
	robot_pt.y = position_get_y_s16(&mainboard.pos);

	DEBUG (E_USER_STRAT, "robot xy %d, %d", robot_pt.x, robot_pt.y);

#else
#warning HOST_VERSION_OA_TEST compilation
	robot_pt.x = robot_x;
	robot_pt.y = robot_y;
#endif

	/* init oa */
	oa_init();

	/* add opponent and 2nd robot polys */
	pol_opp1 = oa_new_poly(4);
	set_opponent_poly(OPP1, pol_opp1, &robot_pt, O_WIDTH, O_LENGTH);
	pol_opp2 = oa_new_poly(4);
	set_opponent_poly(OPP2, pol_opp2, &robot_pt, O_WIDTH, O_LENGTH);

	pol_robot_2nd = oa_new_poly(4);
	set_opponent_poly(ROBOT2ND, pol_robot_2nd, &robot_pt, ROBOT_2ND_WIDTH, ROBOT_2ND_LENGTH);

	pol_platform= oa_new_poly(4);
	set_poly_abs(pol_platform,O_PLATFORM_WIDTH + obs_clerance/2,O_PLATFORM_HEIGHT + obs_clerance/2,PLATFORM_X,PLATFORM_Y+obs_clerance/2);

	pol_stairs= oa_new_poly(4);
	set_poly_abs(pol_stairs,O_STAIRS_WIDTH +obs_clerance,O_STAIRS_HEIGHT + obs_clerance/2,STAIRS_X,STAIRS_Y-obs_clerance/2);

#ifndef HOST_VERSION_OA_TEST
	if(mainboard.our_color== I2C_COLOR_YELLOW){
#endif
		pol_home_yellow= oa_new_poly(4);
		set_poly_abs(pol_home_yellow,O_HOME_WIDTH+ obs_clerance/2 ,O_HOME_HEIGHT + obs_clerance,HOME_YELLOW_X+obs_clerance/2,HOME_YELLOW_Y);

		pol_home_green= oa_new_poly(5);

#ifdef IM_SECONDARY_ROBOT
		set_home_green_poly_abs(pol_home_green,O_HOME_WIDTH+ OBS_CLERANCE/2+71,O_HOME_HEIGHT + OBS_CLERANCE,HOME_GREEN_X-OBS_CLERANCE/2-35,HOME_GREEN_Y);
#else
 		set_home_green_poly_abs(pol_home_green,O_HOME_WIDTH+ OBS_CLERANCE/2,O_HOME_HEIGHT + OBS_CLERANCE,HOME_GREEN_X-OBS_CLERANCE/2,HOME_GREEN_Y);
#endif

#ifndef HOST_VERSION_OA_TEST
 	}else{
 		pol_home_yellow= oa_new_poly(5);
#ifdef IM_SECONDARY_ROBOT
		set_home_yellow_poly_abs(pol_home_yellow,O_HOME_WIDTH+ OBS_CLERANCE/2 +71,O_HOME_HEIGHT + OBS_CLERANCE,HOME_YELLOW_X+OBS_CLERANCE/2+35,HOME_YELLOW_Y);
#else
 		set_home_yellow_poly_abs(pol_home_yellow,O_HOME_WIDTH+ OBS_CLERANCE/2 ,O_HOME_HEIGHT + OBS_CLERANCE,HOME_YELLOW_X+OBS_CLERANCE/2,HOME_YELLOW_Y);


#endif
		pol_home_green= oa_new_poly(4);
		set_poly_abs(pol_home_green,O_HOME_WIDTH+ obs_clerance/2,O_HOME_HEIGHT + obs_clerance,HOME_GREEN_X-obs_clerance/2,HOME_GREEN_Y);
	}
#endif


/* if we are not in the limited area, try to go in it. */
	ret = go_in_area(&robot_pt);

	/* check that destination is in playground */
	p_dst.x = x;
	p_dst.y = y;
	if (!is_in_boundingbox(&p_dst)) {
		NOTICE(E_USER_STRAT, " dst is not in playground");
		return END_ERROR;
	}

	/* check if destination is in any poly */
  	if (is_point_in_poly(pol_opp1, x, y)) {
		NOTICE(E_USER_STRAT, " dst is in opp 1");
		return END_ERROR;
	}
  	if (is_point_in_poly(pol_opp2, x, y)) {
		NOTICE(E_USER_STRAT, " dst is in opp 2");
		return END_ERROR;
	}

 	if (is_point_in_poly(pol_robot_2nd, x, y)) {
		NOTICE(E_USER_STRAT, " dst is in robot 2nd");
		return END_ERROR;
	}
	if(is_point_in_poly(pol_home_green,x ,y)) {
		NOTICE(E_USER_STRAT, " dst is in home green");
		return END_ERROR;
	}
 	if(is_point_in_poly(pol_home_yellow,x ,y)) {
		NOTICE(E_USER_STRAT, " dst is in home yellow");
		return END_ERROR;
	}
	if(is_point_in_poly(pol_platform,x ,y)) {
		NOTICE(E_USER_STRAT, " dst is in platform");
		return END_ERROR;
	}
	if(is_point_in_poly(pol_stairs,x,y)) {
		NOTICE(E_USER_STRAT, " dst is in stairs");
		return END_ERROR;
	}

	/* now start to avoid */
	while (opp1_w && opp1_l && opp2_w && opp2_l)
	{

    	/* escape from polys */
		/* XXX robot_pt is not updated if it fails */
		ret = escape_from_poly(&robot_pt, robot_2nd_x, robot_2nd_y,
				                opp1_x, opp1_y, opp2_x, opp2_y,
				                pol_opp1, pol_opp2,
				                pol_robot_2nd,pol_home_green,
								pol_home_yellow,pol_stairs,pol_platform);


    	/* XXX uncomment in order to skip escape from poly */
		//ret = 0;

		if (ret == 0) {

 			/* reset and set start and end points */
			oa_reset();
			oa_start_end_points(robot_pt.x, robot_pt.y, x, y);
			//oa_dump();

			/* proccesing path */
			len = oa_process();

			if (len >= 0)
			  break;

		}

		/* len < 0, try reduce opponent to get a valid path */
		if (distance_between(robot_pt.x, robot_pt.y, opp1_x, opp1_y) < REDUCE_POLY_THRES ) {
			if (opp1_w == 0)
				opp1_l /= 2;

			opp1_w /= 2;

			NOTICE(E_USER_STRAT, "reducing opponent 1 %d %d", opp1_w, opp1_l);
			set_opponent_poly(OPP1, pol_opp1, &robot_pt, opp1_w, opp1_l);
		}

		if (distance_between(robot_pt.x, robot_pt.y, opp2_x, opp2_y) < REDUCE_POLY_THRES ) {
			if (opp2_w == 0)
				opp2_l /= 2;
			opp2_w /= 2;

			NOTICE(E_USER_STRAT, "reducing opponent 2 %d %d", opp2_w, opp2_l);
			set_opponent_poly(OPP2, pol_opp2, &robot_pt, opp2_w, opp2_l);
		}

		/*if (distance_between(robot_pt.x, robot_pt.y, STAIRS_X, STAIRS_Y) < XXX ) {
			//TODO REDUCE STAIRS
		}
		if (distance_between(robot_pt.x, robot_pt.y, HOME_YELLOW_X, HOME_YELLOW_Y) < XXX ) {
			//TODO REDUCE HOME_YELLOW
		}
		if (distance_between(robot_pt.x, robot_pt.y, HOME_GREEN_X, HOME_GREEN_Y) < XXX ) {
			//TODO REDUCE HOME_GREEN
		}
		if (distance_between(robot_pt.x, robot_pt.y, HEARTFIRE_X, HEARTFIRE_Y) < 600 ) {
      		heartfire_r = HEARTFIRE_RAD2;

			NOTICE(E_USER_STRAT, "reducing heart of fire r=%d", heartfire_r);
			set_heartfire_poly(pol_heartfire, &robot_pt, heartfire_r);
		}
*/
	   /* TODO XXX don't try to reduce robot 2nd */


		if (distance_between(robot_pt.x, robot_pt.y, opp1_x, opp1_y) >= REDUCE_POLY_THRES
		    && distance_between(robot_pt.x, robot_pt.y, opp2_x, opp2_y) >= REDUCE_POLY_THRES)
      	{

            NOTICE(E_USER_STRAT, "oa_process() returned %d", len);
			return END_ERROR;
		}
	}

	if(!(opp1_w && opp1_l && opp2_w && opp2_l)) {

            NOTICE(E_USER_STRAT, "oa_process() returned %d", len);
			return END_ERROR;
	}

	/* execute path */
	p = oa_get_path();
	for (i=0 ; i<len ; i++) {

#ifndef HOST_VERSION_OA_TEST

//		if (d<20){
//			p++;
//			continue;
//		}

		if (direction == GO_AVOID_FORWARD){
			DEBUG(E_USER_STRAT, "With avoidance %d: x=%"PRId32" y=%"PRId32" forward", i, (int32_t)p->x, (int32_t)p->y);
			trajectory_goto_forward_xy_abs(&mainboard.traj, p->x, p->y);
		}
		else if(direction == GO_AVOID_BACKWARD){
			DEBUG(E_USER_STRAT, "With avoidance %d: x=%"PRId32" y=%"PRId32" backward", i, (int32_t)p->x, (int32_t)p->y);
			trajectory_goto_backward_xy_abs(&mainboard.traj, p->x, p->y);
		}
		else {
			DEBUG(E_USER_STRAT, "With avoidance %d: x=%"PRId32" y=%"PRId32" forward", i, (int32_t)p->x, (int32_t)p->y);
			trajectory_goto_xy_abs(&mainboard.traj, p->x, p->y);
		}

		/* no END_NEAR for the last point */
		if (i == len - 1)
			ret = wait_traj_end(flags_final);
		else
			ret = wait_traj_end(flags_intermediate);

		if (ret == END_BLOCKING) {
			DEBUG(E_USER_STRAT, "Retry avoidance %s(%d,%d)",
			      __FUNCTION__, x, y);
			goto *p_retry;
		}
		else if (ret == END_OBSTACLE) {
			/* brake and wait the speed to be slow */
			DEBUG(E_USER_STRAT, "Retry avoidance %s(%d,%d)",
			      __FUNCTION__, x, y);
			goto *p_retry;
		}
		/* else if it is not END_TRAJ or END_NEAR, return */
		else if (!TRAJ_SUCCESS(ret)) {
			return ret;
		}

#endif /* HOST_VERSION_OA_TEST */

		DEBUG(E_USER_STRAT, "With avoidance %d: x=%"PRId32" y=%"PRId32"", i, (int32_t)p->x, (int32_t)p->y);

		/* next point */
		p++;
	}

	return END_TRAJ;
}

#ifndef HOST_VERSION_OA_TEST

/* go to a x,y point. prefer backward but go forward if the point is
 * near and in front of us */
#if 0
uint8_t goto_and_avoid(int16_t x, int16_t y, uint8_t flags_intermediate,
			       uint8_t flags_final)
{
	//double d,a;
	//abs_xy_to_rel_da(x, y, &d, &a);


  //if(robots_are_near()) {
  //    DEBUG(E_USER_STRAT, "Robots near");
		  return __goto_and_avoid(x, y, flags_intermediate,
					  flags_final, GO_AVOID_AUTO);
  //}
  #if 0
  else { /* XXX specific 2014 */
	  if (d < 300 && a < RAD(90) && a > RAD(-90))
		  return __goto_and_avoid(x, y, flags_intermediate,
					  flags_final, GO_AVOID_FORWARD);
	  else
		  return __goto_and_avoid(x, y, flags_intermediate,
					  flags_final, GO_AVOID_BACKWARD);
  }
  #endif
}
#endif


/* go to a x,y point */
uint8_t goto_and_avoid(int16_t x, int16_t y, uint8_t flags_intermediate,
			       uint8_t flags_final)
{
	return __goto_and_avoid(x, y, flags_intermediate, flags_final, GO_AVOID_AUTO);
}

/* go forward to a x,y point. use current speed for that */
uint8_t goto_and_avoid_forward(int16_t x, int16_t y, uint8_t flags_intermediate,
			       uint8_t flags_final)
{
	return __goto_and_avoid(x, y, flags_intermediate, flags_final, GO_AVOID_FORWARD);
}

/* go backward to a x,y point. use current speed for that */
uint8_t goto_and_avoid_backward(int16_t x, int16_t y, uint8_t flags_intermediate,
		       uint8_t flags_final)
{
	return __goto_and_avoid(x, y, flags_intermediate, flags_final, GO_AVOID_BACKWARD);
}



#endif




#if 0
void set_rotated_pentagon(poly_t *pol, int16_t radius,
			      int16_t x, int16_t y, uint8_t special)
{

	double c_a, s_a;
	uint8_t i;
	double px1, py1, px2, py2;
	double a_rad;

	//a_rad = atan2(y - robot_y, x - robot_x);
   a_rad = 0;

	/* generate pentagon  */
	c_a = cos(-2*M_PI/CORN_EDGE_NUMBER);
	s_a = sin(-2*M_PI/CORN_EDGE_NUMBER);

	/*
	px1 = radius;
	py1 = 0;
	*/
	px1 = radius * cos(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));
	py1 = radius * sin(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));


	for (i = 0; i < CORN_EDGE_NUMBER; i++){

		if(special == 1 && (i==0||i==1||i==2))
		   oa_poly_set_point(pol, x + px1, PLAYGROUND_Y_MAX+2, i);
		else if(special == 2 && (i==3))
		   oa_poly_set_point(pol, x + px1, y + py1 - OFFSET_AVOID_RAMPE, i);
		else if(special == 3 && (i==5))
		   oa_poly_set_point(pol, x + px1, y + py1 - OFFSET_AVOID_RAMPE, i);
		else
		   oa_poly_set_point(pol, x + px1, y + py1, i);


		px2 = px1*c_a + py1*s_a;
		py2 = -px1*s_a + py1*c_a;

		px1 = px2;
		py1 = py2;
	}

}

void set_rotated_pentagon_pts(point_t *pt, int16_t radius,
			      int16_t x, int16_t y, uint8_t special)
{

	double c_a, s_a;
	uint8_t i;
	double px1, py1, px2, py2;
	double a_rad;

	//a_rad = atan2(y - robot_y, x - robot_x);
   a_rad = 0;

	/* generate pentagon  */
	c_a = cos(-2*M_PI/CORN_EDGE_NUMBER);
	s_a = sin(-2*M_PI/CORN_EDGE_NUMBER);

	/*
	px1 = radius;
	py1 = 0;
	*/
	px1 = radius * cos(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));
	py1 = radius * sin(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));


	for (i = 0; i < CORN_EDGE_NUMBER; i++){

	  if(special == 1 && (i==0||i==1||i==2)){
			pt[i].x = x + px1;
		  pt[i].y = PLAYGROUND_Y_MAX+2;
		}
		else if(special == 2 && (i==3)){
		  pt[i].x = x + px1;
		  pt[i].y = y + py1 - OFFSET_AVOID_RAMPE;
    }
		else if(special == 3 && (i==5)){
		  pt[i].x = x + px1;
		  pt[i].y = y + py1 - OFFSET_AVOID_RAMPE;
    }
		else{
		  pt[i].x = x + px1;
		  pt[i].y = y + py1;
		}

	  px2 = px1*c_a + py1*s_a;
	  py2 = -px1*s_a + py1*c_a;

		px1 = px2;
		py1 = py2;
	}

}

//#define EDGE_NUMBER 5
//void set_rotated_pentagon(poly_t *pol, const point_t *robot_pt,
//			  int16_t radius, int16_t x, int16_t y)
//{
//
//	double c_a, s_a;
//	uint8_t i;
//	double px1, py1, px2, py2;
//	double a_rad;
//
//	a_rad = atan2(y - robot_pt->y, x - robot_pt->x);
//
//	/* generate pentagon  */
//	c_a = cos(-2*M_PI/EDGE_NUMBER);
//	s_a = sin(-2*M_PI/EDGE_NUMBER);
//
//	/*
//	px1 = radius;
//	py1 = 0;
//	*/
//	px1 = radius * cos(a_rad + 2*M_PI/(2*EDGE_NUMBER));
//	py1 = radius * sin(a_rad + 2*M_PI/(2*EDGE_NUMBER));
//
//
//	for (i = 0; i < EDGE_NUMBER; i++){
//		oa_poly_set_point(pol, x + px1, y + py1, i);
//
//		px2 = px1*c_a + py1*s_a;
//		py2 = -px1*s_a + py1*c_a;
//
//		px1 = px2;
//		py1 = py2;
//	}
//}

#endif
