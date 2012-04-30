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
 *  Revision : $Id: strat_base.c,v 1.7 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  strat_base.c,v 1.7 2009/05/27 20:04:07 zer0 Exp.
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
#include "cmdline.h"
#include "strat_utils.h"
#include "strat_base.h"
#include "strat.h"
#include "sensor.h"
#include "i2c_protocol.h"
#include "beacon.h"

/* true if we want to interrupt a trajectory */
static uint8_t traj_intr=0;

/* filled when a END_OBSTACLE is returned */
struct opponent_obstacle opponent_obstacle;

/* asked speed */
static volatile int16_t strat_speed_a = SPEED_DIST_FAST;
static volatile int16_t strat_speed_d = SPEED_ANGLE_FAST;
static volatile uint16_t strat_limit_speed_a = 0; /* no limit */
static volatile uint16_t strat_limit_speed_d = 0;

static volatile uint8_t strat_limit_speed_enabled = 1;


/* Strings that match the end traj cause */
/* /!\ keep it sync with stat_base.h */
const char *err_tab []= {
	"END_TRAJ",
	"END_BLOCKING",
	"END_NEAR",
	"END_OBSTACLE",
	"END_ERROR",
	"END_INTR",
	"END_TIMER",
	"END_RESERVED",
};

/* return string from end traj type num */
const char *get_err(uint8_t err)
{
	uint8_t i;
	if (err == 0)
		return "SUCCESS";
	for (i=0 ; i<8; i++) {
		if (err & (1 <<i))
			return err_tab[i];
	}
	return "END_UNKNOWN";
}

void strat_hardstop(void) 
{
	trajectory_hardstop(&mainboard.traj);
	pid_reset(&mainboard.angle.pid);
	pid_reset(&mainboard.distance.pid);
	bd_reset(&mainboard.angle.bd);
	bd_reset(&mainboard.distance.bd);

	while ((ABS(mainboard.speed_d) > 200) ||
	       (ABS(mainboard.speed_a) > 200))

	trajectory_hardstop(&mainboard.traj);
	pid_reset(&mainboard.angle.pid);
	pid_reset(&mainboard.distance.pid);
	bd_reset(&mainboard.angle.bd);
	bd_reset(&mainboard.distance.bd);
}

/* go to an x,y point without checking for obstacle or blocking. It
 * should be used for very small dist only. Return END_TRAJ if we
 * reach destination, or END_BLOCKING if the robot blocked more than 3
 * times. */
uint8_t strat_goto_xy_force(int16_t x, int16_t y)
{
	int8_t i, err;
	
#ifdef HOMOLOGATION
	uint8_t hardstop = 0;
	microseconds us = time_get_us2();
	int16_t opp_a, opp_d, opp_x, opp_y;
#ifdef TWO_OPPONENTS
	int16_t opp2_a, opp2_d, opp2_x, opp2_y;
	int8_t err2;
#endif

	while (1) {
#ifdef TWO_OPPONENTS
		err = get_opponent_xyda(&opp_x, &opp_y,
					&opp_d, &opp_a);

		err2 = get_opponent2_xyda(&opp2_x, &opp2_y,
					&opp2_d, &opp2_a);

		if (err == -1 && err2 == -1)
			break;
		if (opp_d < 600 || opp2_d < 600) /* XXX i don't understood */
			break;
#else
		err = get_opponent_xyda(&opp_x, &opp_y,
					&opp_d, &opp_a);

		if (err == -1)
			break;
		if (opp_d < 600) /* XXX i don't understood */
			break;
#endif
		if (hardstop == 0) {
			strat_hardstop();
			hardstop = 1;
		}
		if ((time_get_us2() - us) > 3000000L)
			break;
	}
#endif
	for (i=0; i<3; i++) {
		trajectory_goto_xy_abs(&mainboard.traj, x, y);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (TRAJ_SUCCESS(err))
			return END_TRAJ;
		if (err == END_BLOCKING) {
			time_wait_ms(500);
			strat_hardstop();
		}
	}
	return END_BLOCKING;
}

/* reset position */ 
void strat_reset_pos(int16_t x, int16_t y, int16_t a)
{
	int16_t posx = position_get_x_s16(&mainboard.pos);
	int16_t posy = position_get_y_s16(&mainboard.pos);
	int16_t posa = position_get_a_deg_s16(&mainboard.pos);

	if (x == DO_NOT_SET_POS)
		x = posx;
	if (y == DO_NOT_SET_POS)
		y = posy;
	if (a == DO_NOT_SET_POS)
		a = posa;

	DEBUG(E_USER_STRAT, "reset pos (%s%s%s)",
	      x == DO_NOT_SET_POS ? "" : "x",
	      y == DO_NOT_SET_POS ? "" : "y",
	      a == DO_NOT_SET_POS ? "" : "a");
	position_set(&mainboard.pos, x, y, a);
	DEBUG(E_USER_STRAT, "pos resetted", __FUNCTION__);
}

/* 
 * decrease gain on angle PID, and go forward until we reach the
 * border.
 */
uint8_t strat_calib(int16_t dist, uint8_t flags)
{
	int32_t p = pid_get_gain_P(&mainboard.angle.pid);
	int32_t i = pid_get_gain_I(&mainboard.angle.pid);
	int32_t d = pid_get_gain_D(&mainboard.angle.pid);
	uint8_t err;

	pid_set_gains(&mainboard.angle.pid, 150, 0, 2000);
	trajectory_d_rel(&mainboard.traj, dist);
	err = wait_traj_end(flags);
	pid_set_gains(&mainboard.angle.pid, p, i, d);
	return err;
}

///* escape from zone, and don't brake, so we can continue with another
// * traj */
//uint8_t strat_escape(struct build_zone *zone, uint8_t flags)
//{
//	uint8_t err;
//	uint16_t old_spdd, old_spda;
//
//	strat_get_speed(&old_spdd, &old_spda);
//
//	err = WAIT_COND_OR_TIMEOUT(!opponent_is_behind(), 1000);
//	if (err == 0) {
//		strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
//		trajectory_d_rel(&mainboard.traj, -150);
//		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//		strat_set_speed(old_spdd, old_spda);
//		return err;
//	}
//
//	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
//
//	if (zone->flags & ZONE_F_DISC) {
//		trajectory_d_rel(&mainboard.traj, -350);
//		err = WAIT_COND_OR_TRAJ_END(!robot_is_near_disc(), flags);
//	}
//	else {
//		trajectory_d_rel(&mainboard.traj, -300);
//		err = wait_traj_end(flags);
//	}
//
//	strat_set_speed(old_spdd, old_spda);
//	if (err == 0)
//		return END_NEAR;
//	return err;
//}

static void strat_update_traj_speed(void)
{
	uint16_t d, a;

	d = strat_speed_d;
	if (strat_limit_speed_d && d > strat_limit_speed_d)
		d = strat_limit_speed_d;
	a = strat_speed_a;
	if (strat_limit_speed_a && a > strat_limit_speed_a)
		a = strat_limit_speed_a;
	
	trajectory_set_speed(&mainboard.traj, d, a);
}

void strat_set_speed(uint16_t d, uint16_t a)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	strat_speed_d = d;
	strat_speed_a = a;
	strat_update_traj_speed();
	IRQ_UNLOCK(flags);
}

void strat_get_speed(uint16_t *d, uint16_t *a)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	*d = strat_speed_d;
	*a = strat_speed_a;
	IRQ_UNLOCK(flags);
}

void strat_limit_speed_enable(void)
{
	strat_limit_speed_enabled = 1;
}

void strat_limit_speed_disable(void)
{
	strat_limit_speed_enabled = 0;
}

/* called periodically */
void strat_limit_speed(void)
{
	uint16_t lim_d = 0, lim_a = 0;
	int16_t opp_d, opp_a;
#ifdef TWO_OPPONENTS
	int16_t opp2_d, opp2_a;
#endif

	if (strat_limit_speed_enabled == 0)
		goto update;

#ifdef TWO_OPPONENTS
	get_opponent_da(&opp_d, &opp_a);
	get_opponent2_da(&opp2_d, &opp2_a);

	if(opp_d > opp2_d) {
		opp_d = opp2_d;
		opp_a = opp2_a;
	}

#else
	if (get_opponent_da(&opp_d, &opp_a) == -1)
		goto update;
#endif
		
#ifdef HOMOLOGATION
	if (opp_d < 600) {
		lim_d = SPEED_ANGLE_VERY_SLOW;
		lim_a = SPEED_ANGLE_VERY_SLOW;
	}
#else
	if (opp_d < 500) {
		if (mainboard.speed_d > 0 && (opp_a > 310 || opp_a < 40)) { // XXX tested with +/-70
			lim_d = SPEED_DIST_VERY_SLOW;
			lim_a = SPEED_ANGLE_FAST;
		}
		else if (mainboard.speed_d < 0 && (opp_a < 220 && opp_a > 140)) {
			lim_d = SPEED_DIST_VERY_SLOW;
			lim_a = SPEED_ANGLE_FAST;
		}
		else {
			lim_d = 2000;
			lim_a = SPEED_ANGLE_SLOW;
		}
	}
#endif		
	else if (opp_d < 600) {
		if (mainboard.speed_d > 0 && (opp_a > 310 || opp_a < 40)) {
			lim_d = 3000;
			lim_a = SPEED_ANGLE_FAST;
		}
		else if (mainboard.speed_d < 0 && (opp_a < 220 && opp_a > 140)) {
			lim_d = 3000;
			lim_a = SPEED_DIST_FAST;
		}
	}

 update:
	if (lim_d != strat_limit_speed_d ||
	    lim_a != strat_limit_speed_a) {
		strat_limit_speed_d = lim_d;
		strat_limit_speed_a = lim_a;

		DEBUG(E_USER_STRAT, "new speed limit da=%d,%d", lim_d, lim_a);
		strat_update_traj_speed();
	}
}

/* start the strat */
void strat_start(void)
{ 
	int8_t i;
	uint8_t err;

	strat_preinit();

	/* if start sw not plugged */
	if (sensor_get(S_START_SWITCH)) {
		printf_P(PSTR("No start switch, press a key or plug it\r\n"));

		/* while start sw not plugged */
		while (sensor_get(S_START_SWITCH)) {
			if (! cmdline_keypressed())
				continue;

			for (i=3; i>0; i--) {
				printf_P(PSTR("%d\r\n"), i);
				time_wait_ms(1000);
			}
			break;
		}
	}
	
	/* if start sw plugged */
	if (!sensor_get(S_START_SWITCH)) {
		printf_P(PSTR("Ready, unplug start switch to start\r\n"));
		/* while start sw plugged */
		while (!sensor_get(S_START_SWITCH));
	}

	/* reset infos, set traj speeds, set events ...*/
	strat_init();
	
	/* go to play */
	err = strat_main();
	
	NOTICE(E_USER_STRAT, "Finished !! returned %s", get_err(err));
	strat_exit();
}

/* return true if we have to brake due to an obstacle */
uint8_t __strat_obstacle(uint8_t which)
{
	int16_t x_rel, y_rel;
	int16_t opp_x, opp_y, opp_d, opp_a;
	int8_t ret;

	/* too slow */
	if (ABS(mainboard.speed_d) < 150)
		return 0;


#ifdef HOMOLOGATION
	/* opponent is in front of us */
	if (mainboard.speed_d > 0 && (sensor_get(S_OPPONENT_FRONT_R) || sensor_get(S_OPPONENT_FRONT_L))) {
		DEBUG(E_USER_STRAT, "opponent front");
		//sensor_obstacle_disable();
		return 1;
	}
	/* opponent is behind us */
	if (mainboard.speed_d < 0 && (sensor_get(S_OPPONENT_REAR_R) || sensor_get(S_OPPONENT_REAR_L))) {
		DEBUG(E_USER_STRAT, "opponent behind");
		//sensor_obstacle_disable();
		return 1;
	}
#endif


	if(which == 0)
		ret = get_opponent_xyda(&opp_x, &opp_y,&opp_d, &opp_a);
	else 
		ret = get_opponent2_xyda(&opp_x, &opp_y,&opp_d, &opp_a);

	/* no opponent detected */
	if (ret == -1) {
		return 0;
	}

	/* save obstacle position */
	opponent_obstacle.x = opp_x;
	opponent_obstacle.y = opp_y;
	opponent_obstacle.d = opp_d;
	opponent_obstacle.a = opp_a;

	/* sensor are temporarily disabled */
	if (sensor_obstacle_is_disabled()) 
		return 0;

	/* relative position */
	x_rel = cos(RAD(opp_a)) * (double)opp_d;
	y_rel = sin(RAD(opp_a)) * (double)opp_d;

	/* opponent too far */
	if (opp_d > 600)
		return 0;

	/* opponent is in front of us */
	if (mainboard.speed_d > 0 && (opp_a > 325 || opp_a < 35)) {
		DEBUG(E_USER_STRAT, "opponent front d=%d, a=%d "
		      "xrel=%d yrel=%d (speed_d=%d)", 
		      opp_d, opp_a, x_rel, y_rel, mainboard.speed_d);
		sensor_obstacle_disable();
		return 1;
	}
	/* opponent is behind us */
	if (mainboard.speed_d < 0 && (opp_a < 215 && opp_a > 145)) {
		DEBUG(E_USER_STRAT, "opponent behind d=%d, a=%d xrel=%d yrel=%d", 
		      opp_d, opp_a, x_rel, y_rel);
		sensor_obstacle_disable();
		return 1;
	}

	return 0;
}

/* return true if we have to brake due to an obstacle */
uint8_t strat_obstacle(void)
{
	if(__strat_obstacle(0))
		return 1;
#ifdef TWO_OPPONENTS
	if(__strat_obstacle(1))
		return 1;
#endif
	else
		return 0;
}

void interrupt_traj(void)
{
	traj_intr = 1;
}

void interrupt_traj_reset(void)
{
	traj_intr = 0;
}

uint8_t test_traj_end(uint8_t why)
{ 
	uint16_t cur_timer;
	point_t robot_pt;

	robot_pt.x = position_get_x_s16(&mainboard.pos);
	robot_pt.y = position_get_y_s16(&mainboard.pos);

	/* hardstop ends */
	if ((why & END_BLOCKING) && bd_get(&mainboard.angle.bd)) {
		strat_hardstop();
		return END_BLOCKING;
	}

	if ((why & END_BLOCKING) && bd_get(&mainboard.distance.bd)) {
		strat_hardstop();
		return END_BLOCKING;
	}

	if ((why & END_OBSTACLE) && strat_obstacle()) {
		strat_hardstop();
		return END_OBSTACLE;
	}

	/* interrupt traj by user */
	if ((why & END_INTR) && traj_intr) {
		interrupt_traj_reset();		
		return END_INTR;
	}

	/* traj ends succesfully */
	if ((why & END_TRAJ) && trajectory_finished(&mainboard.traj))
		return END_TRAJ;

	/* trigger an event at 3 sec before the end of the match */
	cur_timer = time_get_s();
	if ((mainboard.flags & DO_TIMER) && (why & END_TIMER)) {
		/* end of match */
		if (cur_timer >= MATCH_TIME)
			return END_TIMER;
	}

	/* we are near the destination point (depends on current
	 * speed) AND the robot is in the area bounding box. */
	if (why & END_NEAR) {
		int16_t d_near = 100;	
		
		if (mainboard.speed_d >= 2000)
			d_near = 150;
		
		if (trajectory_in_window(&mainboard.traj, d_near, RAD(5.0)) &&
		    is_in_boundingbox(&robot_pt))
			return END_NEAR;
	}
	
	return 0;
}

uint8_t __wait_traj_end_debug(uint8_t why, uint16_t line)
{
	uint8_t ret = 0;
	int16_t opp_x, opp_y, opp_d, opp_a;

	while (ret == 0){
		ret = test_traj_end(why);
	}
	if (ret == END_OBSTACLE) {
		if (get_opponent_xyda(&opp_x, &opp_y,
				      &opp_d, &opp_a) != -1)
			DEBUG(E_USER_STRAT, "Got %s at line %d"
			      " xy=(%d,%d) da=(%d,%d)", get_err(ret),
			      line, opp_x, opp_y, opp_d, opp_a);
	}
	else {
		DEBUG(E_USER_STRAT, "Got %s at line %d", 
		      get_err(ret), line);
	}
	return ret;
}
