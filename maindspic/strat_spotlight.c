/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
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
 *  Javier Balias Santos <javier@arc-robots.org> and Silvia Santano
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <dac_mc.h>
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

#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"
#include "cmdline.h"


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)

/**
 * 	TODO
	pickup_cup_rear (x,y)
	pickup_cup_front (x, y, release_after_harvesting)
	release_cup_front ()

	pickup_stands_close_to_wall (x, y, left_blade_angle, right_blade_angle, left_num, right_num, do_calib)
	pickup_stands_and_cup_inpath (x, y, left_blade_angle, right_blade_angle, left_num, right_num, do_cup)
	pickup_stand_generic (x, y, column_side)

	pickup_popcorn_machine_and_calib_y ()

	close_capperboard_on_the_path (x, y, direction, side, hide_stick)
	close_clapperboard_turning (x, y, stick_type, hide_stick)
	close_clapperboard_infront (x, y, hide_stick)

	release_rear_cup_and_stored_popcorns (x, y)
	built_and_release_spotlight (x, y)
 */

/* template, return END_TRAJ if the work is done, err otherwise */
#if 0
uint8_t strat_ (int16_t x, int16_t y, uint8_t foo)
{
//#define DEBUG_STRAT_
#ifdef DEBUG_STRAT_
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif

   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_FAST);
   


end:
	/* end stuff */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}
#endif

/** 
 *	Harvest several the 2 stands and the central cup in a path line 
 *	return END_TRAJ if the work is done, err otherwise 
 */
uint8_t strat_harvest_stands_and_cup_inline (void)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d = 0;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_FAST,SPEED_ANGLE_FAST);
   
	/* TODO enable opponent sensors */

	/* turn to central cup */
	/* XXX done in start position 
	trajectory_turnto_xy(&mainboard.traj, COLOR_X(POPCORNCUP_CENTRE_X), POPCORNCUP_CENTRE_Y);
    err = wait_traj_end(END_INTR | END_TRAJ);
    if (err == END_INTR)
        goto intr;
	*/

	/* prepare for harvesting */
	i2c_slavedspic_mode_ss_harvest(I2C_SIDE_LEFT, 0);
	i2c_slavedspic_mode_ss_harvest(I2C_SIDE_RIGHT, 0);
	i2c_slavedspic_mode_ps(I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_READY);

	/* go close to central cup */
	d = distance_from_robot(COLOR_X(POPCORNCUP_CENTRE_X), POPCORNCUP_CENTRE_Y);
	trajectory_d_rel(&mainboard.traj, d-ROBOT_CENTER_TO_FRONT);
	err = wait_traj_end(TRAJ_FLAGS_STD);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* if traj succesfull, pick up cup */
	i2c_slavedspic_mode_ps(I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_CATCH_AND_DROP);
	time_wait_ms(500);
	i2c_slavedspic_mode_ps(I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_RELEASE);	

	/* NOTE for strat: at this point the 2 first stands and the cup must be checked
	   as done, never mind if we cached or not */

end:
	/* end stuff */
	i2c_slavedspic_mode_ps(I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_HIDE);
	trajectory_d_rel(&mainboard.traj, -80);
	wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	/* ignore error */

	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}


/** 
 *	Harvest stands parallel to the wall
 *	return END_TRAJ if the work is done, err otherwise 
 */
uint8_t strat_harvest_stands_parallel_to_wall (int16_t x, int16_t y,
							 uint8_t side, uint8_t blade_angle,
							 uint8_t calib_x, uint8_t back_to_boundinbox)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d = 0;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_SLOW);

	/* prepare for harvesting */
	if (side == I2C_SIDE_ALL) {
		i2c_slavedspic_mode_ss_harvest(I2C_SIDE_LEFT, blade_angle);
		i2c_slavedspic_mode_ss_harvest(I2C_SIDE_RIGHT, blade_angle);
	}
	else 
		i2c_slavedspic_mode_ss_harvest(side, blade_angle);


	/* turn to stands */
	trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(180));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* harvest */
	d = distance_from_robot(x, y);
	trajectory_d_rel(&mainboard.traj, d-ROBOT_CENTER_TO_MOUTH);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	

	/* calib x position and angle */
	if (calib_x)
	{
		/* calibrate position on the wall */
		strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);

		trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(180));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		time_wait_ms(200);

		err = strat_calib(400, TRAJ_FLAGS_SMALL_DIST);
		strat_reset_pos(COLOR_X(ROBOT_CENTER_TO_FRONT),
						DO_NOT_SET_POS,
						COLOR_A_ABS(180));
	}

	/* go in boundingbox */
	if (back_to_boundinbox) {
		d = distance_from_robot(COLOR_X(OBS_CLERANCE+20), position_get_y_s16(&mainboard.pos));
		trajectory_d_rel(&mainboard.traj, d);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);	
	}

end:
	/* end stuff */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}

/* get the angle and distance to an stand depending on harvesting side */
void get_stand_da (int16_t x, int16_t y, uint8_t side, int16_t *d, int16_t *a)
{
#define L 90.0
	double d1, a1, a2;

	abs_xy_to_rel_da((double)x, (double) y, &d1, &a1);
	a2 = asin(L/d1);
	
	if (side == I2C_SIDE_LEFT)
		*a = DEG((int16_t)(a1 - a2));
	else
		*a = DEG((int16_t)(a1 + a2));

	*d =  (int16_t)sqrt((L*L) + d1*d1);
}

/** 
 *	Harvest stands parallel to the wall
 *	return END_TRAJ if the work is done, err otherwise 
 */
uint8_t strat_harvest_orphan_stands (int16_t x, int16_t y, uint8_t side_target,
									 uint8_t side, uint8_t blade_angle)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d = 0, a = 0;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_SLOW);

	/* prepare for harvesting */
	if (side == I2C_SIDE_ALL) {
		i2c_slavedspic_mode_ss_harvest(I2C_SIDE_LEFT, blade_angle);
		i2c_slavedspic_mode_ss_harvest(I2C_SIDE_RIGHT, blade_angle);
	}
	else
		i2c_slavedspic_mode_ss_harvest(side, blade_angle);

	/* get d,a target */
	if (side_target != I2C_SIDE_ALL)
		get_stand_da (x, y, side_target, &d, &a);

	/* turn to stand */
	trajectory_a_rel(&mainboard.traj, a);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* harvest */
	trajectory_d_rel(&mainboard.traj, d-ROBOT_CENTER_TO_MOUTH);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	

end:
	/* end stuff */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}

/** 
 *	Built a spotlight and release
 *	return END_TRAJ if the work is done, err otherwise 
 */
uint8_t strat_buit_and_release_spotlight (int16_t x, int16_t y, uint8_t side)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d = 0;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_SLOW);

	/* go in home */
	trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(180));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	time_wait_ms (200);

	d = distance_from_robot(x, y);
	trajectory_d_rel(&mainboard.traj, d);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	

	/* built the spotlight */
#ifdef done
	i2c_slavedspic_mode_ss(I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT, COLOR_INVERT(side));
	do {
		ret = i2c_slavedspic_get_ss_status(COLOR_INVERT(side));
	} while (ret!=I2C_SLAVEDSPIC_STATUS_READY &&
		   	 ret!=I2C_SLAVEDSPIC_STATUS_ERROR &&
		     ret!=I2C_SLAVEDSPIC_STATUS_BLOCKING);

	if (ret != I2C_SLAVEDSPIC_STATUS_READY) {
		ERROR (E_USER_STRAT, "ERROR building spotlight, returns %d", ret);
	}
#endif
	/* release spotlight */
	i2c_slavedspic_mode_ss(I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT, COLOR_INVERT(side));

	strat_set_speed (SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);
	trajectory_d_rel(&mainboard.traj, -(ROBOT_CENTER_TO_FRONT-ROBOT_CENTER_TO_MOUTH+10));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	

end:
	/* end stuff */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}




