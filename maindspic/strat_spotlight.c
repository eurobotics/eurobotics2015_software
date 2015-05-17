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

#define ROBOT_CENTER_TO_MOUTH	105
#define STANDS_RADIOUS			30
#define STANDS_READY_TIMEOUT    5000
#define STANDS_STORING_TIMEOUT	700
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
	d = distance_from_robot(COLOR_X(MY_CUP_3_X), MY_CUP_3_Y);
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


/* get the angle and distance to an stand depending on harvesting side */
void get_stand_da (int16_t x, int16_t y, uint8_t side, int16_t *d, int16_t *a)
{
#define L 90.0
	double d1, a1, a2;

	abs_xy_to_rel_da((double)x, (double) y, &d1, &a1);
	a2 = asin(L/d1);
	
	//DEBUG (E_USER_STRAT, "d1=%f, a1=%f, a2=%f", d1, a1, a2);

	if (side == I2C_SIDE_LEFT)
		*a = (int16_t)DEG(a1 - a2);
	else
		*a = (int16_t)DEG(a1 + a2);

	*d =  (int16_t)sqrt((L*L) + (d1*d1));
}

/** 
 *	Harvest orphan stands
 *	return END_TRAJ if the work is done, err otherwise 
 */
uint8_t strat_harvest_orphan_stands (int16_t x, int16_t y, uint8_t side_target,
									 uint8_t side, uint8_t blade_angle, 
									 uint16_t harvest_speed, uint8_t flags)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d = 0, a = 0;
	uint8_t calib_tries = 2;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

try_again:
	/* get d,a target */
	if (side_target != I2C_SIDE_ALL)
		get_stand_da (x, y, side_target, &d, &a);


	//DEBUG (E_USER_STRAT, "d = %d, a = %d", d, a);
	//state_debug_wait_key_pressed();

	/* turn to stand */
	trajectory_a_rel(&mainboard.traj, a);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* prepare for harvesting */
	if (side == I2C_SIDE_ALL) {
        /* wait ready and harvest */
		i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_LEFT, STATUS_READY, STANDS_READY_TIMEOUT);
		i2c_slavedspic_mode_ss_harvest_ready(I2C_SIDE_LEFT, blade_angle);

		i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_RIGHT, STATUS_READY, STANDS_READY_TIMEOUT);
		i2c_slavedspic_mode_ss_harvest_ready(I2C_SIDE_RIGHT, blade_angle);

		/* wait blades ready */
		i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_RIGHT, STATUS_READY, STANDS_READY_TIMEOUT);

	}
	else {
        /* wait ready */
		i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_READY, STANDS_READY_TIMEOUT);
		i2c_slavedspic_mode_ss_harvest_ready(side, blade_angle);
		
		/* wait blades ready */
		i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_READY, STANDS_READY_TIMEOUT);
	}

	/* wait blades ready */
	//time_wait_ms(500);

	/* harvest, go close to stands but without touch */
	strat_set_speed (harvest_speed, SPEED_ANGLE_SLOW);
	d -= (ROBOT_CENTER_TO_MOUTH + STANDS_RADIOUS + 10);
	trajectory_d_rel(&mainboard.traj, d);

	if (strat_smart[MAIN_ROBOT].current_zone == ZONE_MY_STAND_GROUP_1) 
	{
		err = WAIT_COND_OR_TRAJ_END((sensor_get(S_OPPONENT_FRONT_L) || sensor_get(S_OPPONENT_FRONT_R)), TRAJ_FLAGS_NO_NEAR);
		if (err == 0) {
			strat_hardstop();
			time_wait_ms(500);
			goto try_again;
		}
		else if (!TRAJ_SUCCESS(err))
		   ERROUT(err);	
	}
	else {
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			   ERROUT(err);	
	}

	/* debug */
	//state_debug_wait_key_pressed();


	/* harvest stands */
	if (side == I2C_SIDE_ALL) {
//		i2c_slavedspic_mode_blades(SIDE_RIGHT, I2C_STANDS_BLADE_MODE_PUSH_STAND_RIGHT);
//		i2c_slavedspic_wait_ready();
//		i2c_slavedspic_mode_blades(SIDE_LEFT, I2C_STANDS_BLADE_MODE_PUSH_STAND_LEFT);

		i2c_slavedspic_mode_ss_harvest_do(I2C_SIDE_LEFT, blade_angle);
		i2c_slavedspic_mode_ss_harvest_do(I2C_SIDE_RIGHT, blade_angle);

		/* wait storing */
		i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_LEFT, STATUS_STORING, STANDS_STORING_TIMEOUT);
		i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_RIGHT, STATUS_STORING, STANDS_STORING_TIMEOUT);

		
	}
	else {
//		i2c_slavedspic_mode_blades(side, 
//								   side == I2C_SIDE_LEFT? I2C_STANDS_BLADE_MODE_PUSH_STAND_LEFT:
//									 					  I2C_STANDS_BLADE_MODE_PUSH_STAND_RIGHT);

		i2c_slavedspic_mode_ss_harvest_do(side, blade_angle);
		
		/* wait storing */
		i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_STORING, STANDS_STORING_TIMEOUT);
	}

	/* wait storing */
	//time_wait_ms(700);

	/* XXX return to init position */
	if (flags & STANDS_HARVEST_BACK_INIT_POS) {

		if (opponent1_is_behind() || opponent2_is_behind())
			strat_set_speed (SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);
		else
			strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

		trajectory_d_rel(&mainboard.traj, -d);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);	
	}

	/* calib x position and angle */
	if (flags & STANDS_HARVEST_CALIB_X)
	{
        /* TODO: wait for storing status */

		/* calibrate position on the wall */
		strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);

		trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(180));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		time_wait_ms(200);

		d = position_get_x_s16(&mainboard.pos);

		err = strat_calib(400, TRAJ_FLAGS_SMALL_DIST);

#define CALIB_A_OK 6
		if ((ABS(position_get_a_deg_s16 (&mainboard.pos))-COLOR_A_ABS(180)) < CALIB_A_OK)
		{
			strat_reset_pos(COLOR_X(ROBOT_CENTER_TO_FRONT),
							DO_NOT_SET_POS,
							COLOR_A_ABS(180));
		}
		else if (calib_tries) 
		{
			calib_tries --;

			/* go backwards */
			trajectory_d_rel(&mainboard.traj, -(2*STANDS_RADIOUS));
		    err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			/* try to harvest a stand very close to wall */
			x = COLOR_X(STANDS_RADIOUS);
			goto try_again;
		}


	    /* XXX return to init position */
	    if (flags & STANDS_HARVEST_BACK_INIT_POS) {

			if (opponent1_is_behind() || opponent2_is_behind())
				strat_set_speed (SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);
			else
				strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

            d = ABS(d-position_get_x_s16(&mainboard.pos));
		    trajectory_d_rel(&mainboard.traj, -d);
		    err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		    if (!TRAJ_SUCCESS(err))
		       ERROUT(err);	
	    }
	}

end:
	/* wait sensor reinforcement */
	if(!TRAJ_SUCCESS(err))
		time_wait_ms(2500);

	/* end stuff */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}

/** 
 *	Built a spotlight and release
 *	return END_TRAJ if the work is done, err otherwise 
 */
uint8_t strat_buit_and_release_spotlight (int16_t x, int16_t y, uint8_t side, uint8_t flags)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d = 0;
	int16_t x_init,y_init;

	x_init = position_get_x_s16(&mainboard.pos);
	y_init = position_get_y_s16(&mainboard.pos);

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_SLOW);

	/* turn to x,y */
	trajectory_turnto_xy (&mainboard.traj, x, y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* be sure we are in angle */
	time_wait_ms (200);

    /* go to the building position */
	d = distance_from_robot(x, y);
	trajectory_d_rel(&mainboard.traj, d);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	

	/* built the spotlight */
	if (flags & STAND_RELEASE_DO_TOWER) 
	{
		i2c_slavedspic_mode_ss(I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT, side);
		i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_STORING, 20000);

		/* release spotlight */
		i2c_slavedspic_mode_ss(I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT, COLOR_INVERT(side));

		strat_set_speed (SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);
		trajectory_d_rel(&mainboard.traj, -(ROBOT_CENTER_TO_FRONT-ROBOT_CENTER_TO_MOUTH+10));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);	
	}
	else 
	{
		/* open blades */
		i2c_slavedspic_mode_blades(SIDE_RIGHT, I2C_STANDS_BLADE_MODE_CENTER);
		//WAIT_COND_OR_TIMEOUT(i2c_slavedspic_get_status() == STATUS_READY, STANDS_READY_TIMEOUT);    
		time_wait_ms(500);

		i2c_slavedspic_mode_blades(SIDE_LEFT, I2C_STANDS_BLADE_MODE_CENTER);
		//WAIT_COND_OR_TIMEOUT(i2c_slavedspic_get_status() == STATUS_READY, STANDS_READY_TIMEOUT);    
		time_wait_ms(500);

		/* release spotlight left */
		i2c_slavedspic_mode_ss(I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT, COLOR_INVERT(SIDE_LEFT));
		WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ss_test_status(SIDE_LEFT, STATUS_READY|STATUS_BLOCKED), STANDS_READY_TIMEOUT);

		/* go backwards */
		strat_set_speed (300, SPEED_ANGLE_VERY_SLOW);
		trajectory_d_rel(&mainboard.traj, -100);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err)) {
		   ERROUT(err);	
		}

		/* release spotlight right */
		i2c_slavedspic_mode_ss(I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT, COLOR_INVERT(SIDE_RIGHT));
		WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ss_test_status(SIDE_RIGHT, STATUS_READY|STATUS_BLOCKED), STANDS_READY_TIMEOUT);
	}

retry:
	/* go backwards */
	strat_set_speed (300, SPEED_ANGLE_VERY_SLOW);
	//trajectory_d_rel(&mainboard.traj, -200);
	trajectory_goto_xy_abs (&mainboard.traj, x_init, y_init);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err)) {
		while (opponent1_is_behind() || opponent2_is_behind());
	   	goto retry;
		//ERROUT(err);	
	}

end:
	/* end stuff */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}




