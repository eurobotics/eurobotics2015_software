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


#define STANDS_READY_TIMEOUT    5000
#define STANDS_STORING_TIMEOUT	700


/** 
 *	Harvest several the 2 stands and the central cup in a path line 
 *	return END_TRAJ if the work is done, err otherwise 
 */
uint8_t strat_harvest_stands_and_cup_inline (void)
{
#define STAND_4_HARVEST_POS_X   (800-10)
#define STAND_5_HARVEST_POS_X   (1150-10)

#define __ROBOT_CENTER_CUP_FRONT  212
#define CUP_3_HARVEST_DISTANCE  (__ROBOT_CENTER_CUP_FRONT+10)

   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d = 0;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();

    /* set specific MAXIMUN speed and acceleration for DISTANCE */
	quadramp_set_1st_order_vars(&mainboard.distance.qr, SPEED_DIST_VERY_FAST, SPEED_DIST_VERY_FAST); 	/* set speed */
	quadramp_set_2nd_order_vars(&mainboard.distance.qr, ACC_DIST, ACC_DIST); 	                        /* set accel */

    /* set specific MAXIMUN speed and acceleration for ANGLE */
    quadramp_set_1st_order_vars(&mainboard.angle.qr, SPEED_ANGLE_VERY_FAST, SPEED_ANGLE_VERY_FAST); 	/* set speed */
	quadramp_set_2nd_order_vars(&mainboard.angle.qr, ACC_ANGLE, ACC_ANGLE); 		                    /* set accel */

    /* set new traj speeds */
	strat_set_speed (SPEED_DIST_VERY_FAST,SPEED_ANGLE_FAST);

    /* enable obstacle sensors */
    strat_opp_sensor_enable();
  
 
	/* TODO enable opponent sensors */

	/* turn to central cup */
	/* XXX done in start position 
	trajectory_turnto_xy(&mainboard.traj, COLOR_X(POPCORNCUP_CENTRE_X), POPCORNCUP_CENTRE_Y);
    err = wait_traj_end(END_INTR | END_TRAJ);
    if (err == END_INTR)
        goto intr;
	*/

	/* go close to central cup */
	d = distance_from_robot(COLOR_X(MY_CUP_3_X), MY_CUP_3_Y);
	trajectory_d_rel(&mainboard.traj, d-ROBOT_CENTER_TO_FRONT);

	/* prepare for harvesting stands and cup */
	i2c_slavedspic_mode_ss_harvest_ready(I2C_SIDE_LEFT, 0);
	i2c_slavedspic_mode_ss_harvest_ready(I2C_SIDE_RIGHT, 0);
	i2c_slavedspic_mode_ps(I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_READY);

    
    /* harvest stand 4 */
    err = WAIT_COND_OR_TRAJ_END (x_is_more_than(STAND_4_HARVEST_POS_X), TRAJ_FLAGS_NO_NEAR);

    if (!err)
    	i2c_slavedspic_mode_ss_harvest_do(COLOR_INVERT(I2C_SIDE_RIGHT), 0);
    else 
        ERROUT(err);

    /* mark stand as harvested */
    strat_infos.done_flags |= DONE_STAND_4;
                
    /* TODO: set lower speed */

    /* harvest stand 5 */

    /* disable obstacle sensors */
    strat_opp_sensor_disable();
    err = WAIT_COND_OR_TRAJ_END (x_is_more_than(STAND_5_HARVEST_POS_X), TRAJ_FLAGS_NO_NEAR);

    if (!err)
    	i2c_slavedspic_mode_ss_harvest_do(COLOR_INVERT(I2C_SIDE_LEFT), 0);
    else
        ERROUT(err);


    /* mark stand as harvested */
    strat_infos.done_flags |= DONE_STAND_5;

    /* harvest cup 3 */

	/* check sensor before continue */
	if ((sensor_get(S_OPPONENT_FRONT_L) || sensor_get(S_OPPONENT_FRONT_R))) {
		strat_hardstop ();
		ERROUT(END_OBSTACLE);
	}

	/* XXX enable obstacle sensors */
    strat_opp_sensor_enable();

    err = WAIT_COND_OR_TRAJ_END (distance_from_robot(COLOR_X(MY_CUP_3_X), MY_CUP_3_Y) < CUP_3_HARVEST_DISTANCE, TRAJ_FLAGS_NO_NEAR);

    if (!err) {
    	i2c_slavedspic_mode_ps(I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_CATCH_AND_DROP);
		i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, 3000);
	}
    else
        ERROUT(err);

    /* mark cup as harvested */
    strat_infos.done_flags |= DONE_CUP_3;

    /* wait trajectory end */
    err = wait_traj_end(TRAJ_FLAGS_STD);
    if (!TRAJ_SUCCESS(err)) 
        ERROUT(err);

end:
    /* in any case, if trajectory fails */
    if (!TRAJ_SUCCESS(err))
    {
        /* stop */
        strat_hardstop();
        
        /* go backwards and wait a bit */
      	trajectory_d_rel(&mainboard.traj, -OBS_CLERANCE);
        wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

        /* XXX, wait before continue, obstacle should go away */
        time_wait_ms (5000);
    }
	else {

        /* go backwards and wait a bit */
		//time_wait_ms (500);
      	trajectory_d_rel(&mainboard.traj, -OBS_CLERANCE);
        wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	}

  	/* end stuff */

    /* disable obstacle sensors */
    strat_opp_sensor_disable();

    /* restore MAXIMUN speed and acceleration for DISTANCE */
	quadramp_set_1st_order_vars(&mainboard.distance.qr, SPEED_DIST_VERY_FAST, SPEED_DIST_VERY_FAST); 	/* set speed */
	quadramp_set_2nd_order_vars(&mainboard.distance.qr, ACC_DIST, ACC_DIST); 	                        /* set accel */

    /* restore MAXIMUN speed and acceleration for ANGLE */
    quadramp_set_1st_order_vars(&mainboard.angle.qr, SPEED_ANGLE_VERY_FAST, SPEED_ANGLE_VERY_FAST); 	/* set speed */
	quadramp_set_2nd_order_vars(&mainboard.angle.qr, ACC_ANGLE, ACC_ANGLE); 		                    /* set accel */
    
    /* restore traj speeds */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}


/* get the angle and distance to an stand depending on harvesting side */
void get_stand_da (int16_t x, int16_t y, uint8_t side, int16_t *d, int16_t *a)
{
#define L 90.0
	double d1, a1, a2;

	abs_xy_to_rel_da((double)x, (double)y, &d1, &a1);
	a2 = asin(L/d1);
	
	//DEBUG (E_USER_STRAT, "d1=%f, a1=%f, a2=%f", d1, a1, a2);

	if (side == I2C_SIDE_LEFT)
		*a = (int16_t)DEG(a1 - a2);
	else
		*a = (int16_t)DEG(a1 + a2);

	*d =  (int16_t)sqrt((d1*d1)-(L*L));
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
	uint8_t calib_tries = 3;
	int16_t x_init, y_init;

	/* save init position */
	x_init = position_get_x_s16(&mainboard.pos);
	y_init = position_get_y_s16(&mainboard.pos);

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
	if (flags & STANDS_HARVEST_XY_IS_ROBOT_POSITION) {
		trajectory_turnto_xy(&mainboard.traj, x, y);
	}	
	else {
		trajectory_a_rel(&mainboard.traj, a);
	}
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* prepare for harvesting */
	if (side == I2C_SIDE_ALL) {

retry_ready_both:
        /* wait ready and harvest */
		i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_LEFT, STATUS_READY, STANDS_READY_TIMEOUT);
		i2c_slavedspic_mode_ss_harvest_ready(I2C_SIDE_LEFT, blade_angle);

		i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_RIGHT, STATUS_READY, STANDS_READY_TIMEOUT);
		i2c_slavedspic_mode_ss_harvest_ready(I2C_SIDE_RIGHT, blade_angle);

		/* wait blades ready */
		i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_RIGHT, STATUS_READY, STANDS_READY_TIMEOUT);
		if (i2c_slavedspic_ss_test_status(I2C_SIDE_RIGHT, STATUS_ERROR|STATUS_BLOCKED))
			goto retry_ready_both;

		i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_RIGHT, STATUS_READY, STANDS_READY_TIMEOUT);
		if (i2c_slavedspic_ss_test_status(I2C_SIDE_RIGHT, STATUS_ERROR|STATUS_BLOCKED))
			goto retry_ready_both;

	}
	else {
retry_ready_one:
        /* wait ready */
		i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_READY, STANDS_READY_TIMEOUT);
		i2c_slavedspic_mode_ss_harvest_ready(side, blade_angle);
		
		/* wait blades ready */
		i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_READY, STANDS_READY_TIMEOUT);

		if (i2c_slavedspic_ss_test_status(side, STATUS_ERROR|STATUS_BLOCKED))
			goto retry_ready_one;
	}

	/* wait blades ready */
	//time_wait_ms(500);

	/* harvest, go close to stands but without touch */
	strat_set_speed (harvest_speed, SPEED_ANGLE_SLOW);

	if (flags & STANDS_HARVEST_XY_IS_ROBOT_POSITION) {
		d = distance_from_robot(x, y);
	}
	else { 
		d -= (ROBOT_CENTER_TO_MOUTH + STANDS_RADIOUS);
	}
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

		/* TODO: avance a bit for to be sure we harvest it */
	}

	/* debug */
	//state_debug_wait_key_pressed();

harvest_stand:
	/* harvest stands */
	if (side == I2C_SIDE_ALL) {
		/* do harvest */
		i2c_slavedspic_mode_ss_harvest_do(I2C_SIDE_LEFT, blade_angle);
		i2c_slavedspic_mode_ss_harvest_do(I2C_SIDE_RIGHT, blade_angle);

		/* wait storing */
		i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_LEFT, STATUS_STORING, STANDS_STORING_TIMEOUT);
		i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_RIGHT, STATUS_STORING, STANDS_STORING_TIMEOUT);
	}
	else {
		/* do harvest */
		i2c_slavedspic_mode_ss_harvest_do(side, blade_angle);
		
		/* wait storing */
		i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_STORING, STANDS_STORING_TIMEOUT);
	}

	/* XXX wait storing */
	//time_wait_ms(200);

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
	else if (flags & STANDS_HARVEST_STAND_GROUP_3) {

		if (opponent1_is_behind() || opponent2_is_behind())
			strat_set_speed (SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);
		else
			strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

		trajectory_d_rel(&mainboard.traj, -70);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);	

		trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(-85));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
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

		//d = position_get_x_s16(&mainboard.pos);

		err = strat_calib(400, TRAJ_FLAGS_SMALL_DIST);

		DEBUG (E_USER_STRAT, "CALIB: a = %d, diff = %d", position_get_a_deg_s16 (&mainboard.pos),
				ABS(position_get_a_deg_s16 (&mainboard.pos)-COLOR_A_ABS(180)));

#define CALIB_A_OK 10
		if ((ABS(position_get_a_deg_s16 (&mainboard.pos)-COLOR_A_ABS(180))) < CALIB_A_OK)
		{
			strat_reset_pos(COLOR_X(ROBOT_CENTER_TO_FRONT),
							DO_NOT_SET_POS,
							COLOR_A_ABS(180));
		}
		else if (calib_tries) 
		{

			DEBUG (E_USER_STRAT, "WARNING: calib fails, try again");
			calib_tries --;

			/* store a possible stand in mouth */
			if (side == I2C_SIDE_ALL) {
				i2c_slavedspic_mode_ss_harvest_do(I2C_SIDE_LEFT, blade_angle);
				i2c_slavedspic_mode_ss_harvest_do(I2C_SIDE_RIGHT, blade_angle);

				/* wait storing */
				i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_LEFT, STATUS_STORING, STANDS_STORING_TIMEOUT);
				i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_RIGHT, STATUS_STORING, STANDS_STORING_TIMEOUT);
			}
			else {
				i2c_slavedspic_mode_ss_harvest_do(side, blade_angle);
		
				/* wait storing */
				i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_STORING, STANDS_STORING_TIMEOUT);
			}

			/* go backwards */
			trajectory_d_rel(&mainboard.traj, -(2*STANDS_RADIOUS));
		    err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			/* turn infront of wall */
			trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(180));
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

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

			/* go forward */
			trajectory_d_rel(&mainboard.traj, STANDS_RADIOUS);
		    err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			/* try to harvest a stand very close to wall */
			goto harvest_stand;
		}


	    /* XXX return to init position */
	    if (flags & STANDS_HARVEST_BACK_INIT_POS) {

			if (opponent1_is_behind() || opponent2_is_behind())
				strat_set_speed (SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);
			else
				strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

            //d = ABS(d-position_get_x_s16(&mainboard.pos));
            d = distance_from_robot (x_init, y_init);
		    trajectory_d_rel(&mainboard.traj, -d);
		    err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		    if (!TRAJ_SUCCESS(err))
		       ERROUT(err);	
	    }
	}

end:
	/* wait sensor reinforcement */
	//if(!TRAJ_SUCCESS(err))
	//	time_wait_ms(2500);

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
#define SPEED_DIST_RELEASE_STANDS	300
#define TOWER_BUILDINT_TIMEOUT		20000
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

	/* prevent un-harvested stands in mouth */
	i2c_slavedspic_mode_ss_harvest_do(I2C_SIDE_LEFT, 0);
	i2c_slavedspic_mode_ss_harvest_do(I2C_SIDE_RIGHT, 0);

	/* wait storing */
	i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_LEFT, STATUS_STORING, STANDS_STORING_TIMEOUT);
	i2c_slavedspic_ss_wait_status_or_timeout (I2C_SIDE_RIGHT, STATUS_STORING, STANDS_STORING_TIMEOUT);


	/* clean zone, in case of platform */
	if (strat_smart[MAIN_ROBOT].current_zone == ZONE_MY_PLATFORM) {

		trajectory_a_abs(&mainboard.traj, 90);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);	

		/* go and back to the building position */
		d = distance_from_robot(x, y) - ROBOT_CENTER_TO_BACK + STANDS_RADIOUS;
		trajectory_d_rel(&mainboard.traj, -d);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);	

		trajectory_d_rel(&mainboard.traj, d);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);	
	}

	/* turn to x,y */
	trajectory_turnto_xy (&mainboard.traj, x, y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* be sure we are in angle */
	time_wait_ms (200);	

    /* go to the building position */
	d = distance_from_robot(x, y);
    d -= (ROBOT_CENTER_TO_MOUTH + STANDS_RADIOUS);
	trajectory_d_rel(&mainboard.traj, d);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	

	/* built the spotlight */
	if (flags & STANDS_RELEASE_DO_TOWER)
	{
		/* avoid possible popcorn in mouth */
		if (strat_smart[MAIN_ROBOT].current_zone == ZONE_MY_HOME_SPOTLIGHT) 
		{		
			strat_set_speed (SPEED_DIST_RELEASE_STANDS, SPEED_ANGLE_VERY_SLOW);
			trajectory_d_rel(&mainboard.traj, -(2.5*STANDS_RADIOUS));
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			if (!TRAJ_SUCCESS(err))
			   ERROUT(err);	
		}

		i2c_slavedspic_mode_ss(I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT, COLOR_INVERT(SIDE_LEFT));
		i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_READY, TOWER_BUILDINT_TIMEOUT);

#if 0

//		i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_READY|STATUS_BLOCKED, TOWER_BUILDINT_TIMEOUT);

		ret = i2c_slavedspic_get_ps_status();
		if (ret & STATUS_BLOCKING) 
		{

			/* open blades */
			i2c_slavedspic_mode_blades(SIDE_RIGHT, I2C_STANDS_BLADE_MODE_CENTER);
			WAIT_COND_OR_TIMEOUT((i2c_slavedspic_get_status() == STATUS_READY), STANDS_READY_TIMEOUT);    

			i2c_slavedspic_mode_blades(SIDE_LEFT, I2C_STANDS_BLADE_MODE_CENTER);
			WAIT_COND_OR_TIMEOUT(i2c_slavedspic_get_status() == STATUS_READY, STANDS_READY_TIMEOUT);    

			/* go forward */
			strat_set_speed (SPEED_DIST_RELEASE_STANDS, SPEED_ANGLE_VERY_SLOW);
			trajectory_d_rel(&mainboard.traj, (2.5*STANDS_RADIOUS));
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			if (!TRAJ_SUCCESS(err))
			   ERROUT(err);	

			goto try_tower;
		}
#endif

		/* release spotlight */
		i2c_slavedspic_mode_ss(I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT, COLOR_INVERT(SIDE_LEFT));

		strat_set_speed (SPEED_DIST_RELEASE_STANDS, SPEED_ANGLE_VERY_SLOW);
		trajectory_d_rel(&mainboard.traj, -(2*STANDS_RADIOUS));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);	
	}
	else 
	{
		/* open blades */
		//i2c_slavedspic_mode_blades(SIDE_RIGHT, I2C_STANDS_BLADE_MODE_CENTER);
		//WAIT_COND_OR_TIMEOUT(i2c_slavedspic_get_status() == STATUS_READY, STANDS_READY_TIMEOUT);    
		//time_wait_ms(500);

		//i2c_slavedspic_mode_blades(SIDE_LEFT, I2C_STANDS_BLADE_MODE_CENTER);
		//WAIT_COND_OR_TIMEOUT(i2c_slavedspic_get_status() == STATUS_READY, STANDS_READY_TIMEOUT);    
		//time_wait_ms(500);

		/* release spotlight left */
		i2c_slavedspic_mode_ss(I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT, COLOR_INVERT(SIDE_LEFT));
		i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_READY, STANDS_READY_TIMEOUT);

		/* go backwards */
		strat_set_speed (SPEED_DIST_RELEASE_STANDS, SPEED_ANGLE_VERY_SLOW);
		trajectory_d_rel(&mainboard.traj, -(2*STANDS_RADIOUS));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);	

		/* release spotlight right */
		i2c_slavedspic_mode_ss(I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT, COLOR_INVERT(SIDE_RIGHT));
		i2c_slavedspic_ss_wait_status_or_timeout (side, STATUS_READY, STANDS_READY_TIMEOUT);
	}


	/* XXX if TIME_OVER mode, simply exit and block */
	if (flags & STANDS_RELEASE_TIME_OVER) {

		/* go backwards */
		strat_set_speed (SPEED_DIST_RELEASE_STANDS, SPEED_ANGLE_VERY_SLOW);
		trajectory_d_rel(&mainboard.traj, -(2*STANDS_RADIOUS));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

		//strat_exit();
		while(1);
	}

retry:
	/* go backwards */
	strat_set_speed (SPEED_DIST_RELEASE_STANDS, SPEED_ANGLE_VERY_SLOW);
	d = distance_from_robot(x_init, y_init);
	trajectory_d_rel(&mainboard.traj, -d);
//	trajectory_goto_xy_abs (&mainboard.traj, x_init, y_init);
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

/* decides if we need build a tower */
uint8_t strat_need_build_a_tower (void)
{
	if (strat_infos.conf.flags & CONF_FLAG_DO_TOWER)
		return STANDS_RELEASE_DO_TOWER;
	else
		return 0;
}


/* escape from upper zone */
uint8_t strat_escape_form_upper_zone (uint8_t flags)
{
#define X_UP 	950
#define X_DOWN	0
#define Y_UP 	AREA_Y
#define Y_DOWN 	1200

#define MIDDLE_LINE 650

   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t x_init, y_init;

	/* save init position */
	x_init = position_get_x_s16(&mainboard.pos);
	y_init = position_get_y_s16(&mainboard.pos);

    strat_opp_sensor_enable();
	//strat_opp_sensor_middle_enable();

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);

try:
	trajectory_a_abs(&mainboard.traj, -90);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);


	while ((opp1_x_is_more_than(COLOR_X(400)) && opp1_y_is_more_than(1200)) ||
		   (opp2_x_is_more_than(COLOR_X(400)) && opp2_y_is_more_than(1200)));

	strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	trajectory_goto_xy_abs(&mainboard.traj, position_get_x_s16(&mainboard.pos), 1000);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err)) 
	{
		trajectory_goto_xy_abs(&mainboard.traj, x_init, y_init);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		goto try;
	}


#if 0
    /* try to escape with avoid to the center of field */
    do {
        /* XXX */
        clerance_minimum_enable ();

        /* if OPP near stair, escape thru cinema side */
        if ((opponent1_is_in_area(COLOR_X(X_UP), Y_UP, COLOR_X(X_DOWN), Y_DOWN) && opp1_x_is_more_than(MIDDLE_LINE)) ||
            (opponent2_is_in_area(COLOR_X(X_UP), Y_UP, COLOR_X(X_DOWN), Y_DOWN) && opp2_x_is_more_than(MIDDLE_LINE)) )
        {
	        err = goto_and_avoid_forward (COLOR_X(400),
						                  400,
						                  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);

        }
        /* else, if OPP near to cinemas, escape thru stairs side */
        else if ((opponent1_is_in_area(COLOR_X(X_UP), Y_UP, COLOR_X(X_DOWN), Y_DOWN) /*&& !opp1_x_is_more_than(MIDDLE_LINE)*/) ||
                 (opponent2_is_in_area(COLOR_X(X_UP), Y_UP, COLOR_X(X_DOWN), Y_DOWN) /*&& !opp2_x_is_more_than(MIDDLE_LINE)*/) )
        {
	        err = goto_and_avoid_forward (COLOR_X(AREA_Y/2),
						                  AREA_Y/2 + 100,
						                  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
        }

		if (!TRAJ_SUCCESS(err)) {
			time_wait_ms (2000);
			/* TODO: get space */
		}

    } while (!TRAJ_SUCCESS(err));
#endif

end:

    strat_opp_sensor_disable();
	strat_opp_sensor_middle_disable();

	/* end stuff */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}


