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
 *  Javier Balias Santos <javier@arc-robots.org> and Silvia Santano and Miguel Ortiz
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

#define ROBOT_CENTER_CUP_FRONT  212
#define ROBOT_CENTER_CUP_REAR   119

#define POPCORN_FRONT_READY_TIMEOUT 3000
#define POPCORN_REAR_READY_TIMEOUT  1000

/**
 *	Harvest popcorn cups
 *	return END_TRAJ if the work is done, err otherwise
 */
uint8_t strat_harvest_popcorn_cup (int16_t x, int16_t y, uint8_t side, uint8_t flags)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d = 0;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);


	/* check opp */
    if (side == SIDE_FRONT) 
	{
		/* check sensor before continue */
		if ((sensor_get(S_OPPONENT_FRONT_L) || sensor_get(S_OPPONENT_FRONT_R))) {
			ERROUT(END_OBSTACLE);
		}	

        strat_opp_sensor_enable();
	}

    /* prepare cup clamp */
    side == SIDE_FRONT? i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_READY):
                        i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_OPEN);

    /* wait ready */
    side == SIDE_FRONT?
    i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, POPCORN_FRONT_READY_TIMEOUT):
    i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, POPCORN_REAR_READY_TIMEOUT);


    /* turn to cup */
    side == SIDE_FRONT? trajectory_turnto_xy(&mainboard.traj, x, y):
                        trajectory_turnto_xy_behind(&mainboard.traj, x, y);
 
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* wait ready */
    side == SIDE_FRONT?
    i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, POPCORN_FRONT_READY_TIMEOUT):
    i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, POPCORN_REAR_READY_TIMEOUT);

    /* go in clamp range */
    d = distance_from_robot(x, y);
    side == SIDE_FRONT? (d = d-ROBOT_CENTER_CUP_FRONT-10) :
                        (d = -(d-ROBOT_CENTER_CUP_REAR-20));


    /* enable obstacle sensors in front case */
    if (side == SIDE_FRONT) {
	    trajectory_d_rel(&mainboard.traj, d);
	    err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    }
    else {
	    trajectory_d_rel(&mainboard.traj, d);
	    err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	}

    if (!TRAJ_SUCCESS(err)) 
    {
        /* abort */
        if (side == SIDE_FRONT) 
        {
            /* go backwards, get space */
         	trajectory_d_rel(&mainboard.traj, -OBS_CLERANCE);
	        wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

            /* hide clamp */
            i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_HIDE);
        }
        ERROUT(err);
    }


	/* XXX debug step use only for subtraj command */
	//state_debug_wait_key_pressed();

    /* front cup: pick up, drop popcorns in side, and release the cup */
    /* rear cup: pick up */
    side == SIDE_FRONT? i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_CATCH_AND_DROP):
                        i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_CATCH);

    /* wait ready */
    side == SIDE_FRONT?
    i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, POPCORN_FRONT_READY_TIMEOUT):
    i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, POPCORN_REAR_READY_TIMEOUT);

	/* release front cap and hide system */
	if (side == SIDE_FRONT) {

		if (!(flags & POPCORN_CUP_HARVEST_DO_NOT_RELEASE)) {
			time_wait_ms(1500);
			i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_RELEASE);
			i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, 1000);
		}

		trajectory_d_rel(&mainboard.traj, -(OBS_CLERANCE+50));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    	//if (!TRAJ_SUCCESS(err))
	   	//ERROUT(err);

		if (!(flags & POPCORN_CUP_HARVEST_DO_NOT_RELEASE)) {
			i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_HIDE);		
			//i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, 1000);
		}
	}

	/* XXX, at this point the work is done */
	err = END_TRAJ;


end:
    /* enable obstacle sensors */
    strat_opp_sensor_disable();

	/* end stuff */
	strat_set_speed(old_spdd, old_spda);
   	strat_limit_speed_enable();
   	return err;
}

/* release front cup */
void strat_release_popcorn_cup_front (void)
{
	/* release */
	i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_RELEASE);
	i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, 1000);

	/* hide */
	i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_HIDE);		
	//i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, 1000);
}

/** 
 *	Harvest popcorns machine
 *	return END_TRAJ if the work is done, err otherwise 
 */
uint8_t strat_harvest_popcorns_machine (int16_t x, int16_t y)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d = 0;
	int16_t x_init, y_init;
	int16_t x_init2, y_init2;

	/* save init position */
	x_init2 = position_get_x_s16(&mainboard.pos);
	y_init2 = position_get_y_s16(&mainboard.pos);

	

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_SLOW);


	/* turn to machine */
	trajectory_a_abs(&mainboard.traj, -90);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* open system */
	i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_MACHINES_READY);		
	i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, 1000);


	/* go to close to machine and  calibrate position on the wall */
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);

	//trajectory_a_abs(&mainboard.traj, 90);
	//err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	//time_wait_ms(200);

	d = position_get_y_s16(&mainboard.pos);

	err = strat_calib(-400, TRAJ_FLAGS_SMALL_DIST);
	strat_reset_pos(DO_NOT_SET_POS,
					AREA_Y-48-ROBOT_CENTER_TO_BACK,
					-90);

	/* harvest */
	i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_MACHINES_HARVEST);		
	i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, 1000);

	/* save init position */
	x_init = position_get_x_s16(&mainboard.pos);
	y_init = position_get_y_s16(&mainboard.pos);

retrya:

	/* wait popcorn inside */
	time_wait_ms(2000);

	/* XXX check OPP, wait free space or timeout */
	while (opponent1_is_infront() || opponent2_is_infront() || sensor_get(S_OPPONENT_FRONT_L) || sensor_get(S_OPPONENT_FRONT_R) || robots_are_near());
	WAIT_COND_OR_TIMEOUT((!sensor_get(S_OPPONENT_FRONT_L) && !sensor_get(S_OPPONENT_FRONT_R)), 5000);

    /* return to init position */
    d = ABS(d-position_get_y_s16(&mainboard.pos));

	if (robots_are_near())
    	strat_set_speed (SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);
	else    	
		strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

	/* XXX, is possible we push slowly to opponent */
//	trajectory_d_rel(&mainboard.traj, d);
 //   err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);


		trajectory_goto_xy_abs(&mainboard.traj, x_init2, y_init2);
	    err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		
    //err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err)) {

		trajectory_goto_xy_abs(&mainboard.traj, x_init, y_init);
	    err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		goto retrya;
    //  ERROUT(err);	
	}
	
	/* XXX, at this point the machines are done */
	err = END_TRAJ;

end:
	/* close system, TODO before ??? */
	i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_MACHINES_END);		
	i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY, 2000);

	/* end stuff */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}



/** 
 *	Release popcorns in home area
 *	return END_TRAJ if the work is done, err otherwise 
 */
uint8_t strat_release_popcorns_in_home (int16_t x, int16_t y, uint8_t flags)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d = 0;
	int16_t x_init, y_init;
	uint8_t spotlight_timeout;

	/* save init position */
	x_init = position_get_x_s16(&mainboard.pos);
	y_init = position_get_y_s16(&mainboard.pos);

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_SLOW);

	/* turn to target point */
	trajectory_turnto_xy_behind(&mainboard.traj, x, y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* be sure we are in angle */
	time_wait_ms (200);

    /* go inside to the building position */
	d = distance_from_robot(x, y);
	trajectory_d_rel(&mainboard.traj, -d);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	

	/* open rear cup */
	i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_RELEASE);
   	i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY|STATUS_BLOCKED, 1000);

	if (flags & POPCORNS_RELEASE_ONLY_CUP)
	{
		/* release in cinema */
retry1:
		/* return to init position */
		trajectory_goto_xy_abs (&mainboard.traj, x_init, y_init);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

		/* XXX, case can't reach init position, TODO: better interpretation */
		if (!TRAJ_SUCCESS(err)) 
		{
			while(opponent1_is_infront() || opponent2_is_infront());
			goto retry1;
		}
	}
	else {
		
		/* release in home */

		/* open popcorn doors */
		i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_STOCK_DROP);		
	   	i2c_slavedspic_ps_wait_status_or_timeout(STATUS_READY|STATUS_BLOCKED, 1000);

		/* wait for popcorn dump */
		time_wait_ms(1500);

		/* calculate spotlight timeout */
		if (strat_need_build_a_tower())
			spotlight_timeout = TOWER_BUILDING_TIME;
		else
			spotlight_timeout = STAND_RELEASE_TIME;

		/* if timeout, release stands */
		if(time_get_s() >= spotlight_timeout) 
			goto release_stands;

retry2:
		/* return home entrance and close gadgets in the path */	
		trajectory_goto_xy_abs (&mainboard.traj, COLOR_X(530), y_init);

		/* when enough space, close gatgets */
		err = WAIT_COND_OR_TIMEOUT(x_is_more_than(370), 1000);
		if (err)
			i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_STOCK_END);

		/* wait traj end */
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

		/* XXX, case can't reach home outside because of BLOCKING */
		if (!TRAJ_SUCCESS(err)) 
		{
			while ((opponent1_is_infront() || opponent2_is_infront()) &&
				   (time_get_s() < spotlight_timeout));

			if(time_get_s() >= spotlight_timeout) 
			{
				/* go backwards, get space */
				trajectory_goto_xy_abs (&mainboard.traj, COLOR_X(310), position_get_y_s16(&mainboard.pos));
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				//if (!TRAJ_SUCCESS(err))
				//   ERROUT(err);	
	
release_stands:
				/* XXX release stands because time is going over, NEVER RETURNS */
				err = strat_buit_and_release_spotlight (COLOR_X(555), 
														position_get_y_s16(&mainboard.pos),
														COLOR_INVERT(SIDE_LEFT), 
														STANDS_RELEASE_TIME_OVER | strat_need_build_a_tower());
			}
			else
				goto retry2;
		}
	}

end:
	/* be sure, gatgets are closed */
	i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_STOCK_END);

	/* end stuff */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}


