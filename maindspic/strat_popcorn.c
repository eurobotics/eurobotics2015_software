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

    /* prepare cup clamp */
    side == SIDE_FRONT? i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_READY):
                        i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_OPEN);

    /* wait ready */
    side == SIDE_FRONT?
    WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), POPCORN_FRONT_READY_TIMEOUT):
    WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), POPCORN_REAR_READY_TIMEOUT);


    /* turn to cup */
    side == SIDE_FRONT? trajectory_turnto_xy(&mainboard.traj, x, y):
                        trajectory_turnto_xy_behind(&mainboard.traj, x, y);
 
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* wait ready */
    side == SIDE_FRONT?
    WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), POPCORN_FRONT_READY_TIMEOUT):
    WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), POPCORN_REAR_READY_TIMEOUT);


    /* go in clamp range */
    d = distance_from_robot(x, y);
    side == SIDE_FRONT? (d = d-ROBOT_CENTER_CUP_FRONT-10) :
                        (d = -(d-ROBOT_CENTER_CUP_REAR-20));

	trajectory_d_rel(&mainboard.traj, d);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);


	/* XXX debug step use only for subtraj command */
	//state_debug_wait_key_pressed();

    /* front cup: pick up, drop popcorns in side, and release the cup */
    /* rear cup: pick up */
    side == SIDE_FRONT? i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_CATCH_AND_DROP):
                        i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_CATCH);

    /* wait ready */
    side == SIDE_FRONT?
    WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), POPCORN_FRONT_READY_TIMEOUT):
    WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), POPCORN_REAR_READY_TIMEOUT);

	/* release fornt cap and hide system */
	if (side == SIDE_FRONT) {
		time_wait_ms(1500);
		i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_RELEASE);
    	WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), 1000);

		trajectory_d_rel(&mainboard.traj, -d);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    	if (!TRAJ_SUCCESS(err))
	   	ERROUT(err);

		i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_HIDE);		
    	//WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), 1000);
	}


end:
	/* end stuff */
	strat_set_speed(old_spdd, old_spda);
   	strat_limit_speed_enable();
   	return err;
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
	WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), 1000);


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
	WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), 1000);

	/* wait popcorn inside */
	time_wait_ms(2000);

	/* XXX check OPP */
	while (opponent1_is_infront() || opponent2_is_infront());

    /* return to init position */
    d = ABS(d-position_get_y_s16(&mainboard.pos));
    strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
    trajectory_d_rel(&mainboard.traj, d);
    err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err))
       ERROUT(err);	

end:
	/* close system */
	i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_MACHINES_END);		
	WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), 2000);

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

	x_init = position_get_x_s16(&mainboard.pos);
	y_init = position_get_y_s16(&mainboard.pos);

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_SLOW);

	/* turn to home */
	//trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(0));
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
   	WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY|STATUS_BLOCKED), 1000);

	if (!(flags & POPCORN_ONLY_CUP))
	{

		/* open popcorn doors */
		i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_STOCK_DROP);		
	   	WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY|STATUS_BLOCKED), 1000);

		/* wait for popcorn dump */
		time_wait_ms(1500);

retry2:
		/* return to init position and close gadgets in the path */
		//trajectory_d_rel(&mainboard.traj, d);
		//time_wait_ms(500);
		//i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_STOCK_END);
		//err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		//if (!TRAJ_SUCCESS(err))
		//   ERROUT(err);	

		trajectory_goto_xy_abs (&mainboard.traj, x_init, y_init);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err)) {
		   while (opponent1_is_infront() || opponent2_is_infront());
		   goto retry2;
		   //ERROUT(err);	
		}

		i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_STOCK_END);
	   	WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY|STATUS_BLOCKED), 1000);
	}
	else {

retry:
		/* go far cup */
		//trajectory_d_rel(&mainboard.traj, d);
		trajectory_goto_xy_abs (&mainboard.traj, x_init, y_init);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err)) {
		   while (opponent1_is_infront() || opponent2_is_infront());
		   goto retry;
		   //ERROUT(err);	
		}
	}

end:
	/* end stuff */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}

#if 0
#define TIME_TO_RELEASE_2_TOWERS
			if (slavedispic.stands_system[COLOR_INVERT(LEFT)].stored_stands >= 4) 
			{
				if (time_get_s() > 90-TIME_TO_RELEASE_2_TOWERS)
					flags = STANDS_RELEASE_TIME_OVER;
				else
					flags = 0;

				err = strat_buit_and_release_spotlight (COLOR_X(strat_infos.zones[zone_num].x),
														strat_infos.zones[zone_num].y,
														COLOR_INVERT(SIDE_LEFT), flags);
			}
			else {

				if (time_get_s() > 90-TIME_TO_BUILDING_TOWER)
					flags = STANDS_RELEASE_DO_TOWER | STANDS_RELEASE_TIME_OVER;
				else
					flags = STANDS_RELEASE_DO_TOWER;

				err = strat_buit_and_release_spotlight (COLOR_X(strat_infos.zones[zone_num].x),
														strat_infos.zones[zone_num].y,
														COLOR_INVERT(SIDE_LEFT), flags);
			}

			if (slavedispic.stands_system[COLOR_INVERT(LEFT)].stored_stands < 4 ||
				(strat.conf.flags & CONF_FLAG_DO_TOWER)) 
			{
				err = strat_buit_and_release_spotlight (COLOR_X(strat_infos.zones[zone_num].x),
														strat_infos.zones[zone_num].y,
														COLOR_INVERT(SIDE_LEFT), STAND_RELEASE_DO_TOWER);
			}
			else {
				err = strat_buit_and_release_spotlight (COLOR_X(strat_infos.zones[zone_num].x),
														strat_infos.zones[zone_num].y,
														COLOR_INVERT(SIDE_LEFT), 0);
			}
#endif


