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

#define BT_TASK_WAIT_COND_OR_TIMEOUT(cond, timeout)                   \
({                                                            \
        microseconds __us = time_get_us2();                   \
        uint8_t __ret = 1;                                    \
        while(! (cond && !mainboard.bt_task_interrupt )) {      \
                if (time_get_us2() - __us > (timeout)*1000L) {\
                        __ret = 0;                            \
                        break;                                \
                }                                             \
        }    \
	if (mainboard.bt_task_interrupt)        \
		ERROUT(END_INTR); \
	else if (__ret)					      \
		DEBUG(E_USER_STRAT, "bt_task: cond is true at line %d",\
		      __LINE__);			      \
	else						      \
		DEBUG(E_USER_STRAT, "bt_task: timeout at line %d",     \
		      __LINE__);			      \
							      \
        __ret;                                                \
})

#define strat_bt_task_wait_ms(time)		BT_TASK_WAIT_COND_OR_TIMEOUT(mainboard.bt_task_interrupt, time);



/* auto possition depending on color */
void strat_auto_position (void)
{
#define TRESPA_TRIANGLE		 -6	/* XXX align with plastic rollons */
#define TRESPA_BAR			 17
#define HOME_X_EDGE			 70
#define ROBOT_ENCODERS_WIDTH 208

#define HOME_Y_DOWN_EDGE_YELLOW  800
#define HOME_Y_DOWN_EDGE_GREEN   800

	mainboard.our_color == I2C_COLOR_YELLOW?
	strat_reset_pos(COLOR_X(HOME_X_EDGE+TRESPA_TRIANGLE+ROBOT_CENTER_TO_BACK),
					HOME_Y_DOWN_EDGE_YELLOW+TRESPA_BAR+(ROBOT_ENCODERS_WIDTH/2.0),
					COLOR_A_ABS(0)):
	strat_reset_pos(COLOR_X(HOME_X_EDGE+TRESPA_TRIANGLE+ROBOT_CENTER_TO_BACK),
					HOME_Y_DOWN_EDGE_GREEN+TRESPA_BAR+(ROBOT_ENCODERS_WIDTH/2.0),
					COLOR_A_ABS(0));
}


/**
 *	Pickup a popcorn cup
 *	return END_TRAJ if the work is done, err otherwise
 */
uint8_t strat_pickup_cup (int16_t x, int16_t y, uint8_t side)
{
#define CUP_DIAMETER	94
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d=0;
    microseconds us = 0;
    uint8_t old_debug = strat_infos.debug_step;

    /* XXX debug */
    strat_infos.debug_step = 0;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

	/* open clamp */
	side == BT_SIDE_FRONT? cup_front_clamp_set_position (CUP_FRONT_CLAMP_POS_OPEN):
						   cup_rear_clamp_set_position (CUP_REAR_CLAMP_POS_OPEN);
	us = time_get_us2();

	/* turn to cup */
	side == BT_SIDE_FRONT? trajectory_turnto_xy(&mainboard.traj, x, y):
						   trajectory_turnto_xy_behind(&mainboard.traj, x, y);

 	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* wait clamp is opened */
	while ((uint32_t)(time_get_us2()-us) < 500000L);


	/* go forward in clamp range */
	d = distance_from_robot(x, y);

	side == BT_SIDE_FRONT?	trajectory_d_rel(&mainboard.traj, +(d-ROBOT_CENTER_TO_FRONT-(CUP_DIAMETER/2))):
							trajectory_d_rel(&mainboard.traj, -(d-ROBOT_CENTER_TO_BACK-(CUP_DIAMETER/2)));

 	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* XXX debug */
    state_debug_wait_key_pressed(); 

end:
	/* close clamp */
	side == BT_SIDE_FRONT? cup_front_clamp_set_position (CUP_FRONT_CLAMP_POS_CLOSE):
						   cup_rear_clamp_set_position (CUP_REAR_CLAMP_POS_CLOSE);
	time_wait_ms(500);

    /* XXX debug */
    strat_infos.debug_step = old_debug;

	/* end stuff */
	strat_set_speed(old_spdd, old_spda);
   	strat_limit_speed_enable();
   	return err;
}

/**
 *	Release a popcorn cup
 *	return END_TRAJ if the work is done, err otherwise
 */
uint8_t strat_release_cup (int16_t x, int16_t y, uint8_t side)
{
#define CUP_DIAMETER	94
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d;
    uint8_t old_debug = strat_infos.debug_step;

    /* XXX debug */
    strat_infos.debug_step = 0;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

	/* turn to release point */
	side == BT_SIDE_FRONT? trajectory_turnto_xy(&mainboard.traj, x, y):
                           trajectory_turnto_xy_behind(&mainboard.traj, x, y);

 	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* go to point */
	d = distance_from_robot(x, y);
	d -= (side == BT_SIDE_FRONT? (ROBOT_CENTER_TO_FRONT-(CUP_DIAMETER/2)):
                                 (ROBOT_CENTER_TO_BACK-(CUP_DIAMETER/2)));

	side == BT_SIDE_FRONT? trajectory_d_rel(&mainboard.traj, d):
						   trajectory_d_rel(&mainboard.traj, -d);

 	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* XXX debug */
    state_debug_wait_key_pressed(); 

	/* open clamp */
	side == BT_SIDE_FRONT? cup_front_clamp_set_position (CUP_FRONT_CLAMP_POS_OPEN):
                           cup_rear_clamp_set_position (CUP_REAR_CLAMP_POS_OPEN);
	time_wait_ms(2000);

    /* TODO: wait if opponent behind/front?? */

	/* return to init point */ 
	side == BT_SIDE_FRONT?  trajectory_d_rel(&mainboard.traj, -(d+CUP_DIAMETER)):
                            trajectory_d_rel(&mainboard.traj, +(d+CUP_DIAMETER));

 	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

end:
	/* close clamp */
	side == BT_SIDE_FRONT? cup_front_clamp_set_position (CUP_FRONT_CLAMP_POS_CLOSE):
                           cup_rear_clamp_set_position (CUP_REAR_CLAMP_POS_CLOSE);

    /* XXX debug */
    strat_infos.debug_step = old_debug;

	/* end stuff */
	strat_set_speed(old_spdd, old_spda);
   	strat_limit_speed_enable();
   	return err;
}

/**
 *	Put carpets on stairs
 *	return END_TRAJ if the work is done, err otherwise
 */
uint8_t strat_put_carpets (void)
{

/* XXX keep synchronized with main robot */
#define STAIRS_EDGE_Y			(AREA_Y-600) //(AREA_Y-580)
#define STAIRS_EDGE_X			(967)
#define STAIRS_WIDE_HALF		(50)
#define CARPET_LEFT_INFRONT_X	(STAIRS_EDGE_X+STAIRS_WIDE_HALF+ROBOT_CENTER_TO_ARM+20)
#define CARPET_RIGHT_INFRONT_X	((AREA_X/2)-STAIRS_WIDE_HALF-ROBOT_CENTER_TO_ARM-20)

   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d;
    uint8_t old_debug = strat_infos.debug_step;
    static uint8_t first_carpet_done = 0;

    /* XXX debug */
    strat_infos.debug_step = 0;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

	/* turn to first stairs behind */
	trajectory_a_abs(&mainboard.traj, -90);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* XXX debug */
    state_debug_wait_key_pressed(); 

	/* go backwards close to stairs */
	d = distance_from_robot(position_get_x_s16(&mainboard.pos), STAIRS_EDGE_Y);
	trajectory_d_rel(&mainboard.traj, -(d-OBS_CLERANCE));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	

    /* XXX debug */
    state_debug_wait_key_pressed(); 

    /************* first carpet */
    if (!first_carpet_done) {

	    /* turn to left side */
	    trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(0));
	    err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	    if (!TRAJ_SUCCESS(err))
	       ERROUT(err);

        /* XXX debug */
        state_debug_wait_key_pressed(); 

	    /* put carpet */
        arm_set_mode (COLOR_INVERT(ARM_TYPE_LEFT), ARM_MODE_CARPET);
        time_wait_ms(500);
        arm_set_mode (COLOR_INVERT(ARM_TYPE_LEFT), ARM_MODE_HIDE);

        /* mark first carpet as done */
        first_carpet_done = 1;

    }

    /************** second carpet */

	/* turn to right side */
	trajectory_a_rel(&mainboard.traj, COLOR_A_REL(180));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* turn to right side */
	trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(180));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);



    /* XXX debug */
    state_debug_wait_key_pressed(); 

	/* go infront of second stairs */
	d = distance_from_robot(COLOR_X(CARPET_RIGHT_INFRONT_X), position_get_y_s16(&mainboard.pos));
	trajectory_d_rel(&mainboard.traj, -d);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	
	
    //Case: END_OBSTACLE on last movement for second carpet
    //Return error only if opponent is in front of our stairs
    if (!TRAJ_SUCCESS(err))
    {
       if(err==END_OBSTACLE)
       {
       		if(opponents_are_in_area(COLOR_X(1648),STAIRS_EDGE_Y, COLOR_X(967),1000))
	   			ERROUT(err);

			/* go infront of second stairs */
			d = distance_from_robot(COLOR_X(CARPET_RIGHT_INFRONT_X), position_get_y_s16(&mainboard.pos));
			trajectory_d_rel(&mainboard.traj, -d);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
       }
       else
	   	ERROUT(err);
	}

    /* XXX debug */
    state_debug_wait_key_pressed(); 	

	/* put carpet */
	arm_set_mode (COLOR_INVERT(ARM_TYPE_RIGHT), ARM_MODE_CARPET);
	time_wait_ms(500);
	arm_set_mode (COLOR_INVERT(ARM_TYPE_RIGHT), ARM_MODE_HIDE);

end:
	/* end stuff */
	arm_set_mode (ARM_TYPE_LEFT, ARM_MODE_HIDE);
	arm_set_mode (ARM_TYPE_RIGHT, ARM_MODE_HIDE);

    /* XXX debug */
    strat_infos.debug_step = old_debug;

	/* end stuff */
	strat_set_speed(old_spdd, old_spda);
   	strat_limit_speed_enable();
   	return err;
}



/**
 *	Put carpets on stairs
 *	return END_TRAJ if the work is done, err otherwise
 */
uint8_t strat_close_clapperboard (int16_t x, int16_t y)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d, a;
    uint8_t arm;
    uint8_t old_debug = strat_infos.debug_step;
//    uint8_t calib_tries = 2;

    /* XXX debug */
    strat_infos.debug_step = 0;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);


	/* turn to clapperboard behind */
	trajectory_a_abs(&mainboard.traj, 90);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

#if 1
	/* go to clean distance */
	d = distance_from_robot(position_get_x_s16(&mainboard.pos), 0);
	trajectory_d_rel(&mainboard.traj, -(d-CLEAN_CLERANCE-30));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	

	/* arm down */
	arm = (position_get_x_s16(&mainboard.pos) > (AREA_X/2)? ARM_TYPE_RIGHT : ARM_TYPE_LEFT);	
	arm_set_mode (arm, ARM_MODE_CLEAN);
	time_wait_ms(500);

	/* turn 180 degrees and back, for clean */
	a = (position_get_x_s16(&mainboard.pos) > (AREA_X/2)? -180 : 180);
	trajectory_a_rel(&mainboard.traj, a);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	trajectory_a_rel(&mainboard.traj, -a);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	/* turn to clapperboard behind */
	trajectory_a_abs(&mainboard.traj, 90);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);
#endif

	/* go backwards to near wall */
	d = distance_from_robot(position_get_x_s16(&mainboard.pos), 0);
	trajectory_d_rel(&mainboard.traj, -(d-OBS_CLERANCE));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	

	/* open arm */
	arm = (position_get_x_s16(&mainboard.pos) > (AREA_X/2)? ARM_TYPE_RIGHT : ARM_TYPE_LEFT);	
	arm_set_mode (arm, ARM_MODE_CLAPPER);
	time_wait_ms(500);

	/* turn 90 degrees */
	a = (position_get_x_s16(&mainboard.pos) > (AREA_X/2)? 0 : 180);
	trajectory_a_abs(&mainboard.traj, a);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

end:
	/* close arm */
	arm_set_mode (ARM_TYPE_LEFT, ARM_MODE_HIDE);
	arm_set_mode (ARM_TYPE_RIGHT, ARM_MODE_HIDE);

    /* XXX debug */
    strat_infos.debug_step = old_debug;

	/* end stuff */
	strat_set_speed(old_spdd, old_spda);
   	strat_limit_speed_enable();
   	return err;
}

/* TODO */
uint8_t climb_stairs(void)
{
	uint8_t err=0;
    printf_P(PSTR("climb_stairs\r\n"));

	/* TODO */
	strat_bt_task_wait_ms(3000);
	err = END_TRAJ;

end:
    return err;
}


#if 0

	/* TODO: calib y */
	/* calibrate position on the wall */
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_SLOW);
	time_wait_ms(200);

calib:

	/* go to blocking */
	err = strat_calib(-100, TRAJ_FLAGS_SMALL_DIST);

	time_wait_ms(200);
	bd_reset(&mainboard.distance.bd);

	printf ("diff = %d", ABS(position_get_y_s16(&mainboard.pos)-(int16_t)ROBOT_CENTER_TO_BACK));

#define CALIB_D_OK 30
	if (ABS(position_get_y_s16(&mainboard.pos)-ROBOT_CENTER_TO_BACK) < CALIB_D_OK)
	{
		strat_reset_pos(DO_NOT_SET_POS,
                        ROBOT_CENTER_TO_BACK,
						90);

		bd_reset(&mainboard.distance.bd);

	    /* go forward a bit */
	    trajectory_d_rel(&mainboard.traj, OBS_CLERANCE);
	    err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
        if (!TRAJ_SUCCESS(err))
	       ERROUT(err);

	}
	else if (calib_tries) 
    { 
        calib_tries--;

		bd_reset(&mainboard.distance.bd);

	    /* go backwards a bit */
	    trajectory_d_rel(&mainboard.traj, -(OBS_CLERANCE-CALIB_D_OK));
	    err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
        if (!TRAJ_SUCCESS(err))
	       ERROUT(err);	

    
        /* turn 90 degrees, in order to clean space */
        a = (position_get_x_s16(&mainboard.pos) > (AREA_X/2)? 0 : 180);
        trajectory_a_abs(&mainboard.traj, a);
        err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
        if (!TRAJ_SUCCESS(err))
           ERROUT(err);

        /* turn to clapperboard behind */
        trajectory_a_abs(&mainboard.traj, 90);
        err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
        if (!TRAJ_SUCCESS(err))
           ERROUT(err);

        goto calib;
    }
	else {
	    /* go backwards a bit */
	    trajectory_d_rel(&mainboard.traj, -(OBS_CLERANCE-CALIB_D_OK));
	    err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
        if (!TRAJ_SUCCESS(err))
	       ERROUT(err);	
	}

#endif



