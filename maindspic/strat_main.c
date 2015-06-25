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
 *  Javier Bali�as Santos <javier@arc-robots.org> and Silvia Santano
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
#include "bt_protocol.h"


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)



/* Add here the main strategy, the intelligence of robot */

/* debug step to step */
void state_debug_wait_key_pressed (void)
{
	if (!strat_infos.debug_step)
		return;

	DEBUG(E_USER_STRAT,"press a key");
	while(!cmdline_keypressed());
}

/* debug strat step to step */
void strat_debug_wait_key_pressed (uint8_t robot)
{
	if (!strat_infos.debug_step)
		return;

	DEBUG(E_USER_STRAT,"%s, press a key",
			robot==MAIN_ROBOT? "R1":"R2");

	while(!cmdline_keypressed());
}

/* debug strat step to step */
#if 0
uint8_t strat_debug_is_key_pressed (uint8_t robot)
{
	if (!strat_infos.debug_step)
		return 1;

	DEBUG(E_USER_STRAT,"%s, press a key",
			robot==MAIN_ROBOT? "R1":"R2");

	return cmdline_keypressed();
}
#endif

static uint8_t strat_secondary_robot_on = 0;

void strat_secondary_robot_enable (void)
{
	uint8_t flags;

	IRQ_LOCK(flags);
	strat_secondary_robot_on = 1;
	IRQ_UNLOCK(flags);
}

void strat_secondary_robot_disable (void)
{
	uint8_t flags;

	IRQ_LOCK(flags);
	strat_secondary_robot_on = 0;
	IRQ_UNLOCK(flags);
}

uint8_t strat_secondary_robot_is_enabled (void)
{
	return strat_secondary_robot_on;
}
/* return 1 if is a valid zone, 0 if not */
uint8_t strat_is_valid_zone(uint8_t robot, int8_t zone_num)
{
	/* return if zone_num out of range */
	if(zone_num < 0 || zone_num >= ZONES_MAX){
		ERROR (E_USER_STRAT, "ERROR, zone_num out of range");
		return 0;
	}

	/* discard priority 0 */
	if((strat_infos.zones[zone_num].prio == 0) && (zone_num != ZONE_CUP_NEAR_STAIRS)) {
		/*if (robot == MAIN_ROBOT)
			DEBUG(E_USER_STRAT,"Not valid zone, prio=0 in zone: %s",get_zone_name(zone_num));*/
		return 0;
	}

	/* XXX discard current zone */
	if(strat_smart[robot].current_zone == zone_num &&
	   strat_smart[robot].current_zone != ZONE_MY_STAIRWAY)
	{
		/*if (robot == MAIN_ROBOT)
			DEBUG(E_USER_STRAT,"Not valid zone, zone num: %s is current_zone",get_zone_name(zone_num));*/
		return 0;
	}

	/* discard robot zone */
	if(strat_infos.zones[zone_num].robot != robot) {
		/*if (robot == MAIN_ROBOT)
			DEBUG(E_USER_STRAT,"Not valid zone, not valid robot in zone: %s",get_zone_name(zone_num));*/
		return 0;
	}

	/* discard checked zone */
	if(strat_infos.zones[zone_num].flags & ZONE_CHECKED) {
		/*if (robot == MAIN_ROBOT)
			DEBUG(E_USER_STRAT,"Not valid zone, zone num: %s. is checked.", get_zone_name(zone_num));*/
			return 0;
	}

	/* discard avoid zone */
	if(strat_infos.zones[zone_num].flags & ZONE_AVOID) {
		/*if (robot == MAIN_ROBOT)
			DEBUG(E_USER_STRAT,"Not valid zone, zone num: %s. is avoid.", get_zone_name(zone_num));*/
		return 0;
	}

	/* discard one cinema if the other is checked */
	/* Only for sec robot. Main robot may also go to cinema. */
	if(robot == SEC_ROBOT)
	{
		if((zone_num == ZONE_MY_CINEMA_DOWN_SEC) && (strat_infos.zones[ZONE_MY_CINEMA_UP].flags & ZONE_CHECKED)) {
			return 0;
		}
		if((zone_num == ZONE_MY_CINEMA_UP) && (strat_infos.zones[ZONE_MY_CINEMA_DOWN_SEC].flags & ZONE_CHECKED)) {
			return 0;
		}
	}

	return 1;
}


/* return 1 if opponent is in a zone area, 0 if not */
uint8_t strat_is_opp_in_zone (uint8_t zone_num)
{
	//int16_t opp1_x, opp1_y, opp2_x, opp2_y;
	uint8_t ret = 0;

	//get_opponent1_xy(&opp1_x, &opp1_y);
	//get_opponent2_xy(&opp2_x, &opp2_y);

	//DEBUG (E_USER_STRAT, "area x_up=%d, x_down=%d, y_up=%d, y_down=%d", 
	//						COLOR_X(strat_infos.zones[zone_num].x_up), COLOR_X(strat_infos.zones[zone_num].x_down),
 	//						strat_infos.zones[zone_num].y_up, strat_infos.zones[zone_num].y_down);

	/* check if opponent is in zone area */
	ret = opponents_are_in_area(COLOR_X(strat_infos.zones[zone_num].x_up), 	strat_infos.zones[zone_num].y_up,
							 	 COLOR_X(strat_infos.zones[zone_num].x_down),	strat_infos.zones[zone_num].y_down);


	//DEBUG (E_USER_STRAT, "ret = %d", ret);

	return ret;
}

/* return new work zone, STRAT_NO_MORE_ZONES or STRAT_OPP_IS_IN_ZONE */
#define STRAT_NO_MORE_ZONES		-1
#define STRAT_OPP_IS_IN_ZONE	-2
int8_t strat_get_new_zone(uint8_t robot)
{
	uint8_t prio_max = 0;
	int8_t zone_num = STRAT_NO_MORE_ZONES;
	int8_t i=0;

	/* 1. get the valid zone with the maximun priority  */
	for(i=0; i < ZONES_MAX; i++)
	{
		if (strat_is_valid_zone(robot, i) &&
			strat_infos.zones[i].prio >= prio_max)
		{
			prio_max = strat_infos.zones[i].prio;
			zone_num = i;
		}
	}


	/* For secondary robot: check if need to synchronize */
	if(robot==SEC_ROBOT)
	{

		/*if (strat_smart[SEC_ROBOT].current_zone==ZONE_BLOCK_UPPER_SIDE)
		{
			//Free upper zone if it was still blocking
			//zone_num = ZONE_FREE_UPPER_SIDE;
			zone_num = ZONE_MY_STAIRWAY;
			DEBUG(E_USER_STRAT,"R2, going to ZONE_MY_STAIRWAY.");
		}*/

		if(strat_smart_get_msg()==MSG_UPPER_SIDE_FREE)
		{
			DEBUG(E_USER_STRAT,"R2, ZONE_FREE_UPPER_SIDE is FREE.");
			strat_smart_set_msg(MSG_UPPER_SIDE_IS_FREE);
			strat_infos.zones[ZONE_BLOCK_UPPER_SIDE].flags |= ZONE_AVOID;
			zone_num = ZONE_MY_STAIRWAY;
		}
	}

	//DEBUG (E_USER_STRAT, "zone candidate is %s", get_zone_name(zone_num));

	/* 2. check if the maximun priority zone is free */
	if(zone_num != STRAT_NO_MORE_ZONES)
	{
		if (strat_is_opp_in_zone(zone_num)) 
		{
			DEBUG (E_USER_STRAT, "WARNING opp is in zone candidate");
			//if (zone_num != ZONE_BLOCK_UPPER_SIDE)
				zone_num = STRAT_OPP_IS_IN_ZONE;
		}
	}

	if (robot == MAIN_ROBOT)
		strat_debug_wait_key_pressed (MAIN_ROBOT);

	return zone_num;
}

/**
 *  main robot: return END_TRAJ if zone is reached or no where to go, err otherwise
 *  secondary robot: return 0 if command SUCESSED, END_TRAJ no where to go, err otherwise
 */
uint8_t strat_goto_zone(uint8_t robot, uint8_t zone_num)
{
	int8_t err=0;

	/* update strat_infos */
	strat_smart[robot].goto_zone = zone_num;


	/* return if NULL  xy */
	if (strat_infos.zones[zone_num].init_x == INIT_NULL &&
		strat_infos.zones[zone_num].init_y == INIT_NULL) {
		WARNING (E_USER_STRAT, "WARNING, No where to GO (xy is NULL)");
		ERROUT(END_TRAJ);
	}

	/* XXX secondary robot: goto and return */
	if(robot==SEC_ROBOT) {


		/* specific zones */
		if (zone_num == ZONE_MY_HOME_OUTSIDE) {
			bt_robot_2nd_goto_xy_abs(COLOR_X(strat_infos.zones[zone_num].init_x),
									strat_infos.zones[zone_num].init_y);
		}
		/* normaly we go with avoid */
        else if (zone_num == ZONE_MY_STAIRWAY || zone_num == ZONE_MY_CLAP_3 ||
                 zone_num == ZONE_MY_CINEMA_UP ||  zone_num == ZONE_MY_CINEMA_DOWN_SEC)
        {
			DEBUG (E_USER_STRAT, "going backwards");

			/* force go backwards */
			bt_robot_2nd_goto_and_avoid_backward(COLOR_X(strat_infos.zones[zone_num].init_x),
										strat_infos.zones[zone_num].init_y);
        }
        else if (zone_num == ZONE_CUP_MIDDLE)
        {

			/* force go backwards */
			bt_robot_2nd_goto_and_avoid_forward(COLOR_X(strat_infos.zones[zone_num].init_x),
										strat_infos.zones[zone_num].init_y);
        }

		else {
			bt_robot_2nd_goto_and_avoid(COLOR_X(strat_infos.zones[zone_num].init_x),
										strat_infos.zones[zone_num].init_y);
		}

		return 0;
	}


	/* XXX main robot: goto and wait */
#if 0
	DEBUG (E_USER_STRAT, "R1 num zone %s", get_zone_name(zone_num));
	DEBUG(E_USER_STRAT, "R1 last zone %s", get_zone_name(strat_smart[robot].last_zone));
	DEBUG(E_USER_STRAT, "R1 current zone %s", get_zone_name(strat_smart[robot].current_zone));
	DEBUG(E_USER_STRAT, "R1 goto zone %s", get_zone_name(strat_smart[robot].goto_zone));
#endif

    /* TODO TEST: enable obstacle sensors */
    strat_opp_sensor_enable();
	//strat_opp_sensor_middle_enable();
    

	if (strat_smart[robot].current_zone == ZONE_MY_HOME_POPCORNS){

		/* we are at init position */
		err = END_TRAJ;
	}
	else if (strat_smart[robot].current_zone == ZONE_MY_STAND_GROUP_3) /* ||
        	 strat_smart[robot].current_zone == ZONE_MY_POPCORNMAC ||
        	 strat_smart[robot].current_zone == ZONE_MY_STAND_GROUP_4)*/
    {

        /* first try */
		err = goto_and_avoid (COLOR_X(strat_infos.zones[zone_num].init_x),
										strat_infos.zones[zone_num].init_y,
										TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
        /* if not sucessed */
        if (!TRAJ_SUCCESS(err) && (strat_infos.conf.flags & CONF_FLAG_DO_ESCAPE_UPPER_ZONE)) {
			//time_wait_ms (5000);

			/* second try */
			//err = goto_and_avoid (COLOR_X(strat_infos.zones[zone_num].init_x),
			//					strat_infos.zones[zone_num].init_y,
			//					TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

			/* XXX, escape if not sucessed */
            /* XXX, returns only when we are out upper zone */
			//if (!TRAJ_SUCCESS(err))
            	err = strat_escape_form_upper_zone (0);


            /* go init position */
            err = goto_and_avoid (COLOR_X(strat_infos.zones[zone_num].init_x),
								            strat_infos.zones[zone_num].init_y,
								            TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

		}
	}
	else if (zone_num == ZONE_POPCORNCUP_2)
	{
#define X_UP    870
#define X_DOWN  0
#define Y_UP    750    
#define Y_DOWN  0

        /* TODO TEST: if opp is not in down area, skip goto, work directly */
        if (!opponents_are_in_area(COLOR_X(X_UP), Y_UP, COLOR_X(X_DOWN), Y_DOWN) )
        {
            err = END_TRAJ;
        }
        else 
        { 
            /* prepare clamp */
            i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_OPEN);


		    /* by default go with avoidance */
		    err = goto_and_avoid (COLOR_X(strat_infos.zones[zone_num].init_x),
										    strat_infos.zones[zone_num].init_y,
										    TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

            /* close clamp if traj not successed */
            if (!TRAJ_SUCCESS(err)) {
                i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_STOCK_END);

            }
        }
	}
	else if (zone_num == ZONE_MY_CLAP_1 && strat_smart[robot].current_zone == ZONE_MY_STAND_GROUP_2)
	{
        /* TODO: go without avoid */
        trajectory_goto_xy_abs (&mainboard.traj,
                                COLOR_X(strat_infos.zones[zone_num].init_x),
								strat_infos.zones[zone_num].init_y);

        err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    }
	else 
	{
		/* by default go with avoidance */
		err = goto_and_avoid (COLOR_X(strat_infos.zones[zone_num].init_x),
										strat_infos.zones[zone_num].init_y,
										TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	}

end:
	/* update strat_infos */
	strat_smart[robot].last_zone = strat_smart[robot].current_zone;
	strat_smart[robot].goto_zone = -1;

	if (!TRAJ_SUCCESS(err))
		strat_smart[robot].current_zone = -1;
	else
		strat_smart[robot].current_zone=zone_num;


    /* TODO: disable obstacle sensors */
    strat_opp_sensor_disable();    
	strat_opp_sensor_middle_disable();
	return err;
}


/**
 *  main robot: return END_TRAJ if work is done or no wher to work, err otherwise
 *  secondary robot: return 0 if command SUCESSED, END_TRAJ no where to work, err otherwise
 */
uint8_t strat_work_on_zone(uint8_t robot, uint8_t zone_num)
{
	uint8_t err = END_TRAJ;

	/* TODO: return if -1000 xy */
	if (strat_infos.zones[zone_num].x == 0 &&
		strat_infos.zones[zone_num].y == 0) {
		WARNING (E_USER_STRAT, "%s, WARNING, No where to WORK (xy is NULL)",
				 robot == MAIN_ROBOT? "R1":"R2");
		ERROUT(END_TRAJ);
	}

	/* XXX secondary_robot: send work on zone bt task and return */
	if(strat_infos.zones[zone_num].robot == SEC_ROBOT)
	{
		switch (zone_num)
		{
			case ZONE_POPCORNCUP_1:
				bt_robot_2nd_bt_task_pick_cup (COLOR_X(strat_infos.zones[zone_num].x),
											   strat_infos.zones[zone_num].y, BT_SIDE_FRONT);
				break;

			case ZONE_MY_CLAP_3:
				bt_robot_2nd_bt_task_clapperboard(COLOR_X(strat_infos.zones[zone_num].x),
											   strat_infos.zones[zone_num].y);
				break;

			case ZONE_MY_CINEMA_DOWN_SEC:
				bt_robot_2nd_bt_task_bring_cup_cinema(COLOR_X(strat_infos.zones[zone_num].x),
											   strat_infos.zones[zone_num].y, BT_SIDE_FRONT);
				break;

			case ZONE_MY_CINEMA_UP:
				bt_robot_2nd_bt_task_bring_cup_cinema(COLOR_X(strat_infos.zones[zone_num].x),
											   strat_infos.zones[zone_num].y, BT_SIDE_FRONT);
				break;

			case ZONE_MY_STAIRWAY:
				bt_robot_2nd_bt_task_carpet();
				break;

			case ZONE_MY_STAIRS:
				bt_robot_2nd_bt_task_stairs();
				break;

			case ZONE_CUP_NEAR_STAIRS:
				bt_robot_2nd_bt_task_bring_cup_cinema(COLOR_X(strat_infos.zones[zone_num].x),
											   strat_infos.zones[zone_num].y, BT_SIDE_FRONT);
				break;

			default:
				ERROR (E_USER_STRAT, "R2, ERROR zone %d not supported", zone_num);
				ERROUT(END_ERROR);

		}

		/* return SUCCESS */
		ERROUT(0);
	}

	/* main robot, dependin on zone */
	else
	{
		switch (zone_num)
		{
			case ZONE_MY_STAND_GROUP_1:

				/* Set start to sec robot */
				strat_smart[MAIN_ROBOT].current_zone = ZONE_MY_STAND_GROUP_1;

            /* fast harvesting of stand 4, 5 and cup 3 */
            if (strat_infos.conf.flags & CONF_FLAG_DO_STAND_FAST_GROUP_1) {

				DEBUG(E_USER_STRAT,"R1, sending message START.");
				strat_smart_set_msg(MSG_START);

                err = strat_harvest_stands_and_cup_inline();
                strat_infos.conf.flags &= ~(CONF_FLAG_DO_STAND_FAST_GROUP_1);
            }


            /* stand 4 */
			if (!(strat_infos.done_flags & DONE_STAND_4)) {
			    err = strat_harvest_orphan_stands (COLOR_X(MY_STAND_4_X),
											       MY_STAND_4_Y,
											       COLOR_INVERT(SIDE_RIGHT),
								     			   COLOR_INVERT(SIDE_RIGHT),
											       0,
											       SPEED_DIST_SLOW, /* harvest speed */
											       0);				/* flags */
			    /* continue with the next stand */
			    //if (!TRAJ_SUCCESS(err))
			    //   ERROUT(err);

                /* mark stand as harvested */
                strat_infos.done_flags |= DONE_STAND_4;
            }

			if (!(strat_infos.conf.flags & CONF_FLAG_DO_STAND_FAST_GROUP_1)) {
				DEBUG(E_USER_STRAT,"R1, sending message START.");
				strat_smart_set_msg(MSG_START);
			}


			/* XXX debug step use only for subtraj command */
			//strat_debug_wait_key_pressed (MAIN_ROBOT);

            /* stand 5 */
			if (!(strat_infos.done_flags & DONE_STAND_5)) {
			    err = strat_harvest_orphan_stands (COLOR_X(MY_STAND_5_X),
											       MY_STAND_5_Y,
											       COLOR_INVERT(SIDE_LEFT),
								     			   COLOR_INVERT(SIDE_LEFT),
											       0,
											       SPEED_DIST_SLOW, /* harvest speed */
											       0);				/* flags */

			    /* continue with the next stand */
		        //if (!TRAJ_SUCCESS(err))
			    //   ERROUT(err);

                /* mark stand as harvested */
                strat_infos.done_flags |= DONE_STAND_5;

            }


			/* XXX debug step use only for subtraj command */
			//strat_debug_wait_key_pressed (MAIN_ROBOT);

				/* POPCORNCUP_3 */

			
            /* XXX HACK: mark cup as harvested */
            //strat_infos.done_flags |= DONE_CUP_3;

			if (!(strat_infos.done_flags & DONE_CUP_3)) {
			    err = strat_harvest_popcorn_cup (COLOR_X(strat_infos.zones[ZONE_POPCORNCUP_3].x),
									       strat_infos.zones[ZONE_POPCORNCUP_3].y,
										   SIDE_FRONT, 0);

                /* mark cup as harvested */
                strat_infos.done_flags |= DONE_CUP_3;

            }

		    /* XXX debug step use only for subtraj command */
		    //strat_debug_wait_key_pressed (MAIN_ROBOT);

            /* stand 6 */
		    err = strat_harvest_orphan_stands (COLOR_X(MY_STAND_6_X),
										       MY_STAND_6_Y,
										       COLOR_INVERT(SIDE_LEFT),
							     			   COLOR_INVERT(SIDE_LEFT),
										       0,
										       SPEED_DIST_SLOW, /* harvest speed */
										       0);				/* flags */

			break;


			case ZONE_MY_STAND_GROUP_2:
			err = strat_harvest_orphan_stands (COLOR_X(strat_infos.zones[zone_num].x),
											   strat_infos.zones[zone_num].y,
											   COLOR_INVERT(SIDE_LEFT),         /* side target */
								 			   SIDE_ALL,                        /* storing sides */
											   COLOR_A_REL(-10),                /* blade angle */
											   SPEED_DIST_SLOW,                 /* harvest speed */
											   0);
			break;

		case ZONE_MY_STAND_GROUP_3:

	#if 0
			/* only one stand */
			err = strat_harvest_orphan_stands (COLOR_X(MY_STAND_3_X),
											   MY_STAND_3_Y,
											   COLOR_INVERT(SIDE_LEFT),         /* side target */
								 			   COLOR_INVERT(SIDE_LEFT),        /* storing sides */
											   COLOR_A_REL(0),                /* blade angle */
											   SPEED_DIST_SLOW,            /* harvest speed */
											   STANDS_HARVEST_BACK_INIT_POS);	/* flags */
#else
			/* two stands */
			err = strat_harvest_orphan_stands (COLOR_X(strat_infos.zones[zone_num].x),
											   strat_infos.zones[zone_num].y,
											   SIDE_ALL,         				/* side target */
								 			   SIDE_ALL,        				/* storing sides */
											   COLOR_A_REL(0),                	/* blade angle */
											   SPEED_DIST_SLOW,            		/* harvest speed */
											   STANDS_HARVEST_STAND_GROUP_3 | 	/* flags */
											   STANDS_HARVEST_XY_IS_ROBOT_POSITION);
#endif
			break;

		case ZONE_MY_STAND_GROUP_4:

			err = strat_harvest_orphan_stands (COLOR_X(strat_infos.zones[zone_num].x),
											   strat_infos.zones[zone_num].y,
											   COLOR_INVERT(SIDE_RIGHT),        /* side target */
								 			   COLOR_INVERT(SIDE_RIGHT),        /* storing sides */
											   0,                               /* blade angle */
											   SPEED_DIST_SLOW,                 /* harvest speed */
											   STANDS_HARVEST_BACK_INIT_POS |
                                               STANDS_HARVEST_CALIB_X);	        /* flags */

				break;

			case ZONE_MY_HOME_POPCORNS:

				err = strat_release_popcorns_in_home (COLOR_X(strat_infos.zones[zone_num].x),
														strat_infos.zones[zone_num].y, 0);
				break;

		case ZONE_MY_HOME_SPOTLIGHT:
			err = strat_buit_and_release_spotlight (COLOR_X(strat_infos.zones[zone_num].x),
													strat_infos.zones[zone_num].y,
													COLOR_INVERT(SIDE_LEFT), strat_need_build_a_tower());
			break;

		case ZONE_MY_PLATFORM:

			err = strat_buit_and_release_spotlight (COLOR_X(strat_infos.zones[zone_num].x),
													strat_infos.zones[zone_num].y,
													COLOR_INVERT(SIDE_LEFT), strat_need_build_a_tower());
				break;

		case ZONE_POPCORNCUP_1:
			err = strat_harvest_popcorn_cup (COLOR_X(strat_infos.zones[zone_num].x),
										   strat_infos.zones[zone_num].y,
										   SIDE_REAR, 0);
				break;


			case ZONE_POPCORNCUP_2:
			/* release front cup  */
			i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_RELEASE);

			/* harvest on the rear cup  */
			err = strat_harvest_popcorn_cup (COLOR_X(strat_infos.zones[zone_num].x),
										   strat_infos.zones[zone_num].y,
										   SIDE_REAR, 0);


			/* XXX, open a bit the rear tray, popcorns should fall into rear cup  */
			i2c_slavedspic_wait_ready();
			i2c_slavedspic_mode_tray(I2C_POPCORN_TRAY_MODE_CLOSE, -100);

				break;

			case ZONE_POPCORNCUP_3:
				err = strat_harvest_popcorn_cup (COLOR_X(strat_infos.zones[zone_num].x),
										   strat_infos.zones[zone_num].y,
										   SIDE_FRONT, 0);
				break;


			case ZONE_MY_CLAP_1:
				err = strat_close_clapperboards (COLOR_X(strat_infos.zones[zone_num].x),
										   strat_infos.zones[zone_num].y,
										   COLOR_INVERT(SIDE_RIGHT), 0);

			/* hide front clamp */
			i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_HIDE);

				break;

			case ZONE_MY_CLAP_2:
				err = strat_close_clapperboards (COLOR_X(strat_infos.zones[zone_num].x),
										   strat_infos.zones[zone_num].y,
										   COLOR_INVERT(SIDE_RIGHT), 0);
				break;

			case ZONE_MY_CLAP_3:
				err = strat_close_clapperboards (COLOR_X(strat_infos.zones[zone_num].x),
										   strat_infos.zones[zone_num].y,
										   COLOR_INVERT(SIDE_LEFT), 0);
				break;


			case ZONE_MY_POPCORNMAC:

				err = strat_harvest_popcorns_machine (COLOR_X(strat_infos.zones[zone_num].x),
										   strat_infos.zones[zone_num].y);
				break;

			case ZONE_OPP_POPCORNMAC:
				err = strat_harvest_popcorns_machine (COLOR_X(strat_infos.zones[zone_num].x),
										   strat_infos.zones[zone_num].y);
				break;

			case ZONE_MY_CINEMA_UP:
				err = strat_release_popcorns_in_home (COLOR_X(strat_infos.zones[zone_num].x),
														strat_infos.zones[zone_num].y, POPCORNS_RELEASE_ONLY_CUP);
				break;

			case ZONE_MY_CINEMA_DOWN_MAIN:
				err = strat_release_popcorns_in_home (COLOR_X(strat_infos.zones[zone_num].x),
														strat_infos.zones[zone_num].y, POPCORNS_RELEASE_ONLY_CUP);
				break;


			case ZONE_MY_STAIRS:
			case ZONE_MY_STAIRWAY:

				DEBUG(E_USER_STRAT, "R1, Working on zone ... ");
				trajectory_turnto_xy (&mainboard.traj,
									  COLOR_X(strat_infos.zones[zone_num].x),
									  strat_infos.zones[zone_num].y);
				err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

				//time_wait_ms(2000);
				DEBUG(E_USER_STRAT, "R1, ... fishish!! ");
				//ERROUT(END_TRAJ);
				break;

			case ZONE_CUP_NEAR_STAIRS:
				err = strat_harvest_popcorn_cup (COLOR_X(strat_infos.zones[zone_num].x),
										   strat_infos.zones[zone_num].y,
										   SIDE_REAR, 0);
				break;


			default:
				ERROR (E_USER_STRAT, "R1, ERROR zone %d not supported", zone_num);
				//ERROUT(END_TRAJ);
				break;
		}
	}

end:
	return err;
}





/* return 1 if need to wait SYNCHRONIZATION */
/* msg: message we need to wait from secondary robot */
uint8_t strat_wait_sync_main_robot(uint8_t msg)
{
	/* manual syncro */
	if (strat_infos.debug_step)
	{
        /* key trigger */
        if (strat_smart[MAIN_ROBOT].key_trigger) {
            strat_smart[MAIN_ROBOT].key_trigger = 0;
            return 0;
		}
        else
            return 1;
	}

	/* strat syncro */
	switch(msg)
	{
		case MSG_UPPER_SIDE_IS_FREE:
			/* Wait until "is free" from sec robot */
			//if(strat_smart_get_msg() != MSG_UPPER_SIDE_IS_FREE)
			//	return 1;
			break;
		case MSG_CUP_RELEASED:
			/* Wait until cup is released by sec robot */
			if((strat_smart_get_msg() != MSG_CUP_RELEASED) && (strat_smart_get_msg() != MSG_RELEASE_CUP_IMPOSSIBLE))
				return 1;
			break;
		default:
			break;
	}

    return 0;
}


/* implements the strategy motor of the main robot,
   XXX: needs to be called periodically, BLOCKING implementation */
uint8_t strat_smart_main_robot(void)
{
	int8_t zone_num;
	uint8_t err;
	static uint8_t no_more_zones;
//    static microseconds us;
	strat_strategy_time();

	/* get new zone */
	zone_num = strat_get_new_zone(MAIN_ROBOT);


	// Free
	if(zone_num == ZONE_MY_HOME_POPCORNS || zone_num == ZONE_MY_PLATFORM || zone_num == ZONE_MY_CINEMA_DOWN_MAIN)
	{
		if((strat_smart_get_msg() == MSG_UPPER_SIDE_IS_BLOCKED) || (strat_smart_get_msg() == MSG_UPPER_SIDE_FREE))
		{
			DEBUG(E_USER_STRAT,"R1, sending message MSG_UPPER_SIDE_FREE.");
			strat_smart_set_msg(MSG_UPPER_SIDE_FREE);

			// Wait until free
			//if(strat_wait_sync_main_robot(MSG_UPPER_SIDE_IS_FREE))
			//	return END_TRAJ;
		}
	}


	/* zone is on upper side */
	if(zone_num == ZONE_MY_STAND_GROUP_3 || zone_num == ZONE_MY_STAND_GROUP_4 || zone_num == ZONE_MY_POPCORNMAC)
	{




		if (strat_infos.conf.flags & CONF_FLAG_DO_CUP_EXCHANGE)
		{
			// Cup near stairs
			// if main robot has no cup in the back, tell sec robot to release cup near stairs and wait until main robot can take it
			if(!(strat_infos.zones[ZONE_POPCORNCUP_2].flags & ZONE_CHECKED))
			{
				// First time, send message
				if((strat_smart_get_msg() != MSG_CUP_RELEASED) && (strat_smart_get_msg() != MSG_RELEASE_CUP_IMPOSSIBLE))
				{
					strat_infos.zones[ZONE_CUP_NEAR_STAIRS].flags &= (~ZONE_AVOID);
					strat_smart_set_msg(MSG_RELEASE_CUP_NEAR_STAIRS);
				}

				// Wait for response
				if(strat_wait_sync_main_robot(MSG_CUP_RELEASED))
					return END_TRAJ;

				// Sec robot responded
				else
				{
					// If cup has been released, first go and take it
					if(strat_smart_get_msg() == MSG_CUP_RELEASED)
					{
						DEBUG (E_USER_STRAT, "R1, going to ZONE_CUP_NEAR_STAIRS. msg: ",strat_smart_get_msg());
						#define STRAT_WITH_CUP_NEAR_STAIRS 17
						/* wait until sec robot is gone */
						time_wait_ms(3000);
						strat_smart[MAIN_ROBOT].current_strategy = STRAT_WITH_CUP_NEAR_STAIRS;
						strat_change_sequence_qualification(MAIN_ROBOT);
						zone_num = strat_get_new_zone(MAIN_ROBOT);
					}
				}
			}
		}
	}

	/* if no more zones return */
	if (zone_num == STRAT_NO_MORE_ZONES) {

		if (!no_more_zones) {
			no_more_zones = 1;
			DEBUG(E_USER_STRAT,"R1, strat #%d, NO MORE ZONES", strat_smart[MAIN_ROBOT].current_strategy);
		}
		return END_TRAJ;
	}

	/* if no valid zone, change strategy and return */
	if (zone_num == STRAT_OPP_IS_IN_ZONE) {
		DEBUG(E_USER_STRAT,"R1, strat #%d, OPPONENT IN ZONE", strat_smart[MAIN_ROBOT].current_strategy);
		strat_set_next_main_strategy();
		return END_TRAJ;
	}

	DEBUG(E_USER_STRAT,"R1, strat #%d: get zone %s (%d, %d)",
						strat_smart[MAIN_ROBOT].current_strategy,
						get_zone_name(zone_num), zone_num, strat_infos.zones[zone_num].prio);

	/* XXX debug step use only for subtraj command */
	//strat_debug_wait_key_pressed (MAIN_ROBOT);

	/* goto zone */
	DEBUG(E_USER_STRAT,"R1, strat #%d: goto zone %s (%d, %d)",
						strat_smart[MAIN_ROBOT].current_strategy,
						get_zone_name(zone_num), zone_num, strat_infos.zones[zone_num].prio);


	/* goto, if can't reach the zone change the strategy and return */
	err = strat_goto_zone(MAIN_ROBOT, zone_num);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT,"R1, ERROR, goto returned %s", get_err(err));
		strat_set_next_main_strategy();
		return err;
	}

	/* XXX debug step use only for subtraj command */
	//strat_debug_wait_key_pressed (MAIN_ROBOT);


	DEBUG(E_USER_STRAT,"R1,  message: %d", strat_smart_get_msg());
    strat_debug_wait_key_pressed (MAIN_ROBOT);


	/* work on zone */
	DEBUG(E_USER_STRAT,"R1, strat #%d: work on zone %s (%d, %d)",
						strat_smart[MAIN_ROBOT].current_strategy,
						get_zone_name(zone_num), zone_num, strat_infos.zones[zone_num].prio);

	/* work */
	err = strat_work_on_zone(MAIN_ROBOT, zone_num);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT,"R1, ERROR, work returned %s", get_err(err));
		/* XXX should not happen, return END_TRAJ */
		err = END_TRAJ;



	}

	if (TRAJ_SUCCESS(err) && zone_num == ZONE_MY_PLATFORM)
		strat_infos.zones[ZONE_MY_HOME_SPOTLIGHT].flags |= ZONE_CHECKED;


	/* check the zone as DONE */
	strat_infos.zones[zone_num].flags |= ZONE_CHECKED;
	return err;
}


void strat_smart_set_msg (uint8_t msg)
{
	uint8_t flags;

	IRQ_LOCK(flags);
	strat_infos.msg = msg;
	IRQ_UNLOCK(flags);
}


uint8_t strat_smart_get_msg (void)
{
	uint8_t flags;
	uint8_t ret;

	IRQ_LOCK(flags);
	ret=strat_infos.msg;
	IRQ_UNLOCK(flags);

	return ret;
}




/* return 1 if need to wait SYNCHRONIZATION */
uint8_t strat_wait_sync_secondary_robot(void)
{
	/* manual syncro */
	if (strat_infos.debug_step)
	{
        /* key capture */
    	int16_t c;
        c = cmdline_getchar();
        if ((char)c == 'p')
            strat_smart[MAIN_ROBOT].key_trigger = 1;
        else if ((char)c == 't')
            strat_smart[SEC_ROBOT].key_trigger = 1;


        if (strat_smart[SEC_ROBOT].key_trigger) {
            strat_smart[SEC_ROBOT].key_trigger = 0;
            return 0;
		}
        else
            return 1;
	}

	/* strat syncro */
	/* Block until main robot sets start */
	if (strat_smart_get_msg()==MSG_WAIT_START)
		return 1;

	/* Block upper side until "free" message (or timeout) */
#define ZONE_UPPER_SIDE_BLOCKING_TIMEOUT 30

	if ((strat_smart_get_msg() == MSG_UPPER_SIDE_IS_BLOCKED) &&
		(strat_smart[SEC_ROBOT].current_zone == ZONE_BLOCK_UPPER_SIDE) &&
		(time_get_s() < ZONE_UPPER_SIDE_BLOCKING_TIMEOUT))
	{
		return 1;
	}


    return 0;
}


/* implements the strategy motor of the secondary robot,
   XXX: needs to be called periodically, NON-BLOCKING implementation */
uint8_t strat_smart_secondary_robot(void)
{
    	/* static states */
	#define SYNCHRONIZATION 	0
	#define GET_NEW_ZONE	1
	#define GOTO			2
	#define GOTO_WAIT_ACK	3
	#define GOTO_WAIT_END	4
	#define WORK			5
	#define WORK_WAIT_ACK	6
	#define WORK_WAIT_END	7

	static microseconds us = 0;

	uint8_t received_ack;
	uint8_t err=0;

	static int8_t zone_num=STRAT_NO_MORE_ZONES;
	static uint8_t no_more_zones = 0;
	static uint8_t state = SYNCHRONIZATION;
#ifdef DEBUG_STRAT_SECONDARY
	static uint8_t state_saved = 0xFF;

	/* transitions debug */
	if (state != state_saved) {
		state_saved = state;
		DEBUG(E_USER_STRAT,"R2, new state is %d", state);
	}
#endif

	/* strat smart state machine implementation */
	switch (state)
	{
		case SYNCHRONIZATION:
            /* SYNCHRONIZATION mechanism */
            if(strat_wait_sync_secondary_robot())
            {
            #if 0
                if (time_get_us2()-us > 10000000) {
                    DEBUG(E_USER_STRAT,"R2, WAITING syncro");
					if (strat_infos.debug_step)
            			DEBUG(E_USER_STRAT,"R2, press key 't' for continue");
                    us = time_get_us2();
                }
            #endif
				break;
            }

            /* next state */
            state = GET_NEW_ZONE;

#ifdef DEBUG_STRAT_SECONDARY
			state_saved = state;
			DEBUG(E_USER_STRAT,"R2, new state is %d", state);
#endif
			/* XXX: continue without break */
			//break;

		case GET_NEW_ZONE:
			if(strat_smart_get_msg()!=MSG_RELEASE_CUP_NEAR_STAIRS)
				zone_num = strat_get_new_zone(SEC_ROBOT);

			// Received message to release cup near stairs
			else
			{
				DEBUG(E_USER_STRAT,"R2, RECEIVED MSG_RELEASE_CUP_NEAR_STAIRS.");
				if (strat_is_valid_zone(SEC_ROBOT, ZONE_CUP_NEAR_STAIRS) && (strat_infos.match_strategy == STR_QUALIFICATION)
					&& (strat_infos.zones[ZONE_POPCORNCUP_1].flags & ZONE_CHECKED)
					&& !(strat_infos.zones[ZONE_MY_CINEMA_UP].flags & ZONE_CHECKED))
				{
					DEBUG(E_USER_STRAT,"R2, GOING TO RELEASE CUP NEAR STAIRS.");

					#define STRAT_WITH_CUP_NEAR_STAIRS 17
					strat_smart[SEC_ROBOT].current_strategy = STRAT_WITH_CUP_NEAR_STAIRS;
					strat_change_sequence_qualification(SEC_ROBOT);
					zone_num = strat_get_new_zone(SEC_ROBOT);
				}
				else
				{
					DEBUG(E_USER_STRAT,"R2, RELEASE CUP NEAR STAIRS IMPOSSIBLE.");
					strat_smart_set_msg(MSG_RELEASE_CUP_IMPOSSIBLE);
					strat_infos.zones[ZONE_CUP_NEAR_STAIRS].flags |= ZONE_AVOID;
				}
			}

			/* if no more zones, goto SYNCHRONIZATION state XXX???*/
			if(zone_num == STRAT_NO_MORE_ZONES ) {
				if (!no_more_zones) {
					no_more_zones = 1;
					DEBUG(E_USER_STRAT,"R2, strat #%d, NO MORE ZONES", strat_smart[SEC_ROBOT].current_strategy);
				}
				state = SYNCHRONIZATION;
				break;
			}

			/* if no valid zone, change strategy */
 			if(zone_num == STRAT_OPP_IS_IN_ZONE) {
				DEBUG(E_USER_STRAT,"R2, strat #%d, NO VALID ZONE", strat_smart[SEC_ROBOT].current_strategy);
				//DEBUG(E_USER_STRAT,"R2, strat #%d, OPPONENT IN ZONE", strat_smart[SEC_ROBOT].current_strategy);
				strat_set_next_sec_strategy();
				break;
			}


			DEBUG(E_USER_STRAT,"R2, strat #%d: get zone %s (%d, %d)",
						strat_smart[SEC_ROBOT].current_strategy,
						get_zone_name(zone_num), zone_num, strat_infos.zones[zone_num].prio);


			/* update statistics */
			strat_smart[SEC_ROBOT].goto_zone = zone_num;

			/* next state */
			state = GOTO;

#ifdef DEBUG_STRAT_SECONDARY
			state_saved = state;
			DEBUG(E_USER_STRAT,"R2, new state is %d", state);
#endif
			/* XXX: continue without break */
			//break;

		case GOTO:
			/* goto zone */
			DEBUG(E_USER_STRAT,"R2, strat #%d: goto zone %s (%d, %d)",
						strat_smart[SEC_ROBOT].current_strategy,
						get_zone_name(zone_num), zone_num, strat_infos.zones[zone_num].prio);

			err = strat_goto_zone(SEC_ROBOT, zone_num);

            /* END_TRAJ means "no where to go", directly work */
            if (TRAJ_SUCCESS(err)) {
                state = WORK;
                break;
            }else if (err) {
				/* XXX never shoud be reached, infinite loop */
				DEBUG(E_USER_STRAT,"R2, ERROR, goto returned %s at line %d", get_err(err), __LINE__);
				//set new strategy
				strat_set_next_sec_strategy();
				state = GET_NEW_ZONE;
				break;
			}

			/* next state */
			state = GOTO_WAIT_ACK;
			us = time_get_us2();
			break;

		case GOTO_WAIT_ACK:
		    /* return if no minimum time */
			if (time_get_us2() - us < 200000L)
				break;

			/* wait ACK value until ACK, NACK or timeout */
			received_ack = bt_robot_2nd_is_ack_received ();

			if(received_ack == 1)
			{
				/* ACK, wait end trajectory */
				us = time_get_us2();
				state = GOTO_WAIT_END;
			}
			else if (received_ack !=1 && received_ack != 0) {
				/* NACK, retry */
				state = SYNCHRONIZATION;
			}
			else if (time_get_us2() - us > 1000000L) {
				/* timeout, retry */
				state = SYNCHRONIZATION;
			}
			break;

		case GOTO_WAIT_END:
		    /* return if no minimum time */
			if (time_get_us2() - us < 200000L)
				break;

			/* goto zone, if can't reach the zone change the strategy and get new one */
			if(!bt_robot_2nd_is_ret_received())
				break;

			err = bt_robot_2nd_test_end();

			if (!TRAJ_SUCCESS(err)) {
				DEBUG(E_USER_STRAT,"R2, ERROR, goto returned %s", get_err(err));
				strat_smart[SEC_ROBOT].current_zone = -1; /* TODO: why? */
				strat_set_next_sec_strategy();
				state = SYNCHRONIZATION;
				break;
			}

			/* update statistics */
			strat_smart[SEC_ROBOT].last_zone = strat_smart[SEC_ROBOT].current_zone;
			strat_smart[SEC_ROBOT].current_zone = strat_smart[SEC_ROBOT].goto_zone;

			/* send message after done synchronization */
			if((strat_smart[SEC_ROBOT].current_zone == ZONE_BLOCK_UPPER_SIDE) &&
                (strat_smart[SEC_ROBOT].last_zone != ZONE_BLOCK_UPPER_SIDE))
			{
				DEBUG(E_USER_STRAT,"R2, in ZONE_BLOCK_UPPER_SIDE.");
				strat_smart_set_msg(MSG_UPPER_SIDE_IS_BLOCKED);

			}
#if 0
			else if(strat_smart[SEC_ROBOT].current_zone == ZONE_FREE_UPPER_SIDE)
			{
				DEBUG(E_USER_STRAT,"R2, in ZONE_FREE_UPPER_SIDE.");
				strat_smart_set_msg(MSG_UPPER_SIDE_IS_FREE);
			}
#endif

			/* next state */
			if(strat_smart[SEC_ROBOT].current_zone == ZONE_BLOCK_UPPER_SIDE)
			{
				/* update statistics */
				strat_infos.zones[zone_num].flags |= ZONE_CHECKED;
				state = SYNCHRONIZATION;
			}

			else
			{
				state = WORK;
			}

#ifdef DEBUG_STRAT_SECONDARY
			state_old = state;
			DEBUG(E_USER_STRAT,"R2, new state is %d", state);
#endif
			// Must do break
			break;

		case WORK:

			/* FIXME: strat debug on an event */
			//strat_debug_wait_key_pressed (SEC_ROBOT);

			/* work */
			DEBUG(E_USER_STRAT,"R2, strat #%d: work on zone %s (%d, %d)",
						strat_smart[SEC_ROBOT].current_strategy,
						get_zone_name(zone_num), zone_num, strat_infos.zones[zone_num].prio);

			err = strat_work_on_zone(SEC_ROBOT, zone_num);

            /* END_TRAJ means "no where to work", check zone and go directly to synchronize*/
            if (TRAJ_SUCCESS(err)) {
		        /* update statistics */
		        strat_infos.zones[zone_num].flags |= ZONE_CHECKED;

                /* next state */
                state = SYNCHRONIZATION;
			    break;
            }
			else if (err) {
				/* XXX never shoud be reached, infinite loop */
				DEBUG(E_USER_STRAT,"R2, ERROR, (case work) work returned %s at line %d", get_err(err), __LINE__);
				state = GET_NEW_ZONE;
				break;
			}

			/* next state */
			state = WORK_WAIT_ACK;
			us = time_get_us2();
			break;

		case WORK_WAIT_ACK:
		    /* return if no minimum time */
			if (time_get_us2() - us < 200000L)
				break;

			/* wait ACK value until ACK, NACK or timeout */
			received_ack = bt_robot_2nd_is_ack_received ();

			if(received_ack == 1)
			{
				/* ACK, wait end work */
				us = time_get_us2();
				state = WORK_WAIT_END;
			}
			else if (received_ack !=1 && received_ack != 0) {
				/* NACK, retry */
				state = GET_NEW_ZONE;
			}
			else if (time_get_us2() - us > 1000000L) {
				/* timeout, retry */
				state = GET_NEW_ZONE;
			}

			break;

		case WORK_WAIT_END:
		    /* return if no minimum time */
			if (time_get_us2() - us < 200000L)
				break;

			if(!bt_robot_2nd_is_ret_received())
				break;

			err = bt_robot_2nd_test_end();

			if (!TRAJ_SUCCESS(err)) {
				DEBUG(E_USER_STRAT,"R2, ERROR, work returned %s.", get_err(err));

				//If there is an error working in the cinemas don't try it again
				if(strat_smart[SEC_ROBOT].current_zone == ZONE_MY_CINEMA_DOWN_SEC
				|| strat_smart[SEC_ROBOT].current_zone == ZONE_MY_CINEMA_UP){
					strat_infos.zones[zone_num].flags |= ZONE_CHECKED;
				}

				/* timeout. After timeout, change strategy */
				if (time_get_us2() - us > 5000000L) {
					DEBUG(E_USER_STRAT,"R2, changing strategy.");
					strat_set_next_sec_strategy();
				}
				state = GET_NEW_ZONE;
				break;
			}

#if 0
			if(zone_num == ZONE_CUP_NEAR_STAIRS)
			{
				strat_smart_set_msg(MSG_CUP_RELEASED);
				strat_infos.zones[ZONE_CUP_NEAR_STAIRS].robot=MAIN_ROBOT;
			}
#endif

			/* update statistics */
#if 0
			if(zone_num != ZONE_CUP_NEAR_STAIRS)
#endif
			strat_infos.zones[zone_num].flags |= ZONE_CHECKED;

            /* next state */
            state = SYNCHRONIZATION;
			break;


		default:
            state = SYNCHRONIZATION;
			break;
	}

	return err;
}

void strat_opp_tracking (void)
{
#if 0
#define MAX_TIME_BETWEEN_VISITS_MS	4000
#define TIME_MS_TREE				1500
#define TIME_MS_HEART				1500
#define TIME_MS_BASKET				1000
#define UPDATE_ZONES_PERIOD_MS		25

	uint8_t flags;
	uint8_t zone_opp;

    /* check if there are opponents in every zone */
    for(zone_opp = 0; zone_opp <  ZONES_MAX-1; zone_opp++)
    {

	if(opponents_are_in_area(COLOR_X(strat_infos.zones[zone_opp].x_up), strat_infos.zones[zone_opp].y_up,
                                     COLOR_X(strat_infos.zones[zone_opp].x_down), strat_infos.zones[zone_opp].y_down)){

			if(!(strat_infos.zones[zone_opp].flags & (ZONE_CHECKED_OPP)))
			{
				IRQ_LOCK(flags);
				strat_infos.zones[zone_opp].last_time_opp_here=time_get_us2();
				IRQ_UNLOCK(flags);
				if((time_get_us2() - strat_infos.zones[zone_opp].last_time_opp_here) < MAX_TIME_BETWEEN_VISITS_MS*1000L)
				{
					/* Opponent continues in the same zone: */
					/* update zone time */
					IRQ_LOCK(flags);
					strat_infos.zones[zone_opp].opp_time_zone_us += UPDATE_ZONES_PERIOD_MS*1000L;
					IRQ_UNLOCK(flags);

					/* Mark zone as checked and sum points */
					switch(strat_infos.zones[zone_opp].type)
					{
						case ZONE_TYPE_TREE:
							if(strat_infos.zones[zone_opp].opp_time_zone_us>=TIME_MS_TREE*1000L)
							{
								strat_infos.zones[zone_opp].flags |= ZONE_CHECKED_OPP;
								strat_infos.opp_harvested_trees++;
								DEBUG(E_USER_STRAT,"opp_harvested_trees=%d",strat_infos.opp_harvested_trees);
								DEBUG(E_USER_STRAT,"OPP approximated score: %d", strat_infos.opp_score);
							}
							break;
						case ZONE_TYPE_BASKET:
							if(((mainboard.our_color==I2C_COLOR_YELLOW) && (zone_opp==ZONE_BASKET_1)) ||
							((mainboard.our_color==I2C_COLOR_GREEN) && (zone_opp==ZONE_BASKET_2)))
							{
								if(strat_infos.zones[zone_opp].opp_time_zone_us>=TIME_MS_BASKET*1000L)
								{
									if(strat_infos.opp_harvested_trees!=0)
									{
										strat_infos.opp_score += strat_infos.opp_harvested_trees * 3;
										strat_infos.opp_harvested_trees=0;
										DEBUG(E_USER_STRAT,"opp_harvested_trees=%d",strat_infos.opp_harvested_trees);
										DEBUG(E_USER_STRAT,"OPP approximated score: %d", strat_infos.opp_score);
									}
								}
							}
							break;
						case ZONE_TYPE_HEART:
							if(strat_infos.zones[zone_opp].opp_time_zone_us>= TIME_MS_HEART*1000L)
							{
								strat_infos.zones[zone_opp].flags |= ZONE_CHECKED_OPP;
								strat_infos.opp_score += 4;
								DEBUG(E_USER_STRAT,"OPP approximated score: %d", strat_infos.opp_score);
							}
							break;
						default:
							break;
					}
				}

				/* Zone has changed */
				else
				{
					/* reset zone time */
					IRQ_LOCK(flags);
					strat_infos.zones[zone_opp].opp_time_zone_us = 0;
					IRQ_UNLOCK(flags);
				}
			}
		}
	}
#endif
}
