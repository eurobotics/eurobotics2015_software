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


/*
 * Check if the zone is available.
 * @Param zone_num checked zone
 * @Return 1 if is a valid zone, 0 otherwise
 */
int8_t strat_is_valid_zone(uint8_t robot, int8_t zone_num)
{
	/* return if zone_num out of range */
	if(zone_num < 0 || zone_num >= ZONES_MAX){
		ERROR (E_USER_STRAT, "ERROR, %s, zone_num out of range"); 
		return 0;
	}

	/* if the zone has 0 priority then must be avoided */
	if(strat_infos.zones[zone_num].prio == 0)
	{
		//DEBUG(E_USER_STRAT,"zone num: %d. avoid.");
		return 0;
	}

	/* discard current zone */
	if(strat_smart[robot].current_zone == zone_num)
	{
		//DEBUG(E_USER_STRAT,"zone num: %d. current_zone.");
		return 0;
	}

	/* discard if opp is in zone */
	if(opponents_are_in_area(COLOR_X(strat_infos.zones[zone_num].x_up), 	strat_infos.zones[zone_num].y_up,
							 COLOR_X(strat_infos.zones[zone_num].x_down),	strat_infos.zones[zone_num].y_down)) {
		return 0;
	}

	/* discard avoid and checked zones */
	if(strat_infos.zones[zone_num].flags & ZONE_AVOID)
	{
		//DEBUG(E_USER_STRAT,"zone num: %d. avoid.");
		return 0;
	}

	return 1;
}

/* set next SEC_ROBOT_ROBOT strategy */
void strat_set_next_sec_strategy(void)
{
	strat_infos.current_sec_strategy++;

	switch(strat_infos.current_sec_strategy)
	{
		case 1:
			set_strat_sec_1();
			break;
		case 2:
			set_strat_sec_2();
			break;
		case 3:
			set_strat_sec_3();
			strat_infos.current_sec_strategy = 0;
			break;
		default:
			break;
	}
	DEBUG(E_USER_STRAT,"SEC_ROBOT, NEW strat #%d", strat_infos.current_sec_strategy);
}

/* set next MAIN_ROBOT_ROBOT_ROBOT strategy */
void strat_set_next_main_strategy(void)
{
	//strat_infos.current_main_strategy++;
	switch(strat_infos.current_main_strategy){
		case 1:
			set_strat_main_1();
			break;
		/*case 2:
			set_strat_main_2();
			strat_infos.current_sec_strategy = 0;
			break;*/
		default:
			break;
	}
	DEBUG(E_USER_STRAT,"MAIN_ROBOT, NEW strat #%d", strat_infos.current_main_strategy);
}

void set_strat_main_1(void)
{
	strat_infos.current_main_strategy = 1;

	strat_infos.zones[ZONE_MY_STAND_GROUP_1].prio = 100;
	strat_infos.zones[ZONE_POPCORNCUP_3].prio = 90;

	strat_infos.zones[ZONE_MY_CLAP_2].prio = 80;
	strat_infos.zones[ZONE_POPCORNCUP_2].prio = 70;
	strat_infos.zones[ZONE_MY_STAND_GROUP_2].prio = 60;
	strat_infos.zones[ZONE_MY_CLAP_1].prio = 50;

	strat_infos.zones[ZONE_MY_STAND_GROUP_3].prio = 40;
	strat_infos.zones[ZONE_MY_POPCORNMAC].prio = 30;
	strat_infos.zones[ZONE_MY_STAND_GROUP_4].prio = 20;

	strat_infos.zones[ZONE_MY_HOME].prio = 10;
}

void set_strat_main_2(void){
	strat_infos.current_main_strategy = 1;

	strat_infos.zones[ZONE_MY_STAND_GROUP_1].prio = 100;

	strat_infos.zones[ZONE_MY_CLAP_2].prio = 90;
	strat_infos.zones[ZONE_POPCORNCUP_2].prio = 80;
	strat_infos.zones[ZONE_MY_STAND_GROUP_2].prio = 70;
	strat_infos.zones[ZONE_MY_CLAP_1].prio = 60;

	strat_infos.zones[ZONE_MY_STAND_GROUP_3].prio = 50;
	strat_infos.zones[ZONE_MY_POPCORNMAC].prio = 40;
	strat_infos.zones[ZONE_MY_STAND_GROUP_4].prio = 30;

	strat_infos.zones[ZONE_MY_HOME].prio = 10;
}


void set_strat_sec_1(void){
	strat_infos.current_sec_strategy = 1;
	strat_infos.zones[ZONE_POPCORNCUP_1].prio = 100;

	strat_infos.zones[ZONE_MY_CLAP_3].prio = 90;
	//DEBUG(E_USER_STRAT,"strat_sec_1");
	strat_infos.zones[ZONE_MY_CINEMA_DOWN].prio = 80;
	strat_infos.zones[ZONE_MY_CINEMA_UP].prio = 70;
	strat_infos.zones[ZONE_MY_STAIRS].prio = 60;
}
void set_strat_sec_2(void){
	strat_infos.current_sec_strategy = 2;
	//DEBUG(E_USER_STRAT,"strat_sec_2");
	strat_infos.zones[ZONE_POPCORNCUP_1].prio = 100;
	strat_infos.zones[ZONE_MY_CINEMA_UP].prio = 90;

	strat_infos.zones[ZONE_MY_CLAP_3].prio = 80;
	strat_infos.zones[ZONE_MY_CINEMA_DOWN].prio = 70;
	strat_infos.zones[ZONE_MY_STAIRS].prio = 60;
}

void set_strat_sec_3(void){
	strat_infos.current_sec_strategy = 3;
	//DEBUG(E_USER_STRAT,"strat_sec_3");

	strat_infos.zones[ZONE_POPCORNCUP_1].prio = 100;
	strat_infos.zones[ZONE_MY_CINEMA_DOWN].prio = 90;
	strat_infos.zones[ZONE_MY_CLAP_3].prio = 80;
	strat_infos.zones[ZONE_MY_CINEMA_UP].prio = 70;
	strat_infos.zones[ZONE_MY_STAIRS].prio = 60;
}





/* return new work zone, -1 if any zone is found */
#define STRAT_NO_MORE_ZONES	-1
#define STRAT_NO_VALID_ZONE	-2
int8_t strat_get_new_zone(uint8_t robot)
{
	uint8_t prio_max = 0;
	int8_t zone_num = STRAT_NO_MORE_ZONES;
	int8_t i=0;

	/* 1. get the robot NO CHECKED zone with the maximun priority  */
	for(i=0; i < ZONES_MAX; i++)
	{
		if(strat_infos.zones[i].prio >= prio_max && 
		  (strat_infos.zones[i].flags != ZONE_CHECKED) && 
		  (strat_infos.zones[i].robot == robot)) 
		{
			/* check if is a valid zone */
			prio_max = strat_infos.zones[i].prio;
			zone_num = i;
		}
	}

	/* 2. check if the maximun priority zone is valid */
	if(zone_num != STRAT_NO_MORE_ZONES)
	{
		if (!strat_is_valid_zone(robot, zone_num))
			zone_num = STRAT_NO_VALID_ZONE;
	}

	/* XXX: here we have the zone with the maximun priority, and then
			we check if this zone is valid. 

			Why we don't discard the no valid zones in the maximun priority
			zone calculation at point 1. 
	*/

	return zone_num;
}

/* return END_TRAJ if zone is reached, err otherwise */
uint8_t strat_goto_zone(uint8_t robot, uint8_t zone_num)
{
	int8_t err=0;

	/* update strat_infos */
	strat_smart[robot].current_zone = -1;
	strat_smart[robot].goto_zone = zone_num;

	/* return if NULL xy */
	if (strat_infos.zones[zone_num].init_x == NULL ||
		strat_infos.zones[zone_num].init_y == NULL) {
		trajectory_d_rel(&mainboard.traj, 500);
		WARNING (E_USER_STRAT, "WARNING, No where to go (xy is NULL)");
		ERROUT(END_TRAJ);
	}

	/* XXX secondary robot: goto and return */
	if(strat_infos.zones[zone_num].robot==SEC_ROBOT) {
		bt_robot_2nd_goto_and_avoid(COLOR_X(strat_infos.zones[zone_num].init_x),
												strat_infos.zones[zone_num].init_y);
		return 0;
	}

	/* set boundinbox */
	if (zone_num == ZONE_POPCORNCUP_2 || zone_num == ZONE_MY_STAND_GROUP_2)
		strat_set_bounding_box(BOUNDINBOX_WITHOUT_PLATFORM );
	else
		strat_set_bounding_box(BOUNDINBOX_INCLUDES_PLAFORM );


	/* main robot: goto and wait */
	err = goto_and_avoid (COLOR_X(strat_infos.zones[zone_num].init_x),
									strat_infos.zones[zone_num].init_y,
									TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	
	/* update strat_infos */
	strat_smart[robot].last_zone = strat_smart[robot].current_zone;
	strat_smart[robot].goto_zone = -1;

	if (!TRAJ_SUCCESS(err)) {
		strat_smart[robot].current_zone = -1;
	}
	else{
		strat_smart[robot].current_zone=zone_num;
	}
end:
	return err;
}


/* return END_TRAJ if the work is done, err otherwise */
uint8_t strat_work_on_zone(uint8_t robot, uint8_t zone_num)
{
	uint8_t err = END_TRAJ;

	/* return if NULL xy */
	if (strat_infos.zones[zone_num].x == NULL ||
		strat_infos.zones[zone_num].y == NULL) {
		ERROR (E_USER_STRAT, "ERROR zone xy is NULL");
		ERROUT(END_ERROR);
		return err;
	}

	/* XXX secondary_robot: send work on zone bt task and return */
	if(strat_infos.zones[zone_num].robot == robot)
	{
		/* TODO */
		return 0;
	}

	/* main robot, dependin on zone */
	switch (zone_num)
	{
		case ZONE_MY_STAND_GROUP_1:

			/* TODO: call specific function for stand group 1 */
			err = strat_harvest_orphan_stands (COLOR_X(MY_STAND_4_X),
											   MY_STAND_4_Y,
											   COLOR_INVERT(SIDE_RIGHT),
								 			   COLOR_INVERT(SIDE_RIGHT),
											   0,
											   SPEED_DIST_SLOW, /* harvest speed */
											   0);				/* flags */
		    if (!TRAJ_SUCCESS(err))
			   ERROUT(err);

			err = strat_harvest_orphan_stands (COLOR_X(MY_STAND_5_X),
											   MY_STAND_5_Y,
											   COLOR_INVERT(SIDE_LEFT),
								 			   COLOR_INVERT(SIDE_LEFT),
											   0,
											   SPEED_DIST_SLOW, /* harvest speed */
											   0);				/* flags */
		    if (!TRAJ_SUCCESS(err))
			   ERROUT(err);

			err = strat_harvest_orphan_stands (COLOR_X(MY_STAND_6_X),
											   MY_STAND_6_Y,
											   COLOR_INVERT(SIDE_LEFT),
								 			   COLOR_INVERT(SIDE_LEFT),
											   0,
											   SPEED_DIST_SLOW, /* harvest speed */
											   0);				/* flags */
			break;

#if 0
		case ZONE_MY_STAND_GROUP_2:
			err = strat_harvest_orphan_stands (COLOR_X(strat_infos.zones[zone_num].x),
											   strat_infos.zones[zone_num].y,
											   COLOR_INVERT(SIDE_LEFT),         /* side target */
								 			   SIDE_ALL,                        /* storing sides */
											   COLOR_A_REL(-10),                /* blade angle */
											   SPEED_DIST_SLOW,                 /* harvest speed */
											   STANDS_HARVEST_BACK_INIT_POS);	/* flags */
			break;

		case ZONE_MY_STAND_GROUP_3:
			err = strat_harvest_orphan_stands (COLOR_X(strat_infos.zones[zone_num].x),
											   strat_infos.zones[zone_num].y,
											   COLOR_INVERT(SIDE_LEFT),         /* side target */
								 			   SIDE_ALL,                        /* storing sides */
											   COLOR_A_REL(-20),                /* blade angle */
											   SPEED_DIST_SLOW,                 /* harvest speed */
											   STANDS_HARVEST_BACK_INIT_POS);	/* flags */
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

		case ZONE_MY_HOME:
			err = strat_buit_and_release_spotlight (COLOR_X(strat_infos.zones[zone_num].x),
													strat_infos.zones[zone_num].y,
													COLOR_INVERT(SIDE_LEFT));
			break;

#endif
		case ZONE_MY_STAND_GROUP_2:
		case ZONE_MY_STAND_GROUP_3:
		case ZONE_MY_STAND_GROUP_4:
		case ZONE_MY_POPCORNMAC:
		case ZONE_OPP_POPCORNMAC:

		case ZONE_POPCORNCUP_1:
		case ZONE_POPCORNCUP_2:
		case ZONE_POPCORNCUP_3:

		case ZONE_MY_CINEMA_UP:
		case ZONE_MY_CINEMA_DOWN:

		case ZONE_MY_CLAP_1:
		case ZONE_MY_CLAP_2:
		case ZONE_MY_CLAP_3:

			DEBUG(E_USER_STRAT, "Working on zone... ");
			time_wait_ms(2000);
			DEBUG(E_USER_STRAT, "fishish!! ");
			ERROUT(END_TRAJ);

			break;

		default:
			ERROR (E_USER_STRAT, "ERROR %s zone %d not supported", zone_num);
			break;

	}

end:
	return err;
}


/* debug state machines step to step */
void state_debug_wait_key_pressed(void)
{
	if (!strat_infos.debug_step)
		return;

	DEBUG(E_USER_STRAT,"press a key");
	while(!cmdline_keypressed());
}



/* implements the stratety motor of the main robot,
   XXX: needs to be called periodically, BLOCKING implementation */
uint8_t strat_smart_main_robot(void)
{
	int8_t zone_num;
	uint8_t err;
	static uint8_t no_more_zones;
    static microseconds us;

    /* TODO: syncronization mechanism */
    /*
    if( strat_sync_main_robot()) {
        if (time_get_us2()-us > 1000000) {
            DEBUG(E_USER_STRAT,"MAIN_ROBOT, WAITING syncronization");
            us = time_get_us2();
        }
        return END_TRA;
    }
    */

	/* get new zone */
	zone_num = strat_get_new_zone(MAIN_ROBOT);

	/* if no more zones return */
	if (zone_num == STRAT_NO_MORE_ZONES) {

		if (!no_more_zones) {
			no_more_zones = 1;
			DEBUG(E_USER_STRAT,"MAIN_ROBOT, strat #%d, NO MORE ZONES AVAILABLE", strat_infos.current_main_strategy);
		}
		return END_TRAJ;
	}

	/* if no valid zone, change strategy and return */
	if (zone_num == STRAT_NO_VALID_ZONE) {
		DEBUG(E_USER_STRAT,"MAIN_ROBOT, strat #%d, NO VALID ZONE", strat_infos.current_main_strategy);
		strat_set_next_main_strategy();
		return END_TRAJ;
	}


	/* goto zone */
	DEBUG(E_USER_STRAT,"MAIN_ROBOT, strat #%d: goto zone %s (%d, %d)",
						strat_infos.current_main_strategy, 
						get_zone_name(zone_num), zone_num, strat_infos.zones[zone_num].prio);

	/* update statistics */
	strat_smart[MAIN_ROBOT].goto_zone = zone_num;

	/* goto, if can't reach the zone change the strategy and return */
	err = strat_goto_zone(MAIN_ROBOT, zone_num);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT,"MAIN_ROBOT, ERROR, goto returned %s", get_err(err));
		strat_set_next_main_strategy();
		return err;
	}

	/* work on zone */
	DEBUG(E_USER_STRAT,"MAIN_ROBOT, strat #%d: goto zone %s (%d, %d)",
						strat_infos.current_main_strategy, 
						get_zone_name(zone_num), zone_num, strat_infos.zones[zone_num].prio);

	/* update statistis */
	strat_smart[MAIN_ROBOT].last_zone = strat_smart[MAIN_ROBOT].current_zone;
	strat_smart[MAIN_ROBOT].current_zone = strat_smart[MAIN_ROBOT].goto_zone;

	/* work */
	err = strat_work_on_zone(MAIN_ROBOT,zone_num);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT,"MAIN_ROBOT, ERROR, work returned %s", get_err(err));
		/* XXX should doesn't happend, return END_TRAJ */
		err = END_TRAJ;
	}

	/* check the zone as DONE */
	strat_infos.zones[zone_num].flags |= ZONE_CHECKED;
	return err;
}

/*Function to set robot_2nd in waiting state*/
void strat_should_wait_new_order(void){
	switch(strat_smart[SEC_ROBOT].current_zone){

		case ZONE_POPCORNCUP_2:
			strat_infos.strat_smart_sec = IDLE;
		break;

		default:
			strat_infos.strat_smart_sec = GET_NEW_ZONE;
		break;
	}
}

/*Function to set robot_2nd in working state*/
void strat_set_sec_new_order(int8_t zone_num)
{
	switch(zone_num){

		case ZONE_MY_CLAP_2:
			strat_infos.strat_smart_sec = GET_NEW_ZONE;
		break;
		case ZONE_MY_STAND_GROUP_1:
			strat_infos.strat_smart_sec = INIT_ROBOT_2ND;
		break;
		default:
		break;
	}
}


/* implements the strategy motor of the secondary robot,
   XXX: needs to be called periodically, NON-BLOCKING implementation */
void strat_smart_secondary_robot(void)
{
	static microseconds us = 0;
	uint8_t received_ack;
	uint8_t err;

	static int8_t zone_num;
	static uint8_t no_more_zones = 0;
	static uint8_t state_saved = IDLE;

	/* transitions debug */
	if (strat_infos.strat_smart_sec != state_saved) {
		state_saved = strat_infos.strat_smart_sec;
		DEBUG(E_USER_STRAT,"SEC_ROBOT, %s new state is %d", strat_infos.strat_smart_sec);
	}

	/* strat smart state machine implementation */
	switch (strat_infos.strat_smart_sec){

		case IDLE:
            /* TODO: syncronization mechanism */
            /*
            if( strat_sync_secondary_robot()) {
                if (time_get_us2()-us > 1000000) {
                    DEBUG(E_USER_STRAT,"SEC_ROBOT, WAITING syncronization");
                    us = time_get_us2();
                }
            }
            */
            
            /* next state */
            strat_infos.strat_smart_sec = GET_NEW_ZONE;        
			break;

        /* TODO: remove from here, add new zone ZONE_ */
		case INIT_ROBOT_2ND:
#define INIT_ROBOT_2ND_X 400
#define INIT_ROBOT_2ND_Y 1000
			bt_robot_2nd_goto_xy_abs(COLOR_X(INIT_ROBOT_2ND_X) , INIT_ROBOT_2ND_Y);

			/* next state */
			/* XXX no wait either ACK and END_TRAJ */
			strat_infos.strat_smart_sec = IDLE;
			break;


		case GET_NEW_ZONE:
			zone_num = strat_get_new_zone(SEC_ROBOT);

			/* if no more zones, goto idle state */
			if(zone_num == STRAT_NO_MORE_ZONES ) {
				if (!no_more_zones) {
					no_more_zones = 1;
					DEBUG(E_USER_STRAT,"SEC_ROBOT, strat #%d, NO MORE ZONES AVAILABLE", strat_infos.current_sec_strategy);
				}
				strat_infos.strat_smart_sec = IDLE;
				break;
			}

			/* if no valid zone, change strategy */
 			if(zone_num == STRAT_NO_VALID_ZONE ) {
				DEBUG(E_USER_STRAT,"SEC_ROBOT, strat #%d, NO VALID ZONE", strat_infos.current_sec_strategy);
				/* TODO */				
				//set_next_sec_strategy();
				break;
			}
			
			/* update statistics */
			strat_smart[SEC_ROBOT].goto_zone = zone_num;

			/* next state */
			strat_infos.strat_smart_sec = GOTO;

		case GOTO:
			DEBUG(E_USER_STRAT,"SEC_ROBOT, strat #%d: goto zone %s (%d, %d)",
						strat_infos.current_sec_strategy, 
						get_zone_name(zone_num), zone_num, strat_infos.zones[zone_num].prio);

			/* goto, if can't reach the zone change the strategy and return */
			err = strat_goto_zone(SEC_ROBOT, zone_num);
			if (err) {
				/* XXX never shoud be reached, infinite loop */
				DEBUG(E_USER_STRAT,"SEC_ROBOT, ERROR, goto returned %s at line %d", get_err(err), __LINE__);				
				strat_infos.strat_smart_sec = GET_NEW_ZONE;
				break;
			}

			/* next state */
			strat_infos.strat_smart_sec = GOTO_WAIT_ACK;
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
				strat_infos.strat_smart_sec = GOTO_WAIT_END;
			}
			else if (received_ack !=1 && received_ack != 0) {
				/* NACK, retry */
				strat_infos.strat_smart_sec = GET_NEW_ZONE;
			}
			else if (time_get_us2() - us > 1000000L) {
				/* timeout, retry */
				strat_infos.strat_smart_sec = GET_NEW_ZONE;
			}

			break;

		case GOTO_WAIT_END:
		    /* return if no minimum time */
			if (time_get_us2() - us < 200000L)
				break;

			if(!bt_robot_2nd_is_ret_received())
				break;

			err = bt_robot_2nd_test_end();
			if (!TRAJ_SUCCESS(err)) {
				DEBUG(E_USER_STRAT,"SEC_ROBOT, ERROR, goto returned %s", get_err(err));
				/* TODO */
				strat_smart[SEC_ROBOT].current_zone = -1;
				strat_set_next_sec_strategy();
				strat_infos.strat_smart_sec = GET_NEW_ZONE;
				break;
			}

			/* update statistis */
			strat_smart[SEC_ROBOT].last_zone = strat_smart[SEC_ROBOT].current_zone;
			strat_smart[SEC_ROBOT].current_zone = strat_smart[SEC_ROBOT].goto_zone;

			/* next state */
			strat_infos.strat_smart_sec = WORK;

		case WORK:
			err = strat_work_on_zone(SEC_ROBOT, zone_num);
			if (err) {
				/* XXX never shoud be reached, infinite loop */
				DEBUG(E_USER_STRAT,"SEC_ROBOT, ERROR, work returned %s at line %d", get_err(err), __LINE__);
				strat_infos.strat_smart_sec = GET_NEW_ZONE;
				break;
			}

			/* next state */
			strat_infos.strat_smart_sec = WORK_WAIT_ACK;
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
				strat_infos.strat_smart_sec = WORK_WAIT_END;
			}
			else if (received_ack !=1 && received_ack != 0) {
				/* NACK, retry */
				strat_infos.strat_smart_sec = GET_NEW_ZONE;
			}
			else if (time_get_us2() - us > 1000000L) {
				/* timeout, retry */
				strat_infos.strat_smart_sec = GET_NEW_ZONE;
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
				DEBUG(E_USER_STRAT,"SEC_ROBOT, ERROR, work returned %s", get_err(err));
				strat_infos.strat_smart_sec = GET_NEW_ZONE;
				break;
			}

			/* update statistics */
			strat_infos.zones[zone_num].flags |= ZONE_CHECKED;

			/* TODO: remove instead new implemntation of robots syncronization */
			//strat_should_wait_new_order();

            /* next state */
            strat_infos.strat_smart_sec = IDLE;
			break;


		default:
			strat_infos.strat_smart_sec = IDLE;
			break;
	}
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


