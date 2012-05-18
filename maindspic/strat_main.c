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
 *  Javier Baliñas Santos <javier@arc-robots.org>
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


/* return 1 if is a valid zone and 0 otherwise */
uint8_t strat_is_valid_zone(uint8_t zone_num)
{
	/* discard actual zone */
	if(strat_infos.current_zone == zone_num)
		return 0;

	/* discard down side zones depends on strat config */
	if((strat_infos.conf.flags & ENABLE_DOWN_SIDE_ZONES) == 0 
		&& strat_infos.zones[zone_num].init_y < (AREA_Y/2) )
		return 0;

	/* discard if opp is in zone */
	if(opponent_is_in_area(	COLOR_X(strat_infos.zones[zone_num].x_up),strat_infos.zones[zone_num].y_up,
								COLOR_X(strat_infos.zones[zone_num].x_down),	strat_infos.zones[zone_num].y_down)) {
		return 0;
	}

	/* discard our checked zones */
	if(strat_infos.zones[zone_num].flags & ZONE_CHECKED)
		return 0;	

	/* discard opp checked zones */
	if(strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP)
		return 0;	

	/* discard avoid zones */
	if(strat_infos.zones[zone_num].flags & ZONE_AVOID)
		return 0;	

	/* if we have treasure on mouth, we only can send messages, save treasure on ship or pickup middle coins group */
	if(strat_infos.treasure_in_mouth) {
		if(zone_num != ZONE_SHIP_OUR_CAPTAINS_BEDRROM
			&& zone_num != ZONE_SHIP_OUR_DECK_1
			&& zone_num != ZONE_SHIP_OUR_DECK_2
			&& zone_num != ZONE_SHIP_OUR_HOLD
			&& zone_num != ZONE_SAVE_TREASURE
			&& zone_num != ZONE_MIDDLE_COINS_GROUP
			&& zone_num != ZONE_OUR_BOTTLE_1
			&& zone_num != ZONE_OUR_BOTTLE_2 )
		
		return 0;
	}
	/* if we have not treasure on mouth, we have not to save treasure any where */
	else {
		if(zone_num == ZONE_SHIP_OUR_CAPTAINS_BEDRROM
		|| zone_num == ZONE_SHIP_OUR_DECK_1
		|| zone_num == ZONE_SHIP_OUR_DECK_2
		|| zone_num == ZONE_SAVE_TREASURE)
		
		return 0;
	}

	/* TODO depending on goldbars in boot */
	if(!strat_infos.treasure_in_boot && zone_num == ZONE_SHIP_OUR_HOLD)
		return 0;

	return 1;
}

/* return new work zone, -1 if any zone is found */
int8_t strat_get_new_zone(void)
{
	uint16_t prio_max = 0;
	int8_t zone_num = -1;
	int8_t i;
	
	/* evaluate zones */
	for(i=0; i < ZONES_MAX; i++) 
	{
		/* check if is a valid zone */
		if(!strat_is_valid_zone(i))	
			continue;

		/* check priority */
		if(strat_infos.zones[i].prio > prio_max) {

			/* update zone candidate params */
			prio_max = strat_infos.zones[i].prio;
			zone_num = i;
		}
	}

	return zone_num;
}

/* return END_TRAJ if zone is reached, err otherwise */
uint8_t strat_goto_zone(uint8_t zone_num)
{
	//double d_rel = 0.0, a_rel_rad = 0.0;
	//uint8_t arm_type = 0;
	int8_t err;

	/* special cases */
	if(zone_num == ZONE_TOTEM_OPP_SIDE_2) {

		err = goto_and_avoid_forward(COLOR_X(strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	}

	else if(zone_num == ZONE_MIDDLE_COINS_GROUP) {

		if(position_get_x_s16(&mainboard.pos) > (AREA_X/2))
			err = goto_and_avoid_forward((AREA_X - strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
		else
			err = goto_and_avoid_forward(strat_infos.zones[zone_num].init_x, strat_infos.zones[zone_num].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	}

	else if(zone_num == ZONE_SHIP_OUR_DECK_2) {

		/* first goto repick the treasure aparted */		
		err = goto_and_avoid_forward(COLOR_X(strat_infos.zones[ZONE_SAVE_TREASURE].init_x), strat_infos.zones[ZONE_SAVE_TREASURE].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
      if(!TRAJ_SUCCESS(err))
         return err;

      err=strat_pickup_coins_floor(COLOR_X(strat_infos.zones[ZONE_SAVE_TREASURE].x),strat_infos.zones[ZONE_SAVE_TREASURE].y,GROUP);
      if(!TRAJ_SUCCESS(err))
         return err;


		/* goto ship deck init point */
		err = goto_and_avoid_forward(COLOR_X(strat_infos.zones[ZONE_SHIP_OUR_DECK_2].init_x), strat_infos.zones[ZONE_SHIP_OUR_DECK_2].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	}

#if 0
	else if(zone_num == ZONE_OUR_BOTTLE_1) {

	/* TODO */

	}
	else if(zone_num == ZONE_SHIP_OPP_DECK_2 
			&& (strat_infos.zones[ZONE_SAVE_TREASURE].flags & ZONE_CHECKED)) {
		
		/* TODO: goto move the orphan coin first, and goto deck forward with fingers opened */

	}
#endif


	/* by default */
	else {
		err = goto_and_avoid(COLOR_X(strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	}

	return err;
}


/* return END_TRAJ if the work is done, err otherwise */
uint8_t strat_work_on_zone(uint8_t zone_num)
{
	uint8_t err = END_TRAJ;
	int16_t x = strat_infos.zones[zone_num].x;
	int16_t y = strat_infos.zones[zone_num].y;

#ifdef DEBUG_STRAT_SMART
	return END_TRAJ;
#endif

	if(strat_infos.zones[zone_num].type == ZONE_TYPE_TOTEM) {
		err = strat_empty_totem_side(COLOR_X(x), y, STORE_BOOT, 0);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_GOLDBAR) {
		err = strat_pickup_goldbar_floor(COLOR_X(x), y, STORE_BOOT);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_MAP) {
		/* TODO */
		err = END_TRAJ;
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_BOTTLE) {
		if(zone_num == ZONE_OUR_BOTTLE_1)
			x = x + 80;

		err = strat_send_message_bottle(COLOR_X(x), y);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_COIN) {
		err = strat_pickup_coins_floor(COLOR_X(x), y, ONE);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_COINS_GROUP) {
		err = strat_pickup_coins_floor(COLOR_X(x), y, GROUP);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_HOLD) {
		err = strat_save_treasure_in_hold_back(COLOR_X(x), y);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_DECK) {

		err = strat_save_treasure_in_deck_back_blowing(COLOR_X(x), y);
		err = strat_save_treasure_in_deck_back_blowing(COLOR_X(x), y);

		//err = strat_save_treasure_generic(COLOR_X(x), y);
		if(mainboard.our_color == I2C_COLOR_PURPLE)
			err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_RIGHT);
		else
			err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_LEFT);


	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_CAPTAINS_BEDROOM) {
		/* TODO */
		err = END_TRAJ;
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_SAVE) {
		err = strat_save_treasure_generic(COLOR_X(x), y);
	}

	return err;

}

/* debug state machines step to step */
void state_debug_wait_key_pressed(void)
{
	if (!strat_infos.debug_step)
		return;

	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* smart play */
void strat_smart(void)
{
	int8_t zone_num;
	uint8_t err;

	/* XXX DEBUG STEP BY STEP */
	state_debug_wait_key_pressed();

	/* get new zone */
	zone_num = strat_get_new_zone();
	if(zone_num == -1) {
		DEBUG(E_USER_STRAT, "No zone is found");
		return;
	}

	/* goto zone */
	strat_infos.goto_zone = zone_num;
	strat_dump_infos(__FUNCTION__);
	
	err = strat_goto_zone(strat_infos.goto_zone);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT, "Can't reach zone %d", strat_infos.goto_zone);
		return;
	}

	/* work on zone */
	strat_infos.last_zone = strat_infos.current_zone;
	strat_infos.current_zone = strat_infos.goto_zone;
	strat_dump_infos(__FUNCTION__);

	err = strat_work_on_zone(strat_infos.current_zone);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT, "Work on zone %d fails", strat_infos.current_zone);
		return;
	}

	/* check the zone */

 	/* don't check if it's saving zone */
	if(strat_infos.zones[strat_infos.current_zone].type == ZONE_TYPE_HOLD 
		|| strat_infos.zones[strat_infos.current_zone].type == ZONE_TYPE_DECK
		|| strat_infos.zones[strat_infos.current_zone].type == ZONE_TYPE_CAPTAINS_BEDROOM) {
		return;
	}
	else	{
		strat_infos.zones[strat_infos.current_zone].flags |= ZONE_CHECKED;
	}

	DEBUG(E_USER_STRAT, "Work on zone %d successed!", strat_infos.current_zone);
}

