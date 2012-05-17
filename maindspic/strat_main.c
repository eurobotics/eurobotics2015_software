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

#if 0
uint8_t strat_main_loop(void)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save speed and aceleration values */
	strat_get_speed(&old_spdd, &old_spda);

	{
		/* Group of coins */
		DEBUG(E_USER_STRAT, "Go to group of coins");
		trajectory_goto_xy_abs(&mainboard.traj,
			COLOR_X(strat_infos.zones[ZONE_FLOOR_COINS_GROUP].init_x),
			strat_infos.zones[ZONE_FLOOR_COINS_GROUP].init_y);
		err=wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	   if(!TRAJ_SUCCESS(err))
	      ERROUT(err);
	   i2c_slavedspic_wait_ready();
		DEBUG(E_USER_STRAT, "Pick up group of coins");
		strat_limit_speed_enable();
		err=strat_pickup_coins_floor(FLOOR_COINS_GROUP_X,FLOOR_COINS_GROUP_Y,GROUP);
	  	if (!TRAJ_SUCCESS(err))
			ERROUT(err);   	
		DEBUG(E_USER_STRAT, "Save group of coins");
		/* XXX */ /*err=strat_goto_xy_force(COLOR_X(900),1600);*/
	   if(!TRAJ_SUCCESS(err))
	   	ERROUT(err);
	   err=strat_save_treasure_generic(COLOR_X(700),1400);
	   if(!TRAJ_SUCCESS(err))
	   	ERROUT(err);
	}

	#define TRY_OPP_TOTEM_SIDE_1  0
	#define TRY_OPP_TOTEM_SIDE_2  1
	#define TRY_SEND_MESSAGE_2		2
	#define TRY_SEND_MESSAGE_1		3
	#define TRY_COINS_GROUP			4	
	#define TRY_OUR_TOTEM_SIDE_1	5	
	#define TRY_OUR_TOTEM_SIDE_2	6
	#define TRY_SAVE_SEA       	7
	#define TRY_SAVE_DECK			8
	#define TRY_SAVE_HOLD			9
	
	while(1)
	{
		
	}

end:
   /* restore speed values */
	strat_set_speed(old_spdd, old_spda);
   return err;
}



uint8_t try_opp_totem_side_2(void)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save speed and aceleration values */
	strat_get_speed(&old_spdd, &old_spda);
	if(!opponent_is_in_area(strat_infos.zones[opp_totem].x_up,
		                     strat_infos.zones[opp_totem].y_up,
		                     strat_infos.zones[opp_totem].x_down,
		                     strat_infos.zones[opp_totem].y_down))
	{
		/* Empty opponent-totem, side 2 */
	}

end:
   /* restore speed values */
	strat_set_speed(old_spdd, old_spda);
   return err;
}
#endif


/* return 1 if is a valid for work */
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
			&& zone_num != ZONE_SHIP_OUR_HOLD
			&& zone_num != ZONE_SHIP_OUR_DECK_1
			&& zone_num != ZONE_SHIP_OUR_DECK_2
			&& zone_num != ZONE_SAVE_TREASURE
			&& zone_num != ZONE_MIDDLE_COINS_GROUP
			&& zone_num != ZONE_OUR_BOTTLE_1
			&& zone_num != ZONE_OUR_BOTTLE_2 )
		
		return 0;
	}
	/* if we have not treasure on mouth, we have not to save treasure any where */
	else {
		if(zone_num == ZONE_SHIP_OUR_CAPTAINS_BEDRROM
		|| zone_num == ZONE_SHIP_OUR_HOLD
		|| zone_num == ZONE_SHIP_OUR_DECK_1
		|| zone_num == ZONE_SHIP_OUR_DECK_2
		|| zone_num == ZONE_SAVE_TREASURE)
		
		return 0;
	}

	/* TODO depending on goldbars in boot */

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

/* return END_TRAJ if zone is reached */
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

/* return END_TRAJ if the work is done */
uint8_t strat_work_on_zone(uint8_t zone_num)
{
	uint8_t err = END_TRAJ;
	int16_t x = strat_infos.zones[zone_num].x;
	int16_t y = strat_infos.zones[zone_num].y;

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
		err = strat_save_treasure_generic(COLOR_X(x), y);
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
