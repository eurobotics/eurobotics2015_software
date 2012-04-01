/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2012)
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
 *  Javier Baliñas Santos <javier@arc-robots.org> and Silvia Santano
 */
      	

/*Elements handler file*/

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


uint8_t strat_empty_totem_side(int16_t x, int16_t y, uint8_t store_goldbar)
{
   uint8_t err;
	uint16_t old_spdd, old_spda, d;
   #define D_TOTEM_TO_FIRST_LINE 450

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

   /*Turn until we face the totem*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*Open fingers*/
	DEBUG(E_USER_STRAT, "Open fingers floor and fingers totem.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_TOTEM);
   i2c_slavedspic_wait_ready();

   /*Go forward until first line of coins*/
   d=distance_from_robot(x,y);
	trajectory_d_rel(&mainboard.traj, d-D_TOTEM_TO_FIRST_LINE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   
   /*Close fingers floor*/
	DEBUG(E_USER_STRAT, "Close fingers floor to pickup first line of coins.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_COINS_ISLE);
   i2c_slavedspic_wait_ready();

   //XXX
   if(slavedspic.fingers_floor_blocked)
   { 
      i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_TOTEM);
      i2c_slavedspic_wait_ready();
   	trajectory_d_rel(&mainboard.traj, -10);
      i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_COINS_ISLE);
      i2c_slavedspic_wait_ready();
   }

   /*Go forward until blocking with totem*/
	DEBUG(E_USER_STRAT, "Going forward until blocking with totem");
	strat_set_speed(SPEED_DIST_VERY_SLOW,SPEED_ANGLE_VERY_SLOW);
	trajectory_d_rel(&mainboard.traj,200);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (err != END_BLOCKING)
			ERROUT(err);

   /*Go a little backward*/
	trajectory_d_rel(&mainboard.traj, -60);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST|END_BLOCKING);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	DEBUG(E_USER_STRAT, "Coins totem");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_COINS_TOTEM);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_COINS_TOTEM);
   i2c_slavedspic_wait_ready();

	DEBUG(E_USER_STRAT, "Pickup goldbar.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_GOLDBAR_TOTEM);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_GOLDBAR_TOTEM);
   i2c_slavedspic_wait_ready();

   /*Go a little backward*/
	trajectory_d_rel(&mainboard.traj, -50);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   i2c_slavedspic_mode_store(1,I2C_STORE_MODE_GOLDBAR_IN_BOOT);
   i2c_slavedspic_wait_ready();
   

	/* go backward to safe position*/
	trajectory_d_rel(&mainboard.traj, -PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}


uint8_t strat_pickup_coins_floor(int16_t x, int16_t y)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;
   #define D_TO_COIN 200

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
   
   /*Turn until we face the coin*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   
	
   /*Go forward to near the coin*/
	trajectory_d_rel(&mainboard.traj, abs(x-position_get_x_s16(&mainboard.pos)-D_TO_COIN));
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   /*Return if opponent is in front*/
	if(opponent_is_infront())  {
		ERROUT(END_OBSTACLE);
	}

   /*Open fingers floor*/
   /*Wait until they are open*/
	DEBUG(E_USER_STRAT, "Open fingers floor.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_COINS_FLOOR);
   i2c_slavedspic_wait_ready();

   /*If we are taking the coin near to the ship there is no space and we turn*/
   if(x==RED_FLOOR_COIN_3_X || x==PURPLE_FLOOR_COIN_3_X) {
   	trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(-90));
	   err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   	if (!TRAJ_SUCCESS(err))
   			ERROUT(err);
   }

   /*Go forward until we have the coins inside the robot*/
	strat_set_speed(SPEED_DIST_SLOW,SPEED_ANGLE_SLOW);
	trajectory_goto_xy_abs(&mainboard.traj, x+50, y);
	err = WAIT_COND_OR_TRAJ_END(x_is_more_than(x), TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	DEBUG(E_USER_STRAT, "Close fingers floor.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_COINS_FLOOR);
   i2c_slavedspic_wait_ready();


	/* go backward to safe distance from token */
	trajectory_d_rel(&mainboard.traj, -PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}


uint8_t strat_pickup_goldbar_floor(int16_t x, int16_t y, uint8_t store)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
   
   /*Turn until we face the goldbar*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
   /*Open fingers floor*/
	DEBUG(E_USER_STRAT, "Open fingers floor.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_GOLDBAR_FLOOR);
   i2c_slavedspic_wait_ready();

   /*Go forward to near the goldbar*/
	trajectory_d_rel(&mainboard.traj, abs(y-position_get_y_s16(&mainboard.pos)-150));
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);


   /*Go forward until we have the gold bar inside the robot*/
	strat_set_speed(SPEED_DIST_SLOW,SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, 150);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   if (!TRAJ_SUCCESS(err))
	   ERROUT(err);
	
	DEBUG(E_USER_STRAT, "Switch on turbine and take and store goldbar.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_GOLDBAR_FLOOR);
   i2c_slavedspic_wait_ready();
	
   i2c_slavedspic_mode_store(1,I2C_STORE_MODE_GOLDBAR_IN_BOOT);
   i2c_slavedspic_wait_ready();
	

	/*go backward to safe distance from token */
	trajectory_d_rel(&mainboard.traj, -PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}


uint8_t strat_send_message_bottle(int16_t x, int16_t y)
{
   uint8_t err, bottle_is_at_right;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

   /*Turn until position in front of the bottle (depending on which point we use to to send message
   we turn to abs 180 or 0 deg*/
   bottle_is_at_right = (position_get_x_s16(&mainboard.pos) > x);
   bottle_is_at_right ? trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(180)) : trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(0));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   
   /*go to the bottle*/
	trajectory_d_rel(&mainboard.traj, abs(x-position_get_x_s16(&mainboard.pos)));
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*Turn until we have bottle behind*/
   trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(-90));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*change to slow speed*/
   strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);

	/* go to blocking*/
	trajectory_d_rel(&mainboard.traj, -300);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (err != END_BLOCKING)
			ERROUT(err);
   
	/* go forward to safe distance from token */
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, PLACE_D_SAFE);
 	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}

//XXX not modified//

uint8_t strat_save_treasure_generic(int16_t x, int16_t y)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;
   int16_t d;
   /* Minimum distance to point to be able to open fingers*/
   #define MIN_D_TO_POINT 285

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

   /*Turn to point*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*go to the point*/
   //d= distance_from_robot_signed(x,y);
   if(d>0 && d<MIN_D_TO_POINT) d=-d;
	trajectory_d_rel(&mainboard.traj, d-100);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   
   /*Open fingers*/
	DEBUG(E_USER_STRAT, "Open floor fingers.");
	while(!cmdline_keypressed());

   
	/* go backward to safe distance from token */
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, -MIN_D_TO_POINT);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*Close fingers*/
	DEBUG(E_USER_STRAT, "Close floor fingers.");
	while(!cmdline_keypressed());

   end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}


uint8_t strat_save_treasure_in_deck_back(int16_t x, int16_t y)
{   
   uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

   /*Turn to abs 180 or 0 deg*/
   trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(0));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*go to the ship*/
	trajectory_d_rel(&mainboard.traj, -abs(x-position_get_x_s16(&mainboard.pos)));
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*Slave in mode empty*/
	DEBUG(E_USER_STRAT, "Mode empty.");
	while(!cmdline_keypressed());

	wait_ms(2000);

   /*Turn a little to each side to let tokens fall down*/
   trajectory_a_rel(&mainboard.traj,60);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   trajectory_a_rel(&mainboard.traj,-120);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   trajectory_a_rel(&mainboard.traj,60);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	wait_ms(1000);

	/* go to safe distance*/
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*End of mode empty*/
	DEBUG(E_USER_STRAT, "End of mode empty.");
	while(!cmdline_keypressed());

   end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}

uint8_t strat_save_treasure_in_hold_back(int16_t x, int16_t y)
{   
   uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

   /*Turn to abs 180 or 0 deg*/
   trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(0));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*Spread hook out*/
	DEBUG(E_USER_STRAT, "Spread hook out.");
	while(!cmdline_keypressed());

   /*Deactivate cs angle*/
   /**/

   /*Go backward until blocking*/
	trajectory_d_rel(&mainboard.traj, -300);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (err != END_BLOCKING)
			ERROUT(err);

   /*Activate cs angle*/
   /**/

   /*Lift hook to lift the lid*/
	DEBUG(E_USER_STRAT, "Lift hook to lift the lid.");
	while(!cmdline_keypressed());

   /*Slave in mode empty*/
	DEBUG(E_USER_STRAT, "Mode empty.");
	while(!cmdline_keypressed());
	wait_ms(3000);

   /*Put down hook*/
	DEBUG(E_USER_STRAT, "Put down hook.");
	while(!cmdline_keypressed());

	/* go to safe distance*/
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*Gather hook in*/
	DEBUG(E_USER_STRAT, "Gather hook in.");
	while(!cmdline_keypressed());

   /*End of mode empty*/
	DEBUG(E_USER_STRAT, "End of mode empty.");

   end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}


uint8_t strat_raise_window(uint8_t window)
{
   return 1;
}

uint8_t strat_steal_treasure_hold(void)
{
   return 1;
}


uint8_t strat_game(void)
{
   uint8_t err;

   DEBUG(E_USER_STRAT, "Game started!");
   DEBUG(E_USER_STRAT, "Going to group of coins");
   trajectory_goto_xy_abs(&mainboard.traj,strat_infos.zones[ZONE_FLOOR_COINS_GROUP].init_x, strat_infos.zones[ZONE_FLOOR_COINS_GROUP].init_y);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   err=strat_pickup_coins_floor(FLOOR_COINS_GROUP_X,FLOOR_COINS_GROUP_Y);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   err=strat_save_treasure_generic(2360,1700);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   DEBUG(E_USER_STRAT, "Going to totem 1");
   trajectory_goto_xy_abs(&mainboard.traj,strat_infos.zones[ZONE_TOTEM_1_SIDE_2].init_x, strat_infos.zones[ZONE_TOTEM_1_SIDE_2].init_y);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   err=strat_empty_totem_side(TOTEM_1_X,TOTEM_1_Y,0);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   

   DEBUG(E_USER_STRAT, "Going to coin 3");
   trajectory_goto_xy_abs(&mainboard.traj,strat_infos.zones[ZONE_PURPLE_FLOOR_COIN_3].init_x, strat_infos.zones[ZONE_PURPLE_FLOOR_COIN_3].init_y);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   err=strat_pickup_coins_floor(PURPLE_FLOOR_COIN_3_X,PURPLE_FLOOR_COIN_3_Y);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);


   DEBUG(E_USER_STRAT, "Going to pick up the group of coins apart");
   err=strat_pickup_coins_floor(2360,1700);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);


   DEBUG(E_USER_STRAT, "Going to save treasure in deck");
   err = strat_save_treasure_generic(2700,1000);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	DEBUG(E_USER_STRAT, "Press a key to continue with part 2");



   DEBUG(E_USER_STRAT, "Going to coin 1");
   trajectory_goto_xy_abs(&mainboard.traj,strat_infos.zones[ZONE_PURPLE_FLOOR_COIN_1].init_x, strat_infos.zones[ZONE_PURPLE_FLOOR_COIN_1].init_y);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   err=strat_pickup_coins_floor(PURPLE_FLOOR_COIN_1_X,PURPLE_FLOOR_COIN_1_Y);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   DEBUG(E_USER_STRAT, "Going to totem 1 side 1");
   trajectory_goto_xy_abs(&mainboard.traj,strat_infos.zones[ZONE_TOTEM_1_SIDE_1].init_x, strat_infos.zones[ZONE_TOTEM_1_SIDE_1].init_y);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   err=strat_empty_totem_side(TOTEM_1_X,TOTEM_1_Y,0);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   DEBUG(E_USER_STRAT, "Going to coin 2");
   trajectory_goto_xy_abs(&mainboard.traj,strat_infos.zones[ZONE_PURPLE_FLOOR_COIN_2].init_x, strat_infos.zones[ZONE_PURPLE_FLOOR_COIN_2].init_y);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   err=strat_pickup_coins_floor(PURPLE_FLOOR_COIN_2_X,PURPLE_FLOOR_COIN_2_Y);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   DEBUG(E_USER_STRAT, "Going to save treasure in deck");
   err = strat_save_treasure_generic(2700,1000);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   DEBUG(E_USER_STRAT, "Going to send message bottle");
   trajectory_goto_xy_abs(&mainboard.traj,strat_infos.zones[ZONE_PURPLE_BOTTLE_1].init_x, strat_infos.zones[ZONE_PURPLE_BOTTLE_1].init_y);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   err=strat_pickup_coins_floor(PURPLE_BOTTLE_1_X,BOTTLE_Y);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   

   DEBUG(E_USER_STRAT, "Mission completed!!");
	while(!cmdline_keypressed());

   end:
   return err;
}



