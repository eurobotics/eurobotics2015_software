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


uint8_t strat_store_goldbar(uint8_t where);


uint8_t strat_empty_totem_side(int16_t x, int16_t y, uint8_t store_goldbar, uint8_t store_coins)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
   
   /*Turn until we face the totem*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

   /*Open arms*/
	printf_P(PSTR("Open lower arms.\r\n"));
	printf_P(PSTR("Open upper arms. Press a key...\r\n"));
	while(!cmdline_keypressed());
   /*Wait until they are open*/

   /*Cart in mode pickup goldbar*/
	printf_P(PSTR("Cart in mode pickup goldbar. Press a key...\r\n"));
	while(!cmdline_keypressed());

   /*Go forward until first line of coins*/
	trajectory_d_rel(&mainboard.traj, 100);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   
   /*Close lower arms*/
	printf_P(PSTR("Close lower arms to pickup first line of coins. Press a key...\r\n"));
	while(!cmdline_keypressed());

   /*Go forward until blocking with totem*/
	trajectory_d_rel(&mainboard.traj, 300);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (err != END_BLOCKING)
			ERROUT(err);

   /*Go a little backward*/
	trajectory_d_rel(&mainboard.traj, -50);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

   /*Put cart up*/
	printf_P(PSTR("Put cart up. Press a key...\r\n"));
	while(!cmdline_keypressed());

   /*Close upper arms*/
	printf_P(PSTR("Close upper arms. Press a key...\r\n"));
	while(!cmdline_keypressed());

   /*Open upper arms*/
	printf_P(PSTR("Open upper arms. Press a key...\r\n"));
	while(!cmdline_keypressed());
   /*Wait until they are open*/

   /*Cart in mode pickup goldbar*/
	printf_P(PSTR("Cart in mode pickup goldbar. Press a key...\r\n"));
	while(!cmdline_keypressed());

   /*Pickup goldbar*/
	printf_P(PSTR("Pickup goldbar. Press a key...\r\n"));
	while(!cmdline_keypressed());
   /*Wait until we have the gold bar*/

   /*Go a little backward*/
	trajectory_d_rel(&mainboard.traj, -100);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

   /*Store goldbar*/
   if(store_goldbar)
   {
      /*store_goldbar may be: DONT_STORE, STORE_FRONT, STORE_BACK*/
      strat_store_goldbar(store_goldbar);
   }
   
   /*Put cart up*/
   /*wait until cart is up*/
   /*close arms*/

	/* go backward to safe position*/
	strat_set_speed(500, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, -PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*Store coins*/
   if(store_coins)
   {
      /*store_coins may be: 0, 1, 2 (times)*/
      //strat_store_coins(store_coins);
   }

   end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}


uint8_t strat_pickup_coins_floor(int16_t x, int16_t y, uint8_t store)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
   
   /*Turn until we face the coin*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   
   /*Open lower arms*/
   /*Wait until they are open*/
	printf_P(PSTR("Open lower arms. Press a key...\r\n"));
	while(!cmdline_keypressed());

   /*Go forward to near the coin*/
	trajectory_d_rel(&mainboard.traj, 100);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   /*Return if opponent is in front*/
	if(opponent_is_infront())  {
		ERROUT(END_OBSTACLE);
	}

   /*Go forward until we have the coins inside the robot*/
   /*If we are taking the coin near to the ship there is no space and we turn*/
   if(x==RED_FLOOR_COIN_3_X || x==PURPLE_FLOOR_COIN_3_X) {
   	trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(-90));
	   err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   }

	trajectory_goto_xy_abs(&mainboard.traj, x+50, y);
	err = WAIT_COND_OR_TRAJ_END(x_is_more_than(x-50), TRAJ_FLAGS_SMALL_DIST);

   /*close lower arms*/
	printf_P(PSTR("Open lower arms. Press a key...\r\n"));
	while(!cmdline_keypressed());

   /*Store coins*/
   if(store)
   {
      /*store_coins may be: 0, 1, 2 (times)*/
      //strat_store_coins(store_coins);
   }

	/* go backward to safe distance from token */
	strat_set_speed(500, SPEED_ANGLE_FAST);
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
   
   /*Turn until we face the gold bar*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

   /*Open lower arms*/
	printf_P(PSTR("Open lower arms. Press a key...\r\n"));
	while(!cmdline_keypressed());
   /*Wait until they are open*/

   /*Go forward until we have the gold bar inside the robot*/
	trajectory_d_rel(&mainboard.traj, 300);
	//err = WAIT_COND_OR_TRAJ_END(token_catched(), TRAJ_FLAGS_SMALL_DIST);
   

   /*Close lower arms*/
	printf_P(PSTR("Close lower arms to center goldbar. Press a key...\r\n"));
	while(!cmdline_keypressed());
   /*Wait until they are closed*/

   /*Open lower arms*/
	printf_P(PSTR("Open lower arms. Press a key...\r\n"));
	while(!cmdline_keypressed());

   /*Store goldbar*/
   if(store)
   {
      /*store_goldbar may be: 0, FRONT, BACK*/
      strat_store_goldbar(store);
   }

   /*put cart up*/
	printf_P(PSTR("Put cart up. Press a key...\r\n"));
	while(!cmdline_keypressed());
   /*wait until cart is up*/

   /*close lower arms*/
	printf_P(PSTR("Close lower arms. Press a key...\r\n"));
	while(!cmdline_keypressed());

	/*go backward to safe distance from token */
	strat_set_speed(500, SPEED_ANGLE_FAST);
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
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   
   /*go to the bottle*/
	trajectory_d_rel(&mainboard.traj, abs(x-position_get_x_s16(&mainboard.pos)));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

   /*Turn until we have bottle behind*/
   trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(-90));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

   /*change to slow speed*/
   strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);

	/* go to blocking*/
	trajectory_d_rel(&mainboard.traj, -300);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (err != END_BLOCKING)
			ERROUT(err);
   
	/* go backward to safe distance from token */
	strat_set_speed(500, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}


uint8_t strat_save_treasure_in_deck_front(int16_t x, int16_t y)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

   /*Turn to abs 180 or 0 deg*/
   trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(180));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   
   /*Open arms*/
	printf_P(PSTR("Open lower arms. Press a key...\r\n"));
	while(!cmdline_keypressed());

   /*go to the ship*/
	trajectory_d_rel(&mainboard.traj, abs(x-position_get_x_s16(&mainboard.pos)));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

   
	/* go backward to safe distance from token */
	strat_set_speed(500, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, -PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*Close arms*/
	printf_P(PSTR("Open lower arms. Press a key...\r\n"));
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
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

   /*go to the ship*/
	trajectory_d_rel(&mainboard.traj, -abs(x-position_get_x_s16(&mainboard.pos)));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

   /*Slave in mode empty*/
	printf_P(PSTR("Mode empty. Press a key...\r\n"));
	while(!cmdline_keypressed());

	wait_ms(2000);

   /*Turn a little to each side to let tokens fall down*/
   trajectory_a_rel(&mainboard.traj,60);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   trajectory_a_rel(&mainboard.traj,-120);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   trajectory_a_rel(&mainboard.traj,60);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	wait_ms(1000);

	/* go to safe distance*/
	strat_set_speed(500, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*End of mode empty*/
	printf_P(PSTR("End of mode empty. Press a key...\r\n"));
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
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

   /*Spread finger out*/
	printf_P(PSTR("Spread finger out. Press a key...\r\n"));
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

   /*Lift finger to lift the lid*/
	printf_P(PSTR("Lift finger to lift the lid. Press a key...\r\n"));
	while(!cmdline_keypressed());

   /*Slave in mode empty*/
	printf_P(PSTR("Mode empty. Press a key...\r\n"));
	while(!cmdline_keypressed());
	wait_ms(3000);

   /*Put down finger*/
	printf_P(PSTR("Put down finger. Press a key...\r\n"));
	while(!cmdline_keypressed());

	/* go to safe distance*/
	strat_set_speed(500, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*Gather finger in*/
	printf_P(PSTR("Gather finger in. Press a key...\r\n"));
	while(!cmdline_keypressed());

   /*End of mode empty*/
	printf_P(PSTR("End of mode empty.\r\n"));

   end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}


uint8_t strat_raise_window(uint8_t window)
{
   return 1;
}


uint8_t strat_steal_treasure_hold()
{
   return 1;
}


uint8_t strat_store_goldbar(uint8_t where)
{
   switch(where){
      case STORE_FRONT:
      default:
         /*Put cart down to let gold bar in the floor*/
      	printf_P(PSTR("Turn cart to let gold bar in the floor. Press a key...\r\n"));
      	while(!cmdline_keypressed());
      	printf_P(PSTR("Put cart down. Press a key...\r\n"));
      	while(!cmdline_keypressed());
         break;

      case STORE_BACK:
         /*Turn cart to let gold bar inside the robot*/
      	printf_P(PSTR("Turn cart to let gold bar inside the robot. Press a key...\r\n"));
      	while(!cmdline_keypressed());
         break;

      case DONT_STORE:
         break;
   }
   return 1;
}

