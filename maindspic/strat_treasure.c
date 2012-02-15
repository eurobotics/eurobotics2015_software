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


/*Points where the robot should get positioned on*/
#define D_EMPTY_TOTEM	        300 
#define BOTTLE_RED_1_X          2360
#define BOTTLE_RED_2_X          1117
#define BOTTLE_PURPLE_1_X       640
#define BOTTLE_PURPLE_2_X       1883
#define BOTTLE_Y                1800

#define PICKUP_SPEED		        1000 
#define PICKUP_CAUGHT_TIME	  	  100

typedef struct {
{
   uint16_t x;
   uint16_t y;
   uint8_t side[2];        /*Indicates whether each side of the two is empty(0) or not(1)*/
   uint8_t color;          /*color red(0), color purple(1)*/
}  totem;

totem totem_red, totem_purple;
totem_purple.x=TOTEM_1_X;
totem_purple.y=TOTEM_1_Y;
totem_red.x=TOTEM_2_X;
totem_red.y=TOTEM_2_Y;


typedef struct {
   uint16_t x;
   uint16_t y;
   uint8_t coin_color;     /*Black(0), white(1)*/
   uint8_t floor;          /*Is still in the floor (1), not(0)*/
} coin_floor;

uint8_t bottle[2];         /*Indicates whether each bottle of our two is sent(1) or not(0)*/

uint8_t strat_empty_totem_side(totem *tot, uint8_t side)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;
   uint16_t d_totem, totem_empty_point_x, totem_empty_point_y;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

	/* return if this side is already empty from this totem */
	if(!tot->side[side])
		ERROUT(END_TRAJ);
   
   /*Go to the totem. Stops on a defined point for each side of each totem.*/
   tot->color ? totem_empty_point_x=1100: totem_empty_point_x=1900;
   tot->side ? totem_empty_point_y=1450: totem_empty_point_y=550;
   trajectory_goto_xy_abs(&mainboard.traj,totem_empty_point_x,totem_empty_point_y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

   /*Turn until we face the totem*/
   trajectory_turnto_xy(&mainboard.traj,tot->x,tot->y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

   /*Spread upper and lower arms preparing to pickup coins*/
	//i2c_slavedspic_mode_coins_take();
   //wait until arms are opened
   i2cproto_wait_update();

   /*Set speed slow to get closer to the totem*/
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);

   /*Get closer enough to empty totem */
	d_totem = distance_from_robot(tot->x, tot->y);
	if(d_totem > D_EMPTY_TOTEM)
   {
		trajectory_d_rel(&mainboard.traj, d_token-D_EMPTY_TOTEM);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}

   /*Prepare turbine to catch the gold bar*/
	//i2c_slavedspic_mode_goldbar_take();
   //wait until is ready
   i2cproto_wait_update();

   /*Catch bar with the turbine*/
	strat_set_speed(PICKUP_SPEED, SPEED_ANGLE_FAST);
	//i2c_slavedspic_goldbar_take();


   /*Close upper and lower arms to catch coins*/
	strat_set_speed(PICKUP_SPEED, SPEED_ANGLE_FAST);
	//i2c_slavedspic_coins_take();


   /*Check if we got it*/
	//WAIT_COND_OR_TIMEOUT(coins_caught(), PICKUP_CAUGHT_TIME);
	//if(coins_caught(side)) {
		//DEBUG(E_USER_STRAT, "Coins caught!");
		//ERROUT(END_TRAJ);
   //}
	/*Not caught yet*/

   /*XXX Handle the tresure once it's caught. */


 end:
	/* side empty */
   tot->side[side]=0;
	strat_set_speed(old_spdd, old_spda);
	return err;
}


uint8_t strat_pickup_coins_floor(coin_floor *coin)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;
   uint16_t d_coin;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

	/* return if this side is already empty from this totem */
	if(!coin->floor)
		ERROUT(END_TRAJ);
   
   /*We suppose the robot is already on a good position to catch the coin (?)*/
   /*Turn until we face the coin*/
   trajectory_turnto_xy(&mainboard.traj,coin->x,coin->y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

   /*Turn until we face the coin*/
   trajectory_turnto_xy(&mainboard.traj,coin->x,coin->y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

   /*XXX Not finished*/

 end:
	/* coin picked */
	strat_set_speed(old_spdd, old_spda);
	return err;
}


uint8_t strat_pickup_goldbar_floor(int16_t x, int16_t y)
{
}


uint8_t strat_send_message_bottle(int8_t bot)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;
   uint16_t send_message_point_x, send_message_point_y;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

	/* return if this bottle message is already send */
	if(bottle[bot])
		ERROUT(END_TRAJ);
   
   /*Go to the bottle. Stops on a defined point.*/
   if (mainboard.our_color == I2C_COLOR_RED) 
      bot? send_message_point_x=BOTTLE_RED_1_X: send_message_point_x=BOTTLE_RED_2_X;
   else 
      bot? send_message_point_x=BOTTLE_PURPLE_1_X: send_message_point_x=BOTTLE_PURPLE_1_X;

   trajectory_goto_backward_xy_abs(&mainboard.traj,send_message_point_x,BOTTLE_Y);
	err = wait_traj_end(TRAJ_FLAGS);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

   /*Turn until we are looking at the opposite of the bottle*/
   trajectory_turnto_xy_behind(&mainboard.traj,send_message_point_x,BOTTLE_Y+300);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

   /*Set speed slow*/
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

   /*Go push the bottle */
	trajectory_d_rel(&mainboard.traj, 280);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

 end:
	/* message bottle sent */
   bottle[bot]=1;
	strat_set_speed(old_spdd, old_spda);
	return err;
}


uint8_t strat_pickup_goldbar_floor(int16_t x, int16_t y)
{
}


uint8_t strat_treasure_safe_place_front(uint8_t ship_point)
{
}


uint8_t strat_treasure_safe_place_back(uint8_t ship_point)
{
}


uint8_t strat_raise_window(uint8_t window)
{
}


uint8_t strat_steal_treasure_hold()
{
}