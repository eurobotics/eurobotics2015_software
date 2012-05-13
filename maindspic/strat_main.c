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
		err=strat_goto_xy_force(COLOR_X(900),1600);
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


#if 0
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
