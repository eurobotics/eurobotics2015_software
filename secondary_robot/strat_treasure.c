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
#include <pwm_mc.h>
#include <pwm_mc.h>
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



/* pick up the piece of fabric that discover the map */
uint8_t strat_pickup_map(void)
{
#define PICKUP_D				160
#define PICKUP_SPEED_SLOW	200
#define PICKUP_SPEED_FAST	2800

   uint8_t err;
	uint16_t old_spdd, old_spda;
   int16_t d;

	/* disable limit speed */
	strat_limit_speed_disable();

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

	/* turn to infron of map */
	trajectory_a_abs(&mainboard.traj, -90);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	/* prepare arm and teeth */
	arm_set_pos(ARM_POS_PICKUP_MAP);
	teeth_set_pos(TEETH_POS_OPEN);
	time_wait_ms(250);	

	/* go near map */
	d = position_get_y_s16(&mainboard.pos);
	trajectory_d_rel(&mainboard.traj, d-PICKUP_D);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);		

	/* disable torque */
	arm_disable_torque();

	/* go slow until blocking */
	d = position_get_y_s16(&mainboard.pos);
	strat_set_speed(PICKUP_SPEED_SLOW, old_spda);
	trajectory_d_rel(&mainboard.traj, PICKUP_D*2);
	//trajectory_d_rel(&mainboard.traj, d-(ROBOT_LENGTH/2));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	//if (!TRAJ_SUCCESS(err))
	//		ERROUT(err);		

	/* close teeth */
	teeth_set_pos(TEETH_POS_CLOSE);
	time_wait_ms(250);

	/* go backwards faster */
	strat_set_speed(PICKUP_SPEED_FAST, old_spda);
	trajectory_d_rel(&mainboard.traj, -PICKUP_D);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);		

	/* hold map */
	arm_set_pos(ARM_POS_HOLD_MAP);

end:	strat_set_speed(old_spdd, old_spda);	strat_limit_speed_enable(); 
  return err;
}

/* empty our totem down side and save treasure on ship */
uint8_t strat_empty_totem(void)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;


	/* disable limit speed */
	strat_limit_speed_disable();

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

#if 0
	/* go near orphan coin */
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1000), 357);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	/* go near totem */
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1460), 710);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
#endif

	/* turn depends on color */
	//if(mainboard.our_color == I2C_COLOR_PURPLE) {
		trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(0));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);
	//}

	/* move first coin */
	//trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1335), 710);
	trajectory_d_rel(&mainboard.traj, -125);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);


	/* turn depends on color and open comb */
	comb_set_pos(COMB_POS_OPEN);
	if(mainboard.our_color == I2C_COLOR_PURPLE) {
		trajectory_a_rel(&mainboard.traj, -180);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);
	}

	/* goto push a little the goldbar */
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
	if(mainboard.our_color == I2C_COLOR_PURPLE)
		trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1045), 710);
	else
		trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1045+130), 710);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	/* shoot goldbar */
	trajectory_a_rel(&mainboard.traj, COLOR_A_REL(-90));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	trajectory_a_rel(&mainboard.traj, COLOR_A_REL(140));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);


	/* go near group of treasure */
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(840), 710);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

end:	strat_set_speed(old_spdd, old_spda);	strat_limit_speed_enable(); 
  return err;
}

/* save treasure after empty totem */
uint8_t strat_save_treasure_on_ship(void)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;


	/* XXX disable limit speed */
	strat_limit_speed_disable();

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);

	/* goto ship */
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(250), 800);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	/* try to push treasure inside ship */
	trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(180));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(400), 800);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

#if 0
	trajectory_a_rel(&mainboard.traj, COLOR_A_REL(-270));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	trajectory_d_rel(&mainboard.traj, COLOR_SIGN(200));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

#else
	trajectory_d_rel(&mainboard.traj, COLOR_SIGN(-200));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
#endif

	/* get out of ship */
	comb_set_pos(COMB_POS_CLOSE);
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(700), 600);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

end:	strat_set_speed(old_spdd, old_spda);	strat_limit_speed_enable(); 
  return err;
}

