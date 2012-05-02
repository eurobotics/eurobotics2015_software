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
	strat_set_speed(PICKUP_SPEED_SLOW, old_spda);
	trajectory_d_rel(&mainboard.traj, PICKUP_D*2);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	//if (!TRAJ_SUCCESS(err))
	//		ERROUT(err);		

	/* close teeth */
	teeth_set_pos(TEETH_POS_CLOSE);

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

