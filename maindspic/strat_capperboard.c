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
 *  Javier Balias Santos <javier@arc-robots.org> and Silvia Santano and Miguel Ortiz
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


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)

/**
 *	Close clapper board
 *	return END_TRAJ if the work is done, err otherwise
 */
uint8_t strat_close_clapperboards (int16_t x, int16_t y, uint8_t side, uint8_t flags)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;

#define CLOSE_CLAPPER_A_ABS 180

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);

    /* XXX */
    if (strat_smart[MAIN_ROBOT].current_zone != ZONE_MY_CLAP_1)
   	    strat_limit_speed_disable ();

	strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);



	/* turn to in parallel with the claper oppenin the stick */
	if (side == SIDE_RIGHT) {

		trajectory_a_abs(&mainboard.traj, 180);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);
#ifndef HOST_VERSION
		pwm_servo_set(&gen.pwm_servo_oc2, 450);
#endif
		trajectory_a_rel(&mainboard.traj, CLOSE_CLAPPER_A_ABS);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);
	}
	else {

		trajectory_a_abs(&mainboard.traj, 0);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);

#ifndef HOST_VERSION
		pwm_servo_set(&gen.pwm_servo_oc1, 700);
#endif
		trajectory_a_rel(&mainboard.traj, -CLOSE_CLAPPER_A_ABS);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
		   ERROUT(err);
	}

end:
	/* end stuff and close sticks */
#ifndef HOST_VERSION
	pwm_servo_set(&gen.pwm_servo_oc1, 20);
	pwm_servo_set(&gen.pwm_servo_oc2, 0);
#endif
	strat_set_speed(old_spdd, old_spda);
   	strat_limit_speed_enable();
   	return err;
}


