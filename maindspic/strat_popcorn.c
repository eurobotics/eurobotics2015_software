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

#define ROBOT_CENTER_CUP_FRONT  0
#define ROBOT_CENTER_CUP_REAR   0

/** 
 *	Harvest orphan stands
 *	return END_TRAJ if the work is done, err otherwise 
 */

uint8_t strat_harvest_popcorn_cup (int16_t x, int16_t y, uint8_t side, uint8_t flags)
{
   	uint8_t err = 0;
	uint16_t old_spdd, old_spda;
	int16_t d = 0;

	/* set local speed, and disable speed limit */
	strat_get_speed (&old_spdd, &old_spda);
   	strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

	//DEBUG (E_USER_STRAT, "d = %d, a = %d", d, a);
	//state_debug_wait_key_pressed();

    /* prepare cup clamp */
    side == SIDE_FRONT? i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_READY):
                        i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_OPEN);

    /* turn to cup */
    side == SIDE_FRONT? trajectory_turnto_xy(&mainboard.traj, x, y):
                        trajectory_turnto_xy_behind(&mainboard.traj, x, y);

	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	
    
    /* go in clamp range */
    d = distance_from_robot(x, y);
    side == SIDE_FRONT? d = d-ROBOT_CENTER_CUP_FRONT-10 :
                        d = -(d-ROBOT_CENTER_CUP_REAR-50); 

	trajectory_d_rel(&mainboard.traj, d);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);	


    /* front cup: pick up, drop popcorns in side, and release the cup */
    /* rear cup: pick up */
    side == SIDE_FRONT? i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_CATCH_AND_DROP):
                        i2c_slavedspic_mode_ps (I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_CATCH);

    /* wait ready */
    side == SIDE_FRONT? 
    WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), POPCORN_FRONT_READY_TIMEOUT):
    WAIT_COND_OR_TIMEOUT(i2c_slavedspic_ps_test_status(STATUS_READY), POPCORN_REAR_READY_TIMEOUT);


end:
	/* end stuff */
	strat_set_speed(old_spdd, old_spda);	
   	strat_limit_speed_enable();
   	return err;
}




