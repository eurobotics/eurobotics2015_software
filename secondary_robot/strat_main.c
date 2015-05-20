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
 *  Javier Bali?as Santos <javier@arc-robots.org> and Silvia Santano
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
#include <pwm_mc.h>
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
#include <scheduler.h>


#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"
//#include "../common/bt_commands.h"

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


/* interrupt a bt task */
void strat_bt_task_interrupt (void)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	mainboard.bt_task_interrupt = 1;
	interrupt_traj();
	IRQ_UNLOCK(flags);
}

/* reset bt task interrup */
void strat_bt_task_interrupt_reset (void)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	mainboard.bt_task_interrupt = 0;
	interrupt_traj_reset();
	IRQ_UNLOCK(flags);
}


/* set current bt task */
void strat_bt_task_rqst (uint8_t task_id,
						int16_t a, int16_t b,
						int16_t c, int16_t d, int16_t e)
{
	uint8_t flags;

	/* interrup current bt task */
	strat_bt_task_interrupt();

	/* new task request */
	IRQ_LOCK (flags);
	mainboard.bt_task_id_rqst = task_id;

	mainboard.bt_task_args[0] = a;
	mainboard.bt_task_args[1] = b;
	mainboard.bt_task_args[2] = c;
	mainboard.bt_task_args[3] = d;
	mainboard.bt_task_args[4] = e;

	mainboard.bt_task_new_rqst = 1;
	IRQ_UNLOCK (flags);
}

/* never returns */
void strat_bt_task_scheduler (void)
{
    uint8_t flags, ret = 0;
    microseconds us;

    /* init bt_task */
	strat_bt_task_rqst (BT_TASK_NONE, 0,0,0,0,0);

	while(1)
	{

		/* check if task request */
		IRQ_LOCK(flags);
		if (mainboard.bt_task_new_rqst) {
			mainboard.bt_task_id = mainboard.bt_task_id_rqst;
			mainboard.bt_task_id_rqst = 0;
			mainboard.bt_task_new_rqst = 0;

			strat_bt_task_interrupt_reset();
		}
		IRQ_UNLOCK(flags);

		/* continue if non task */
		if (mainboard.bt_task_id == BT_TASK_NONE)
			continue;

		/* get time mark */
		us = time_get_us2();

		/* schedule task */
		switch(mainboard.bt_task_id)
		{
			default:
				break;
			case BT_AUTO_POSITION:
				strat_auto_position ();
				ret = END_TRAJ;
				break;

			case  BT_TASK_PICK_CUP:
				ret = strat_pickup_cup (mainboard.bt_task_args[0], mainboard.bt_task_args[1]);
				break;

			case  BT_TASK_CARPET:
				ret = strat_put_carpets();
				break;

			case  BT_TASK_STAIRS:
				ret = climb_stairs();
				break;

			case  BT_TASK_BRING_CUP:
				ret = strat_release_cup (mainboard.bt_task_args[0], mainboard.bt_task_args[1]);
				break;

			case  BT_TASK_CLAP:
				ret = strat_close_clapperboard (mainboard.bt_task_args[0], mainboard.bt_task_args[1]);
				break;

			case  BT_GOTO:
				ret = wait_traj_end(TRAJ_FLAGS_STD);
				break;

			case BT_GOTO_AVOID_FW:
				ret= goto_and_avoid_forward(mainboard.bt_task_args[0], mainboard.bt_task_args[1],
                                            TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
				break;

			case BT_GOTO_AVOID_BW:
				ret= goto_and_avoid_backward(mainboard.bt_task_args[0], mainboard.bt_task_args[1],
                                            TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
				break;

			case BT_GOTO_AVOID:
				ret= goto_and_avoid(mainboard.bt_task_args[0], mainboard.bt_task_args[1],
                                    TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

				break;
		}

		/* parse end value */
		if (ret != END_INTR) {
			while (time_get_us2() - us < 200000L);
			bt_status_set_cmd_ret (ret);
			IRQ_LOCK(flags);
			mainboard.bt_task_id = BT_TASK_NONE;
			IRQ_UNLOCK(flags);
		}
	}
}
