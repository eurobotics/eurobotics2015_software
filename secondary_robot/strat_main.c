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
#include "../maindspic/strat_avoid.h"
#include "../maindspic/strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"
#include "cmdline.h"


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)

/* Add here the main strategic, the inteligence of robot */


/* auto possition depending on color */
void strat_auto_position (void)
{
#define AUTOPOS_SPEED_FAST 	1000
#define BASKET_WIDTH		300

	uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save & set speeds */
	interrupt_traj_reset();
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(AUTOPOS_SPEED_FAST, AUTOPOS_SPEED_FAST);

	/* goto blocking to y axis */
	trajectory_d_rel(&mainboard.traj, -200);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (err == END_INTR)
		goto intr;
	wait_ms(100);

	/* set y */
	strat_reset_pos(DO_NOT_SET_POS, BASKET_WIDTH + ROBOT_CENTER_TO_BACK, 90);

	/* prepare to x axis */
	trajectory_d_rel(&mainboard.traj, 60);
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;

	trajectory_a_rel(&mainboard.traj, COLOR_A_REL(-90));
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;


	/* goto blocking to x axis */
	trajectory_d_rel(&mainboard.traj, -700);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (err == END_INTR)
		goto intr;
	wait_ms(100);

	/* set x and angle */
	strat_reset_pos(COLOR_X(ROBOT_CENTER_TO_BACK), DO_NOT_SET_POS, COLOR_A_ABS(0));

	/* goto start position */
	trajectory_d_rel(&mainboard.traj, 150);
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;
	wait_ms(100);

	/* restore speeds */
	strat_set_speed(old_spdd, old_spda);
	return;

intr:
	strat_hardstop();
	strat_set_speed(old_spdd, old_spda);
}

//////////////////////////////////////////////////////////////////////////////
// BT TASKS
//////////////////////////////////////////////////////////////////////////////

//TODO: inside these functions, current_task flag must be continuously checked
//in blocking functions to be able to react and stop tasks when a new one comes

uint8_t pick_popcorn_cup(void)
{
	uint8_t ret=0;
    printf_P(PSTR("pick_popcorn_cup\r\n"));
    return ret;
}


uint8_t extend_carpet(void)
{
	uint8_t ret=0;
    printf_P(PSTR("extend_carpet\r\n"));
    return ret;
}


uint8_t climb_stairs(void)
{
	uint8_t ret=0;
    printf_P(PSTR("climb_stairs\r\n"));
    return ret;
}


uint8_t bring_cup_to_cinema(void)
{
	uint8_t ret=0;
    printf_P(PSTR("bring_cup_to_cinema\r\n"));
    return ret;
}


uint8_t close_clapperboard(void)
{
	uint8_t ret=0;
    printf_P(PSTR("clapperboard\r\n"));
    return ret;
}


/******************  BT TASKS ************************************************/

#if 0
/* set current bt task */
void strat_set_bt_task (uint8_t task, int16_t )
{
	uint8_t flags;
	IRQ_LOCK (flags);
	maindspic.strat_bt_task = task;
	IRQ_UNLOCK (flags);
}

void strat_bt_task_interrupt (void)
{

}


/* never returns */
void strat_bt_task_scheduler (void)
{

	/* init bt_task */
	current_bt_task=0;
	strat_bt_goto_avoid_x = -1; strat_bt_goto_avoid_y = -1; strat_bt_goto_avoid_checksum = -1;


	while(1)
	{

		switch(current_bt_task)
		{
			case  BT_TASK_NONE:
			default:
				break;

			case  BT_TASK_PICK_CUP:
				ret=pick_popcorn_cup();
				break;

			case  BT_TASK_CARPET:
				ret=extend_carpet();
				break;

			case  BT_TASK_STAIRS:
				ret=climb_stairs();
				break;

			case  BT_TASK_BRING_CUP:
				ret=bring_cup_to_cinema();
				break;

			case  BT_TASK_CLAP:
				ret=close_clapperboard();
				break;

			case  BT_GOTO:

				//TODO: check task
				ret = wait_traj_end(TRAJ_FLAGS_STD);
				printf_P("BT_GOTO");

				break;

			case BT_GOTO_AVOID_FW:
				//TODO: check task???????????
				ret=bt_goto_and_avoid_forward(strat_bt_goto_avoid_x, strat_bt_goto_avoid_y, strat_bt_goto_avoid_checksum);
				strat_bt_goto_avoid_x = -1;
				strat_bt_goto_avoid_y = -1;
				strat_bt_goto_avoid_checksum = -1;

				break;

			case BT_GOTO_AVOID_BW:
				ret=bt_goto_and_avoid_backward (strat_bt_goto_avoid_x, strat_bt_goto_avoid_y, strat_bt_goto_avoid_checksum);
				strat_bt_goto_avoid_x = -1;
				strat_bt_goto_avoid_y = -1;
				strat_bt_goto_avoid_checksum = -1;

				break;

			case BT_GOTO_AVOID:

				printf_P("BT_GOTO_AVOID");

				ret=bt_goto_and_avoid (strat_bt_goto_avoid_x, strat_bt_goto_avoid_y, strat_bt_goto_avoid_checksum);
				strat_bt_goto_avoid_x = -1;
				strat_bt_goto_avoid_y = -1;
				strat_bt_goto_avoid_checksum = -1;

				printf_P("BT_GOTO_AVOID	: %d",ret);
				break;
		}

		// Return value from the functions indicating finish, to inform main robot.
		if(current_bt_task!=BT_TASK_NONE && ret != 0){

			printf_P("Sending...");
			time_wait_ms(200);
			bt_status_set_cmd_ret (ret);

			ret=0;
			current_bt_task=BT_TASK_NONE;
		}
	}
}

#endif



