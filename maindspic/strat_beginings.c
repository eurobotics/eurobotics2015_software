/*  
 *  Copyright Droids, Microb Technology (2008)
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
 *  Revision : $Id: strat_static_columns.c,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
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
#include <pwm_servo.h>
#include <dac_mc.h>
#include <time.h>
#include <encoders_dspic.h>

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
#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_utils.h"
#include "strat_avoid.h"
#include "sensor.h"
#include "i2c_protocol.h"

#define ERROUT(e) do {				\
		err = e;			\
		goto end;			\
	} while(0)

#define BIG_DIST 5000

/*
 * must be called from start area.
 * get 4 static columns and build a temple on the disc
 */
uint8_t strat_first_tomato(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;

	DEBUG(E_USER_STRAT, "%s(%d)", __FUNCTION__);

	strat_get_speed(&old_spdd, &old_spda);

	i2c_slavedspic_mode_prepare_harvest_ball();

	/* go straight. total distance is less than 5 meters */
	strat_set_speed(8000, 8000);
	trajectory_d_rel(&mainboard.traj, BIG_DIST);

	/* when y > 50, break */
	err = WAIT_COND_OR_TRAJ_END(y_is_more_than(290), TRAJ_FLAGS_STD);
	if (TRAJ_SUCCESS(err)) /* we should not reach end */
		ERROUT(END_ERROR);
	else if (err)
		ERROUT(err);

	/* turn to 29° abs while going forward */
	DEBUG(E_USER_STRAT, "turn now");
	strat_set_speed(8000, 3000);
	trajectory_only_a_abs(&mainboard.traj, COLOR_A(29));

	/* when y > 100, check the presence of column 4 */
	err = WAIT_COND_OR_TRAJ_END(y_is_more_than(1050), TRAJ_FLAGS_STD);
	if (TRAJ_SUCCESS(err)) /* we should not reach end */
		ERROUT(END_ERROR);
	else if (err)
		ERROUT(err);

	interrupt_traj_reset();		

	ERROUT(END_TRAJ);
 
 end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

