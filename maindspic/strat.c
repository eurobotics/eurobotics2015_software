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

#ifndef HOST_VERSION
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

#endif

struct strat_infos strat_infos = { 
	/* conf */
	.conf = {
		.flags = 0,
	},

	/* grid slots 
	.slot[X][Y] = { .x,	.y ,  	.color,       		.prio,   					.flags, 			.flags_poly},  */
	.slot[0][0] = { 200,	200,		SLOT_BLUE,			SLOT_PRIO_GREEN,			0, 				0, },
	.slot[0][1] = { 200,	690,		SLOT_GREEN_BLUE,	SLOT_PRIO_GREEN,			SLOT_BUSY, 		0, },
	.slot[0][2] = { 200,	970,		SLOT_GREEN_BLUE,	SLOT_PRIO_GREEN,			SLOT_BUSY, 		0, },
	.slot[0][3] = { 200,	1250,		SLOT_GREEN_BLUE,	SLOT_PRIO_GREEN,			SLOT_BUSY, 		0, },
	.slot[0][4] = { 200,	1530,		SLOT_GREEN_BLUE,	SLOT_PRIO_GREEN,			SLOT_BUSY, 		0, },
	.slot[0][5] = { 200,	1810,		SLOT_GREEN_BLUE,	SLOT_PRIO_GREEN,			SLOT_BUSY, 		0, },

	.slot[1][0] = { 625,	175,		SLOT_RED, 			SLOT_PRIO_CORNER,			0, 				0, },
	.slot[1][1] = { 625,	525,		SLOT_BLUE,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[1][2] = { 625,	875,		SLOT_RED,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[1][3] = { 625,	1225,		SLOT_BLUE,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[1][4] = { 625,	1575,		SLOT_RED,			SLOT_PRIO_CORNER,			0, 				0, },
	.slot[1][5] = { 625,	1865+10,	SLOT_BLUE,			SLOT_PRIO_SAFE,			SLOT_SAFE,		0, },

	.slot[2][0] = { 975,	175,		SLOT_BLUE, 			SLOT_PRIO_WALL,			0, 				0, },
	.slot[2][1] = { 975,	525,		SLOT_RED,			SLOT_PRIO_BONUS,			0, 				0, },
	.slot[2][2] = { 975,	875,		SLOT_BLUE,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[2][3] = { 975,	1225,		SLOT_RED,			SLOT_PRIO_BONUS,			0, 				0, },
	.slot[2][4] = { 975,	1575,		SLOT_BLUE,			SLOT_PRIO_NEAR_SAFE,		0, 				0,	},
	.slot[2][5] = { 975, 1865+10,	SLOT_RED,			SLOT_PRIO_SAFE,			SLOT_SAFE, 		0, },

	.slot[3][0] = { 1325, 175,		SLOT_RED, 			SLOT_PRIO_WALL,			0, 				0, },
	.slot[3][1] = { 1325, 525,		SLOT_BLUE,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[3][2] = { 1325, 875,		SLOT_RED,			SLOT_PRIO_CENTER,			0,					0, },
	.slot[3][3] = { 1325, 1225,	SLOT_BLUE,			SLOT_PRIO_CENTER,			0, 				0, },
	.slot[3][4] = { 1325, 1575,	SLOT_RED,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[3][5] = { 1325, 1925,	SLOT_BLUE,			SLOT_PRIO_BONUS_WALL,	0, 				0, },

	.slot[4][0] = { 1675, 175,		SLOT_BLUE, 			SLOT_PRIO_WALL,			0, 				0, },
	.slot[4][1] = { 1675, 525,		SLOT_RED,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[4][2] = { 1675, 875,		SLOT_BLUE,			SLOT_PRIO_CENTER,			0, 				0, },
	.slot[4][3] = { 1675, 1225,	SLOT_RED,			SLOT_PRIO_CENTER,			0, 				0, },
	.slot[4][4] = { 1675, 1575,	SLOT_BLUE,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[4][5] = { 1675, 1925,	SLOT_RED,			SLOT_PRIO_BONUS_WALL,	0, 				0, },

	.slot[5][0] = { 2025, 175,		SLOT_RED, 			SLOT_PRIO_WALL,			0, 				0, },
	.slot[5][1] = { 2025, 525,		SLOT_BLUE,			SLOT_PRIO_BONUS,			0, 				0, },
	.slot[5][2] = { 2025, 875,		SLOT_RED,			SLOT_PRIO_PATH,			0, 				0, },
	.slot[5][3] = { 2025, 1225,	SLOT_BLUE,			SLOT_PRIO_BONUS,			0, 				0, },
	.slot[5][4] = { 2025, 1575,	SLOT_RED,			SLOT_PRIO_NEAR_SAFE,		0, 				0, },
	.slot[5][5] = { 2025, 1865+10,SLOT_BLUE,			SLOT_PRIO_SAFE,			SLOT_SAFE, 		0, },

	.slot[6][0] = { 2375, 175,		SLOT_BLUE, 			SLOT_PRIO_CORNER,			0, 				0, },
	.slot[6][1] = { 2375, 525,		SLOT_RED,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[6][2] = { 2375, 875,		SLOT_BLUE,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[6][3] = { 2375, 1225,	SLOT_RED,			SLOT_PRIO_NEAR_GREEN,	0, 				0, },
	.slot[6][4] = { 2375, 1575,	SLOT_BLUE,			SLOT_PRIO_CORNER,			0, 				0, },
	.slot[6][5] = { 2375, 1865+10,SLOT_RED,			SLOT_PRIO_SAFE,			SLOT_SAFE, 		0, },

	.slot[7][0] = { 2800, 200,		SLOT_RED,			SLOT_PRIO_GREEN,			0, 				0, },
	.slot[7][1] = { 2800, 690,		SLOT_GREEN_RED,	SLOT_PRIO_GREEN,			SLOT_BUSY, 		0, },
	.slot[7][2] = { 2800, 970,		SLOT_GREEN_RED,	SLOT_PRIO_GREEN,			SLOT_BUSY, 		0, },
	.slot[7][3] = { 2800, 1250,	SLOT_GREEN_RED,	SLOT_PRIO_GREEN,			SLOT_BUSY, 		0, },
	.slot[7][4] = { 2800, 1530,	SLOT_GREEN_RED,	SLOT_PRIO_GREEN,			SLOT_BUSY, 		0, },
	.slot[7][5] = { 2800, 1810,	SLOT_GREEN_RED,	SLOT_PRIO_GREEN,			SLOT_BUSY, 		0, },

	/* grid lines */
	.grid_line_x = { 0, 450, 800, 1150, 1500, 1850, 2200, 2550, 3000 },
	.grid_line_y = { 0, 350, 700, 1050, 1400, 1750, 2100 },

#ifdef ZONES_HAS_FNCS
	/* zones[] = x, y, x_up, y_up, x_down, y_down, num_visits, total_time_ms, do_before, do_after */
	.zones[ZONE_OPP_NEAR_HOME] = { 2025, 525, 	1500, 1050, 2550, 0, 		1, 0,
											 strat_place_figure_near_opp_home, NULL },
	.zones[ZONE_OPP_NEAR_SAFE] = { 2025, 1400, 	1500, 1750, 2550, 1050, 	1, 0,
											 strat_place_on_near_opp_safe_slot, strat_place_on_opp_safe_slot },
	.zones[ZONE_NEAR_HOME] 		= { 975, 525, 		450, 1050, 1500, 0, 			0, 0,
											 NULL, NULL },
	.zones[ZONE_NEAR_SAFE] 		= { 975, 1400, 	450, 1750, 1500, 1050, 		0, 0,
											 NULL, NULL },
	.zones[ZONE_WALL_BONUS] 	= { 1675, 1575, 	1150, 2100, 1850, 1750, 	0, 0,
											 strat_pickup_bonus_near_wall, NULL },
#else
	/* zones[] = x, y, x_up, y_up, x_down, y_down, num_visits, total_time_ms, do_before, do_after */
	.zones[ZONE_OPP_NEAR_HOME] = { 1500, 1050, 2550, 0, 			0, 0 },
	.zones[ZONE_OPP_NEAR_SAFE] = { 1500, 2100, 2550, 1050, 		0, 0 },
	.zones[ZONE_NEAR_HOME] 		= { 450,  1050, 1500, 0, 			0, 0 },
	.zones[ZONE_NEAR_SAFE] 		= { 450,  2100, 1500, 1050, 		0, 0 },
	.zones[ZONE_WALL_BONUS] 	= { 1150, 2100, 1850, 1750-175, 	0, 0 },

#endif
};


/*************************************************************/

/*                  INIT                                     */

/*************************************************************/
void strat_set_bounding_box(uint8_t type)
{
	strat_infos.area_bbox.x1 = 400 + OBS_CLERANCE;
	strat_infos.area_bbox.y1 = OBS_CLERANCE;
	strat_infos.area_bbox.x2 = 3000 - 400 - OBS_CLERANCE;
	strat_infos.area_bbox.y2 = 2000 - 44 - OBS_CLERANCE;

	polygon_set_boundingbox(strat_infos.area_bbox.x1,
				strat_infos.area_bbox.y1,
				strat_infos.area_bbox.x2,
				strat_infos.area_bbox.y2);
}

#ifndef HOST_VERSION

/* called before each strat, and before the start switch */
void strat_preinit(void)
{
	time_reset();
	interrupt_traj_reset();
	mainboard.flags =  DO_ENCODERS | DO_CS | DO_RS |
							 DO_POS | DO_BD | DO_POWER | DO_OPP;

	strat_dump_conf();
	strat_dump_infos(__FUNCTION__);
}

/* display curret strat configuration */
void strat_dump_conf(void)
{
	if (!strat_infos.dump_enabled)
		return;

	printf_P(PSTR("-- conf --\r\n"));

	/* add here configuration dump */
}


/* display current information about the state of the game */
void strat_dump_infos(const char *caller)
{
	if (!strat_infos.dump_enabled)
		return;

	printf(PSTR("%s() dump strat infos:\r\n"), caller);

	/* add here print infos */
}

/* init current area state before a match. Dump update user conf
 * here */
void strat_reset_infos(void)
{
	/* bounding box */
	strat_set_bounding_box(AREA_BBOX_4X4);

	/* add here other infos resets */
}

/* call it just before launching the strat */
void strat_init(void)
{
	strat_reset_infos();

	/* we consider that the color is correctly set */

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	time_reset();
	interrupt_traj_reset();

	/* init other devices (lasers...) */

	/* used in strat_base for END_TIMER */
	mainboard.flags = DO_ENCODERS | DO_CS | DO_RS | 
		DO_POS | DO_BD | DO_TIMER | DO_POWER | DO_OPP;
}


/* call it after each strat */
void strat_exit(void)
{
	uint8_t flags;

	/* stop robot, disable timer */
	mainboard.flags &= ~(DO_TIMER);
	strat_hardstop();
	time_reset();

	/* disable CS, and motors */
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_CS);
	dac_mc_set(LEFT_MOTOR, 0);
	dac_mc_set(RIGHT_MOTOR, 0);
	IRQ_UNLOCK(flags);

	/* stop beacon */
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_OPP);
	IRQ_UNLOCK(flags);
	beacon_cmd_beacon_off();

	/* slavespic exit */

	/* turn off other devices (lasers...) */

	/* stop beacon */
	beacon_cmd_beacon_off();
	beacon_cmd_beacon_off();
	beacon_cmd_beacon_off();
	beacon_cmd_beacon_off();
}

/* called periodically */
void strat_event(void *dummy)
{
	/* XXX in parallel with main strat, 
	 *	disable/enable events depends on case or protect with IRQ_LOCK.
	 */

	/* limit speed when opponent are close */
	strat_limit_speed();

}

/* dump state (every 5 s max) XXX */
#define DUMP_RATE_LIMIT(dump, last_print)		\
	do {						\
		if (time_get_s() - last_print > 5) {	\
			dump();				\
			last_print = time_get_s();	\
		}					\
	} while (0)


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)	


/* strat main loop */
uint8_t strat_main(void)
{
	return END_TRAJ;
}

#endif /* HOST_VERSION */


