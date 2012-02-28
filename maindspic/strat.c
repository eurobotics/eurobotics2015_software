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

   /*zones[W] =                       { x_up,          y_up, x_down,          y_down, init_x,         init_y, prio, flags }; */
	.zones[ZONE_OUR_TOTEM_SIDE_1]=     { COLOR_X(2300), 1070, COLOR_X(1500),   650   , COLOR_X(1900),  500, ZONE_PRIO_3, ZONE_WITH_TREASURE };
	.zones[ZONE_OUR_TOTEM_SIDE_2]=     { COLOR_X(2300), 1490, COLOR_X(1500),   1070  , COLOR_X(1900),  900, ZONE_PRIO_3, ZONE_WITH_TREASURE };
	.zones[ZONE_OUR_FLOOR_COIN_1]=     { COLOR_X(2100), 600,  COLOR_X(1900),   400   , COLOR_X(2000),  300, ZONE_PRIO_3, ZONE_WITH_TREASURE };
	.zones[ZONE_OUR_FLOOR_COIN_2]=     { COLOR_X(2300), 1100, COLOR_X(2100),   900   , COLOR_X(2400),  1000, ZONE_PRIO_3, ZONE_WITH_TREASURE };
	.zones[ZONE_OUR_FLOOR_COIN_3]=     { COLOR_X(2650), 1800, COLOR_X(2450),   1600  , COLOR_X(2350),  1700, ZONE_PRIO_3, ZONE_WITH_TREASURE };
	.zones[ZONE_OUR_FLOOR_GOLDBAR]=    { COLOR_X(2700), 1000, COLOR_X(2500),   650   , COLOR_X(2350),  785,  ZONE_PRIO_3, ZONE_WITH_TREASURE };
	.zones[ZONE_OUR_BOTTLE_1]=         { COLOR_X(2550), 2000, COLOR_X(2170),   1850  , COLOR_X(2360),  1850, ZONE_PRIO_3, ZONE_WITH_TREASURE };
	.zones[ZONE_OUR_BOTTLE_2]=         { COLOR_X(1307), 2000, COLOR_X(1027),   1850  , COLOR_X(1117),  1850, ZONE_PRIO_3, ZONE_WITH_TREASURE };
	.zones[ZONE_OUR_MAP]=              { COLOR_X(1800), 300,  COLOR_X(1500),      0  , COLOR_X(1600),  300,  ZONE_PRIO_3, ZONE_WITH_TREASURE };
	.zones[ZONE_FLOOR_COINS_GROUP]=    { COLOR_X(1700), 1950, COLOR_X(1300),   1450  , COLOR_X(1800),  1700, ZONE_PRIO_3, ZONE_WITH_TREASURE };
	.zones[ZONE_MIDDLE_FLOOR_GOLDBAR]= { COLOR_X(1650), 1453, COLOR_X(1350),   1253  , COLOR_X(1500),  1500, ZONE_PRIO_3, ZONE_WITH_TREASURE };

	.zones[ZONE_OPP_TOTEM_SIDE_1]=     { 3000-COLOR_X(2300), 1070, 3000-COLOR_X(1500),   650   , 3000-COLOR_X(1900),  500, ZONE_PRIO_1, ZONE_WITH_TREASURE };
	.zones[ZONE_OPP_TOTEM_SIDE_2]=     { 3000-COLOR_X(2300), 1490, 3000-COLOR_X(1500),   1070  , 3000-COLOR_X(1900),  900, ZONE_PRIO_1, ZONE_WITH_TREASURE };
	.zones[ZONE_OPP_FLOOR_COIN_1]=     { 3000-COLOR_X(2100), 600,  3000-COLOR_X(1900),   400   , 3000-COLOR_X(2000),  300, ZONE_PRIO_1, ZONE_WITH_TREASURE };
	.zones[ZONE_OPP_FLOOR_COIN_2]=     { 3000-COLOR_X(2300), 1100, 3000-COLOR_X(2100),   900   , 3000-COLOR_X(2400),  1000, ZONE_PRIO_1, ZONE_WITH_TREASURE };
	.zones[ZONE_OPP_FLOOR_COIN_3]=     { 3000-COLOR_X(2650), 1800, 3000-COLOR_X(2450),   1600  , 3000-COLOR_X(2350),  1700, ZONE_PRIO_1, ZONE_WITH_TREASURE };
	.zones[ZONE_OPP_FLOOR_GOLDBAR]=    { 3000-COLOR_X(2700), 1000, 3000-COLOR_X(2500),   650   , 3000-COLOR_X(2350),  785,  ZONE_PRIO_1, ZONE_WITH_TREASURE };
	
   /* ship zones */
   .zones[ZONE_SHIP_OUR_CAPTAINS_BEDRROM]= { COLOR_X(3000),     500,  COLOR_X(2500),       0     , 3000-COLOR_X(2300),   250, ZONE_PRIO_2, ZONE_WITH_TREASURE };
	.zones[ZONE_SHIP_OUR_DECK]=             { 3000-COLOR_X(350), 2000, 3000-COLOR_X(0),     1360  , 3000-COLOR_X(550) ,   1200, ZONE_PRIO_2, ZONE_WITH_TREASURE };
   .zones[ZONE_SHIP_OUR_LOADING_DECK]=     { 3000-COLOR_X(425), 1360, 3000-COLOR_X(0),     500   , 3000-COLOR_X(640) ,   600, ZONE_PRIO_2, ZONE_WITH_TREASURE };
	.zones[ZONE_SHIP_OPP_DECK]=             { COLOR_X(350),      2000, COLOR_X(0),          1360  , COLOR_X(550)      ,   1200, ZONE_PRIO_2, ZONE_WITH_TREASURE };
	.zones[ZONE_SHIP_OPP_LOADING_DECK]=     { COLOR_X(425),      1360, COLOR_X(0),          500   , COLOR_X(640)      ,   600, ZONE_PRIO_2, ZONE_WITH_TREASURE };
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


