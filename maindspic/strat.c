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

	if(type == AREA_BBOX_6X5) {
		/* area 6x5 */
		strat_infos.area_bbox.x1 = 625-25;
		strat_infos.area_bbox.y1 = 220;
		strat_infos.area_bbox.x2 = 2375+25;
		strat_infos.area_bbox.y2 = 1575;
	}
	else {
		/* area 4x4	*/
		strat_infos.area_bbox.x1 = 945;
		strat_infos.area_bbox.y1 = 495;
		strat_infos.area_bbox.x2 = 2055;
		strat_infos.area_bbox.y2 = 1575;
	}

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

	/* flags */
	printf(PSTR("line1: \r\n"));
	if(strat_infos.conf.flags & LINE1_TOKENS_ON_BONUS_OLD)
		printf(PSTR(" TOKENS_ON_BONUS_OLD \r\n"));
	else if(strat_infos.conf.flags & LINE1_TOKENS_NEAR_WALL)
		printf(PSTR(" TOKENS_NEAR_WALL \r\n"));
	else
		printf(PSTR(" TOKENS_ON_BONUS \r\n"));	

	if(strat_infos.conf.flags & LINE1_OPP_TOKEN_BEFORE_PLACE)
		printf(PSTR(" OPP_TOKEN_BEFORE_PLACE \r\n"));
	if(strat_infos.conf.flags & LINE1_OPP_TOKEN_AFTER_PLACE)
		printf(PSTR(" OPP_TOKEN_AFTER_PLACE \r\n"));
	else if((strat_infos.conf.flags & (LINE1_OPP_TOKEN_BEFORE_PLACE | LINE1_OPP_TOKEN_AFTER_PLACE)) == 0)
		printf(PSTR(" NOT PLACE OPP_TOKEN \r\n"));

	/* place thresholds */
	printf(PSTR("place thresholds: \r\n"));
	printf(PSTR(" th_place_prio = %d \r\n"), strat_infos.conf.th_place_prio);
	printf(PSTR(" th_token_score = %d \r\n"),strat_infos.conf.th_token_score);

	
}

int8_t strat_print_flag(uint8_t i, uint8_t j)
{

	/* robot slot position */
	if(strat_infos.slot[i][j].flags & SLOT_ROBOT)
		return 'R';	

	/* opponent slot position */
	if(strat_infos.slot[i][j].flags & SLOT_OPPONENT)
		return 'P';	

	/* slot busy */
	if(strat_infos.slot[i][j].flags & SLOT_BUSY)
		return 'O';	

	/* slot checked */
	if(strat_infos.slot[i][j].flags & SLOT_CHECKED)
		return 'X';	

	/* slot figure */
	if(strat_infos.slot[i][j].flags & SLOT_FIGURE)
		return 'F';	


	/* green area */
	if( (i == 0 || i == 7) && j > 0)
		return ' '; 

	/* any slot */
	return '_'; 

}

/* display current information about the state of the game */
void strat_dump_infos(const char *caller)
{
	int8_t j;

	if (!strat_infos.dump_enabled)
		return;

	printf(PSTR("%s() dump strat infos:\r\n"), caller);

	/* slots flags */
	for(j=5; j>=0; j--) { 

		/* first column: playground info */

		//	printf(PSTR("y  _______________  \r\n"));
		//	printf(PSTR("5 | |___|_|_|___| | \r\n"));
		//	printf(PSTR("4 | |_|_|_|_|_|_| | \r\n"));
		//	printf(PSTR("3 | |P|_|_|X|O|X| | \r\n"));
		//	printf(PSTR("2 | |_|_|_|O|R|O| | \r\n"));
		//	printf(PSTR("1 |_|_|_|_|X|O|X|_| \r\n"));
		//	printf(PSTR("0 |_|_|_|_|_|_|_|_| \r\n"));
		//	printf(PSTR("   0 1 2 3 4 5 6 7 x\r\n"));

		if(j==5) {
			printf(PSTR("y  _______________  \r\n"));
			printf(PSTR("%d |%c|%c_%c|%c|%c|%c_%c|%c|"),
					j,
					strat_print_flag(0, j),
					strat_print_flag(1, j),
					strat_print_flag(2, j),
					strat_print_flag(3, j),
					strat_print_flag(4, j),
					strat_print_flag(5, j),
					strat_print_flag(6, j),
					strat_print_flag(7, j) );
		}
		else {
			printf(PSTR("%d |%c|%c|%c|%c|%c|%c|%c|%c|"),
					j,
					strat_print_flag(0, j),
					strat_print_flag(1, j),
					strat_print_flag(2, j),
					strat_print_flag(3, j),
					strat_print_flag(4, j),
					strat_print_flag(5, j),
					strat_print_flag(6, j),
					strat_print_flag(7, j) );
		}

		/* second column */
		switch(j) {
			case 5:
				/* tokens catched */
				printf(PSTR("  num_tokens = %d"), (token_catched(SIDE_FRONT) + token_catched(SIDE_REAR)) );
				break;
			case 4:
				printf(PSTR("  num_towers = %d"), strat_infos.num_towers);
				break;
			case 3:
				/* robot slot position */
				printf(PSTR("  robot_slot = (%d,%d) -> (%d,%d)"),
						strat_infos.slot_before.i, strat_infos.slot_before.j,
						strat_infos.slot_actual.i, strat_infos.slot_actual.j);
				break;
			case 2:
				/* opponent slot position */
				printf(PSTR("  opp_slot = (%d,%d) -> (%d,%d)"),
						strat_infos.opp_slot_before.i, strat_infos.opp_slot_before.j,
						strat_infos.opp_slot_actual.i, strat_infos.opp_slot_actual.j);
				break;
			case 1:
				printf(PSTR("  opp_zone = (%d) -> (%d, %ld ms)"),
						strat_infos.opp_before_zone, strat_infos.opp_actual_zone, strat_infos.opp_time_zone_ms);
				break;
			case 0:
				break;
			default:
				break;
		}

		printf(PSTR(" \r\n"));
	}

	printf(PSTR("   0 1 2 3 4 5 6 7 x \r\n"));


	/* towers found info */
	if(strat_infos.num_towers) {
		printf(PSTR(" tower n: i, j, x, y, w c \r\n"));		
		for(j = 0; j < strat_infos.num_towers; j++) {
			printf(PSTR(" tower %d: %d, %d, %.4d, %.4d, %.3d %.2d"), j+1,
					 strat_infos.towers[j].i, strat_infos.towers[j].j,
					 strat_infos.towers[j].x, strat_infos.towers[j].y,
					 strat_infos.towers[j].w, strat_infos.towers[j].c);
			printf(PSTR(" \r\n"));
		}		
	}

	/* figures found info */
	for(j = 1; j < 6; j++) {
		if(strat_infos.slot[0][j].flags & SLOT_FIGURE)
			printf(PSTR(" figure @ %d\n\r"), j);	
	}

	/* zones visited by opponent */
	for(j = 0; j < NB_ZONES_MAX; j++) {
		if(strat_infos.zones[j].num_visits) {

			printf(PSTR(" zone %d: %d times, %ld ms\n\r"), j,
							 strat_infos.zones[j].num_visits,  strat_infos.zones[j].total_time_ms);
		}
	}
}

/* init current area state before a match. Dump update user conf
 * here */
void strat_reset_infos(void)
{
	uint8_t i, j;

	/* bounding box */
	strat_set_bounding_box(AREA_BBOX_4X4);
	
	/* reset flags of slots */
	for(i=0; i<NB_SLOT_X; i++) {
		for(j=0; j<NB_SLOT_Y; j++) {
			strat_infos.slot[i][j].flags = 0;
		}
	}

	/* specific flags */
	strat_infos.slot[1][5].flags = SLOT_SAFE;
	strat_infos.slot[2][5].flags = SLOT_SAFE;
	strat_infos.slot[5][5].flags = SLOT_SAFE;
	strat_infos.slot[6][5].flags = SLOT_SAFE;

	strat_infos.slot[0][1].flags = SLOT_BUSY;
	strat_infos.slot[0][2].flags = SLOT_BUSY;
	strat_infos.slot[0][3].flags = SLOT_BUSY;
	strat_infos.slot[0][4].flags = SLOT_BUSY;
	strat_infos.slot[0][5].flags = SLOT_BUSY;

	strat_infos.slot[7][1].flags = SLOT_BUSY;
	strat_infos.slot[7][2].flags = SLOT_BUSY;
	strat_infos.slot[7][3].flags = SLOT_BUSY;
	strat_infos.slot[7][4].flags = SLOT_BUSY;
	strat_infos.slot[7][5].flags = SLOT_BUSY;

	/* slot position */
	if(get_color() == I2C_COLOR_BLUE) {
		strat_infos.slot_actual.i = 0;
		strat_infos.slot_actual.j = 0;

		strat_infos.opp_slot_actual.i = 7;
		strat_infos.opp_slot_actual.j = 0;
	}
	else {
		strat_infos.slot_actual.i = 7;
		strat_infos.slot_actual.j = 0;

		strat_infos.opp_slot_actual.i = 0;
		strat_infos.opp_slot_actual.j = 0;
	}
	strat_infos.slot_before = strat_infos.slot_actual;
	strat_infos.opp_slot_before = strat_infos.opp_slot_actual;

	/* towers found */
	strat_infos.num_towers = 0;
	memset(&strat_infos.towers, 0, sizeof(strat_infos.towers));	

	/* opponet zone */
	strat_infos.opp_actual_zone = ZONE_OPP_NEAR_HOME;
	strat_infos.opp_before_zone = ZONE_OPP_NEAR_HOME;
}

/* call it just before launching the strat */
void strat_init(void)
{
	strat_reset_infos();

	/* we consider that the color is correctly set */

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	time_reset();
	interrupt_traj_reset();

	/* lasers off */
	lasers_set_off();

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

	/* power off lasers */
	lasers_set_off();

	/* stop slavespic */
	i2c_slavedspic_mode_token_stop(SIDE_FRONT);
	i2c_slavedspic_mode_token_stop(SIDE_REAR);

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

	/* update robot slot position */
	strat_update_slot_position(TYPE_ROBOT, GRID_MARGIN, 
									     0, NB_GRID_LINES_X-1,
									     0, NB_GRID_LINES_Y-1);	

	/* update opponent slot position */
	strat_update_slot_position(TYPE_OPPONENT, GRID_MARGIN, 
									     2, 6,						/* central rectangle (includes bonus near wall and half safe zones */
									     1, 6);	
	strat_update_slot_position(TYPE_OPPONENT, 100, 
									     0, 8,						/* near wall */
									     0, 1);	
	strat_update_slot_position(TYPE_OPPONENT, 100, 
									     0, 2,						/* near green blue */
									     0, NB_GRID_LINES_Y-1);	
	strat_update_slot_position(TYPE_OPPONENT, 100, 
									     6, 8,						/* near green red */
									     0, NB_GRID_LINES_Y-1);	


	/* update zones */
	strat_update_zones(); 

	/* manage mirrors position */
	mirrors_state_machine();
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


/* begining trajs related with static elements */
uint8_t strat_beginning(void)
{
	uint8_t err = 0;
	uint16_t old_spdd, old_spda;

	/* set new speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* go out of start position */
	trajectory_d_rel(&mainboard.traj, 250);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);

	/* pick & place tokens on line 1 */
	err = strat_harvest_line1();
	if (!TRAJ_SUCCESS(err)) {
		wait_until_opponent_is_far();
		//err = strat_harvest_green_area_smart(get_opponent_color());
		//ERROUT(err);		
	}

	/* pick & place tokens on line 2 */
	err = strat_harvest_line2();
	if (!TRAJ_SUCCESS(err) && !opp_x_is_more_than(975)) {
		wait_until_opponent_is_far();
		//err = strat_harvest_green_area_smart(get_opponent_color());
		//ERROUT(err);		
	}

	/* pick & place tokens on green area */
	if((strat_infos.conf.flags & GREEN_OPP_ZONE_FIRST))
		err = strat_harvest_green_area_smart(get_opponent_color());
	else
		err = strat_harvest_green_area_smart(get_color());		
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);

//end:
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* strat main loop */
uint8_t strat_main(void)
{
	uint8_t err; //i, why=0;
//	uint8_t side;

	/* pick & place our static tokens */
	err = strat_beginning();

//#ifndef HOMOLOGATION
//	/* place two token on the other side */
//	err = strat_bonus_point();
//#endif

#ifdef TEST_EXIT
	i2c_slavedspic_mode_token_take(SIDE_FRONT);
	i2c_slavedspic_mode_token_take(SIDE_REAR);
	lasers_set_on();
#endif

#ifdef PLACE_FIGURES_ON_OUR_SIDE
	/* place figures on our side */
	/* go near */
	err = goto_and_avoid(COLOR_X(strat_infos.slot[2][3].x),
									  	  strat_infos.slot[2][3].y, 
						  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);

	/* set place thresholds */
	strat_infos.conf.th_place_prio = SLOT_PRIO_GREEN;
	strat_infos.conf.th_token_score = PLACE_ALL_SCORE;

	/* place */
	err = strat_place_near_slots(0,0);
#endif

#ifdef HARVEST_OPP_GREEN_AREA
	err = strat_harvest_green_area_smart(get_opponent_color());		
#endif

#ifdef Q5_WAIT_OPP_TO_BUILD_A_TOWER
	/* go to waiting position */
	err = goto_and_avoid(COLOR_X(1675),1400, 
						  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);

	/* wait the moment */
	while( (time_get_s() < 40 && !opp_y_is_more_than(875))
			 || (time_get_s() < 60));

	/* go to near opp zone */
	err = goto_and_avoid(COLOR_X(2025),525, 
						  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);

	/* pickup tower */
	err = strat_pickup_or_push_near_slots(MODE_ALL);

	/* if no tokens */
	if(strat_get_num_tokens() == 0) {
		/* go to near opp zone */
		err = goto_and_avoid(COLOR_X(2025),1225, 
							  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
	
		/* pickup tower */
		err = strat_pickup_or_push_near_slots(MODE_ALL);
	}

	strat_infos.conf.th_place_prio = SLOT_PRIO_NEAR_GREEN;
	strat_infos.conf.th_token_score = NOPLACE_SCORE;
#endif

//#define PLAYOFF_8
#ifdef PLAYOFF_8

	strat_set_bounding_box(AREA_BBOX_6X5);

	/* place figure */
	err = goto_and_avoid(COLOR_X(strat_infos.slot[5][1].x),
									  	  strat_infos.slot[5][1].y, 
						  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);

	err = strat_place_token_auto(COLOR_X(strat_infos.slot[6][0].x),
									  	          strat_infos.slot[6][0].y, &side, GO_FORWARD);

	/* pickup/place green pion */
	err = strat_pickup_green_token(TYPE_PION, get_opponent_color());
	err = goto_and_avoid(COLOR_X(strat_infos.slot[5][4].x),
									  	  strat_infos.slot[5][4].y, 
						  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);

	if(token_side_score(SIDE_FRONT) >= token_side_score(SIDE_REAR))
		side = SIDE_REAR;
	else
		side = SIDE_FRONT;

	err = strat_place_token(COLOR_X(strat_infos.slot[5][5].x),
									  	     strat_infos.slot[5][5].y, side, GO_FORWARD);


	/* pickup/place green pion */
	err = strat_pickup_green_token(TYPE_PION, get_opponent_color());
	err = goto_and_avoid(COLOR_X(strat_infos.slot[6][2].x),
									  	  strat_infos.slot[6][2].y, 
						  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);

	if(token_side_score(SIDE_FRONT) >= token_side_score(SIDE_REAR))
		side = SIDE_REAR;
	else
		side = SIDE_FRONT;

	err = strat_place_token(COLOR_X(strat_infos.slot[5][1].x),
									  	     strat_infos.slot[5][1].y, side, GO_FORWARD);

	/* place green figure trie 1 */
	err = strat_pickup_green_token(TYPE_FIGURE, get_opponent_color());

	if(strat_get_num_tokens() < 2)
		err = strat_pickup_green_token(TYPE_FIGURE, get_opponent_color());

	if(opp_y_is_more_than(875)) {
		err = goto_and_avoid(COLOR_X(strat_infos.slot[4][1].x),
										  	  strat_infos.slot[4][1].y, 
							  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
	}
	else {
		err = goto_and_avoid(COLOR_X(strat_infos.slot[4][3].x),
										  	  strat_infos.slot[4][3].y, 
							  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
	}

	strat_set_bounding_box(AREA_BBOX_6X5);

	strat_infos.conf.th_place_prio = SLOT_PRIO_NEAR_GREEN;
	strat_infos.conf.th_token_score = NOPLACE_SCORE;

#endif


	/* autoplay depends on opponent */
	while (1) {
		err = strat_play_with_opp();

		if(err == END_TRAJ) {
			break;
		}
		
		/* check end of match */
		else if (err == END_TIMER) {
			DEBUG(E_USER_STRAT, "End of time");
			strat_exit();
			break;
		}
	}

//	strat_set_speed(1000, 1000);
//
////	while(time_get_s() < 80) {
//	
//		err = goto_and_avoid(COLOR_X(strat_infos.slot[5][1].x),
//											  	  strat_infos.slot[5][1].y, 
//								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
//
//		err = goto_and_avoid(COLOR_X(strat_infos.slot[5][3].x),
//											  	  strat_infos.slot[5][3].y, 
//								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
//
//		err = goto_and_avoid(COLOR_X(strat_infos.slot[4][3].x),
//											  	  strat_infos.slot[4][3].y, 
//								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
//
//		err = goto_and_avoid(COLOR_X(strat_infos.slot[4][1].x),
//											  	  strat_infos.slot[4][1].y, 
//								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);


//		if(opp_y_is_more_than(875)) {
//			err = goto_and_avoid(COLOR_X(strat_infos.slot[2][3].x),
//											  	  strat_infos.slot[2][3].y, 
//								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
//
//		}
//		else {
//			err = goto_and_avoid(COLOR_X(strat_infos.slot[2][1].x),
//											  	  strat_infos.slot[2][1].y, 
//								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
//		}
//	}

//		err = goto_and_avoid(COLOR_X(strat_infos.slot[5][1].x),
//											  	  strat_infos.slot[5][1].y, 
//								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);

//	while(time_get_s() < 80);
//	strat_set_speed(4000, 4000);


	/* try to place on bonus */
	while(1) {
		err = strat_big_final();
		err = wait_traj_end(TRAJ_FLAGS_STD);

		/* check end of match */
		if (err == END_TIMER) {
			DEBUG(E_USER_STRAT, "End of time");
			strat_exit();
			break;
		}
	}

	return END_TRAJ;
}

#endif /* HOST_VERSION */


