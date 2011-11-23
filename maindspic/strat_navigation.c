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
#include "cmdline.h"

#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)

/* update slot position */
#define TYPE_ROBOT		0
#define TYPE_OPPONENT	1
#define GRID_MARGIN 		10

void strat_update_slot_position(uint8_t type, int16_t margin, 
									     int8_t x_line_init, int8_t x_line_end,
									     int8_t y_line_init, int8_t y_line_end)
{
	int16_t x,y;
	int8_t x_index = -1, y_index = -1;
	int8_t i;
	slot_index_t slot_actual;
	slot_index_t slot_before;
	int8_t flags;
	static microseconds opp_time_us = 0;

	/* depends on type */
	if(type == TYPE_ROBOT) {

		/* get robot possition */
		x = position_get_x_s16(&mainboard.pos);
		y = position_get_y_s16(&mainboard.pos);

		/* and slots infos */
		slot_actual = strat_infos.slot_actual;
		slot_before = strat_infos.slot_before;

	} 
	else { /* TYPE_OPPONENT */

		/* get opponent possition */
		if(get_opponent_xy(&x, &y) == -1) {

			/* return if opp not there */
			return;
		}

		/* and slots infos */
		slot_actual = strat_infos.opp_slot_actual;
		slot_before = strat_infos.opp_slot_before;	
	}


	/* get x grid index */
	for(i = x_line_init; i < x_line_end; i++) {

		if( (x > (strat_infos.grid_line_x[i] + margin)) &&
		    (x < (strat_infos.grid_line_x[i+1] - margin)) ) {
			
			x_index = i;
			break; 
		}
	}		

	/* get y grid index */
	for(i = y_line_init; i < y_line_end; i++) {

		if( (y > (strat_infos.grid_line_y[i] + margin)) &&
		    (y < (strat_infos.grid_line_y[i+1] - margin)) ) {
			
			y_index = i;
			break; 
		}
	}			

	/* return if not found both index */
	if( (x_index == -1) || (y_index == -1) )
		return;

	/* if it's changed */
	if( (slot_actual.i != x_index) ||
		 (slot_actual.j != y_index) ) {

		/* update slot index */
		slot_before = slot_actual;
		slot_actual.i = x_index; 
		slot_actual.j = y_index; 
		
		/* update slots index depends on type */
		if(type == TYPE_ROBOT) {

			IRQ_LOCK(flags);
	
			/* save last position */
			strat_infos.slot_before = slot_before;
			strat_infos.slot_actual = slot_actual;
			
			/* update flags */
			strat_infos.slot[strat_infos.slot_before.i][strat_infos.slot_before.j].flags &= ~(SLOT_ROBOT);
			strat_infos.slot[strat_infos.slot_actual.i][strat_infos.slot_actual.j].flags &= ~(SLOT_BUSY|SLOT_AVOID);
			strat_infos.slot[strat_infos.slot_actual.i][strat_infos.slot_actual.j].flags |= (SLOT_ROBOT|SLOT_CHECKED);
			
			IRQ_UNLOCK(flags);

		}
		else { /* TYPE_OPPONENT */

			if(time_get_us2() - opp_time_us < 320000UL)
				return;

			opp_time_us = time_get_us2();

			IRQ_LOCK(flags);
	
			/* save last position */
			strat_infos.opp_slot_before = slot_before;
			strat_infos.opp_slot_actual = slot_actual;
			
			/* update flags */
			strat_infos.slot[strat_infos.opp_slot_before.i][strat_infos.opp_slot_before.j].flags &= ~(SLOT_OPPONENT);
			strat_infos.slot[strat_infos.opp_slot_actual.i][strat_infos.opp_slot_actual.j].flags &= ~(SLOT_CHECKED|SLOT_BUSY|SLOT_FIGURE|SLOT_AVOID);
			strat_infos.slot[strat_infos.opp_slot_actual.i][strat_infos.opp_slot_actual.j].flags |= (SLOT_OPPONENT);
			
			IRQ_UNLOCK(flags);

		}

		DEBUG(E_USER_STRAT, "new %s slot (%d,%d), before (%d, %d)",
					type == TYPE_ROBOT? "ROBOT" : "OPPONENT",
					slot_actual.i, slot_actual.j,
					slot_before.i, slot_before.j);
	}

}

/* update opponent zone infos */
#define UPDATE_ZONES_PERIOD_MS	25	/* XXX syncronized with EVENT_PERIOD_STRAT */

void strat_update_zones(void) 
{
	int8_t i;
	int16_t x_opp, y_opp;
	uint8_t flags;

	/* get opponent position, return if not there */
	if(get_opponent_xy(&x_opp, &y_opp) == -1)
		return;

	/* get actual zone */
	for(i = 0; i < NB_ZONES_MAX; i++) {

		if(opponent_is_in_area(COLOR_X(strat_infos.zones[i].x_up), strat_infos.zones[i].y_up,
									  COLOR_X(strat_infos.zones[i].x_down), strat_infos.zones[i].y_down)) {
			break;
		}
	}

	/* return if no valid zone ? */
	if(i == NB_ZONES_MAX)
		return;

	/* special case: bonus wall zone */
	if(opponent_is_in_area(COLOR_X(strat_infos.zones[ZONE_WALL_BONUS].x_up), strat_infos.zones[ZONE_WALL_BONUS].y_up,
								  COLOR_X(strat_infos.zones[ZONE_WALL_BONUS].x_down), strat_infos.zones[ZONE_WALL_BONUS].y_down)) {
		i = ZONE_WALL_BONUS;
	}

	/* if zone has changed */
	if(i != strat_infos.opp_actual_zone) {

		IRQ_LOCK(flags);

		/* update zone index */
		strat_infos.opp_before_zone = strat_infos.opp_actual_zone;
		strat_infos.opp_actual_zone = i;

		/* update num visites */
		strat_infos.zones[i].num_visits ++;

		/* reset actual zone time */
		strat_infos.opp_time_zone_ms = 0;

		IRQ_UNLOCK(flags);
	}
	else { /* opponent continues in the same zone */

		IRQ_LOCK(flags);

		/* update total zone time */
		strat_infos.zones[i].total_time_ms += UPDATE_ZONES_PERIOD_MS;

		/* update actual zone time */
		strat_infos.opp_time_zone_ms += UPDATE_ZONES_PERIOD_MS;

		IRQ_UNLOCK(flags);
	}

}

uint8_t strat_play_with_opp(void)
{
#define GOTO_NEAR_OPP			0
#define WAITING_OPP				1
#define WORK_NEAR_OPP_HOME		2
#define WORK_NEAR_OPP_SAFE		3
#define WORK_NEAR_HOME			4
#define WORK_NEAR_SAFE			5
#define WORK_ON_BONUS_WALL		6
#define PICKUP_TOWERS			7

//#define DEBUG_PLAY
#ifdef DEBUG_PLAY
	/* debug */
	static uint8_t state = WORK_ON_BONUS_WALL;
#else
	static uint8_t state = 0;
#endif
	static uint8_t opp_side_actual;
	static uint8_t opp_side_before;
	uint8_t side = 0;
	uint16_t old_spdd = 0, old_spda = 0;
	int8_t th_place_prior_old = 0;
	int8_t th_token_score_old = 0;
	uint8_t err;
	uint8_t our_bonus_token_not_there = 0;

#ifdef LOOK_FOR_TOWERS
	int8_t i_tower, j_tower, i_place_tower, j_place_tower;
	uint8_t opp_actual_zone;
#endif

#define TIMEOUT_OPP_IS_WORKING	2000
#define TIMEOUT_OPP_WAS_WORKING	2000 
#define TIMEOUT_GOTO_NEAR_OPP		3000

#define TIMEOUT_RETURN				(90-10)
#define TIMEOUT_STOP_PLACE			(90-30)


#define ZONE_OUR_SIDE	0
#define ZONE_OPP_SIDE	1
	
//while(!cmdline_keypressed());

	/* at timeout return */
#ifndef DEBUG_PLAY
	if(time_get_s() > TIMEOUT_RETURN)
		return END_TRAJ;

	/* at timeout stop of place tokens */
	if(time_get_s() > TIMEOUT_STOP_PLACE) {
		strat_infos.conf.th_token_score = NOPLACE_SCORE;
	}
#endif

	/* play */
	switch(state) {

		case GOTO_NEAR_OPP:

			/* if opp is on opp near home zone */
			if(strat_infos.opp_actual_zone == ZONE_OPP_NEAR_HOME
					&& strat_infos.opp_time_zone_ms > TIMEOUT_GOTO_NEAR_OPP) {

				DEBUG(E_USER_STRAT, "GOTO_NEAR_OPP");

				/* goto beside side */
				err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[4][3].x),
														  		    strat_infos.slot[4][3].y, 
														  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
	
				if (!TRAJ_SUCCESS(err)) {
					break;
				}

//				/* work */
//				err = strat_pickup_or_push_near_slots(MODE_ALL);
//				if (!TRAJ_SUCCESS(err)) {
//					state = GOTO_NEAR_OPP;
//					break;
//				}

				state = WAITING_OPP;
			}
			/* if opp is on opp near safe zone */
			else if(strat_infos.opp_actual_zone == ZONE_OPP_NEAR_SAFE
					&& strat_infos.opp_time_zone_ms > TIMEOUT_GOTO_NEAR_OPP) {

				DEBUG(E_USER_STRAT, "GOTO_NEAR_OPP");

				/* goto beside side */
				err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[4][1].x),
														  		    strat_infos.slot[4][1].y, 
														  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					break;
				}

//				/* work */
//				err = strat_pickup_or_push_near_slots(MODE_ALL);
//				if (!TRAJ_SUCCESS(err)) {
//					state = GOTO_NEAR_OPP;
//					break;
//				}

				state = WAITING_OPP;
			}
			/* if opp is on our near home zone */
			else if(strat_infos.opp_actual_zone == ZONE_NEAR_HOME
					&& strat_infos.opp_time_zone_ms > TIMEOUT_GOTO_NEAR_OPP) {

				DEBUG(E_USER_STRAT, "GOTO_NEAR_OPP");

				/* goto beside side */
				err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[3][3].x),
														  		    strat_infos.slot[3][3].y, 
														  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					break;
				}

//				/* work */
//				err = strat_pickup_or_push_near_slots(MODE_ALL);
//				if (!TRAJ_SUCCESS(err)) {
//					state = GOTO_NEAR_OPP;
//					break;
//				}

				state = WAITING_OPP;
			}
			/* if opp is on our near safe zone */
			else if(strat_infos.opp_actual_zone == ZONE_NEAR_SAFE
					&& strat_infos.opp_time_zone_ms > TIMEOUT_GOTO_NEAR_OPP) {

				DEBUG(E_USER_STRAT, "GOTO_NEAR_OPP");

				/* goto beside side */
				err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[3][1].x),
														  		    strat_infos.slot[3][1].y, 
														  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					break;
				}

//				/* work */
//				err = strat_pickup_or_push_near_slots(MODE_ALL);
//				if (!TRAJ_SUCCESS(err)) {
//					state = GOTO_NEAR_OPP;
//					break;
//				}

				state = WAITING_OPP;
			}
			/* if opp is on wall bonus zone */
			else if(strat_infos.opp_actual_zone == ZONE_WALL_BONUS
					&& strat_infos.opp_time_zone_ms > TIMEOUT_GOTO_NEAR_OPP) {

				DEBUG(E_USER_STRAT, "GOTO_NEAR_OPP");
				state = WAITING_OPP;
			}
			/* goto waiting opponent */
			if(state == WAITING_OPP)
				DEBUG(E_USER_STRAT, "WAITING_OPP");
			break;

		case WAITING_OPP:

			/* update opp side */
			if(strat_infos.opp_actual_zone == ZONE_OPP_NEAR_SAFE
				|| strat_infos.opp_actual_zone == ZONE_OPP_NEAR_HOME)
				opp_side_actual = ZONE_OPP_SIDE;
			else
				opp_side_actual = ZONE_OUR_SIDE;

#ifdef LOOK_FOR_TOWERS
			/* if found towers */
			if(strat_get_best_tower_ij(&i_tower, &j_tower)) {
				/* goto pickup towers */
				state = PICKUP_TOWERS;
				break;
			}
#endif

			/* if opp visited bonus wall and was on homes side */
#ifndef HOMOLOGATION
//			if((strat_infos.opp_actual_zone == ZONE_NEAR_HOME
//				|| strat_infos.opp_actual_zone == ZONE_OPP_NEAR_HOME)
//				&& (strat_infos.zones[ZONE_WALL_BONUS].num_visits 
//				&& strat_infos.zones[ZONE_WALL_BONUS].total_time_ms > TIMEOUT_OPP_WAS_WORKING)) {

			if((strat_infos.opp_actual_zone != ZONE_WALL_BONUS)
				&& (strat_infos.zones[ZONE_WALL_BONUS].num_visits 
				&& strat_infos.zones[ZONE_WALL_BONUS].total_time_ms > TIMEOUT_OPP_WAS_WORKING)) {

				/* goto work on bonus wall */
				state = WORK_ON_BONUS_WALL;
				break;
			}
#endif



			/* if opp has change the side or if is on bonus wall zone during N seconds */
			if( (opp_side_actual != opp_side_before) 
				|| (strat_infos.opp_actual_zone == ZONE_WALL_BONUS
				&& strat_infos.opp_time_zone_ms > TIMEOUT_OPP_IS_WORKING)) {

				/* update opp side before */
				opp_side_before = opp_side_actual;

				/* if opp was working during N seconds on zone before */
				if(strat_infos.zones[strat_infos.opp_before_zone].num_visits 
					&& strat_infos.zones[strat_infos.opp_before_zone].total_time_ms > TIMEOUT_OPP_WAS_WORKING) {

					/* got work zone */
					if(strat_infos.opp_before_zone == ZONE_OPP_NEAR_SAFE)
						state = WORK_NEAR_OPP_SAFE;
					else if(strat_infos.opp_before_zone == ZONE_OPP_NEAR_HOME)
						state = WORK_NEAR_OPP_HOME;
					else if(strat_infos.opp_before_zone == ZONE_NEAR_SAFE)
						state = WORK_NEAR_SAFE;
					else if(strat_infos.opp_before_zone == ZONE_NEAR_HOME)
						state = WORK_NEAR_HOME;
					else if(strat_infos.opp_before_zone == ZONE_WALL_BONUS)
						state = WORK_ON_BONUS_WALL;
					break;
				}
				else {
					/* goto near opp */
					state = GOTO_NEAR_OPP;
					break;
				}
			}

			/* if opp is in his side */
			if(opp_side_actual == ZONE_OPP_SIDE) {

				/* if is near opp safe zone during N seconds */
				if(strat_infos.opp_actual_zone == ZONE_OPP_NEAR_SAFE
					&& strat_infos.opp_time_zone_ms > TIMEOUT_OPP_IS_WORKING) {

					/* goto work on the zone beside if opp visited it before */
					if(strat_infos.zones[ZONE_OPP_NEAR_HOME].num_visits 
						&& strat_infos.zones[ZONE_OPP_NEAR_HOME].total_time_ms > TIMEOUT_OPP_WAS_WORKING) {
						DEBUG(E_USER_STRAT, "WORK_OPP_NEAR_HOME");					
						state = WORK_NEAR_OPP_HOME;
						break;
					}
				}
				/* if is near opp home zone during N seconds */
				else if(strat_infos.opp_actual_zone == ZONE_OPP_NEAR_HOME
					&& strat_infos.opp_time_zone_ms > TIMEOUT_OPP_IS_WORKING) {

					/* goto work on the zone beside if opp visited it before */
					if(strat_infos.zones[ZONE_OPP_NEAR_SAFE].num_visits 
						&& strat_infos.zones[ZONE_OPP_NEAR_SAFE].total_time_ms > TIMEOUT_OPP_WAS_WORKING) {
						DEBUG(E_USER_STRAT, "WORK_OPP_NEAR_SAFE");					
						state = WORK_NEAR_OPP_SAFE;
						break;
					}
				}
			}
			else {

				/* if is near our safe zone during N seconds */
				if(strat_infos.opp_actual_zone == ZONE_NEAR_SAFE
					&& strat_infos.opp_time_zone_ms > TIMEOUT_OPP_IS_WORKING) {

					/* goto work on the zone beside */
					if(strat_infos.zones[ZONE_NEAR_HOME].num_visits 
						&& strat_infos.zones[ZONE_NEAR_HOME].total_time_ms > TIMEOUT_OPP_WAS_WORKING) {
						DEBUG(E_USER_STRAT, "WORK_NEAR_HOME");
						state = WORK_NEAR_HOME;
						break;
					}

				}
				/* if is near our home zone during N seconds */
				else 	if(strat_infos.opp_actual_zone == ZONE_NEAR_HOME
					&& strat_infos.opp_time_zone_ms > TIMEOUT_OPP_IS_WORKING) {


					/* goto work on the zone beside */
					if(strat_infos.zones[ZONE_NEAR_SAFE].num_visits 
						&& strat_infos.zones[ZONE_NEAR_SAFE].total_time_ms > TIMEOUT_OPP_WAS_WORKING) {
						DEBUG(E_USER_STRAT, "WORK_NEAR_SAFE");
						state = WORK_NEAR_SAFE;
						break;
					}

				}
			}
#ifdef LOOK_FOR_TOWERS
			/* find towers */
			lasers_set_on();
			strat_look_for_towers_enable();

			strat_get_speed(&old_spdd, &old_spda);
			strat_set_speed(old_spdd, 500);

			opp_actual_zone = strat_infos.opp_actual_zone;

			trajectory_only_a_rel(&mainboard.traj, COLOR_A_REL(360));
			err = WAIT_COND_OR_TRAJ_END((opp_actual_zone != strat_infos.opp_actual_zone) 
										|| strat_get_best_tower_ij(&i_tower, &j_tower)
										, TRAJ_FLAGS_SMALL_DIST);
			
			if (TRAJ_SUCCESS(err) && !strat_get_best_tower_ij(&i_tower, &j_tower)) {	
				trajectory_only_a_rel(&mainboard.traj, COLOR_A_REL(-360));
				err = WAIT_COND_OR_TRAJ_END((opp_actual_zone != strat_infos.opp_actual_zone) 
											|| strat_get_best_tower_ij(&i_tower, &j_tower)
											, TRAJ_FLAGS_SMALL_DIST);
			}

			strat_set_speed(old_spdd, old_spda);
			strat_look_for_towers_disable();
			lasers_set_off();

			if(strat_get_best_tower_ij(&i_tower, &j_tower) 
				&& strat_get_num_tokens() == 2) {

				strat_infos.conf.th_place_prio = SLOT_PRIO_PATH;
				strat_infos.conf.th_token_score = PLACE_ALL_SCORE;

				err = strat_place_near_slots(1, 0);

			}
#endif
			break;

		case WORK_NEAR_OPP_HOME:	
			/* goto */
			err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[5][1].x),
													  		    strat_infos.slot[5][1].y, 
													  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}

			/* work */
			//err = strat_place_near_slots(1, FIGURE_SCORE);
			err = strat_place_near_slots(0,0);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}			

			err = strat_pickup_or_push_near_slots(MODE_ALL);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}			

			/* reset zone statistics */
			strat_infos.zones[ZONE_OPP_NEAR_HOME].num_visits = 0; 
			strat_infos.zones[ZONE_OPP_NEAR_HOME].total_time_ms = 0;
			
			/* goto near opp */
			state = GOTO_NEAR_OPP;
			break;

		case WORK_NEAR_OPP_SAFE:
			/* goto */
			err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[5][3].x),
													  		    strat_infos.slot[5][3].y, 
													  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}

			/* work */
			//err = strat_place_near_slots(1, FIGURE_SCORE);
			err = strat_place_near_slots(0,0);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}			

			err = strat_pickup_or_push_near_slots(MODE_ALL);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}			

			/* reset zone statistics */
			strat_infos.zones[ZONE_OPP_NEAR_SAFE].num_visits = 0; 
			strat_infos.zones[ZONE_OPP_NEAR_SAFE].total_time_ms = 0;

			/* goto near opp */
			state = GOTO_NEAR_OPP;
			break;

		case WORK_NEAR_HOME:
			/* goto */
			err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[2][1].x),
													  		    strat_infos.slot[2][1].y, 
													  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}

			/* work */
			err = strat_place_near_slots(0,0);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}			

			err = strat_pickup_or_push_near_slots(MODE_ALL);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}			

			/* reset zone statistics */
			strat_infos.zones[ZONE_NEAR_HOME].num_visits = 0; 
			strat_infos.zones[ZONE_NEAR_HOME].total_time_ms = 0;

			/* goto near opp */
			state = GOTO_NEAR_OPP;
			break;

		case WORK_NEAR_SAFE:
			/* goto */
			err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[2][3].x),
													  		    strat_infos.slot[2][3].y, 
													  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}

			/* work */
			err = strat_place_near_slots(0, PLACE_ALL_SCORE);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}			

			err = strat_pickup_or_push_near_slots(MODE_ALL);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}	

			/* reset zone statistics */
			strat_infos.zones[ZONE_NEAR_SAFE].num_visits = 0; 
			strat_infos.zones[ZONE_NEAR_SAFE].total_time_ms = 0;

			/* goto near opp */
			state = GOTO_NEAR_OPP;
			break;

		case WORK_ON_BONUS_WALL:
			
			/* save score and slot priority thresholds */
			th_place_prior_old = strat_infos.conf.th_place_prio;
			th_token_score_old = strat_infos.conf.th_token_score;

			/* save speed */
			strat_get_speed(&old_spdd, &old_spda);

			/* if sides has figure o towers, place one */
			if(token_side_score(SIDE_FRONT) >= FIGURE_SCORE
				&& token_side_score(SIDE_REAR) >= FIGURE_SCORE) {

				err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[5][3].x),
														  		    strat_infos.slot[5][3].y, 
														  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					state = GOTO_NEAR_OPP;
					break;
				}
				
				/* apply new thresholds */
				strat_infos.conf.th_token_score = PLACE_ALL_SCORE;

				/* place one token */
				err = strat_place_near_slots(1, 0);

				/* restore thresholds */
				strat_infos.conf.th_place_prio = th_place_prior_old;
				strat_infos.conf.th_token_score = th_token_score_old;

				if (!TRAJ_SUCCESS(err)) {
					state = GOTO_NEAR_OPP;
					break;
				}			
			}

			/* goto in front bonus wall */
			err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[4][4].x+75),
													  		    strat_infos.slot[4][4].y-100, 
													  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}


			/* if we have place one token */
			if (token_catched(SIDE_FRONT) && token_catched(SIDE_REAR)){

				/* apply new thresholds */
				strat_infos.conf.th_place_prio = SLOT_PRIO_CENTER;
				strat_infos.conf.th_token_score = PLACE_ALL_SCORE;

				/* place pion */
				err = strat_place_near_slots(1, PION_SCORE);

				/* restore thresholds */
				strat_infos.conf.th_place_prio = th_place_prior_old;
				strat_infos.conf.th_token_score = th_token_score_old;

				if (!TRAJ_SUCCESS(err)) {
					state = GOTO_NEAR_OPP;
					break;
				}	

				/* goto in front bonus wall */
				err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[4][4].x+75),
														  		    strat_infos.slot[4][4].y-100, 
														  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					state = GOTO_NEAR_OPP;
					break;
				}
			}


			/* possible token 4 */
			side = strat_turnto_pickup_token(&mainboard.traj,
						 COLOR_X(strat_infos.slot[4][5].x+80),
									strat_infos.slot[4][5].y );
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			strat_d_rel_side(&mainboard.traj, 95, side);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			
			//i2c_slavedspic_mode_token_out(side);
			strat_set_speed(2000, 2000);
			trajectory_only_a_rel(&mainboard.traj, COLOR_A_REL(45));
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			

			/* if our token bonus disapared place one */
			side = strat_turnto_pickup_token(&mainboard.traj,
						 COLOR_X(strat_infos.slot[3][5].x),
									strat_infos.slot[3][5].y );
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			if(!sensor_token_side(side))
				our_bonus_token_not_there = 1;


			/* pickup possible token protecting, XXX displacement important */
			err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[4][5].x-100),
													  		  strat_infos.slot[4][5].y-160, &side); //100 160

			/* go backwards to avoid crash on turning */
			strat_d_rel_side(&mainboard.traj, -70, side);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_NEAR_OPP;
				break;
			}

			/* there is a token protecting */
			if(token_catched(side) &&  strat_get_num_tokens() == 2) {

				/* back to origin */
				err = goto_and_avoid_empty_side(COLOR_X(strat_infos.slot[4][4].x),
												  		    strat_infos.slot[4][4].y-30, 
												  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					state = GOTO_NEAR_OPP;
					break;
				}

				/* apply new thresholds */
				strat_infos.conf.th_place_prio = SLOT_PRIO_CENTER;
				strat_infos.conf.th_token_score = PLACE_ALL_SCORE;

				/* place pion */
				err = strat_place_near_slots(1, PION_SCORE);

				/* restore thresholds */
				strat_infos.conf.th_place_prio = th_place_prior_old;
				strat_infos.conf.th_token_score = th_token_score_old;

				if (!TRAJ_SUCCESS(err)) {
					state = GOTO_NEAR_OPP;
					break;
				}	
			}		

			/* goto in front bonus wall */
			trajectory_goto_xy_abs(&mainboard.traj,
													COLOR_X(strat_infos.slot[4][4].x),
												           strat_infos.slot[4][4].y-30);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			side = strat_turnto_pickup_token(&mainboard.traj,
									 COLOR_X(strat_infos.slot[4][5].x),
												strat_infos.slot[4][5].y );
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);


			if(side == SIDE_FRONT) {
				if(!sensor_get(S_TOKEN_FRONT_R) && !sensor_get(S_TOKEN_FRONT_L))
					goto end_bonus_wall;
			}
			else {
				if(!sensor_get(S_TOKEN_REAR_R) && !sensor_get(S_TOKEN_REAR_L))
					goto end_bonus_wall;
			}

			strat_d_rel_side(&mainboard.traj, 130, side);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			strat_set_speed(200, 200);

			if(get_color() == I2C_COLOR_BLUE)
				i2c_slavedspic_mode_token_push_l(side);	
			else
				i2c_slavedspic_mode_token_push_r(side);

			strat_d_rel_side(&mainboard.traj, 80, side);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			i2c_slavedspic_mode_token_take(side);
			trajectory_only_a_rel(&mainboard.traj, COLOR_A_REL(-8));
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			if(err & END_BLOCKING) {
				trajectory_only_a_rel(&mainboard.traj, COLOR_A_REL(8));
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			}

			i2c_slavedspic_mode_token_take(side);
			strat_d_rel_side(&mainboard.traj, 130, side);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//			if(err & END_BLOCKING) {
//				time_wait_ms(100);
//				if(!token_catched(side)) {
//					trajectory_only_a_rel(&mainboard.traj, COLOR_A_REL(10));
//					err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//				}
//			}

			strat_set_speed(old_spdd, old_spda);
			strat_d_rel_side(&mainboard.traj, -200, side);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			/* goto in front bonus wall */
			trajectory_goto_xy_abs(&mainboard.traj,
													COLOR_X(strat_infos.slot[4][4].x),
												           strat_infos.slot[4][4].y-30);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			/* place on our bonus */
			if(our_bonus_token_not_there) {
 				err = strat_place_token_auto(COLOR_X(strat_infos.slot[3][5].x-60),
									 		                 strat_infos.slot[3][5].y+60, &side, GO_FORWARD);

				trajectory_goto_xy_abs(&mainboard.traj,
														COLOR_X(strat_infos.slot[4][4].x),
													           strat_infos.slot[4][4].y-30);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			}

end_bonus_wall:
			/* restore speed */
			strat_set_speed(old_spdd, old_spda);

			i2c_slavedspic_mode_token_stop(SIDE_FRONT);
			i2c_slavedspic_mode_token_stop(SIDE_REAR);

			/* reset zone statistics */
			strat_infos.zones[ZONE_WALL_BONUS].num_visits = 0; 
			strat_infos.zones[ZONE_WALL_BONUS].total_time_ms = 0;

			/* goto near opp */
			state = GOTO_NEAR_OPP;
			break;

		case PICKUP_TOWERS:
#ifdef LOOK_FOR_TOWERS
			strat_get_slot_to_place(i_tower, j_tower, &i_place_tower, &j_place_tower);

			if(i_place_tower == 0 || i_place_tower == 7)
				strat_set_bounding_box(AREA_BBOX_6X5);

			err = goto_and_avoid(strat_infos.slot[i_place_tower][j_place_tower].x,
										strat_infos.slot[i_place_tower][j_place_tower].y, 
									  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err)) {
				strat_set_bounding_box(AREA_BBOX_4X4);
				state = GOTO_NEAR_OPP;
				break;
			}

			err = strat_pickup_token_auto(strat_infos.slot[i_place_tower][j_place_tower].x,
													strat_infos.slot[i_place_tower][j_place_tower].y, &side);

			state = GOTO_NEAR_OPP;
			break;
#endif
		default:
			state = GOTO_NEAR_OPP;
			break;
	}

	return 0;
}

uint8_t strat_big_final(void)
{
	static uint8_t state = 0;
//	uint8_t side;
	uint8_t err;

#define INIT								0
#define GOTO_BEST_PLACE_POSITION		1
#define PLACE_FROM_CENTER_SLOT		2
#define PLACE_FROM_BONUS_NEAR_SAFE	3
#define PLACE_FROM_BONUS_NEAR_HOME	4
#define VISIT_OPP_BONUS					5
#define GOTO_BETWEEN_BONUS				6
#define GOTO_BONUS_NEAR_SAFE			7
#define GOTO_BONUS_NEAR_HOME			8


	switch(state) {
		case INIT:
			/* thresholds */
			strat_infos.conf.th_place_prio = SLOT_PRIO_GREEN;
			strat_infos.conf.th_token_score = PLACE_ALL_SCORE;
			state = GOTO_BETWEEN_BONUS;
			break;

		case GOTO_BETWEEN_BONUS:
			err = goto_and_avoid(COLOR_X(strat_infos.slot[5][2].x),
											  	  strat_infos.slot[5][2].y, 
								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_BONUS_NEAR_SAFE;
				break;
			}
			state = PLACE_FROM_CENTER_SLOT;
			break;

		case GOTO_BONUS_NEAR_SAFE:
			err = goto_and_avoid(COLOR_X(strat_infos.slot[5][3].x),
											  	  strat_infos.slot[5][3].y, 
								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_BONUS_NEAR_HOME;
				break;
			}
			state = PLACE_FROM_BONUS_NEAR_SAFE;
			break;

		case GOTO_BONUS_NEAR_HOME:
			err = goto_and_avoid(COLOR_X(strat_infos.slot[5][1].x),
											  	  strat_infos.slot[5][1].y, 
								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err)) {
				state = GOTO_BETWEEN_BONUS;
				break;
			}
			state = PLACE_FROM_BONUS_NEAR_HOME;
			break;

		case GOTO_BEST_PLACE_POSITION:
			if(!opp_x_is_more_than(1500+350)) {
				err = goto_and_avoid(COLOR_X(strat_infos.slot[5][2].x),
												  	  strat_infos.slot[5][2].y, 
									  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
				if (TRAJ_SUCCESS(err)) {
					state = PLACE_FROM_CENTER_SLOT;
					break;
				}
			}
			else if(opp_x_is_more_than(1500)) {

				if(opp_y_is_more_than(875)) {
					err = goto_and_avoid(COLOR_X(strat_infos.slot[5][1].x),
													  	  strat_infos.slot[5][1].y, 
										  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
					if (TRAJ_SUCCESS(err)) {
						state = PLACE_FROM_BONUS_NEAR_HOME;
						break;
					}
				}
				else {
			
					err = goto_and_avoid(COLOR_X(strat_infos.slot[5][3].x),
													  	  strat_infos.slot[5][3].y, 
										  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
					if (TRAJ_SUCCESS(err)) {
						state = PLACE_FROM_BONUS_NEAR_SAFE;
						break;
					}
				}
			}
			else {
				err = goto_and_avoid(COLOR_X(strat_infos.slot[5][2].x),
												  	  strat_infos.slot[5][2].y, 
									  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
				if (TRAJ_SUCCESS(err)) {
					state = PLACE_FROM_CENTER_SLOT;
					break;
				}
			}
			break;

		case PLACE_FROM_CENTER_SLOT:

#ifdef FORCE_PLACE
			/* place */
			strat_infos.slot[5][3].flags &= ~(SLOT_BUSY);
			strat_infos.slot[5][1].flags &= ~(SLOT_BUSY);
			err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][3].x),
									              strat_infos.slot[5][3].y, &side, GO_FORWARD);
			err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][1].x),
									              strat_infos.slot[5][1].y, &side, GO_FORWARD);
#else
			err = strat_place_near_slots(0,0);	

//			/* place */
//			strat_infos.slot[5][3].flags &= ~(SLOT_BUSY);
//			err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][3].x),
//									              strat_infos.slot[5][3].y, &side, GO_FORWARD);

#endif
			/* protect tokens */
			trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1675), 875);
			err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
			trajectory_only_a_abs(&mainboard.traj, 90);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

//			if(opp_y_is_more_than(1050))
//				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1675), 1250);
//			else if(!opp_y_is_more_than(1050)) 
//				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1675), 525);
//		
//			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			state = VISIT_OPP_BONUS;
			break;

		case PLACE_FROM_BONUS_NEAR_SAFE:
			/* goto near */
			err = goto_and_avoid(COLOR_X(strat_infos.slot[4][3].x),
											  	  strat_infos.slot[4][3].y, 
								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err)) {
				state = PLACE_FROM_BONUS_NEAR_HOME;
				break;
			}

#ifdef FORCE_PLACE
			/* place */
			strat_infos.slot[4][4].flags &= ~(SLOT_BUSY);
			strat_infos.slot[5][3].flags &= ~(SLOT_BUSY);

			err = strat_place_token_auto(COLOR_X(strat_infos.slot[4][4].x),
									              strat_infos.slot[4][4].y, &side, GO_FORWARD);
			err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][3].x),
									              strat_infos.slot[5][3].y, &side, GO_FORWARD);
#else
			err = strat_place_near_slots(0,0);	
#endif

			/* protect tokens */
			strat_set_speed(2000, SPEED_ANGLE_FAST);
			trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1850), 1050);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

			state = VISIT_OPP_BONUS;
			break;
	
		case PLACE_FROM_BONUS_NEAR_HOME:
			/* goto near */
			err = goto_and_avoid(COLOR_X(strat_infos.slot[4][1].x),
											  	  strat_infos.slot[4][1].y, 
								  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err)) {
				state = PLACE_FROM_BONUS_NEAR_SAFE;
				break;
			}

#ifdef FORCE_PLACE
			/* place */
			strat_infos.slot[4][2].flags &= ~(SLOT_BUSY);
			strat_infos.slot[5][1].flags &= ~(SLOT_BUSY);

			err = strat_place_token_auto(COLOR_X(strat_infos.slot[4][2].x),
									              strat_infos.slot[4][2].y, &side, GO_FORWARD);
			err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][1].x),
									              strat_infos.slot[5][1].y, &side, GO_FORWARD);
#else
			err = strat_place_near_slots(0,0);
#endif
			/* protect tokens */
			strat_set_speed(2000, SPEED_ANGLE_FAST);
			trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1850), 700);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

			state = VISIT_OPP_BONUS;
			break;
			
		case VISIT_OPP_BONUS:
			return END_TRAJ;
		default:
			state = 0;
			break;
	
	}
	
	return 0;
}



///* work on a zone */
//uint8_t strat_work_on_zone(zone_t * z)
//{
//	uint8_t err;
//
//	/* goto zone pt with avoidance, return if we can't reach the pt */
//	err = goto_and_avoid_busy_side(COLOR_X(z->x), z->y,
//								    TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);
//
//	/* do before task */
//	if(z->do_before)
//		z->do_before();
//
//	/* pickup or push tokens */
//	strat_pickup_or_push_near_slots();
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);
//
//	/* do after task */
//	if(z->do_after)
//		z->do_after();
//
// end:
//	return err;
//}
//
///* XXX create strat_place_best_token(i,j) */
//uint8_t strat_place_figure_near_opp_home(void)
//{
//	uint8_t err = END_TRAJ;
//	uint8_t side;
//	int8_t i;
//	
//	/* return if slot busy */
//	i = (get_color() == I2C_COLOR_BLUE? 6 : 1);
//	if(strat_infos.slot[i][0].flags & SLOT_BUSY) {
//		ERROUT(err);
//	}
//
//	/* TODO: place the token with higher score */
//
//	/* we prefer place a figure */
//	if(sensor_get(S_TOKEN_FRONT_FIGURE)) {
//
//		side = SIDE_FRONT;
//		err = strat_place_token(COLOR_X(strat_infos.slot[6][0].x), 
//								strat_infos.slot[6][0].y, side, GO_FORWARD);
//	}
//	else if(sensor_get(S_TOKEN_REAR_FIGURE)) {
//
//		side = SIDE_REAR;
//		err = strat_place_token(COLOR_X(strat_infos.slot[6][0].x), 
//								strat_infos.slot[6][0].y, side, GO_FORWARD);
//	}
//	else {
//		err = strat_place_token_auto(COLOR_X(strat_infos.slot[6][0].x), 
//						strat_infos.slot[6][0].y, &side, GO_FORWARD);
//	}
//
//
//	/* avoid opponent */
//	while(opponent_is_behind_side(side));
//
//	/* back in front of bonus wall */
//	trajectory_goto_xy_abs(&mainboard.traj,
//								 COLOR_X(strat_infos.slot[5][1].x), 
//								 strat_infos.slot[5][1].y);
//	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);
//
//end:
//	return err;
//}
//
///* XXX create strat_place_best_token(i,j) */
//uint8_t strat_place_on_near_opp_safe_slot(void)
//{
//	uint8_t err = END_TRAJ;
//	uint8_t side;
//	int8_t i;
//	
//	/* return if slot busy */
//	i = (get_color() == I2C_COLOR_BLUE? 6 : 1);
//	if(strat_infos.slot[i][0].flags & SLOT_BUSY) {
//		ERROUT(err);
//	}
//
//	/* TODO: place the token with higher score */
//
//	/* we prefer place a figure */
//	if(sensor_get(S_TOKEN_FRONT_FIGURE)) {
//
//		side = SIDE_FRONT;
//		err = strat_place_token(COLOR_X(strat_infos.slot[6][4].x), 
//								strat_infos.slot[6][4].y, side, GO_FORWARD);
//	}
//	else if(sensor_get(S_TOKEN_REAR_FIGURE)) {
//
//		side = SIDE_REAR;
//		err = strat_place_token(COLOR_X(strat_infos.slot[6][4].x), 
//								strat_infos.slot[6][4].y, side, GO_FORWARD);
//	}
//	else {
//		err = strat_place_token_auto(COLOR_X(strat_infos.slot[6][4].x), 
//						strat_infos.slot[6][4].y, &side, GO_FORWARD);
//	}
//
//
//	/* avoid opponent */
//	while(opponent_is_behind_side(side));
//
//	/* back in front of bonus wall */
//	trajectory_goto_xy_abs(&mainboard.traj,
//								 COLOR_X(strat_infos.slot[5][3].x), 
//								 strat_infos.slot[5][3].y);
//	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);
//
//end:
//	return err;
//}
//
///* XXX improve */
//uint8_t strat_place_on_opp_safe_slot(void)
//{
//	uint8_t err = END_TRAJ;
//	uint8_t side = 0;
//	int8_t i;
//	
//	/* return if slot busy */
//	i = (get_color() == I2C_COLOR_BLUE? 5 : 2);
//	if(strat_infos.slot[i][5].flags & SLOT_BUSY) {
//		ERROUT(err);
//	}
//
//	/* place a pion */
//	if(!sensor_get(S_TOKEN_FRONT_FIGURE)) {
//		side = SIDE_FRONT;
//		err = strat_place_token(COLOR_X(strat_infos.slot[5][5].x), 
//								strat_infos.slot[5][5].y, side, GO_FORWARD);
//	}
//	else if(!sensor_get(S_TOKEN_REAR_FIGURE)) {
//		side = SIDE_FRONT;
//		err = strat_place_token(COLOR_X(strat_infos.slot[5][5].x), 
//								strat_infos.slot[5][5].y, side, GO_FORWARD);
//	}
//
//	/* avoid opponent */
//	while(opponent_is_behind_side(side));
//
//	/* back to origin slot */
//	trajectory_goto_xy_abs(&mainboard.traj,
//								 COLOR_X(strat_infos.slot[5][3].x), 
//								 strat_infos.slot[5][3].y);
//	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);
//end:
//	return err;
//}

//uint8_t strat_pickup_bonus_near_wall(void)
//{
//	int16_t delta = 0;
//	uint8_t side = 0;
//	uint8_t err;
//	uint8_t our_token_disapeared = 0;
//	
//	/* initial side */
//	side = (token_side_is_lower_score(SIDE_FRONT)? SIDE_FRONT : SIDE_REAR);
//
//start:
//	/* take a look if our token is still here */
//	side = strat_turnto_pickup_token(&mainboard.traj,
//										COLOR_X(strat_infos.slot[3][5].x), 
//										strat_infos.slot[3][5].y);
//	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);
//
//	if(!sensor_token_side(side))
//		our_token_disapeared = 1;
//
//	/* try pickup opp token */
//	side = strat_turnto_pickup_token(&mainboard.traj,
//										COLOR_X(strat_infos.slot[4][5].x), 
//										strat_infos.slot[4][5].y);
//	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);
//
//	/* go forwards slowly */
//	i2c_slavedspic_mode_token_take(side);
//	strat_set_speed(300, SPEED_ANGLE_FAST);
//	strat_d_rel_side(&mainboard.traj, 260, side);
//
//	/* wait token catched or end traj */
//	err = WAIT_COND_OR_TRAJ_END(sensor_token_side(side), TRAJ_FLAGS_NO_NEAR);
//
//	/* restore speed */
//	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
//
//	/* two tokens on opponent bonus */
//	if(sensor_token_side(side) && !y_is_more_than(1750)) {
//
//		/* back in front of bonus wall */
//		trajectory_goto_xy_abs(&mainboard.traj,
//									 COLOR_X(strat_infos.slot[4][4].x), 
//									 strat_infos.slot[4][4].y);
//		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//		if (!TRAJ_SUCCESS(err))
//			ERROUT(err);
//
//		/* for place token catched in null position */
//		delta = 100;
//
//		goto start;
//	}
//
//	/* avoid opponent */
//	while(opponent_is_in_area(COLOR_X(1500), 1750,
//									  COLOR_X(1850), 1050));
//
//	/* back in front of bonus wall */
//	trajectory_goto_xy_abs(&mainboard.traj,
//								 COLOR_X(strat_infos.slot[4][4].x), 
//								 strat_infos.slot[4][4].y);
//	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);
//
//	/* TODO if our token bonus disapared place one */
//
//end:
//	return err;
//}



#ifdef ALCALA_STRAT
/* place two tokens in bonus points,
	return END_TRAJ if sucess o END_TIMER (only one token in bonus */

#define DATA_UPDATE_TIME 50
uint8_t strat_bonus_point(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	uint8_t side;
	int8_t opp_there; 
	int16_t opp_x, opp_y;
	uint8_t path;
	uint8_t nb_tokens_catched = 0;

#define CHECK_OPPONENT						0
#define GO_LEFT_PATH							1
#define GO_RIGHT_PATH						2
#define WAIT_OPP								3
#define PLACE_TWO_ON_BONUS_POINT			4
#define PLACE_ONE_ON_BONUS_POINT_UP		5
#define PLACE_ONE_ON_BONUS_POINT_DOWN	6
#define PICK_AND_PLACE_NEAR_TOKENS		7
#define CHECK_OPP_BONUS_TOKEN				8
#define PICKUP_OPP_BONUS_TOKEN			9

	uint8_t state = CHECK_OPPONENT;

#define WAIT_OPP_TIMEOUT (90-15)

	while(1) {

		switch(state) {
			case CHECK_OPPONENT:		
				DEBUG(E_USER_STRAT, "CHECK_OPPONENT");
				/* chech opponent */
				time_wait_ms(300);
				opp_there = get_opponent_xy(&opp_x, &opp_y);
	
				/* decide path */
				if(opp_there != -1) {

					if(opp_y < 1050) {
						state = GO_LEFT_PATH;
						path = GO_LEFT_PATH;
					}
					else {
						state = GO_RIGHT_PATH;
						path = GO_RIGHT_PATH;
					}
				}
				else {
					state = GO_RIGHT_PATH;
					path = GO_RIGHT_PATH;
				}
				break;
	
			case GO_LEFT_PATH:
				DEBUG(E_USER_STRAT, "GO_LEFT_PATH");
				/* position 1 */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[3][4].x),
											 strat_infos.slot[3][4].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err)) {
//					/* escape 1 */
//					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[2][3].x),
//											 strat_infos.slot[2][3].y);
//					err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
//
//					path = GO_RIGHT_PATH;
//					state = GO_RIGHT_PATH;
					break;
				}
	
				/* position 2 */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][3].x),
											 strat_infos.slot[4][3].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err)) {
					/* escape 2 */
//					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[3][4].x),
//											 strat_infos.slot[3][4].y);
//					err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
//
//					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[2][3].x),
//											 strat_infos.slot[2][3].y);
//					err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
//
//					path = GO_RIGHT_PATH;
//					state = GO_RIGHT_PATH;	
					break;			
				}
	
				DEBUG(E_USER_STRAT, "WAIT_OPP");
				state = WAIT_OPP;		
				break;
	
			case GO_RIGHT_PATH:
				DEBUG(E_USER_STRAT, "GO_RIGHT_PATH");
				/* position 1 */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][1].x-175),
											 strat_infos.slot[4][1].y+175);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err)) {
					/* escape 1 */
//					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[2][3].x),
//											 strat_infos.slot[2][3].y);
//					err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
//					path = GO_LEFT_PATH;
//					state = GO_LEFT_PATH;
					break;
				}
	
				/* position 2 */
				strat_limit_speed_disable();
				strat_get_speed(&old_spdd, &old_spda);
				strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);

				strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y, GO_FORWARD);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

				strat_set_speed(old_spdd, old_spda);
				strat_limit_speed_enable();

				if (!TRAJ_SUCCESS(err)) {
					break;
				}

				DEBUG(E_USER_STRAT, "WAIT_OPP");
				state = WAIT_OPP;		
				break;
	
			case WAIT_OPP:

				/* test opponent xy */
				time_wait_ms(300);
				opp_there = get_opponent_xy(&opp_x, &opp_y);
				if(opp_there != -1) {
					if((mainboard.our_color == I2C_COLOR_BLUE)
						 && ((opp_x < 1500) || (opp_y > 1400))) {
						state = PLACE_TWO_ON_BONUS_POINT;
					}
					else if((mainboard.our_color == I2C_COLOR_RED)
						 && ((opp_x > 1500) || (opp_y > 1400))) {
						state = PLACE_TWO_ON_BONUS_POINT;
					}
				}
					
				/* test time */
				if(time_get_s() >= WAIT_OPP_TIMEOUT) {
					if(path == GO_LEFT_PATH) {
						state = PLACE_ONE_ON_BONUS_POINT_UP;
					}
				 	else {	
						state = PLACE_ONE_ON_BONUS_POINT_DOWN;
					}
				}
				break;
	
			case PLACE_TWO_ON_BONUS_POINT:
				DEBUG(E_USER_STRAT, "PLACE_TWO_ON_BONUS_POINT");			
				/* go in the middle */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err))
					break;
	
				/* place one token */
			 	err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][3].x),
												 strat_infos.slot[5][3].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;
	
				/* place the other */
			 	err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][1].x),
												 strat_infos.slot[5][1].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;
	
				state = PICK_AND_PLACE_NEAR_TOKENS;
				break;
	
			case PLACE_ONE_ON_BONUS_POINT_UP:
				DEBUG(E_USER_STRAT, "PLACE_ONE_ON_BONUS_POINT_UP");
				/* goto place slot */
				strat_limit_speed_disable();
				strat_get_speed(&old_spdd, &old_spda);
				strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][3].x),
											 strat_infos.slot[6][3].y);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					break;

				strat_set_speed(old_spdd, old_spda);
				strat_limit_speed_enable();
	
				/* place one token */
			 	err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][3].x),
												 strat_infos.slot[5][3].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;
	
				/* goto place slot */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][3].x),
											 strat_infos.slot[6][3].y);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					break;

				/* place the other */
			 	err = strat_place_token_auto( COLOR_X(strat_infos.slot[6][4].x),
												 strat_infos.slot[6][4].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;

				/* goto place slot */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][3].x),
											 strat_infos.slot[6][3].y);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					break;

				return END_TRAJ;
	
			case PLACE_ONE_ON_BONUS_POINT_DOWN:
				DEBUG(E_USER_STRAT, "PLACE_ONE_ON_BONUS_POINT_DOWN");
				/* goto place slot */
				strat_limit_speed_disable();
				strat_get_speed(&old_spdd, &old_spda);
				strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_VERY_SLOW);

				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][1].x),
											 strat_infos.slot[6][1].y);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					break;

				strat_set_speed(old_spdd, old_spda);
				strat_limit_speed_enable();

				/* place one token */
			 	err = strat_place_token_auto( COLOR_X(strat_infos.slot[5][1].x),
												 strat_infos.slot[5][1].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;

				/* goto place slot */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][1].x),
											 strat_infos.slot[6][1].y);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
					break;

				/* place the other */
			 	err = strat_place_token_auto( COLOR_X(strat_infos.slot[6][0].x),
												 strat_infos.slot[6][0].y, &side, GO_FORWARD);
				if (!TRAJ_SUCCESS(err))
					break;
	
				return END_TRAJ;
	

			case PICK_AND_PLACE_NEAR_TOKENS:
				DEBUG(E_USER_STRAT, "PICK_AND_PLACE_NEAR_TOKENS");
				/* slot 1 */
				err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[6][1].x),
							 strat_infos.slot[6][1].y, &side);

				WAIT_COND_OR_TIMEOUT(token_catched(side), DATA_UPDATE_TIME);
				if(token_catched(side))
					nb_tokens_catched++;

				/* back */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;


				/* slot 2 */
				err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[4][3].x),
							 strat_infos.slot[4][3].y, &side);

				WAIT_COND_OR_TIMEOUT(token_catched(side), DATA_UPDATE_TIME);
				if(token_catched(side))
					nb_tokens_catched++;		
				
				/* back */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;

				/* slot 3 */
				if(nb_tokens_catched == 2) {
					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][1].x),
							 						strat_infos.slot[4][1].y);
					err = wait_traj_end(TRAJ_FLAGS_STD);
				}
				else {
					err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[4][1].x),
								 strat_infos.slot[4][1].y, &side);

					WAIT_COND_OR_TIMEOUT(token_catched(side), DATA_UPDATE_TIME);
					if(token_catched(side))
						nb_tokens_catched++;
				}

				/* back */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;


				/* slot 4 */
				if(nb_tokens_catched == 2) {
					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][3].x),
							 						strat_infos.slot[6][3].y);
					err = wait_traj_end(TRAJ_FLAGS_STD);
				}
				else {
					err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[6][3].x),
								 strat_infos.slot[6][3].y, &side);
	
					WAIT_COND_OR_TIMEOUT(token_catched(side), DATA_UPDATE_TIME);
					if(token_catched(side))
						nb_tokens_catched++;
				}

				/* back */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;


				/* place tokens */
				if(nb_tokens_catched == 2) {
					err = strat_place_token_auto(COLOR_X(strat_infos.slot[4][2].x),
								 strat_infos.slot[4][2].y, &side, GO_FORWARD);

					/* back */
					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
												 strat_infos.slot[5][2].y);
					err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
					if (!TRAJ_SUCCESS(err))
						break;

					err = strat_place_token_auto(COLOR_X(strat_infos.slot[6][2].x),
								 strat_infos.slot[6][2].y, &side, GO_FORWARD);
				}	
				else if(nb_tokens_catched == 1) {
					err = strat_place_token_auto(COLOR_X(strat_infos.slot[6][2].x),
								 strat_infos.slot[6][2].y, &side, GO_FORWARD);
				}		

				/* back */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[5][2].x),
											 strat_infos.slot[5][2].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;

				state = CHECK_OPP_BONUS_TOKEN;
				break;

			case CHECK_OPP_BONUS_TOKEN:
				DEBUG(E_USER_STRAT, "CHECK_OPP_BONUS_TOKEN");
				/* chech opponent */
				time_wait_ms(300);
				opp_there = get_opponent_xy(&opp_x, &opp_y);
				if(opp_there != -1) {
					if((mainboard.our_color == I2C_COLOR_BLUE)
						 && (opp_x < 1500)) {
						state = PICKUP_OPP_BONUS_TOKEN;
					}
					else if((mainboard.our_color == I2C_COLOR_RED)
						 && (opp_x > 1500)) {
						state = PICKUP_OPP_BONUS_TOKEN;
					}
				}
				break;
			
			case PICKUP_OPP_BONUS_TOKEN:
				DEBUG(E_USER_STRAT, "PICKUP_OPP_BONUS_TOKEN");

				/* go near */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][3].x),
						 						strat_infos.slot[4][3].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;

				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][4].x),
						 						strat_infos.slot[4][4].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;

				/* pickup */
				err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[4][5].x),
							 strat_infos.slot[4][5].y, &side);

				/* go near */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[4][3].x),
						 						strat_infos.slot[4][3].y);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					break;

				/* wait time */
				while(time_get_s() < 85);

				/* place */
				WAIT_COND_OR_TIMEOUT(token_catched(side), DATA_UPDATE_TIME);
				err = strat_place_token(COLOR_X(strat_infos.slot[4][4].x),
									 strat_infos.slot[4][4].y, side, GO_FORWARD);
	
				return END_TRAJ;

			default:
				return 0;
		}
	}
	return 0;
}


uint8_t strat_traj_slots(slot_index_t *go, slot_index_t *back)
{
	uint8_t err;

	/* go path */
	while(go->i != NULL) {
		trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[go->i][go->j].x),
									 strat_infos.slot[go->i][go->j].y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err); 

		go++;
	}
	
	return END_TRAJ;

end:
	/* back path */
	while(back->i != NULL) {
		trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[back->i][back->j].x),
									 strat_infos.slot[go->i][go->j].y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err); 
		back++;
	}
	
	return err;
}
#endif

