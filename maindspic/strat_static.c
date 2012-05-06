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


#if 1
#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)
#else
#define ERROUT(e) do {err = 0} while(0) /* XXX no interrups trajs are possible */
#endif

/* pick and place tokens on line 1 */

#define LINE1_START_X			1150
#define LINE1_START_Y			350
#define LINE1_END_X				1150
#define LINE1_END_Y				(1400-100)	//(1050+100)
#define LINE1_LAST_TOKEN_X		1150
#define LINE1_LAST_TOKEN_Y		(1750-50)

#define LINE2_LAST_TOKEN_X		800
#define LINE2_LAST_TOKEN_Y		350

#define LINE_CATCHED_TIME 		10

#define SPEED_PICKUP_TOKEN		1500 //1800
#define PUSH_TOKEN_ANGLE		45

#define TIMEOUT_WAIT_OPP		3000
#define NB_OBSTACLE_TRIES		3

uint8_t line2_only_one_token = 0;

uint8_t line1_push_last_opp_token(void)
{
	uint8_t err = END_OBSTACLE;

	if(!opponent_is_in_area(COLOR_X(1150), 1750, COLOR_X(2200-175), 1400)) {
	
		/* goto near opp safe zone */
		strat_set_speed(1800, SPEED_ANGLE_FAST);
		trajectory_goto_forward_xy_abs(&mainboard.traj, COLOR_X(1900), 1575); // XXX tested 1850
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
		//if (!TRAJ_SUCCESS(err))
		//	return err;	
	
		/* push token */;
		strat_set_speed(1800, 1800);
		trajectory_a_rel(&mainboard.traj, COLOR_A_REL(PUSH_TOKEN_ANGLE));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		//if (!TRAJ_SUCCESS(err))
		//	return err;	
	
		/* check slot as busy */
		if(get_color() == I2C_COLOR_BLUE)
			strat_infos.slot[5][5].flags |= (SLOT_BUSY|SLOT_AVOID);
		else
			strat_infos.slot[2][5].flags |= (SLOT_BUSY|SLOT_AVOID);
	}

	return err;
}

void goto_push_green_opp (void)
{
	uint8_t err;
	uint8_t one_token = 0;
	uint8_t side;
	int16_t x, y;

	strat_set_bounding_box(AREA_BBOX_6X5);

	if(strat_get_num_tokens() == 1)
		one_token = 1;

	while(!opp_y_is_more_than(1050));

	err = goto_and_avoid(COLOR_X(2375),
								  	  700, 
					  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);


	err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[7][2].x),
								  	  strat_infos.slot[7][2].y,&side);

	strat_set_speed(2000, 2000);

	trajectory_d_rel(&mainboard.traj, -100);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	trajectory_only_a_rel(&mainboard.traj, COLOR_A_REL(-90));	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	trajectory_only_a_rel(&mainboard.traj, COLOR_A_REL(90));	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	trajectory_d_rel(&mainboard.traj, 100);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	x = position_get_x_s16(&mainboard.pos); 
	y = position_get_y_s16(&mainboard.pos);
	
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[7][4].x),
								  	  strat_infos.slot[7][4].y);
	err = wait_traj_end(TRAJ_FLAGS_STD);

	trajectory_goto_xy_abs(&mainboard.traj,x,
								  	  y);
	err = wait_traj_end(TRAJ_FLAGS_STD);


	while(!opp_y_is_more_than(1050) && opp_y_is_more_than(2200));

	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(2375),
			 						700);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	//if (!TRAJ_SUCCESS(err))
	//	break;

//	if(opp_y_is_more_than(1050)) {
//		trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][1].x),
//				 						strat_infos.slot[6][1].y);
//		err = wait_traj_end(TRAJ_FLAGS_STD);
//
//	}
//	else {
		trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[6][1].x),
				 						strat_infos.slot[6][1].y);
		err = wait_traj_end(TRAJ_FLAGS_STD);
//	}

	strat_set_speed(4000, 4000);

//	if(!opp_y_is_more_than(1050)) {
//		err = goto_and_avoid(COLOR_X(strat_infos.slot[5][4].x),
//								  	  strat_infos.slot[5][4].y-50, 
//					  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
//
//		err = strat_place_token_auto(COLOR_X(strat_infos.slot[5][5].x),
//				 						strat_infos.slot[5][5].y, &side, GO_FORWARD);
////	}
//	else {

		err = goto_and_avoid(COLOR_X(strat_infos.slot[6][1].x),
								  	  strat_infos.slot[6][1].y, 
					  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);

		err = strat_place_token_auto(COLOR_X(strat_infos.slot[6][0].x),
				 						strat_infos.slot[6][0].y, &side, GO_FORWARD);
//	}

	//if (!TRAJ_SUCCESS(err))
	//	break;

	err = goto_and_avoid(COLOR_X(1150),
								  	  350, 
					  TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
}


uint8_t strat_harvest_line1(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	uint8_t side = SIDE_FRONT;
	uint8_t nb_tokens_catched = 0;
	uint8_t line1_last_pos_token = 0;
	uint8_t cnt_tries = 0;

	/* reset variables */
	line2_only_one_token = 0;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* XXX set mirrors mode */
	mirrors_set_mode(MODE_LOOK_FOR_FIGURES);

	/* look to last token of line 2 */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,  COLOR_X(LINE2_LAST_TOKEN_X), LINE2_LAST_TOKEN_Y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* be sure that the token is there */
	time_wait_ms(50);
	if(sensor_token_side(side)) {

		/* try to pick up last token of line 2 */
		err = strat_pickup_token(COLOR_X(LINE2_LAST_TOKEN_X), LINE2_LAST_TOKEN_Y, side);		
		if (token_catched(SIDE_FRONT)) {
			line2_only_one_token = 1;
		}
	}

	/* goto line start */
	side = strat_turnto_pickup_token(&mainboard.traj,  COLOR_X(LINE1_START_X), LINE1_START_Y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj,  COLOR_X(LINE1_START_X), LINE1_START_Y);		
	i2c_slavedspic_mode_token_take(side);

	/* wait traj end or token detected */
	err = WAIT_COND_OR_TRAJ_END(sensor_token_side(side), TRAJ_FLAGS_SMALL_DIST);

	/* token detected, down speed */
	if(sensor_token_side(side)) {
		/* down speed and wait end traj */
		strat_set_speed(SPEED_PICKUP_TOKEN, SPEED_ANGLE_FAST);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
		/* restore speed */
		strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	}
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);			

	/* check token catched at first position */
	if(token_catched(side)) { 
		nb_tokens_catched ++;
		DEBUG(E_USER_STRAT, "token catched at first possition (%d)", nb_tokens_catched);
	}

	/* check if we had found the last token of line 2 at begins */
	if(line2_only_one_token) {
		/* place token */
		err = strat_place_token(COLOR_X(strat_infos.slot[2][0].x-50),
							 strat_infos.slot[2][0].y-70, 
							 SIDE_FRONT, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* back to line start */
		wait_until_opponent_is_far();
		trajectory_goto_xy_abs(&mainboard.traj,  COLOR_X(LINE1_START_X), LINE1_START_Y);		
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);			
	}


	//goto_push_green_opp();

	/* XXX enable look for figures */
	lasers_set_on();
	strat_look_for_figures_enable();

	/* wait beacon update */
	time_wait_ms(100);

	/* do straight line harvesting */
	do {

		/* go to line end harvesting tokens	*/
		wait_until_opponent_is_far();
		side = strat_turnto_pickup_token(&mainboard.traj,  COLOR_X(LINE1_END_X), LINE1_END_Y);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* set speed */
		if(!sensor_token_side(side))
			strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
		else
			strat_set_speed(SPEED_PICKUP_TOKEN, SPEED_ANGLE_FAST);

		wait_until_opponent_is_far();
		trajectory_goto_xy_abs(&mainboard.traj,  COLOR_X(LINE1_END_X), LINE1_END_Y);		
		i2c_slavedspic_mode_token_take(side);

		/* wait traj end or token detected */
		err = WAIT_COND_OR_TRAJ_END(sensor_token_side(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	
		/* token detected */
		if(sensor_token_side(side)) {

			/* down speed */
			strat_set_speed(SPEED_PICKUP_TOKEN, SPEED_ANGLE_FAST);
			DEBUG(E_USER_STRAT, "token detected, down speed");

			/* wait token catched or end traj */
			err = WAIT_COND_OR_TRAJ_END(token_catched(side), TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	
			/* if token catched */
			if(token_catched(side)) {
		
				trajectory_stop(&mainboard.traj);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
					ERROUT(err);
	
				nb_tokens_catched ++;
				DEBUG(E_USER_STRAT, "token catched in line (%d)", nb_tokens_catched);
				continue;
			}
		}

		if (TRAJ_SUCCESS(err))
			break;
		else if (err & END_OBSTACLE) {

			cnt_tries ++;
			if(cnt_tries == NB_OBSTACLE_TRIES) {
				cnt_tries = 0;
				ERROUT(err);
			}

			wait_ms(1000);	/* wait to sensor re-enable, we not avoid the opponent */
			continue;		/* TODO: second way / scape sequence */
		}
		else if (!TRAJ_SUCCESS(err))
			ERROUT(err);	

	} while(nb_tokens_catched < 2);

	/* reset cnt_tries */
	cnt_tries = 0;

	/* some times last token is not catched in line */
	if(nb_tokens_catched == 0) {
		err = strat_pickup_token_auto(COLOR_X(1150), 1500, &side);
		//if (!TRAJ_SUCCESS(err))
		//	ERROUT(err);	

		if(token_catched(side)) {	
			nb_tokens_catched ++;
			DEBUG(E_USER_STRAT, "token catched at end of line (%d)", nb_tokens_catched);
		}
		else
			ERROUT(END_ERROR);
	}

	/* need one more token? */
	if( nb_tokens_catched < 2) {

		/* set flag of last token */
		line1_last_pos_token = 1;

		wait_until_opponent_is_far();
//		side = strat_turnto_pickup_token(&mainboard.traj,  COLOR_X(LINE1_END_X), LINE1_END_Y);
//		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//		if (!TRAJ_SUCCESS(err))
//			ERROUT(err);

		err = strat_pickup_token_auto(COLOR_X(LINE1_LAST_TOKEN_X), LINE1_LAST_TOKEN_Y, &side);
		if(token_catched(side)) {	
			nb_tokens_catched ++;
			DEBUG(E_USER_STRAT, "token catched at last position (%d)", nb_tokens_catched);
		}
		else
			ERROUT(END_ERROR);
			
	}

	/* restore speed */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

read_fig4:

	/* goto the position to read figure 4 */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(1150),  
								 1575);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (err & END_OBSTACLE) {

		cnt_tries ++;
		if(cnt_tries == NB_OBSTACLE_TRIES) {
			cnt_tries = 0;
			ERROUT(err);
		}

		wait_ms(5000);			/* XXX wait to sensor re-enable, we not avoid the opponent */
		goto read_fig4;
	}
	else if (!TRAJ_SUCCESS(err))
		ERROUT(err);		

	/* reset cnt_tries */
	cnt_tries = 0;

	/* XXX disable look for figures */
	strat_look_for_figures_disable();
	lasers_set_off();
	strat_dump_infos(__FUNCTION__);

	/* if token in last position, try to place oppopent token in safe zone */
	if(line1_last_pos_token 
		&& (strat_infos.conf.flags & LINE1_OPP_TOKEN_BEFORE_PLACE)) {

		/* push opp token */
		err = line1_push_last_opp_token();

		/* reset flag */
		if (TRAJ_SUCCESS(err)) {
			line1_last_pos_token = 0;
		}
	}

	/* restore speed */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);


	/* PLACE TOKENS **************************************************/

	/* place two tokens in bonus slot SLOWLY */
	if(!opponent_is_in_area(COLOR_X(1500), 2100, COLOR_X(2025), 1400) 
		&& (strat_infos.conf.flags & LINE1_TOKENS_ON_BONUS_OLD)) {

		/* goto near place slot */
place_pt_1:
		wait_until_opponent_is_far();
		trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1500), 1750);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (err & END_OBSTACLE) {

			cnt_tries ++;
			if(cnt_tries == NB_OBSTACLE_TRIES) {
				cnt_tries = 0;
				ERROUT(err);
			}

			time_wait_ms(1000);			/* XXX wait to sensor re-enable, we not avoid the opponent */
			goto place_pt_1;
		}
		else if (!TRAJ_SUCCESS(err))
				ERROUT(err);
	
		/* reset cnt_tries */
		cnt_tries = 0;

		/* place first token in bonus slot */
		wait_until_opponent_is_far();
		err = strat_place_token(COLOR_X(strat_infos.slot[3][5].x-50),
							 strat_infos.slot[3][5].y+50, 
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
		/* go backwards */
		if(side == SIDE_FRONT)
			trajectory_d_rel(&mainboard.traj, -20);
		else
			trajectory_d_rel(&mainboard.traj, 20);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
		/* place second token, force blocking */
		wait_until_opponent_is_far();
		side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[3][5].x+70),
							 (strat_infos.slot[3][5].y-70), GO_FORWARD);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
		/* eject token */
		i2c_slavedspic_mode_token_out(side);
		
		/* push slowly until blocking */
		strat_set_speed(1000, 1000);
		trajectory_goto_xy_abs(&mainboard.traj,
									COLOR_X(strat_infos.slot[3][5].x+60),
							 		(strat_infos.slot[3][5].y-60));
		err = wait_traj_end(END_BLOCKING);
		
		/* go backwards slowly */
		i2c_slavedspic_mode_token_stop(side);

		/* avoid opponent */
		WAIT_COND_OR_TIMEOUT(!opponent_is_behind_side(side), TIMEOUT_WAIT_OPP);
		wait_until_opponent_is_far();

		i2c_slavedspic_mode_token_out(side);
		strat_set_speed(200, 200);
		trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(strat_infos.slot[3][5].x+350), //68
							 	 (strat_infos.slot[3][5].y-350));
		time_wait_ms(800);
		strat_set_speed(1800, 1800);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

		//if (!TRAJ_SUCCESS(err))
		//	ERROUT(err);
	
		/* stop belts */
		i2c_slavedspic_mode_token_stop(side);

		/* avoid blocking with near tokens */
		wait_until_opponent_is_far();
		trajectory_a_rel(&mainboard.traj, COLOR_A_REL(-180));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		//if (!TRAJ_SUCCESS(err))
		//	ERROUT(err);

	} 

	/* place two tokens near wall */
	else if(!opponent_is_in_area(COLOR_X(1500), 2100, COLOR_X(1850), 1400) 
		     && (strat_infos.conf.flags & LINE1_TOKENS_NEAR_WALL)) {

		/* goto near place slot */
place_pt_2:
		wait_until_opponent_is_far();
		trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1500), 1750);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (err & END_OBSTACLE) {

			cnt_tries ++;
			if(cnt_tries == NB_OBSTACLE_TRIES) {
				cnt_tries = 0;
				ERROUT(err);
			}

			time_wait_ms(1000);			/* XXX wait to sensor re-enable, we not avoid the opponent */
			goto place_pt_2;
		}
		else if (!TRAJ_SUCCESS(err))
				ERROUT(err);
	
		/* reset cnt_tries */
		cnt_tries = 0;

		/* place first token in bonus slot */
		wait_until_opponent_is_far();
		err = strat_place_token(COLOR_X(strat_infos.slot[3][5].x-50),
							 strat_infos.slot[3][5].y+50, 
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
		/* go backwards */
		trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1500), 1750);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* turn pushing firsT token placed */
		strat_set_speed(SPEED_DIST_FAST, 1500);
		wait_until_opponent_is_far();
		trajectory_only_a_rel(&mainboard.traj, COLOR_A_REL(180));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* place second token near first */
		wait_until_opponent_is_far();
		strat_clear_slot_flags(COLOR_X(1500-30),
							 strat_infos.slot[3][5].y + 60, SLOT_BUSY);
		err = strat_place_token_auto(COLOR_X(1500-30),
							 strat_infos.slot[3][5].y + 60, 
							 &side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* restore speed */
		strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

		/* return to grid */
		wait_until_opponent_is_far();
		trajectory_goto_xy_abs(&mainboard.traj,
									 COLOR_X(1500), 1575);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	}
	else { /* place two tokens in bonus slot FASTER */

place_pt_3:

		/* go to place origin */
		wait_until_opponent_is_far();
		trajectory_goto_xy_abs(&mainboard.traj,
									 COLOR_X(strat_infos.slot[3][4].x),  
									 strat_infos.slot[3][4].y);
	
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (err & END_OBSTACLE) {

			cnt_tries ++;
			if(cnt_tries == NB_OBSTACLE_TRIES) {
				cnt_tries = 0;
				ERROUT(err);
			}

			wait_ms(5000);			/* XXX wait to sensor re-enable, we not avoid the opponent */
			goto place_pt_3;
		}
		else if (!TRAJ_SUCCESS(err))
			ERROUT(err);	

		/* reset cnt_tries */
		cnt_tries = 0;

		/* place one token on bonus slot */
		wait_until_opponent_is_far();
		err = strat_place_token(COLOR_X(strat_infos.slot[3][5].x),
							 strat_infos.slot[3][5].y, 
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
		/* invert side */
		side ^= 0x01;
	

		wait_until_opponent_is_far();
#ifdef HOMOLOGATION
		/* and the other in front */
		err = strat_place_token(COLOR_X(strat_infos.slot[3][3].x),
							 strat_infos.slot[3][3].y, 
							 side, GO_FORWARD);
#else
		/* and the other protecting the last */
		strat_clear_slot_flags(COLOR_X(strat_infos.slot[3][5].x + 70),
							 strat_infos.slot[3][5].y - 60, SLOT_BUSY);	// XXX tested with 70
		err = strat_place_token(COLOR_X(strat_infos.slot[3][5].x + 70),
							 strat_infos.slot[3][5].y - 60, 					// XXX tested with 70
							 side, GO_FORWARD);
#endif
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	}

	/* if token in last position, try to place oppopent token in safe zone */
	if(line1_last_pos_token 
		&& (strat_infos.conf.flags & LINE1_OPP_TOKEN_AFTER_PLACE)) {

		/* push opp token */
		err = line1_push_last_opp_token();

		/* reset flag */
		line1_last_pos_token = 0;
	}

	/* back to place origin */		
back_origin:
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(1150+60),  
								 strat_infos.slot[3][4].y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER);
	if (err & END_OBSTACLE) {

		cnt_tries ++;
		if(cnt_tries == NB_OBSTACLE_TRIES) {
			cnt_tries = 0;
			ERROUT(err);
		}

		time_wait_ms(1000);
		goto back_origin;
	}
	else if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* reset cnt_tries */
	cnt_tries = 0;


 end:
	strat_look_for_figures_disable();
	lasers_set_off();
	//mirrors_set_mode(MODE_HIDE_MIRRORS);
	i2c_slavedspic_mode_token_stop(SIDE_FRONT);
	i2c_slavedspic_mode_token_stop(SIDE_REAR);
	strat_set_speed(old_spdd, old_spda);
	return err;
}


/* pick and place tokens on line 2 */

#define LINE2_START_X			800
#define LINE2_START_Y			1400
#define LINE2_END_X				800
#define LINE2_END_Y				700
	
#define LINE2_FIRST_TOKEN_X	800
#define LINE2_FIRST_TOKEN_Y	1750

uint8_t strat_harvest_line2(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;
	uint8_t side;
	uint8_t nb_tokens_catched = 0;
	uint8_t cnt_tries = 0;


	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* check token catched carry */
	if(line2_only_one_token)
		nb_tokens_catched = 1;

	/* try to pick up token */
	err = strat_pickup_token_auto(COLOR_X(LINE2_FIRST_TOKEN_X), LINE2_FIRST_TOKEN_Y, &side);
	if (token_catched(side)) {
		nb_tokens_catched ++;
	}

	/* check flag first possition */
	if(nb_tokens_catched < 2) {

		/* goto line start */
		//wait_until_opponent_is_far();
		side = strat_turnto_pickup_token(&mainboard.traj,  COLOR_X(LINE2_START_X), LINE2_START_Y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		//wait_until_opponent_is_far();
		i2c_slavedspic_mode_token_take(side);
		trajectory_goto_xy_abs(&mainboard.traj,  COLOR_X(LINE2_START_X), LINE2_START_Y);		

		/* wait traj end or token detected */
		err = WAIT_COND_OR_TRAJ_END(sensor_token_side(side), TRAJ_FLAGS_NO_NEAR);

		/* token detected */
		if(sensor_token_side(side)) {
			/* down speed and wait end traj */
			strat_set_speed(SPEED_PICKUP_TOKEN, SPEED_ANGLE_FAST);
			err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		}
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);	
	
		/* restore speed */
		strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);	

		/* check token catched  */
		WAIT_COND_OR_TIMEOUT(token_catched(side), LINE_CATCHED_TIME);
		if(token_catched(side)) { 
			nb_tokens_catched ++;
			DEBUG(E_USER_STRAT, "token catched at second possition (%d)", nb_tokens_catched);
		}
	
		/* do straight line harvesting */
		while(nb_tokens_catched < 2){
	
			/* go to line end harvesting tokens	*/
			//wait_until_opponent_is_far();
			side = strat_turnto_pickup_token(&mainboard.traj,  COLOR_X(LINE2_END_X), LINE2_END_Y);
			err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err))
				ERROUT(err);

			/* set speed */
			if(!sensor_token_side(side))
				strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
			else
				strat_set_speed(SPEED_PICKUP_TOKEN, SPEED_ANGLE_FAST);	

			//wait_until_opponent_is_far();
			i2c_slavedspic_mode_token_take(side);
			trajectory_goto_xy_abs(&mainboard.traj,  COLOR_X(LINE2_END_X), LINE2_END_Y);		

			/* wait traj end or token catched */
			err = WAIT_COND_OR_TRAJ_END(sensor_token_side(side), TRAJ_FLAGS_NO_NEAR);
		
			/* token detected */
			if(sensor_token_side(side)) {
	
				/* down speed */
				strat_set_speed(SPEED_PICKUP_TOKEN, SPEED_ANGLE_FAST);
	
				/* wait token catched or end traj */
				err = WAIT_COND_OR_TRAJ_END(token_catched(side), TRAJ_FLAGS_NO_NEAR);
		
				/* if token catched */
				if(token_catched(side)) {
			
					trajectory_stop(&mainboard.traj);
					err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
					if (!TRAJ_SUCCESS(err))
						ERROUT(err);
			
					nb_tokens_catched ++;
					DEBUG(E_USER_STRAT, "token catched in line (%d)", nb_tokens_catched);
					continue;
				}
			}
			
			if (TRAJ_SUCCESS(err))
				break;
//			else if (err & END_OBSTACLE) {
//
//				cnt_tries ++;
//				if(cnt_tries == NB_OBSTACLE_TRIES) {
//					cnt_tries = 0;
//					ERROUT(err);
//				}
//
//				time_wait_ms(1000);		/* XXX wait to sensor re-enable, we not avoid the opponent */
//				continue;					/* TODO: sencond way / scape sequence */
//			}
			else if (!TRAJ_SUCCESS(err))
				ERROUT(err);		
		}

//		/* reset cnt_tries */
//		cnt_tries = 0;

		/* may be last token is pushed and not catched */
		if(nb_tokens_catched < 2) {
			err = strat_pickup_token_auto(COLOR_X(LINE2_END_X), LINE2_END_Y-175, &side);
			if(token_catched(side)) {	
				nb_tokens_catched ++;
				DEBUG(E_USER_STRAT, "token catched at end of line (%d)", nb_tokens_catched);
			}
			else
				ERROUT(END_ERROR);
		}
	}

	/* restore speed */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* go to place origin */
place_pt:
	//wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj,
								 COLOR_X(strat_infos.slot[2][1].x),  
								 strat_infos.slot[2][1].y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (err & END_OBSTACLE) {

		cnt_tries ++;
		if(cnt_tries == NB_OBSTACLE_TRIES) {
			cnt_tries = 0;
			ERROUT(err);
		}

		wait_ms(1000);			/* XXX wait to sensor re-enable, we not avoid the opponent */
		goto place_pt;		
	}
	else if (!TRAJ_SUCCESS(err))
		ERROUT(err);	

	/* reset cnt_tries */
	cnt_tries = 0;

	/* place tokens */
	if(!line2_only_one_token) {
		//wait_until_opponent_is_far();
		err = strat_place_token(COLOR_X(strat_infos.slot[2][0].x),
							 strat_infos.slot[2][0].y-50, 
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* back to place position */
		trajectory_goto_xy_abs(&mainboard.traj,
									 COLOR_X(strat_infos.slot[2][1].x),  
									 strat_infos.slot[2][1].y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* invert side */
		side ^= 0x01;
	}

	/* choose location of tokens depends on opponent */
	//time_wait_ms(100);
	WAIT_COND_OR_TIMEOUT(!(opponent_is_in_area(COLOR_X(1500), 350, COLOR_X(1850), 0)
		|| opponent_is_in_area(COLOR_X(1150), 700, COLOR_X(1500), 0)), 2000);

	if(opponent_is_in_area(COLOR_X(1500), 350, COLOR_X(1850), 0)
		|| opponent_is_in_area(COLOR_X(1150), 700, COLOR_X(1500), 0)) {

		/* place token in right slot */
		//wait_until_opponent_is_far();
		err = strat_place_token(COLOR_X(strat_infos.slot[2][2].x),
							 strat_infos.slot[2][2].y, 
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);	
	}
	else {

		/* turn to slot near wall */
		//wait_until_opponent_is_far();
		side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[4][0].x),
							 					strat_infos.slot[4][0].y-30, GO_FORWARD);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* go near slot */
		//wait_until_opponent_is_far();
		strat_d_rel_side(&mainboard.traj, 400, side);
		err = wait_traj_end(TRAJ_FLAGS_STD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* place token near wall */
		//wait_until_opponent_is_far();
		err = strat_place_token(COLOR_X(strat_infos.slot[4][0].x),
							 strat_infos.slot[4][0].y-30,
							 side, GO_FORWARD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* go near slot */
		//wait_until_opponent_is_far();
		strat_d_rel_side(&mainboard.traj, -600, side);
		err = wait_traj_end(TRAJ_FLAGS_STD);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* go to place origin */
		//wait_until_opponent_is_far();
		trajectory_goto_xy_abs(&mainboard.traj,
									 COLOR_X(strat_infos.slot[2][1].x),  
									 strat_infos.slot[2][1].y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);	
	}

 end:
	//mirrors_set_mode(MODE_HIDE_MIRRORS);
	i2c_slavedspic_mode_token_stop(SIDE_FRONT);
	i2c_slavedspic_mode_token_stop(SIDE_REAR);
	strat_set_speed(old_spdd, old_spda);
	return err;
}


 

/* pick and place tokens on green area */

#define TOKENS_GREEN_START_X 			625
#define TOKENS_GREEN_START_Y 			690
#define TOKENS_GREEN_D_AVOID_WALL	150
#define TOKEN_INSIDE_TIME				800

uint8_t strat_harvest_green_area(void)
{
#ifdef COMPILE_HARVEST_GREEN_AREA

	uint8_t err;
	uint16_t old_spdd, old_spda;
	uint8_t side;
	uint8_t cnt_tries = 0;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* goto init possition */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(TOKENS_GREEN_START_X), TOKENS_GREEN_START_Y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);	

	/* pick token 1 */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,
									 COLOR_X(strat_infos.slot[0][1].x-100),
									 strat_infos.slot[0][1].y );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_pickup_token(COLOR_X(strat_infos.slot[0][1].x-100),
									 strat_infos.slot[0][1].y, side );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* wait token inside */
	WAIT_COND_OR_TIMEOUT(token_inside(side), TOKEN_INSIDE_TIME);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, -TOKENS_GREEN_D_AVOID_WALL);
	else
		trajectory_d_rel(&mainboard.traj, TOKENS_GREEN_D_AVOID_WALL);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

#ifndef GREEN_AREA_V1
	/* avoid crash with near tokens */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj,COLOR_X(strat_infos.slot[1][2].x),
								  strat_infos.slot[1][2].y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);	
#endif

	/* place token 1 */
	wait_until_opponent_is_far();
 	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[1][1].x),
									 strat_infos.slot[1][1].y, GO_FORWARD);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[1][1].x),
									 strat_infos.slot[1][1].y, side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

#ifdef GREEN_AREA_V1
	/* pick token 2 */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,
									 COLOR_X(strat_infos.slot[0][2].x),
									 strat_infos.slot[0][2].y );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_pickup_token(COLOR_X(strat_infos.slot[0][2].x),
									 strat_infos.slot[0][2].y, side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* wait token inside */ 
	WAIT_COND_OR_TIMEOUT(token_inside(side), TOKEN_INSIDE_TIME);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, -(TOKENS_GREEN_D_AVOID_WALL-50));
	else
		trajectory_d_rel(&mainboard.traj, (TOKENS_GREEN_D_AVOID_WALL-50));

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* goto intermediate possition */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, 
									COLOR_X(strat_infos.slot[1][2].x),
									strat_infos.slot[1][2].y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place token 2 */
	wait_until_opponent_is_far();
 	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[2][2].x),
									 strat_infos.slot[2][2].y, GO_FORWARD);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[2][2].x),
									 strat_infos.slot[2][2].y, side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* pick token 3 */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,
									 COLOR_X(strat_infos.slot[0][3].x),
									 strat_infos.slot[0][3].y );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_pickup_token(COLOR_X(strat_infos.slot[0][3].x),
									 strat_infos.slot[0][3].y, side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* wait token inside */
	WAIT_COND_OR_TIMEOUT(token_inside(side), TOKEN_INSIDE_TIME);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, -TOKENS_GREEN_D_AVOID_WALL);
	else
		trajectory_d_rel(&mainboard.traj, TOKENS_GREEN_D_AVOID_WALL);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* pick token 4 */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,
									 COLOR_X(strat_infos.slot[0][4].x),
									 strat_infos.slot[0][4].y );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_pickup_token(COLOR_X(strat_infos.slot[0][4].x),
									 strat_infos.slot[0][4].y, side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* wait token inside */
	WAIT_COND_OR_TIMEOUT(token_inside(side), TOKEN_INSIDE_TIME);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, -TOKENS_GREEN_D_AVOID_WALL);
	else
		trajectory_d_rel(&mainboard.traj, TOKENS_GREEN_D_AVOID_WALL);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);


	/* place token 3 */
	wait_until_opponent_is_far();
 	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[1][3].x),
									 strat_infos.slot[1][3].y, GO_FORWARD);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[1][3].x),
									 strat_infos.slot[1][3].y, side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* go to intermediate possition */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(450), 1575);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	

	/* pick token 5 */
	wait_until_opponent_is_far();
	side = strat_turnto_pickup_token(&mainboard.traj,
									 COLOR_X(strat_infos.slot[0][5].x),
									 strat_infos.slot[0][5].y );
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_pickup_token(COLOR_X(strat_infos.slot[0][5].x),
									 strat_infos.slot[0][5].y, side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* wait token inside */
	WAIT_COND_OR_TIMEOUT(token_inside(side), TOKEN_INSIDE_TIME);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	if(side == SIDE_FRONT)
		trajectory_d_rel(&mainboard.traj, -TOKENS_GREEN_D_AVOID_WALL);
	else
		trajectory_d_rel(&mainboard.traj, TOKENS_GREEN_D_AVOID_WALL);

	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place token 4 */
	wait_until_opponent_is_far();
 	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[2][4].x),
									 strat_infos.slot[2][4].y, GO_FORWARD);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[2][4].x),
									 strat_infos.slot[2][4].y, side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place token 5 */
	wait_until_opponent_is_far();
 	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[1][5].x),
									 strat_infos.slot[1][5].y, GO_FORWARD);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	err = strat_place_token(COLOR_X(strat_infos.slot[1][5].x),
									 strat_infos.slot[1][5].y+12, side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

#else /* GREEN_AREA_V2 */
	
	/* go in front of token 2 */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(625),
								  strat_infos.slot[0][2].y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* pick token 2 */
	wait_until_opponent_is_far();
	err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[0][2].x-100),
									 strat_infos.slot[0][2].y, &side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	strat_d_rel_side(&mainboard.traj, -(TOKENS_GREEN_D_AVOID_WALL-50), side);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);	

	/* go in front of token 3 */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(625),
								  strat_infos.slot[0][3].y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* pick token 3 */
	wait_until_opponent_is_far();
	err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[0][3].x-100),
									 strat_infos.slot[0][3].y, &side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* go backwards to avoid wall */
	wait_until_opponent_is_far();
	strat_d_rel_side(&mainboard.traj, -(TOKENS_GREEN_D_AVOID_WALL-50), side);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* goto place point */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[1][4].x+25),
								  strat_infos.slot[1][4].y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place pion (if there is one) on safe zone */
	side = strat_turnto_place_token(&mainboard.traj, COLOR_X(strat_infos.slot[1][5].x+25),
									  strat_infos.slot[1][5].y, GO_FORWARD);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	if(side == SIDE_FRONT) {
		if(!sensor_get(S_TOKEN_FRONT_FIGURE)) {
			err = strat_place_token(COLOR_X(strat_infos.slot[1][5].x),
									  strat_infos.slot[1][5].y, SIDE_FRONT, GO_FORWARD);
		}
		else if(!sensor_get(S_TOKEN_REAR_FIGURE)) {
			err = strat_place_token(COLOR_X(strat_infos.slot[1][5].x),
									  strat_infos.slot[1][5].y, SIDE_REAR, GO_FORWARD);
		}
		else {
			err = strat_place_token(COLOR_X(strat_infos.slot[1][5].x),
									  strat_infos.slot[1][5].y, SIDE_FRONT, GO_FORWARD);
		}
	}
	else {
		if(!sensor_get(S_TOKEN_REAR_FIGURE)) {
			err = strat_place_token(COLOR_X(strat_infos.slot[1][5].x),
									  strat_infos.slot[1][5].y, SIDE_REAR, GO_FORWARD);
		}
		else if(!sensor_get(S_TOKEN_FRONT_FIGURE)) {
			err = strat_place_token(COLOR_X(strat_infos.slot[1][5].x),
									  strat_infos.slot[1][5].y, SIDE_FRONT, GO_FORWARD);
		}
		else {
			err = strat_place_token(COLOR_X(strat_infos.slot[1][5].x),
									  strat_infos.slot[1][5].y, SIDE_REAR, GO_FORWARD);
		}
	}
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* place the other token on slot near green area */
	err = strat_place_token_auto(COLOR_X(strat_infos.slot[1][3].x),
								  strat_infos.slot[1][3].y, &side, GO_FORWARD);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* avoid crash with near tokens */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[1][4].x),
								  strat_infos.slot[1][4].y-27);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* pick token 4 */
	wait_until_opponent_is_far();
	err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[0][4].x-100),
									 strat_infos.slot[0][4].y, &side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* avoid wall and crash with token 5 */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(450),
								  strat_infos.slot[1][4].y-27);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(400),
								  1510);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* pick token 5 */
	wait_until_opponent_is_far();
	err = strat_pickup_token_auto(COLOR_X(strat_infos.slot[0][5].x),
									 strat_infos.slot[0][5].y, &side );
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* go backwards to avoid wall and crash */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(400),
								  1510);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);


	/* goto end position */
	wait_until_opponent_is_far();
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[1][4].x),
								  strat_infos.slot[1][4].y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
//	wait_until_opponent_is_far();
//	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(strat_infos.slot[2][4].x),
//								  strat_infos.slot[2][4].y);
//	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//	if (!TRAJ_SUCCESS(err))
//		ERROUT(err);

#endif


 end:
	i2c_slavedspic_mode_token_stop(SIDE_FRONT);
	i2c_slavedspic_mode_token_stop(SIDE_REAR);
	strat_set_speed(old_spdd, old_spda);
	return err;
#else
	DEBUG(E_USER_STRAT, "%s not compiled", __FUNCTION__);
	return END_TRAJ;
#endif
}



/* take a token from green area */

//#define TYPE_PION		0
//#define TYPE_FIGURE	1

uint8_t strat_pickup_green_token(uint8_t type, uint8_t color)
{

#define D_PUSH_TOKEN_4		250
#define OPPONENT_TIMEOUT	5000

	uint8_t err = END_TRAJ;
	uint16_t old_spdd, old_spda;
	uint8_t side;
	int8_t i, j;
	int16_t x_infront;
//	int16_t x_escape, y_escape;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* depends on color */
	i = (color == I2C_COLOR_BLUE? 0 : 7);
	x_infront = (color == I2C_COLOR_BLUE? 625 : 2375);

	/* if we are not in the correct side depends on color */
	if(color == I2C_COLOR_BLUE && x_is_more_than(1500)) {
		err = goto_and_avoid(x_infront, 875,
								TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}
	else if(color == I2C_COLOR_RED && !x_is_more_than(1500)) {
		err = goto_and_avoid(x_infront, 875,
								TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}
		
	/* get pion slot index */
	for(j = 1; j < 6; j++) {

		if(type == TYPE_PION) {
			/* we look for a slot without a figure and not checked before */
			if((strat_infos.slot[i][j].flags & SLOT_FIGURE) == 0
				&& (strat_infos.slot[i][j].flags & SLOT_BUSY)) {
				
				break;
			}
		}
		else { /* type == TYPE_FIGURE */
			/* we look for a slolt with a figure and not checked before */
			if((strat_infos.slot[i][j].flags & SLOT_FIGURE)
				&& (strat_infos.slot[i][j].flags & SLOT_BUSY) == 0) {
				
				break;
			}
		}
	}	

	/* special case: two figures on positions 1 and 2 */
	if(type == TYPE_FIGURE) {
		if(j == 1 && (strat_infos.slot[i][2].flags & SLOT_FIGURE)) {
			/* pickup first figure on position 2 */
			j = 2;
		}	
	}

	/* return if no tokens found */
	if(j == 6) {
		DEBUG(E_USER_STRAT, "%s not found in gree area", type == TYPE_PION? "PION" : "FIGURE");
		ERROUT(END_TRAJ);
	}


	/* XXX check opponent */
	//WAIT_COND_OR_TIMEOUT(!opponent_is_infront_side(side), OPPONENT_TIMEOUT);
	//WAIT_COND_OR_TIMEOUT(!opponent_is_in_area(COLOR_X(450), 2100, COLOR_X(800), 350), OPPONENT_TIMEOUT);
	//if(opponent_is_infront_side(side))
	//	ERROUT(END_OBSTACLE);

	if(opponent_is_in_area(COLOR_X(450), 2100, COLOR_X(800), 350)) {
		ERROUT(END_OBSTACLE);
	}

	/* turn to infront of token */
	side = strat_turnto_pickup_token(&mainboard.traj, x_infront, strat_infos.slot[i][j].y);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* goto infront of token */
	if(j == 1 && type == TYPE_FIGURE) {
		/* special case */
		if(strat_infos.slot_actual.j != 2) {
			trajectory_goto_xy_abs(&mainboard.traj, x_infront, strat_infos.slot[i][2].y);
			err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

			if (!TRAJ_SUCCESS(err)) {
				ERROUT(err);
			}
		}
	}
	else if(j == 5) {
		/* special case */
		if(strat_infos.slot_actual.j != 4) {
			trajectory_goto_xy_abs(&mainboard.traj, x_infront, strat_infos.slot[i][4].y-50);
			err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err))
				ERROUT(err);
		}
		/* turn to token */
		side = strat_turnto_pickup_token(&mainboard.traj, strat_infos.slot[i][j].x, strat_infos.slot[i][j].y);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* push token 4, if is there */
		strat_set_speed(SPEED_ANGLE_SLOW, SPEED_ANGLE_FAST);
		strat_d_rel_side(&mainboard.traj, D_PUSH_TOKEN_4, side);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);	
	}
	else {
		trajectory_goto_xy_abs(&mainboard.traj, x_infront, strat_infos.slot[i][j].y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}


	/* pick token */
	DEBUG(E_USER_STRAT, "pickup token @ (%d, %d)", i,j);
	err = strat_pickup_token_auto(strat_infos.slot[i][j].x,
									 		strat_infos.slot[i][j].y, &side);
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);

	
	/* wait if opponent is behind */
	while(opponent_is_behind_side(side));

//	WAIT_COND_OR_TIMEOUT(!opponent_is_behind_side(side), OPPONENT_TIMEOUT);	
//	if(opponent_is_behind_side(side)) {
//
//		/* escape from opponent */
//		strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
//
//		x_escape = (color == I2C_COLOR_BLUE? 625 : 2375);
//		y_escape = (y_is_more_than(1050)? 350 : 1400);
//
//		DEBUG(E_USER_STRAT, "escape from opp (%d, %d)", x_escape, y_escape);
//
//		err = strat_goto_xy_force(x_escape, y_escape);
//		if (!TRAJ_SUCCESS(err))
//			ERROUT(err);
//
//		ERROUT(END_OBSTACLE);
//	}

	/* return to infront position */
	if(j == 1 && type == TYPE_FIGURE) {
		trajectory_goto_xy_abs(&mainboard.traj, x_infront, strat_infos.slot[i][2].y);
	}
	else if(j == 5) {
		trajectory_goto_xy_abs(&mainboard.traj, x_infront, strat_infos.slot[i][4].y);
	}
	else {
		trajectory_goto_xy_abs(&mainboard.traj, x_infront, strat_infos.slot[i][j].y);
	}
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

 end:
	i2c_slavedspic_mode_token_stop(SIDE_FRONT);
	i2c_slavedspic_mode_token_stop(SIDE_REAR);
	strat_set_speed(old_spdd, old_spda);
	return err;
}

/* harvest green area ending with two figures inside */
uint8_t strat_harvest_green_area_smart(uint8_t color)
{

#define NOT_SUCCESS_TRIES_MAX 3
#define TIME_WAIT_OPP_ms		1000

	uint8_t err;
	uint16_t old_spdd, old_spda;
	uint8_t side;
//	uint8_t color = get_color();
	int8_t i, j;
	int16_t x_opp, y_opp;
	uint8_t num_tries = 0;

	strat_set_speed(2000, 2000);

	/* bounding box */
	strat_set_bounding_box(AREA_BBOX_6X5);

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	/* goto near green zone */
near_green:
	i = (color == I2C_COLOR_BLUE? 1 : 6);
	j = 2;
	num_tries = 0;

try_near_green:
	err = goto_and_avoid(strat_infos.slot[i][j].x, strat_infos.slot[i][j].y,
								TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err)) {
		
		num_tries ++;
		if(num_tries == NOT_SUCCESS_TRIES_MAX) {
			if((x_is_more_than(1500) && opp_x_is_more_than(450)) 
				|| (!x_is_more_than(1500) && !opp_x_is_more_than(450))) {
				color = (color == get_color()? get_opponent_color() : get_color());
			}
			goto near_green;
		}	
		else {
			time_wait_ms(TIME_WAIT_OPP_ms);
			j ++;
			if(j > 3) {
				j = 1; 
			}
			goto try_near_green;
		}
	}

	/* pickup two pions */
retry_pickup_first:
	if( strat_get_num_tokens() < 2) {
		err = strat_pickup_green_token(TYPE_PION, color);
		if (!TRAJ_SUCCESS(err)) {
			if((x_is_more_than(1500) && opp_x_is_more_than(450)) 
				|| (!x_is_more_than(1500) && !opp_x_is_more_than(450))) {
				color = (color == get_color()? get_opponent_color() : get_color());
			}
			goto retry_pickup_first;
		}
	}

retry_pickup_second:
	if( strat_get_num_tokens() < 2) {
		err = strat_pickup_green_token(TYPE_PION, color);
		if (!TRAJ_SUCCESS(err)) {
			if((x_is_more_than(1500) && opp_x_is_more_than(450)) 
				|| (!x_is_more_than(1500) && !opp_x_is_more_than(450))) {
				color = (color == get_color()? get_opponent_color() : get_color());
			}
			goto retry_pickup_second;
		}
	}

	/* goto near the nearest place slot */
retry_place_first:
	if(color == get_color()) {
		/* our side */
		i = COLOR_I(1);
		j = (y_is_more_than(1050)? 4 : 2);
	}
	else {
		/* opponent side */
		if(!opponent_is_in_area(COLOR_X(1500), 1750, COLOR_X(3000), 1050)
			&& (strat_infos.slot[COLOR_I(5)][5].flags & SLOT_BUSY) == 0) {

			i = COLOR_I(5);
			j = 4;			
		}
		else {
			i = COLOR_I(6);
			j = 1;
		}
	}
//	trajectory_goto_xy_abs(&mainboard.traj, strat_infos.slot[i][j].x, strat_infos.slot[i][j].y);
//	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

	err = goto_and_avoid(strat_infos.slot[i][j].x, strat_infos.slot[i][j].y,
								TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err)) {
		if((x_is_more_than(1500) && opp_x_is_more_than(450)) 
			|| (!x_is_more_than(1500) && !opp_x_is_more_than(450))) {
			color = (color == get_color()? get_opponent_color() : get_color());
		}
		goto retry_place_first;
	}
	else if (TRAJ_SUCCESS(err)) {

		/* place */
		if(color == get_color()) {
			/* our side */
			i = COLOR_I(1);
			j = (y_is_more_than(1050)? 5 : 1);
		}
		else {
			/* opponent side */
			if(y_is_more_than(1050)) {
	
				i = COLOR_I(5);
				j = 5;			
			}
			else {
				/* infront opp home */
				i = COLOR_I(6);
				j = 0;
			}
		}

		/* depends if we have pions */
		if(token_side_score(SIDE_FRONT) == PION_SCORE
			&& token_side_score(SIDE_REAR) == PION_SCORE) {
			err = strat_place_token_auto(strat_infos.slot[i][j].x,
									           strat_infos.slot[i][j].y, &side, GO_FORWARD);
		}
		else if(token_side_score(SIDE_FRONT) == PION_SCORE) {
			err = strat_place_token(strat_infos.slot[i][j].x,
									           strat_infos.slot[i][j].y, SIDE_FRONT, GO_FORWARD);
		}
		else if(token_side_score(SIDE_REAR) == PION_SCORE) {
			err = strat_place_token(strat_infos.slot[i][j].x,
									           strat_infos.slot[i][j].y, SIDE_REAR, GO_FORWARD);
		}
		//if (!TRAJ_SUCCESS(err))
		//	ERROUT(err);		
	}
	
	/* goto the other place position */
retry_place_second:
	if(color == get_color()) {
		/* our side */
		i = COLOR_I(1);
		j = (y_is_more_than(1050)? 2 : 4);
	}
	else {
		/* opponent side */
		if(!y_is_more_than(1050)
			&& (strat_infos.slot[COLOR_I(5)][5].flags & SLOT_BUSY) == 0) {

			i = COLOR_I(5);
			j = 4;			
		}
		else {
			i = COLOR_I(6);
			j = 1;
		}
	}
//	trajectory_goto_xy_abs(&mainboard.traj, strat_infos.slot[i][j].x, strat_infos.slot[i][j].y);
//	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	err = goto_and_avoid(strat_infos.slot[i][j].x, strat_infos.slot[i][j].y,
								TRAJ_FLAGS_NO_NEAR, TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err)) {
		if((x_is_more_than(1500) && opp_x_is_more_than(450)) 
			|| (!x_is_more_than(1500) && !opp_x_is_more_than(450))) {
			color = (color == get_color()? get_opponent_color() : get_color());
		}
		goto retry_place_second;
	}
	else if (TRAJ_SUCCESS(err)) {

		/* place */
		if(color == get_color()) {
			/* our side */
			i = COLOR_I(1);
			j = (y_is_more_than(1050)? 5 : 1);
		}
		else {
			/* opponent side */
			if(y_is_more_than(1050)) {
				/* on safe slot */
				i = COLOR_I(5);
				j = 5;			
			}
			else {	
				if((strat_infos.slot[COLOR_I(4)][0].flags & SLOT_BUSY) == 0) {
					/* near wall */
					i = COLOR_I(4);
					j = 0;
				}
				else {
					/* on bonus */
					i = COLOR_I(5);
					j = 1;
				}
			}
		}

		/* XXX we suppose that both are pions */
		err = strat_place_token_auto(strat_infos.slot[i][j].x,
								           strat_infos.slot[i][j].y, &side, GO_FORWARD);
		//if (!TRAJ_SUCCESS(err))
		//	ERROUT(err);		
	}


	/* place tokens if we have */
	if( strat_get_num_tokens() !=0 ) {
		/* XXX never will be case */
		i2c_slavedspic_mode_token_out(SIDE_REAR);		i2c_slavedspic_mode_token_out(SIDE_FRONT);
	}

	/* pickup two figures */
retry_pickup_fig1:
	if( strat_get_num_tokens() < 2) {
		err = strat_pickup_green_token(TYPE_FIGURE, color);
		if (!TRAJ_SUCCESS(err)) {
			if((x_is_more_than(1500) && opp_x_is_more_than(450)) 
				|| (!x_is_more_than(1500) && !opp_x_is_more_than(450))) {
				color = (color == get_color()? get_opponent_color() : get_color());
			}
			goto retry_pickup_fig1;
		}
	}


retry_pickup_fig2:
	if( strat_get_num_tokens() < 2) {
		err = strat_pickup_green_token(TYPE_FIGURE, color);
		if (!TRAJ_SUCCESS(err)) {
			if((x_is_more_than(1500) && opp_x_is_more_than(450)) 
				|| (!x_is_more_than(1500) && !opp_x_is_more_than(450))) {
				color = (color == get_color()? get_opponent_color() : get_color());
			}
			goto retry_pickup_fig2;
		}
	}

	/* goto in area */
	if(color == get_color()) {
		/* our side */
		i = COLOR_I(2);
	}
	else {
		/* opponent side */
		i = COLOR_I(5);
	}

	if(get_opponent_xy(&x_opp, &y_opp) == -1) {

		/* depends on robot position */
		if(y_is_more_than(1050)) {
			trajectory_goto_xy_abs(&mainboard.traj, strat_infos.slot[i][0].x, 1400);
		}
		else {
			trajectory_goto_xy_abs(&mainboard.traj, strat_infos.slot[i][0].x, 700);
		}
	}
	else {
		/* depends on opponent position */
		if(y_opp < 1050) {
			trajectory_goto_xy_abs(&mainboard.traj, strat_infos.slot[i][0].x, 1400);
		}
		else {
			trajectory_goto_xy_abs(&mainboard.traj, strat_infos.slot[i][0].x, 700);
		}

	}
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

end:	
	/* bounding box */
	strat_set_bounding_box(AREA_BBOX_4X4);
	//mirrors_set_mode(MODE_HIDE_MIRRORS);
	strat_set_speed(old_spdd, old_spda);
	return err;
}

