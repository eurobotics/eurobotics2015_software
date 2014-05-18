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
 *  Javier Bali�as Santos <javier@arc-robots.org> and Silvia Santano
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
#include <trajectory_manager_utils.h>
//#include <trajectory_manager_core.h>
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
#include "../maindspic/strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"
#include "cmdline.h"


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)


/* Add here the main strategic, the inteligence of robot */

#if notyet /* TODO 2014 */

/* return 1 if is a valid zone and 0 otherwise */
uint8_t strat_is_valid_zone(uint8_t zone_num)
{
#define OPP_WAS_IN_ZONE_TIMES	

	//static uint16_t opp_times[ZONES_MAX];
	//static microseconds opp_time_us = 0;

	/* discard actual zone */
	//if(strat_infos.current_zone == zone_num)
	//	return 0;

	/* discard down side zones depends on strat config */
	if((strat_infos.conf.flags & ENABLE_DOWN_SIDE_ZONES) == 0 
		&& strat_infos.zones[zone_num].init_y < (AREA_Y/2)
		&& zone_num != ZONE_SHIP_OUR_DECK_2
		&& zone_num != ZONE_SHIP_OUR_DECK_1 )
		return 0;
/*
	else if((strat_infos.conf.flags & ENABLE_DOWN_SIDE_ZONES) 
		&& strat_infos.zones[zone_num].init_y > (AREA_Y/2) 
		&& zone_num != ZONE_SHIP_OUR_DECK_2
		&& zone_num != ZONE_SHIP_OUR_DECK_1 )
		return 0;
*/
	/* discard if opp is in zone */
	if(opponent_is_in_area(	COLOR_X(strat_infos.zones[zone_num].x_up),strat_infos.zones[zone_num].y_up,
								COLOR_X(strat_infos.zones[zone_num].x_down),	strat_infos.zones[zone_num].y_down)) {

#if 0
		if(time_get_us2() - opp_time_us < 100000UL)
		{
			opp_time_us = time_get_us2();

			opp_times[zone_num]++;
			if(opp_times[zone_num] > OPP_WAS_IN_ZONE_TIMES)
				strat_infos.zones[zone_num].flags |= ZONE_CHECKED_OPP;
		}
#endif
		DEBUG(E_USER_STRAT, "Discarted zone %s, opp inside", numzone2name[zone_num]);
		return 0;
	}

	/* discard our checked zones */
	if(strat_infos.zones[zone_num].flags & ZONE_CHECKED)
		return 0;	

	/* discard opp checked zones */
	if((strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP)
		&& zone_num != ZONE_SHIP_OUR_CAPTAINS_BEDRROM 
		&& zone_num != ZONE_SHIP_OUR_HOLD 
		&& zone_num != ZONE_SHIP_OUR_DECK_2 
		&& zone_num != ZONE_SHIP_OUR_DECK_1 )

		return 0;	

	/* discard avoid zones */
	if(strat_infos.zones[zone_num].flags & ZONE_AVOID)
		return 0;	

	/* if we have treasure on mouth, we only can send messages, save treasure on ship or pickup middle coins group */
	if(strat_infos.treasure_in_mouth) {
		if(zone_num != ZONE_SHIP_OUR_CAPTAINS_BEDRROM
			&& zone_num != ZONE_SHIP_OUR_DECK_1
			&& zone_num != ZONE_SHIP_OUR_DECK_2
			&& zone_num != ZONE_SHIP_OUR_HOLD
			&& zone_num != ZONE_SAVE_TREASURE
			&& zone_num != ZONE_MIDDLE_COINS_GROUP
			&& zone_num != ZONE_OUR_BOTTLE_1
			&& zone_num != ZONE_OUR_BOTTLE_2 )
		
		return 0;
	}
	/* if we have not treasure on mouth, we have not to save treasure any where */
	else {
		if(zone_num == ZONE_SHIP_OUR_CAPTAINS_BEDRROM
		|| zone_num == ZONE_SHIP_OUR_DECK_1
		|| zone_num == ZONE_SHIP_OUR_DECK_2
		|| zone_num == ZONE_SAVE_TREASURE)
		
		return 0;
	}

	/* TODO depending on goldbars in boot */
	if(!strat_infos.treasure_in_boot && zone_num == ZONE_SHIP_OUR_HOLD)
		return 0;

	return 1;
}

/* return new work zone, -1 if any zone is found */
int8_t strat_get_new_zone(void)
{
	uint16_t prio_max = 0;
	int8_t zone_num = -1;
	int8_t i;
	
	/* evaluate zones */
	for(i=0; i < ZONES_MAX; i++) 
	{
		/* check if is a valid zone */
		if(!strat_is_valid_zone(i))	
			continue;

		/* check priority */
		if(strat_infos.zones[i].prio > prio_max) {

			/* update zone candidate params */
			prio_max = strat_infos.zones[i].prio;
			zone_num = i;
		}
	}

	return zone_num;
}

/* return END_TRAJ if zone is reached, err otherwise */
uint8_t strat_goto_zone(uint8_t zone_num)
{
	//double d_rel = 0.0, a_rel_rad = 0.0;
	//uint8_t arm_type = 0;
	int8_t err;

	/* special cases */

	if(zone_num == ZONE_TOTEM_OPP_SIDE_2) {

		if(!opp_y_is_more_than(1200) && !opp2_y_is_more_than(1200))
			strat_limit_speed_disable();

		err = goto_and_avoid_forward(COLOR_X(strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

		strat_limit_speed_enable();
	}

	else if(zone_num == ZONE_MIDDLE_COINS_GROUP) {
		if(position_get_x_s16(&mainboard.pos) > (AREA_X/2))
			err = goto_and_avoid_forward((AREA_X - strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
		else
			err = goto_and_avoid_forward(strat_infos.zones[zone_num].init_x, strat_infos.zones[zone_num].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_BOTTLE) {
		err = goto_and_avoid_backward(COLOR_X(strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	}

	else if(zone_num == ZONE_SHIP_OUR_DECK_1 || zone_num == ZONE_SHIP_OUR_DECK_2) {
		err = goto_and_avoid_forward(COLOR_X(strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

	}
	/* by default */
	else {
		err = goto_and_avoid(COLOR_X(strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	}

	return err;
}


/* return END_TRAJ if the work is done, err otherwise */
uint8_t strat_work_on_zone(uint8_t zone_num)
{
	uint8_t err = END_TRAJ;
	int16_t x = strat_infos.zones[zone_num].x;
	int16_t y = strat_infos.zones[zone_num].y;

	static uint8_t first_time = 1;

#ifdef DEBUG_STRAT_SMART
	return END_TRAJ;
#endif

	if(strat_infos.zones[zone_num].type == ZONE_TYPE_TOTEM) {

#if 0
		if(zone_num == ZONE_TOTEM_OUR_SIDE_1) {
			err = strat_empty_totem_side(COLOR_X(x), y, STORE_BOOT, step_our_totem_1);

		}
		else if(zone_num == ZONE_TOTEM_OUR_SIDE_2) {

		}
		else if(zone_num == ZONE_TOTEM_OPP_SIDE_1) {

		}
		else if(zone_num == ZONE_TOTEM_OPP_SIDE_2) {

		}
#else
		err = strat_empty_totem_side(COLOR_X(x), y, STORE_BOOT, 0);
#endif
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_GOLDBAR) {
		err = strat_pickup_goldbar_floor(COLOR_X(x), y, STORE_BOOT);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_MAP) {
		/* TODO */
		err = END_TRAJ;
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_BOTTLE) {
		if(zone_num == ZONE_OUR_BOTTLE_1)
			x = x + 80;

		err = strat_send_message_bottle(COLOR_X(x), y);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_COIN) {
		err = strat_pickup_coins_floor(COLOR_X(x), y, ONE);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_COINS_GROUP) {
		err = strat_pickup_coins_floor(COLOR_X(x), y, GROUP);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_HOLD) {
		err = strat_save_treasure_in_hold_back(COLOR_X(x), y);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_DECK) {

		if(zone_num == ZONE_SHIP_OUR_DECK_2) {
			
			if(time_get_s() < LAST_SECONDS_TIME + 5) {

				if(mainboard.our_color == I2C_COLOR_PURPLE)
					err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_RIGHT);
				else
					err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_LEFT);
	
				if(!(first_time) && strat_infos.treasure_in_boot)
					err = strat_save_treasure_in_deck_back_blowing(COLOR_X(x), y);
				first_time = 0;
			}
			else {
				if(!(first_time) && strat_infos.treasure_in_boot)
					err = strat_save_treasure_in_deck_back_blowing(COLOR_X(x), y);
				first_time = 0;

				if(mainboard.our_color == I2C_COLOR_PURPLE)
					err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_RIGHT);
				else
					err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_LEFT);
			}
		}
		else if(zone_num == ZONE_SHIP_OUR_DECK_1) {
			
			if(mainboard.our_color == I2C_COLOR_PURPLE)
				err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_LEFT);
			else
				err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_RIGHT);

			if(strat_infos.treasure_in_boot)
				err = strat_save_treasure_in_deck_back_blowing(COLOR_X(x), y);
		}
		else {
			err = strat_stole_opp_treasure(COLOR_X(x), y);
		}	
	}


	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_CAPTAINS_BEDROOM) {
		/* TODO */
		err = END_TRAJ;
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_SAVE) {
		err = strat_save_treasure_generic(COLOR_X(x), y);
	}

	return err;

}

/* debug state machines step to step */
void state_debug_wait_key_pressed(void)
{
	if (!strat_infos.debug_step)
		return;

	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* smart play */
uint8_t strat_smart(void)
{
	int8_t zone_num;
	uint8_t err;

	/* XXX DEBUG STEP BY STEP */
	state_debug_wait_key_pressed();


	/* get new zone */
	if(time_get_s() > LAST_SECONDS_TIME 
		&& (strat_infos.zones[ZONE_OUR_BOTTLE_1].flags & ZONE_CHECKED)
		&& (strat_infos.zones[ZONE_OUR_BOTTLE_2].flags & ZONE_CHECKED))
		zone_num = ZONE_SHIP_OUR_DECK_2;
	else
		zone_num = strat_get_new_zone();

		
	if(zone_num == -1) {
		DEBUG(E_USER_STRAT, "No zone is found");
		DEBUG(E_USER_STRAT, "Down side zones enabled");	
		strat_infos.conf.flags |= ENABLE_DOWN_SIDE_ZONES;	
		strat_infos.conf.flags |= ENABLE_R2ND_POS;

		/* Enable all paths */
		DEBUG(E_USER_STRAT, "All possible paths enabled");	
		strat_set_bounding_box(-1);	
		return END_TRAJ;
	}

	/* goto zone */
	strat_infos.goto_zone = zone_num;
	strat_dump_infos(__FUNCTION__);
	
	err = strat_goto_zone(strat_infos.goto_zone);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT, "Can't reach zone %d", strat_infos.goto_zone);
		return END_TRAJ;
	}

	/* work on zone */
	strat_infos.last_zone = strat_infos.current_zone;
	strat_infos.current_zone = strat_infos.goto_zone;
	strat_dump_infos(__FUNCTION__);

	err = strat_work_on_zone(strat_infos.current_zone);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT, "Work on zone %d fails", strat_infos.current_zone);

		if(strat_infos.zones[zone_num].type == ZONE_TYPE_TOTEM) 
		{
			strat_infos.zones[zone_num].flags |= ZONE_CHECKED;
         i2c_slavedspic_mode_turbine_angle(0,200);
         i2c_slavedspic_wait_ready();
         i2c_slavedspic_mode_lift_height(30);
			i2c_slavedspic_wait_ready();
         i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM,I2C_FINGERS_MODE_HOLD,0);
			i2c_slavedspic_wait_ready();
         i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR,I2C_FINGERS_MODE_CLOSE,0);
         i2c_slavedspic_wait_ready();
		}
		return err;
	}

	/* special case */
//	if(strat_infos.current_zone == ZONE_SHIP_OUR_DECK_2 
//		&& strat_infos.zones[ZONE_SHIP_OUR_DECK_2].prio == ZONE_PRIO_80)
//		strat_infos.zones[ZONE_SHIP_OUR_DECK_2].prio = ZONE_PRIO_50;

	/* check the zone */

 	/* don't check if it's saving zone */
	if(strat_infos.zones[strat_infos.current_zone].type == ZONE_TYPE_HOLD 
		|| strat_infos.zones[strat_infos.current_zone].type == ZONE_TYPE_DECK
		|| strat_infos.zones[strat_infos.current_zone].type == ZONE_TYPE_CAPTAINS_BEDROOM) {
		DEBUG(E_USER_STRAT, "Work on zone %d successed!", strat_infos.current_zone);
		return END_TRAJ;
	}
	else	{
		strat_infos.zones[strat_infos.current_zone].flags |= ZONE_CHECKED;
	}

	DEBUG(E_USER_STRAT, "Work on zone %d successed!", strat_infos.current_zone);
	return END_TRAJ;
}


#endif /* notyet TODO 2014 */



uint8_t patrol_and_paint_fresco(void)
{
	#define BEGIN_FRESCO_X	1295
	#define REFERENCE_DISTANCE_TO_ROBOT 800
	static uint8_t fresco_done =0;
	int16_t d,d2;
	int16_t a,a2;
	int16_t opp_x, opp_y, opp2_x, opp2_y;
	
	get_opponent_da(&d,&a);
	get_opponent2_da(&d2,&a2);
	
	if(d2>REFERENCE_DISTANCE_TO_ROBOT && d>REFERENCE_DISTANCE_TO_ROBOT && fresco_done!=1)
		return (fresco_done=paint_fresco());
		
	else
	{
		get_opponent_xy(&opp_x, &opp_y);
		get_opponent2_xy(&opp2_x, &opp2_y);
		if(opp_y<300 || opp2_y<300 && fresco_done!=1)
			return (fresco_done=paint_fresco());
			
		else
			return patrol_between(COLOR_X(BEGIN_FRESCO_X),300,COLOR_X(BEGIN_FRESCO_X),900);
	}
}


uint8_t paint_fresco(void)
{
	static uint8_t state = 1;
	uint16_t old_spdd, old_spda, temp_spdd, temp_spda;
	uint8_t err = 0;
#define BEGIN_LINE_Y 	450
#define BEGIN_FRESCO_X	1295

	switch (state)
	{
		/* go in front of fresco */
		/*case 0:
			printf_P("State 0\n");
			trajectory_goto_forward_xy_abs (&mainboard.traj, 
										BEGIN_FRESCO_X, BEGIN_LINE_Y);
			err = wait_traj_end(TRAJ_FLAGS_STD);
			if (!TRAJ_SUCCESS(err))
					ERROUT(err);

			state ++;	
			return 0;		
			break;*/

		/* turn to fresco 1*/
		case 1:
			printf_P("State 1\n");
			trajectory_a_abs (&mainboard.traj, COLOR_A_ABS(90));
			err = wait_traj_end(TRAJ_FLAGS_STD);
			if (!TRAJ_SUCCESS(err))
					ERROUT(err);

			state ++;	
			return 0;		
			break;

		/* paint fresco 1*/
		case 2:
			printf_P("State 2\n");
			trajectory_goto_backward_xy_abs (&mainboard.traj, 
										COLOR_X(BEGIN_FRESCO_X), 430);
			err = wait_traj_end(TRAJ_FLAGS_STD);
			if (!TRAJ_SUCCESS(err))
					ERROUT(err);
			state ++;		
			return 0;	
			break;

		/* paint fresco 2*/
		case 3:
			printf_P("State 3\n");
			sensor_obstacle_enable();
			if (sensor_get (S_OBS_REAR_L) || sensor_get (S_OBS_REAR_R))
				ERROUT(END_OBSTACLE);
				
			trajectory_goto_backward_xy_abs (&mainboard.traj, 
										COLOR_X(BEGIN_FRESCO_X), ROBOT_CENTER_TO_BACK + 100);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			if (!TRAJ_SUCCESS(err))
					ERROUT(err);

			/* go backwards */
			strat_get_speed(&old_spdd, &old_spda);
			strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
			trajectory_d_rel(&mainboard.traj, -200);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

			strat_set_speed(old_spdd, old_spda);

			trajectory_goto_forward_xy_abs (&mainboard.traj, 
										COLOR_X(BEGIN_FRESCO_X), ROBOT_CENTER_TO_BACK + 100);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			if (!TRAJ_SUCCESS(err))
					ERROUT(err);
					
					
			trajectory_goto_forward_xy_abs (&mainboard.traj, COLOR_X(BEGIN_FRESCO_X),600);
			err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
			if (!TRAJ_SUCCESS(err))
					ERROUT(err);
					
			state++;
			return 1;
			break;
			
		default:
			break;
	}
end:
	return err;
}


uint8_t patrol_between(int16_t x1, int16_t y1,int16_t x2, int16_t y2)
{	
	#define REFERENCE_DISTANCE_TO_ROBOT 800
	int8_t opp_there, r2nd_there;
	int16_t opp_x, opp_y, opp2_x, opp2_y, reference_x;
	uint16_t old_spdd, old_spda, temp_spdd, temp_spda;
	uint8_t err = 0;
	int16_t d,d2;
	int16_t a,a2;
	
	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
  	strat_limit_speed_disable ();
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_VERY_SLOW);
	
	/* get robot coordinates */
	opp_there = get_opponent_xy(&opp_x, &opp_y);
	get_opponent2_xy(&opp2_x, &opp2_y);
	/*if(opp_there == -1 )
		printf_P("Opp is not there");*/
	
	get_opponent_da(&d,&a);
	get_opponent2_da(&d2,&a2);
	
	if(d<(REFERENCE_DISTANCE_TO_ROBOT))
	{
		if((opp_y_is_more_than(y1)&&!opp_y_is_more_than(y2))||(!opp_y_is_more_than(y1)&&opp_y_is_more_than(y2)))
		{
			trajectory_goto_xy_abs (&mainboard.traj,(x1+x2)/2,opp_y); 
		}
	}else if(d2<(REFERENCE_DISTANCE_TO_ROBOT))
	{
		if((opp2_y_is_more_than(y1)&&!opp2_y_is_more_than(y2))||(!opp2_y_is_more_than(y1)&&opp2_y_is_more_than(y2)))
		{
			trajectory_goto_xy_abs (&mainboard.traj,(x1+x2)/2,opp2_y); 
		}
	}
		
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
end:
	strat_set_speed(old_spdd, old_spda);	
	strat_limit_speed_enable();
	return err;
}

uint8_t shoot_mamooth(uint8_t balls_mamooth_1, uint8_t balls_mamooth_2)
{
#define BEGIN_LINE_Y 	450
#define BEGIN_MAMOOTH_X	750
#define SERVO_SHOOT_POS_UP 80
#define SERVO_SHOOT_POS_DOWN 300

    uint8_t err = 0;

	if(mainboard.our_color == I2C_COLOR_RED)
	{
		uint8_t aux=balls_mamooth_1;
		balls_mamooth_1=balls_mamooth_2;
		balls_mamooth_2=aux;
	}

	if(balls_mamooth_1 > 0)
	{
		trajectory_goto_forward_xy_abs (&mainboard.traj, COLOR_X(BEGIN_MAMOOTH_X), BEGIN_LINE_Y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);
	
		trajectory_a_abs (&mainboard.traj, COLOR_A_ABS(-90));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);
				
		printf_P("Shooting %d balls to mamooth 1\r\n", balls_mamooth_1);
		/* TODO: on slavedspic: possibility to select the number of balls to be thrown */
		#ifndef HOST_VERSION
			pwm_servo_set(&gen.pwm_servo_oc3, SERVO_SHOOT_POS_UP);
			pwm_servo_set(&gen.pwm_servo_oc4, SERVO_SHOOT_POS_DOWN);
			time_wait_ms(1000);
		#else
			time_wait_ms(2000);
		#endif
	}
	
	if(balls_mamooth_2 > 0)
	{
		trajectory_goto_forward_xy_abs (&mainboard.traj, COLOR_X(3000-BEGIN_MAMOOTH_X), BEGIN_LINE_Y);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
		trajectory_a_abs (&mainboard.traj, COLOR_A_ABS(-90));
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);
				
		printf_P("Shooting %d balls to mamooth 2\r\n", balls_mamooth_2);
		#ifndef HOST_VERSION
			pwm_servo_set(&gen.pwm_servo_oc3, SERVO_SHOOT_POS_UP);
			pwm_servo_set(&gen.pwm_servo_oc4, SERVO_SHOOT_POS_DOWN);
			time_wait_ms(1000);
		#else
			time_wait_ms(2000);
		#endif
	}
end:	
	return err;
}

