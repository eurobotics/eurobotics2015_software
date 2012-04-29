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
#include "actuator.h"
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


uint8_t strat_goto_and_avoid_harvesting(int16_t x, int16_t y, uint8_t harvest_flags)
{	
	uint8_t err;
	static uint8_t retry_flag=0;
	
	/* set strat conf */
	strat_infos.conf.flags |= harvest_flags;

 retry:
	err = goto_and_avoid(x, y, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	
	if(err == END_INTR){

//		/* manage static corns events */
//		if((strat_infos.conf.flags & STRAT_CONF_HARVEST_STATIC_CORNS) &&
//				strat_infos.static_corn_event){			
//			
//			/* harvest static corn */
//			strat_harvest_corn(strat_infos.corn_detect_num);			
//			
//			retry_flag = 1;
////			goto retry;
//		}


		/* manage fall corns event */
		if((strat_infos.conf.flags & STRAT_CONF_HARVEST_FALL_CORNS)&& (strat_infos.fall_corn_event == ON)){
			strat_infos.fall_corn_event == OFF;
			retry_flag = 1;	
//			goto retry;
		}
		else if((strat_infos.conf.flags & STRAT_CONF_HARVEST_TOMATOES)){
			retry_flag = 1;
//			goto retry;
		}
		
		
		if(retry_flag){
			retry_flag = 0;
			goto retry;
		}
	}
	/* clear strat conf */
	strat_infos.conf.flags &= (~harvest_flags);

	return err;
}


uint8_t strat_harvest_oranges(void)
{
	uint8_t err;
	uint16_t old_spdd, old_spda;

	strat_infos.event_elements_enable = 0;

	i2c_slavedspic_mode_show_arm();	

	/* goto rampe center */
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(500), 250);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	/* goto rampe center in angle */
	trajectory_a_abs(&mainboard.traj, COLOR_A2(0));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	/* goto up rampe */
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1010), 250);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	/* change aceleration and angle speed */
	quadramp_set_2nd_order_vars(&mainboard.angle.qr, 5, 5);
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST, 700);

	/* orange 1 */
	trajectory_a_abs(&mainboard.traj, COLOR_A2(23));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	i2c_slavedspic_mode_harvest_orange(60, -50, 0);
	wait_ms(100);
//	WAIT_COND_OR_TIMEOUT(((slavedspic.status==I2C_IDLE)), 5000);    
	
	/* orange 3 */
	trajectory_a_abs(&mainboard.traj, COLOR_A2(5));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	i2c_slavedspic_mode_harvest_orange(40, -80, 0); //40
	wait_ms(100);
//	WAIT_COND_OR_TIMEOUT(((slavedspic.status==I2C_IDLE)), 5000);    
	
	/* orange 4 */
	trajectory_a_abs(&mainboard.traj, COLOR_A2(-9));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	i2c_slavedspic_mode_harvest_orange(70, -75, 0); //-60
	wait_ms(100);
//	WAIT_COND_OR_TIMEOUT(((slavedspic.status==I2C_IDLE)), 5000);    

	/* orange 6 */
	trajectory_a_abs(&mainboard.traj, COLOR_A2(-25));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	i2c_slavedspic_mode_harvest_orange(30, -30, 0); // 50 -30?
	wait_ms(100);
//	WAIT_COND_OR_TIMEOUT(((slavedspic.status==I2C_IDLE)), 5000);    
	
	/* goto rampe center in angle */
	trajectory_a_abs(&mainboard.traj, COLOR_A2(0));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	/* goto out rampe */
	trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(350), 250);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	/* change aceleration and angle speed */
	quadramp_set_2nd_order_vars(&mainboard.angle.qr, 10, 10);
	strat_set_speed(old_spdd, old_spda);

	/* goto rampe center in angle */
	trajectory_a_abs(&mainboard.traj, COLOR_A2(90));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	strat_infos.event_elements_enable = 1;

	return err;
}

uint8_t strat_goto_avoid_retry(int16_t x, int16_t y, uint8_t tries)
{
	uint8_t err;
	uint8_t n_tries = tries;
	
 retry:	
	err = goto_and_avoid(x, y, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
 	
 	if (!TRAJ_SUCCESS(err)){
	 	n_tries--;
		if(n_tries != 0)
			goto retry;
		else
			return err;
	}
	
	return err;
}

uint8_t strat_goto_basket(void)
{
	uint8_t err;
	uint8_t state = 0;
	double a, d;

	strat_infos.event_elements_enable = 0;

	while(1){
		
	switch(state){
#define BEGIN_POS 				0
#define SECOND_BEGIN_POS 	1
#define TEST_ANGLE 				10
#define TEST_ANGLE_SECOND 11
#define BASKET_POS  			20
		case BEGIN_POS:
			err = strat_goto_avoid_retry(COLOR_X(X(10)), Y(5), 3); 	
		 	if (TRAJ_SUCCESS(err))
				state = TEST_ANGLE;
			else
				state = SECOND_BEGIN_POS;
			break;

		case SECOND_BEGIN_POS:
			err = strat_goto_avoid_retry(COLOR_X(X(10)), Y(3), 3); 	
		 	if (TRAJ_SUCCESS(err))
				state = TEST_ANGLE_SECOND;
			else
				state = BEGIN_POS;
			break;

		case TEST_ANGLE:
			abs_xy_to_rel_da(COLOR_X(X(11)), Y(7), &d, &a);

			if((a < RAD(75)) && (a > RAD(-75))){
				err = strat_goto_avoid_retry(COLOR_X(X(11)), Y(7), 2);
			 	
			 	if (TRAJ_SUCCESS(err))
					state = BASKET_POS;
				else
					state = SECOND_BEGIN_POS;				 	
		 	}	
		 	else{
				err = strat_goto_avoid_retry(COLOR_X(X(11)), Y(3), 1);
			 	
			 	if (TRAJ_SUCCESS(err)){
					err = strat_goto_avoid_retry(COLOR_X(X(11)), Y(7), 1);
				 	if (TRAJ_SUCCESS(err))
						state = BASKET_POS;
					else
						state = SECOND_BEGIN_POS;
				}
				else
					state = SECOND_BEGIN_POS;
		 	}
			break;	
				
		case TEST_ANGLE_SECOND:
			abs_xy_to_rel_da(COLOR_X(X(11)), Y(7), &d, &a);
			if((a < RAD(75)) && (a > RAD(-75))){
				err = strat_goto_avoid_retry(COLOR_X(X(11)), Y(7), 2);
			 	
			 	if (TRAJ_SUCCESS(err))
					state = BASKET_POS;
				else
					state = BEGIN_POS;				 	
		 	}	
		 	else{
				err = strat_goto_avoid_retry(COLOR_X(X(11)), Y(0), 1);
			 	
			 	if (TRAJ_SUCCESS(err)){
					err = strat_goto_avoid_retry(COLOR_X(X(11)), Y(7), 1);
				 	if (TRAJ_SUCCESS(err))
						state = BASKET_POS;
					else
						state = BEGIN_POS;
				}
				else
					state = SECOND_BEGIN_POS;
		 	}
			break;
			
		case BASKET_POS:
			abs_xy_to_rel_da(COLOR_X(X(11)), Y(9), &d, &a);
			if((a < RAD(30)) && (a > RAD(-30))){		
				err = strat_goto_avoid_retry(COLOR_X(X(11)), (Y(9)-10), 10); 	
			 	if (TRAJ_SUCCESS(err)){
					trajectory_a_rel(&mainboard.traj, COLOR_A(-10));
					err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
					ball_lids_open();
					i2c_slavedspic_mode_out_corns();
					wait_ms(200);
//					WAIT_COND_OR_TIMEOUT(((slavedspic.status==I2C_IDLE)), 5000);    
					i2c_slavedspic_mode_hide_arm();
					wait_ms(200);
//					WAIT_COND_OR_TIMEOUT(((slavedspic.status==I2C_IDLE)), 5000);    

					trajectory_d_rel(&mainboard.traj, 120);
					err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

					trajectory_d_rel(&mainboard.traj, -120);
					err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
										
					strat_infos.event_elements_enable = 1;
					return err;
				}
				else
					state = BEGIN_POS;
			}			
			else
				state = BEGIN_POS;
				
			break;			
			
		default:
			state = BEGIN_POS;
			break;
	
		}
	}
	return err;
}

uint8_t strat_goto_avoid_harvesing_retry(int16_t x, int16_t y, uint8_t tries)
{
	uint8_t err;
	uint8_t n_tries = tries;
	
 retry:	
	err = strat_goto_and_avoid_harvesting(x, y,
	 (STRAT_CONF_HARVEST_TOMATOES | STRAT_CONF_HARVEST_FALL_CORNS));// | STRAT_CONF_HARVEST_STATIC_CORNS));
 	
 	if (!TRAJ_SUCCESS(err)){
	 	n_tries--;
		if(n_tries != 0)
			goto retry;
		else
			return err;
	}
	
	return err;
}


uint8_t test_angle(int16_t x_tarjet, int16_t y_tarjet ,int16_t x1, int16_t y1, int16_t x2, int16_t y2,uint8_t tries )
{
	uint8_t err;
	uint8_t state = 0;
	double a, d;

	abs_xy_to_rel_da(x_tarjet, y_tarjet , &d, &a);

	if((a < RAD(75)) && (a > RAD(-75))){
		err = strat_goto_avoid_retry(x1, y1, tries);
 	}	
 	else{
		err = strat_goto_avoid_retry(x2, y2, tries);
	}
	
	return err;
}

uint8_t strat_goto_diagonal(void)
{
	uint8_t err;
	uint8_t state = 0;
	double robot_a;

	strat_infos.event_elements_enable = 1;

	err = strat_goto_avoid_harvesing_retry(COLOR_X(X(6)), Y(5), 3); 	
	err = test_angle(COLOR_X(X(8)), Y(7), 
									COLOR_X(X(6)), Y(5), 
									COLOR_X(X(7)), Y(3),5);

	err = strat_goto_avoid_harvesing_retry(COLOR_X(X(8)), Y(7), 3); 	
	err = test_angle(COLOR_X(X(10)), Y(9),
									COLOR_X(X(8)), Y(7), 
									COLOR_X(X(9)), Y(5),5);


	err = strat_goto_avoid_harvesing_retry(COLOR_X(X(10)), Y(9), 3); 	
	//err = test_angle(COLOR_X(X()), Y(), COLOR_X(X()), Y(),5)

	strat_infos.event_elements_enable = 0;

	return err;	
}
