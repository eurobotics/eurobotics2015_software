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


void strat_event_tomato(void)
{
//	static uint16_t old_spdd, old_spda;
//	
//	if(strat_infos.conf.flags & STRAT_CONF_HARVEST_TOMATOES){	
//	
//		/* detect tomato */
//		if(((sensor_get(S_FAR_HI_R) && sensor_get(S_NEAR_LOW_R)) ||
//				(sensor_get(S_FAR_HI_L) && sensor_get(S_NEAR_LOW_L)) ) &&
//				(!sensor_get(S_BARRIER)))
//		{		
//				/* harvest tomato */
//				if((slavedspic.status == I2C_IDLE)){// && (strat_infos.tomato_event == OFF)){
//	
//					//DEBUG(E_USER_STRAT, "harvest tomato ON");
//					i2c_slavedspic_mode_harvest_tomato();		
//					
//					/* new speed */
//					strat_get_speed(&old_spdd, &old_spda);
//					strat_set_speed(300, old_spda);
//	
//					/* interrupt traj */									
//					trajectory_stop(&mainboard.traj);
//					interrupt_traj();				
//						
//					strat_infos.tomato_event = ON;			
//				}
//		}
//		else if((slavedspic.status == I2C_IDLE) && (strat_infos.tomato_event == ON)){
//			/* restore speed */	
//			strat_set_speed(old_spdd, old_spda);
//		
//			/* interrupt traj */
//			trajectory_stop(&mainboard.traj);
//			interrupt_traj();
//											
//			strat_infos.tomato_event = OFF;
//			
//			//DEBUG(E_USER_STRAT, "harvest tomato OFF");
//			
//		}
//	}
//	else if(strat_infos.tomato_event == ON){
//		if(slavedspic.status == I2C_IDLE){
//			/* restore speed */	
//			strat_set_speed(old_spdd, old_spda);										
//			strat_infos.tomato_event = OFF;		
//		}	
//	}
//	else
//		strat_infos.tomato_event = OFF;
//
}

void strat_enable_harvest_tomatoes(void)
{
//	WAIT_COND_OR_TIMEOUT((slavedspic.status == I2C_IDLE), 1000);
	strat_infos.conf.flags |= STRAT_CONF_HARVEST_TOMATOES;	
}

void strat_disable_harvest_tomatoes(void)
{
//	WAIT_COND_OR_TIMEOUT((slavedspic.status == I2C_IDLE), 1000);
	strat_infos.conf.flags &= (~STRAT_CONF_HARVEST_TOMATOES);	
}


int8_t strat_is_corn(uint8_t side, double robot_x, double robot_y, double robot_a)
{
#define SIDE_LEFT		0
#define SIDE_RIGHT	1
	
	double corn_x, corn_y;
	uint8_t i;
	
	/* corn position */
	if(side == SIDE_LEFT){
		corn_x = (double)CORN_LEFT_X;
		corn_y = (double)CORN_LEFT_Y;
	}
	else{
		corn_x = (double)CORN_RIGHT_X;
		corn_y = (double)CORN_RIGHT_Y;		
	}
	rotate(&corn_x, &corn_y, robot_a);
	corn_x += robot_x;
	corn_y += robot_y;


//	DEBUG(E_USER_STRAT, "corn position x = %.2f , y = %.2f", corn_x, corn_y);
	
	/* match with corn positions */
	for(i=0; i<18; i++){
		if((ABS(strat_infos.corn[i].x-(int16_t)corn_x) < CORN_MATCH_MARGIN) &&
			 (ABS(strat_infos.corn[i].y-(int16_t)corn_y) < CORN_MATCH_MARGIN)){
//			DEBUG(E_USER_STRAT, "corn %d matched position x = %d , y = %d", i, 
//						strat_infos.corn[i].x, strat_infos.corn[i].y);
//			DEBUG(E_USER_STRAT, "corn dif x = %d , y = %d",
//			 			i, ABS(strat_infos.corn[i].x-(int16_t)corn_x), ABS(strat_infos.corn[i].y-(int16_t)corn_y));
			return i;
		}
	}
	
	return -1;
}

void strat_event_static_corn(void)
{
	double robot_x, robot_y, robot_a;
	int8_t ret;
	
	if(strat_infos.conf.flags & STRAT_CONF_HARVEST_STATIC_CORNS)
	{	
		/* detect static corn */
		if(sensor_get(S_CORN_LEFT) && (strat_infos.static_corn_event == OFF)){
			
			/* save robot position */
			robot_x =	position_get_x_double(&mainboard.pos),
			robot_y = position_get_y_double(&mainboard.pos),
			robot_a = position_get_a_rad_double(&mainboard.pos);
			
			//DEBUG(E_USER_STRAT, "static left corn detected");
			
			ret = strat_is_corn(SIDE_LEFT, robot_x, robot_y, robot_a);
			if(ret != -1){
				//DEBUG(E_USER_STRAT, "static corn %d matched", ret);
				strat_infos.corn_detect_num = ret;
				trajectory_stop(&mainboard.traj);
				interrupt_traj();
				strat_infos.static_corn_event = LEFT_ON;
			}		
		}
		else if(sensor_get(S_CORN_RIGHT) && (strat_infos.static_corn_event == OFF)){
			/* save robot position */
			robot_x =	position_get_x_double(&mainboard.pos),
			robot_y = position_get_y_double(&mainboard.pos),
			robot_a = position_get_a_rad_double(&mainboard.pos);
			
			//DEBUG(E_USER_STRAT, "static right corn detected");
			
			ret = strat_is_corn(SIDE_RIGHT, robot_x, robot_y, robot_a);
			if(ret != -1){
				DEBUG(E_USER_STRAT, "static corn %d matched", ret);
				strat_infos.corn_detect_num = ret;
				trajectory_stop(&mainboard.traj);
				interrupt_traj();
				strat_infos.static_corn_event = RIGHT_ON;
			}		
		}
	}
	else
		strat_infos.static_corn_event = OFF;
}

void strat_enable_harvest_static_corns(void)
{
	strat_infos.conf.flags |= STRAT_CONF_HARVEST_STATIC_CORNS;	
}

void strat_disable_harvest_static_corns(void)
{
	strat_infos.conf.flags &= (~STRAT_CONF_HARVEST_STATIC_CORNS);	
}

void strat_event_fall_corn(void)
{
//	if(strat_infos.conf.flags & STRAT_CONF_HARVEST_FALL_CORNS)
//	{		
//		/* detect fall corn */
//		if(((!sensor_get(S_FAR_HI_R) && sensor_get(S_NEAR_LOW_R)) ||
//				(!sensor_get(S_FAR_HI_L) && sensor_get(S_NEAR_LOW_L)) ) &&
//				(!sensor_get(S_BARRIER)))
//		{
//			
//			/* harvest corn */
//			if((slavedspic.status == I2C_IDLE) && (strat_infos.fall_corn_event == 0)){
//				//DEBUG(E_USER_STRAT, "harvest fall corn");
//				i2c_slavedspic_mode_harvest_corn();	
//				strat_infos.fall_corn_event = ON;				
//			}
//		}
//			
//	}
//	else
//			strat_infos.fall_corn_event = OFF;
}

void strat_enable_harvest_fall_corns(void)
{
	strat_infos.conf.flags |= STRAT_CONF_HARVEST_FALL_CORNS;	
}

void strat_disable_harvest_fall_corns(void)
{
	strat_infos.conf.flags &= (~STRAT_CONF_HARVEST_FALL_CORNS);	
}


uint8_t strat_harvest_corn(uint8_t num)
{
	
	uint8_t err;
	double a, d;
	uint16_t old_spdd, old_spda;
	int16_t d_back;

	strat_infos.event_elements_enable = 0;

	/* go	backward */
	trajectory_d_rel(&mainboard.traj, -150);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	/* turn to corn */
	abs_xy_to_rel_da(strat_infos.corn[num].x, strat_infos.corn[num].y , &d, &a);
	trajectory_a_rel(&mainboard.traj, DEG(a));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	/* go fordward near */
	d_back = (d-60);
	trajectory_d_rel(&mainboard.traj, (d-60));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	/* go fordward slow harvesting */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(200, old_spda);

	i2c_slavedspic_mode_harvest_corn();	
	d_back += 100;
	trajectory_d_rel(&mainboard.traj, 100);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
//	WAIT_COND_OR_TIMEOUT((slavedspic.status == I2C_IDLE), 2000);
	
//	/* if corn still in entrance */
//	if(sensor_get(S_BARRIER) && !sensor_get(S_OBSTACLE_FRONT))
//	{
//		i2c_slavedspic_mode_harvest_corn();
//		wait_ms(100);
//
//		d_back += 100;
//		trajectory_d_rel(&mainboard.traj, 100);
//		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);		
//		WAIT_COND_OR_TIMEOUT((slavedspic.status == I2C_IDLE), 3000);	
//	}

	/* go backward distance */
	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	trajectory_d_rel(&mainboard.traj, d_back);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	/* turn to begin */
	trajectory_a_rel(&mainboard.traj, DEG(-a));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	strat_infos.event_elements_enable = 1;

	return err;		
}

