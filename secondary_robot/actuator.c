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

#include <aversive.h>
#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <ax12.h>

#include "i2c_protocol.h"
#include "actuator.h"
#include "ax12_user.h"
#include "main.h"

void dac_set_and_save(void *dac, int32_t val)
{
#ifdef EUROBOT2011_BOARD
//#define RIGHT_MOTOR_OFFSET 2000 
#define LEFT_MOTOR_OFFSET  0
#else
#define RIGHT_MOTOR_OFFSET	0 
//#define LEFT_MOTOR_OFFSET  3500
#define LEFT_MOTOR_OFFSET  0
#endif

#define RIGHT_MOTOR_MAX		(65535-LEFT_MOTOR_OFFSET)
#define LEFT_MOTOR_MAX		(65535-RIGHT_MOTOR_OFFSET)
	
	if (dac == LEFT_MOTOR) {
		/* apply offset */
		//val = val > 0? (val + LEFT_MOTOR_OFFSET):(val - LEFT_MOTOR_OFFSET);

		/* we need to do the saturation here, before saving the value */
		if (val > LEFT_MOTOR_MAX)
			val = LEFT_MOTOR_MAX;
		if (val < -LEFT_MOTOR_MAX)
			val = -LEFT_MOTOR_MAX;

		/* save value */
		mainboard.pwm_l = val;
	}
	else if (dac == RIGHT_MOTOR){
		/* apply offset */
		//val = val > 0? (val + RIGHT_MOTOR_OFFSET):(val - RIGHT_MOTOR_OFFSET);

		/* we need to do the saturation here, before saving the value */
		if (val > RIGHT_MOTOR_MAX)
			val = RIGHT_MOTOR_MAX;
		if (val < -RIGHT_MOTOR_MAX)
			val = -RIGHT_MOTOR_MAX;

		/* save value */
		mainboard.pwm_r = val;
	}

	/* set value */
	pwm_mc_set(dac, val);
}


void actuators_init(void)
{
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_SHUTDOWN, 0x24);	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_LED, 0x24);
	/* specific config for mirror ax12, angle is limited */ 
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x1);	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CW_ANGLE_LIMIT_L, 0x00);	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CCW_ANGLE_LIMIT_L, 0x3FF);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_MOVING_SPEED_L, 0x3FF);
}


/* AX12 stuff */
#define AX12_PULLING_TIME_us		5000L
#define AX12_WINDOW_POSITION		15
#define AX12_BLOCKING_TIMEOUT_us	1300000L

typedef struct {
	uint8_t id;
	uint16_t pos;
} ax12_actuator_t;

ax12_actuator_t arm = {
	.id = AX12_ID_ARM,
};

ax12_actuator_t teeth = {
	.id = AX12_ID_TEETH,
};

/* set finger position depends on mode */
static uint8_t ax12_set_pos(ax12_actuator_t *ax12, uint16_t pos)
{
	uint8_t err;
		
	ax12->pos = pos;
	
	/* apply to ax12 */
	err = ax12_user_write_int(&gen.ax12, ax12->id, AA_GOAL_POSITION_L, ax12->pos);
	if(err) 	
		return err;

	return 0;
}

/* return 1 if mode is done */
static uint8_t ax12_check_mode_done(ax12_actuator_t *ax12)
{
	static microseconds us = 0;
	uint16_t ax12_pos;

	/* ax12 position pulling */
	if(time_get_us2() - us < AX12_PULLING_TIME_us)
		return 0;

	/* update time */
	us = time_get_us2();

	/* read positions */
	if(ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12_pos))
		return 0;

	/* check if position is inside window */
	if(ABS(ax12->pos - ax12_pos) < AX12_WINDOW_POSITION)	
		return END_TRAJ;
	
	return 0;
}

/* return END_TRAJ or END_BLOCKING */
static uint8_t ax12_wait_end(ax12_actuator_t *ax12)
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = ax12_check_mode_done(ax12);

	return ret;
}

static uint8_t ax12_disable_torque(ax12_actuator_t *ax12) 
{
	uint8_t err;
	err =ax12_user_write_byte(&gen.ax12, ax12->id, AA_TORQUE_ENABLE, 0);

	if(err) 	
		return err;

	return 0;
}

static uint8_t ax12_enable_torque(ax12_actuator_t *ax12) {
	uint8_t err;
	err =ax12_user_write_byte(&gen.ax12, ax12->id, AA_TORQUE_ENABLE, 0xFF);

	if(err) 	
		return err;

	return 0;
}


inline uint8_t arm_set_pos(uint16_t pos){	return ax12_set_pos(&arm, pos);
}

inline uint8_t arm_wait_end(void) {
	return ax12_wait_end(&arm);
}

inline uint8_t arm_disable_torque(void){	return ax12_disable_torque(&arm);
}

inline uint8_t arm_enable_torque(void){	return ax12_enable_torque(&arm);
}


inline uint8_t teeth_set_pos(uint16_t pos){	return ax12_set_pos(&teeth, pos);
}

inline uint8_t teeth_wait_end(void) {
	return ax12_wait_end(&teeth);
}

