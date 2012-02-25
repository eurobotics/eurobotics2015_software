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
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>
#include <rdline.h>

#include "../common/i2c_commands.h"
#include "actuator.h"
#include "ax12_user.h"
#include "state.h"
#include "main.h"

/* XXX ax12 management is done out of scheduler because there aren`t
 * other scheduler process that uses the servos ax12.
 */

#if 0
/* put in/out a token or nice token turn right/left ;) */
void belts_mode_set(uint8_t side, uint8_t mode, uint16_t speed)
{
	uint8_t ax12_left_id, ax12_right_id;

	/* set ax12 id */
	if(side == BELTS_SIDE_FRONT){
		ax12_left_id = AX12_FRONT_BELT_L;
		ax12_right_id = AX12_FRONT_BELT_R;
	}
	else{
		ax12_left_id = AX12_REAR_BELT_L;
		ax12_right_id = AX12_REAR_BELT_R;
	}

	/* manage ax12 servos */
	if(mode == BELTS_MODE_IN){
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_MOVING_SPEED_L, speed);		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_MOVING_SPEED_L, 0x400|speed);
	}
	else if(mode == BELTS_MODE_OUT){
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_MOVING_SPEED_L, 0x400|speed);		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_MOVING_SPEED_L, speed);
	}
	else if(mode == BELTS_MODE_RIGHT){
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_MOVING_SPEED_L, 0x400|speed);		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_MOVING_SPEED_L, 0x400|speed);
	}
	else if(mode == BELTS_MODE_LEFT){
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_MOVING_SPEED_L, speed);		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_MOVING_SPEED_L, speed);
	}
	else{
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_MOVING_SPEED_L, 0);		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_MOVING_SPEED_L, 0);
	}}	


/* get average load of a pair of belts */
uint16_t belts_load_get(uint8_t side)
{
	uint8_t ax12_left_id, ax12_right_id;
	uint16_t ax12_left_load, ax12_right_load;

	/* set ax12 id */
	if(side == BELTS_SIDE_FRONT){
		ax12_left_id = AX12_FRONT_BELT_L;
		ax12_right_id = AX12_FRONT_BELT_R;
	}
	else{
		ax12_left_id = AX12_REAR_BELT_L;
		ax12_right_id = AX12_REAR_BELT_R;
	}

	/* get load of each ax12 servo */
	ax12_user_read_int(&gen.ax12, ax12_left_id, AA_PRESENT_LOAD_L, &ax12_left_load);
	ax12_user_read_int(&gen.ax12, ax12_right_id, AA_PRESENT_LOAD_L, &ax12_right_load);

	/* absolute load value */
	ax12_left_load &= (~0xFC00);
	ax12_right_load &= (~0xFC00);

	/* return average load */
	//return (uint16_t)((ax12_left_load+ax12_right_load)/2);

	/* return max load */
	/*
	if(ax12_left_load > ax12_right_load)
		return (uint16_t)ax12_left_load;
	else
		return (uint16_t)ax12_right_load;
	*/

	/* XXX return only left load because right front 
	 * is very hard
	 */
	if(side == BELTS_SIDE_FRONT)
		return (uint16_t)ax12_left_load;
	else
		return (uint16_t)ax12_left_load;
}

/* set position of a mirror (left or right) and update cmd position on slavedspic structure */
void mirror_pos_set(uint8_t side, uint16_t pos)
{
	if(side == I2C_MIRROR_SIDE_RIGHT) {
		ax12_user_write_int(&gen.ax12, AX12_RIGHT_MIRROR, AA_GOAL_POSITION_L, pos);
		slavedspic.mirror_cmd_pos_right = pos;
	}
	else {
		ax12_user_write_int(&gen.ax12, AX12_LEFT_MIRROR, AA_GOAL_POSITION_L, pos);
		slavedspic.mirror_cmd_pos_left = pos;
	}
}
#endif

void actuator_init(void)
{
#ifdef notyet
	/* global ax12 servos init, set free running mode */	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x00);	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CW_ANGLE_LIMIT_L, 0x00);	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CCW_ANGLE_LIMIT_L, 0x00);
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_SHUTDOWN, 0x24);	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_LED, 0x24);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_MOVING_SPEED_L, 0);

	/* specific config for mirror ax12, angle is limited */ 
	ax12_user_write_byte(&gen.ax12, AX12_RIGHT_MIRROR, AA_TORQUE_ENABLE, 0xFF);	ax12_user_write_int(&gen.ax12, AX12_RIGHT_MIRROR, AA_CW_ANGLE_LIMIT_L, 0x00);	ax12_user_write_int(&gen.ax12, AX12_RIGHT_MIRROR, AA_CCW_ANGLE_LIMIT_L, 0x3FF);
	ax12_user_write_int(&gen.ax12, AX12_RIGHT_MIRROR, AA_MOVING_SPEED_L, 0x3FF);

	ax12_user_write_byte(&gen.ax12, AX12_LEFT_MIRROR, AA_TORQUE_ENABLE, 0xFF);	ax12_user_write_int(&gen.ax12, AX12_LEFT_MIRROR, AA_CW_ANGLE_LIMIT_L, 0x00);	ax12_user_write_int(&gen.ax12, AX12_LEFT_MIRROR, AA_CCW_ANGLE_LIMIT_L, 0x3FF);
	ax12_user_write_int(&gen.ax12, AX12_LEFT_MIRROR, AA_MOVING_SPEED_L, 0x3FF);#endif
}
