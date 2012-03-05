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

#define ACTUATORS_DEBUG(args...) 	DEBUG(E_USER_ACTUATORS, args)
#define ACTUATORS_NOTICE(args...) 	NOTICE(E_USER_ACTUATORS, args)
#define ACTUATORS_ERROR(args...) 	ERROR(E_USER_ACTUATORS, args)

#ifdef notyet
/**** lift functions ********************************************************/

void lift_hard_stop(struct cs_block *lift){	cs_set_consign(&lift->cs, encoders_dspic_get_value(LIFT_ENCODER));	lift->qr.previous_var = 0;	lift->qr.previous_out = encoders_dspic_get_value(LIFT_ENCODER);}
void lift_autopos(struct cs_block_t *lift)
{
#define AUTOPOS_SPEED			1
#define AUTOPOS_ACCEL			100
#define AUTOPOS_DIST_mm			400
#define AUTOPOS_BD_TIMEOUT_ms	5000

	int8_t ret;	int16_t kp, ki, kd;

	/* save and set PID constants */
	kp = pid_get_gain_P(&slavedspic.lift.pid);
	ki = pid_get_gain_I(&slavedspic.lift.pid);
	kd = pid_get_gain_D(&slavedspic.lift.pid);	pid_set_gains(&lift.pid, 2000, 0, 5000);
	/* set low speed, and hi acceleration for fast response */
	quadramp_set_1st_order_vars(&slavedspic.lift.qr, AUTOPOS_SPEED, AUTOPOS_SPEED);
	quadramp_set_2nd_order_vars(&slavedspic.lift.qr, AUTOPOS_ACCEL, AUTOPOS_ACCEL);
	ACTUATORS_DEBUG("Down speed and acceleration");
	/* goto zero with cs */
	maindspic.flags |= DO_CS;
	cs_set_consign(&slavedspic.lift.cs, (int32_t)(AUTOPOS_DIST_MM * LIFT_K_IMP_MM));
	ACTUATORS_DEBUG("Goto zero");	

	/* wait end blocking */
	ret = WAIT_COND_OR_TIMEOUT(bd_get(&slavedspic.lift.bd), AUTOPOS_BD_TIMEOUT_ms);
	if(!ret) {
		ACTUATORS_DEBUG("Blocking not reached");
		return;
	}

	/* reset encoder */
	encoders_dspic_set_value(LIFT_ENCODER, LIFT_CALIB_IMP_MAX);
	lift_hard_stop(&slavedspic.lift.cs);
	pid_reset(&slavedspic.lift.pid);
	bd_reset(&slavedspic.lift.pid);	ACTUATORS_DEBUG("End blocking, encoder is reset to zero");	
	/* restore speed, acceleration and PID */
	quadramp_set_1st_order_vars(&slavedspic.lift.qr, LIFT_SPEED, LIFT_SPEED);
	quadramp_set_2nd_order_vars(&slavedspic.lift.qr, LIFT_ACCEL, LIFT_ACCEL);
	pid_set_gains(&slavedspic.lift.pid, kp, ki, kd);
	ACTUATORS_DEBUG("Restored speed, acceleration and PID");
	ACTUATORS_DEBUG("%s ends", __FUNCTION__);

	/* set calibration flag */
	lift->calibrated = 1;
	return;}

void lift_set_height(struct cs_block_t *lift,  int16_t height_mm)
{
	/* check calibration flag */
	if(!lift->calibrated) {
		ACTUATORS_ERROR("Can't set height, lift is NOT CALIBRATED");
		return;
	}

	/* saturate heigh */
	if(heigh_mm > LIFT_HEIGH_MAX_mm)
		heigh_mm = LIFT_HEIGH_MAX_mm;
	if(heigh_mm < LIFT_HEIGH_MIN_mm)
		heigh_mm = LIFT_HEIGH_MIN_mm;

	/* apply consign */
	cs_set_consign(&lift->cs, (int32_t)(heigh_mm * LIFT_K_IMP_MM));
}

int16_t lift_get_height(struct cs_block_t *lift)
{
	return (int16_t)(encoders_dspic_get_value(LIFT_ENCODER)/LIFT_K_IMP_MM);
}

uint8_t lift_wait_heigh_reached(struct cs_block *lift)
{	uint8_t ret=0;
	while(1) 
	{
		/* test consign end */		if(cs_get_consign(&lift->cs) == cs_get_filtered_consign(&lift->cs)){			return END_LIFT;		}
		/* test blocking */		ret = bd_get(&lift->bd);		if(ret){			hard_stop(lift, LIFT_ENCODER);			pid_reset(&lift->pid);			bd_reset(&lift->bd);			ACTUATORS_ERROR("Lift ENDS BLOCKING");			return END_BLOCKING;		}	}}

/**** turbine funcions *********************************************************/

void turbine_power_on(turbine_t *turbine) {
	turbine->power = TURBINE_POWER_OFF;
	TURBINE_POWER_PIN = turbine->power;
}
void turbine_power_off(void) {
	turbine->power = TURBINE_POWER_OFF;
	TURBINE_POWER_PIN = turbine->power;
}

void turbine_set_angle(turbine_t *turbine, uint16_t angle_deg, uint16_t wait_ms)
{
	uint16_t angle_pos;

	/* position */
	/* TODO: set sign */
	angle_pos = TURBINE_POS_ANGLE_ZERO + (uint16_t)(angle_deg * TURBINE_K_POS_DEG);

	/* set and save consign */
	turbine->angle_pos = pwm_servo_set(TURBINE_ANGLE_PWM_SERVO, turbine->angle_pos);
	/* TODO: set sign */
	turbine->angle_deg = (turbine->angle_pos - TURBINE_POS_ANGLE_ZERO) / TURBINE_K_POS_DEG;

	/* wait to reach the position */
	if(wait_ms)
		time_wait_ms(wait_ms);
}
uint16_t turgine_get_angle(turbine_t *turbine) {
	return turbine->angle_deg;
}

void turbine_set_speed(turbine_t *turbine, uint16_t speed) {
	turbine->speed = pwm_servo_set(TURBINE_SPEED_PWM_SERVO, speed);
}
uint16_t turbine_get_speed(turbine_t *turbine) {
	return turbine->speed;
}

/**** trays funcions *********************************************************/


/**** mouth funcions *********************************************************/

struct {
	uint8_t type;
	uint8_t mode;
	uint8_t ax12_pos_l;
	uint8_t ax12_pos_r;
} mouth_t;

#define MOUTH_MODE_OPEN			0
#define MOUTH_MODE_CLOSE		1
#define MOUTH_MODE_HOLD			2
#define MOUTH_MODE_PUSH_IN		3

void mouth_set_mode(mouth_t *mouth, uint8_t mode)
{
	uint8_t ax12_left_id, ax12_right_id;

	/* set ax12 ids */
	if(type == MOUTH_TYPE_UP) {
		ax12_left_id = AX12_MOUTH_UP_L;
		ax12_right_id = AX12_MOUTH_UP_R;
	}
	else {
		ax12_left_id = AX12_MOUTH_DOWN_L;
		ax12_right_id = AX12_MOUTH_DOWN_R;
	}
		
	/* set ax12 possitions depends on mode */
	mouth->mode = mode;
	switch(mouth->mode) {
		case MOUTH_MODE_OPEN:
			mouth->ax12_pos_l = AX12_OPEN_L;
			mouth->ax12_pos_r = AX12_OPEN_R;
			break;

		case MOUTH_MODE_CLOSE:
			mouth->ax12_pos_l = AX12_CLOSE_L;
			mouth->ax12_pos_r = AX12_CLOSE_R;
			break;

		case MOUTH_MODE_HOLD:
			mouth->ax12_pos_l = AX12_HOLD_L;
			mouth->ax12_pos_r = AX12_HOLD_R;
			break;

		case MOUTH_MODE_PUSH_IN:
			mouth->ax12_pos_l = AX12_PUSH_IN_L;
			mouth->ax12_pos_r = AX12_PUSH_IN_R;
			break;

		default:
			ACTUATOR_ERROR("UNKNOW mouth mode);
			break;
	}
	
	/* apply to ax12 */
	ax12_user_write_int(&gen.ax12, ax12_left_id, AA_PRESENT_POSITION_L, mouth->ax12_pos_l);	ax12_user_write_int(&gen.ax12, ax12_right_id, AA_PRESENT_POSITION_L, mouth->ax12_pos_r);
}

uint8_t mouth_check_mode_done(mouth_t *mouth)
{
	static microseconds us = 0;
	int16_t ax12_pos_l = 0, ax12_pos_r = 0;
	uint8_t ax12_left_id, ax12_right_id;

	/* ax12 position pulling */
	if(time_get_us2() - us < PULLING_TIME_us)
		return 0;

	/* update time */
	us = time_get_us2;

	/* set ax12 ids */
	if(type == MOUTH_TYPE_UP) {
		ax12_left_id = AX12_MOUTH_UP_L;
		ax12_right_id = AX12_MOUTH_UP_R;
	}
	else {
		ax12_left_id = AX12_MOUTH_DOWN_L;
		ax12_right_id = AX12_MOUTH_DOWN_R;
	}

	/* read positions */
	ax12_user_read_int(&gen.ax12, ax12_left_id, AA_PRESENT_LOAD_L, &ax12_pos_l);
	ax12_user_read_int(&gen.ax12, ax12_right_id, AA_PRESENT_LOAD_L, &ax12_pos_r);

	/* check mode positions */
	if(ABS(mouth->ax12_pos_l - ax12_pol_l) < X && ABS(mouth->ax12_pos_l - ax12_pol_l) < X)	
		return 1;
	
	return 0;
}
#endif


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
