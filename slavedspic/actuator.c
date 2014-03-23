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
 *  Javier Bali�as Santos <javier@arc-robots.org>
 */

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>
#include <rdline.h>

#include "../common/i2c_commands.h"
#include "actuator.h"
#include "ax12_user.h"
#include "state.h"
#include "main.h"

#define ACTUATORS_DEBUG(args...) 	DEBUG(E_USER_ACTUATORS, args)
#define ACTUATORS_NOTICE(args...) 	NOTICE(E_USER_ACTUATORS, args)
#define ACTUATORS_ERROR(args...) 	ERROR(E_USER_ACTUATORS, args)

/* AX12 stuff */
#define AX12_PULLING_TIME_us		5000L
#define AX12_WINDOW_POSITION		15
#define AX12_BLOCKING_TIMEOUT_us	600000L



/**** lift functions ********************************************************/

/* stop without rampe */
void lift_hard_stop(void){	cs_set_consign(&slavedspic.lift.cs, encoders_dspic_get_value(LIFT_ENCODER));	slavedspic.lift.qr.previous_var = 0;	slavedspic.lift.qr.previous_out = encoders_dspic_get_value(LIFT_ENCODER);}
/* calibrate initial position */
void lift_calibrate(void)
{
#define AUTOPOS_SPEED			50
#define AUTOPOS_ACCEL			1
#define AUTOPOS_BIG_DIST_mm	50000
#define AUTOPOS_BD_TIMEOUT_ms	5000

	int8_t ret;	int16_t kp, ki, kd;

	/* save and set PID constants */
	kp = pid_get_gain_P(&slavedspic.lift.pid);
	ki = pid_get_gain_I(&slavedspic.lift.pid);
	kd = pid_get_gain_D(&slavedspic.lift.pid);	pid_set_gains(&slavedspic.lift.pid,  5000, 0, 0);
	/* set low speed, and hi acceleration for fast response */
	quadramp_set_1st_order_vars(&slavedspic.lift.qr, AUTOPOS_SPEED, AUTOPOS_SPEED);
	quadramp_set_2nd_order_vars(&slavedspic.lift.qr, AUTOPOS_ACCEL, AUTOPOS_ACCEL);
	ACTUATORS_DEBUG("Down speed and acceleration");
	/* goto zero with cs */
	slavedspic.flags |= DO_CS;
	//cs_set_consign(&slavedspic.lift.cs, (int32_t)(AUTOPOS_BIG_DIST_mm * LIFT_K_IMP_mm));
	cs_set_consign(&slavedspic.lift.cs, (int32_t)(AUTOPOS_BIG_DIST_mm));
	ACTUATORS_DEBUG("Goto zero");	

	/* wait end blocking */
	ret = WAIT_COND_OR_TIMEOUT(bd_get(&slavedspic.lift.bd), AUTOPOS_BD_TIMEOUT_ms);
	if(!ret) {
		ACTUATORS_DEBUG("Blocking not reached");
		return;
	}

	/* reset encoder */
	encoders_dspic_set_value(LIFT_ENCODER, LIFT_CALIB_IMP_MAX);
	lift_hard_stop();
	pid_reset(&slavedspic.lift.pid);
	bd_reset(&slavedspic.lift.bd);	ACTUATORS_DEBUG("End blocking, encoder is reset to zero");	
	/* restore speed, acceleration and PID */
	quadramp_set_1st_order_vars(&slavedspic.lift.qr, LIFT_SPEED, LIFT_SPEED);
	quadramp_set_2nd_order_vars(&slavedspic.lift.qr, LIFT_ACCEL, LIFT_ACCEL);
	pid_set_gains(&slavedspic.lift.pid, kp, ki, kd);

	ACTUATORS_DEBUG("Restored speed, acceleration and PID");
	ACTUATORS_DEBUG("%s ends", __FUNCTION__);

	/* set calibration flag */
	slavedspic.lift.calibrated = 1;
	return;}

/* set height in mm */
void lift_set_height(int32_t height_mm)
{
	/* check calibration flag */
	if(!slavedspic.lift.calibrated) {
		ACTUATORS_ERROR("Can't set height, lift is NOT CALIBRATED");
		return;
	}

	/* saturate heigh */
	if(height_mm > LIFT_HEIGHT_MAX_mm)
		height_mm = LIFT_HEIGHT_MAX_mm;
	if(height_mm < LIFT_HEIGHT_MIN_mm)
		height_mm = LIFT_HEIGHT_MIN_mm;

	/* apply consign */
	cs_set_consign(&slavedspic.lift.cs, (int32_t)(height_mm * LIFT_K_IMP_mm));
	slavedspic.lift.blocking = 0;
}

/* return heigh in mm */
int32_t lift_get_height(void)
{
	return (int32_t)(encoders_dspic_get_value(LIFT_ENCODER)/LIFT_K_IMP_mm);
}

/* return 1 if height reached, -1 if blocking and zero if no ends yet */
int8_t lift_check_height_reached(void)
{
	/* test consign end */	if(cs_get_consign(&slavedspic.lift.cs) == cs_get_filtered_consign(&slavedspic.lift.cs)){		return END_TRAJ;	}
	/* test blocking */	if(bd_get(&slavedspic.lift.bd)) {		lift_hard_stop();		pid_reset(&slavedspic.lift.pid);		bd_reset(&slavedspic.lift.bd);
		slavedspic.lift.blocking = 1;		ACTUATORS_ERROR("Lift ENDS BLOCKING");		return END_BLOCKING;	}

	return 0;}

/* return END_TRAJ or END_BLOCKING */
uint8_t lift_wait_end()
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = lift_check_height_reached();

	return ret;
}

#if 0

/**** combs funcions *********************************************************/

uint16_t combs_ax12_pos_l [COMBS_MODE_MAX] = {
	[COMBS_MODE_HIDE] 			= POS_COMB_L_HIDE, 
	[COMBS_MODE_OPEN] 			= POS_COMB_L_OPEN, 
	[COMBS_MODE_HARVEST_CLOSE] = POS_COMB_L_HARVEST_CLOSE, 
	[COMBS_MODE_HARVEST_OPEN] 	= POS_COMB_L_HARVEST_OPEN, 
};

uint16_t combs_ax12_pos_r [COMBS_MODE_MAX] = {
	[COMBS_MODE_HIDE] 			= POS_COMB_R_HIDE, 
	[COMBS_MODE_OPEN] 			= POS_COMB_R_OPEN, 
	[COMBS_MODE_HARVEST_CLOSE] = POS_COMB_R_HARVEST_CLOSE, 
	[COMBS_MODE_HARVEST_OPEN] 	= POS_COMB_R_HARVEST_OPEN, 
};

/* set finger position depends on mode */
int8_t combs_set_mode(combs_t *combs, uint8_t mode, int16_t pos_offset)
{
	uint8_t ax12_left_id, ax12_right_id, err1=0, err2=0;

	/* set ax12 ids */
	ax12_left_id = AX12_ID_COMB_L;
	ax12_right_id = AX12_ID_COMB_R;

	/* set ax12 possitions depends on mode and type */
	if(mode >= COMBS_MODE_MAX) {
		ACTUATORS_ERROR("Unknow COMBS MODE");
		return -1;
	}

   /* ax12 possitions */
	combs->mode_old = combs->mode;
	combs->mode = mode;
	combs->ax12_pos_l = combs_ax12_pos_l[combs->mode] + pos_offset;
	combs->ax12_pos_r = combs_ax12_pos_r[combs->mode] + pos_offset;

	/* set speed */
#if 0
   if(combs->mode == COMBS_MODE_CLOSE || combs->mode == COMBS_MODE_HOLD)) {
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_MOVING_SPEED_L, 300);
		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_MOVING_SPEED_L, 300);
	}
	else if(combs->type == COMBS_TYPE_TOTEM) {
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_MOVING_SPEED_L, 0x3ff);
		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_MOVING_SPEED_L, 0x3ff);
	}
#endif

	/* saturate to position range */
	if(combs->ax12_pos_l > combs_ax12_pos_l[COMBS_MODE_L_POS_MAX])
		combs->ax12_pos_l = combs_ax12_pos_l[COMBS_MODE_L_POS_MAX];
	if(combs->ax12_pos_l < combs_ax12_pos_l[COMBS_MODE_L_POS_MIN])
		combs->ax12_pos_l = combs_ax12_pos_l[COMBS_MODE_L_POS_MIN];

	if(combs->ax12_pos_r > combs_ax12_pos_r[COMBS_MODE_R_POS_MAX])
		combs->ax12_pos_r = combs_ax12_pos_r[COMBS_MODE_R_POS_MAX];
	if(combs->ax12_pos_r < combs_ax12_pos_r[COMBS_MODE_R_POS_MIN])
		combs->ax12_pos_r = combs_ax12_pos_r[COMBS_MODE_R_POS_MIN];
 
	/* apply to ax12 */
	err1 = ax12_user_write_int(&gen.ax12, ax12_left_id, AA_GOAL_POSITION_L, combs->ax12_pos_l);
   err2 = ax12_user_write_int(&gen.ax12, ax12_right_id, AA_GOAL_POSITION_L, combs->ax12_pos_r);

	/* update time for timeout and reset blocking */
	combs->blocking = 0;
	combs->time_us = time_get_us2();

	if(err1) return err1;
	if(err2) return err2;

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
int8_t combs_check_mode_done(combs_t *combs)
{
	static microseconds us = 0;
	uint16_t ax12_pos_l = 0, ax12_pos_r = 0;
	uint8_t ax12_left_id, ax12_right_id;

	/* ax12 position pulling */
	if(time_get_us2() - us < AX12_PULLING_TIME_us)
		return 0;

	/* update time */
	us = time_get_us2();

	/* set ax12 ids */
	ax12_left_id = AX12_ID_COMB_L;
	ax12_right_id = AX12_ID_COMB_R;

	/* read positions */
	if(ax12_user_read_int(&gen.ax12, ax12_left_id, AA_PRESENT_POSITION_L, &ax12_pos_l))
		return 0;

	if(ax12_user_read_int(&gen.ax12, ax12_right_id, AA_PRESENT_POSITION_L, &ax12_pos_r))
		return 0;

	/* check if positions are inside window */
	if(ABS(combs->ax12_pos_l - ax12_pos_l) < AX12_WINDOW_POSITION 
		&& ABS(combs->ax12_pos_r - ax12_pos_r) < AX12_WINDOW_POSITION)	
		return END_TRAJ;
	
	/* ax12 blocking timeout */
	if(time_get_us2() - combs->time_us > AX12_BLOCKING_TIMEOUT_us) {
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_GOAL_POSITION_L, ax12_pos_l);		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_GOAL_POSITION_L, ax12_pos_r);
		combs->blocking = 1;
		return END_BLOCKING;
	}

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t combs_wait_end(combs_t *combs)
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = combs_check_mode_done(combs);

	return ret;
}

/**** sticks funcions *********************************************************/
uint16_t stick_ax12_pos[STICK_TYPE_MAX][STICK_MODE_MAX] = {
	[STICK_TYPE_RIGHT][STICK_MODE_HIDE] 				= POS_STICK_R_HIDE,
	[STICK_TYPE_RIGHT][STICK_MODE_PUSH_FIRE] 			= POS_STICK_R_PUSH_FIRE,
	[STICK_TYPE_RIGHT][STICK_MODE_PUSH_TORCH_FIRE]	= POS_STICK_R_PUSH_TORCH_FIRE,
	[STICK_TYPE_RIGHT][STICK_MODE_CLEAN_FLOOR] 		= POS_STICK_R_CLEAN_FLOOR,
	[STICK_TYPE_RIGHT][STICK_MODE_CLEAN_HEART] 		= POS_STICK_R_CLEAN_HEART,

	[STICK_TYPE_LEFT][STICK_MODE_HIDE] 					= POS_STICK_L_HIDE,
	[STICK_TYPE_LEFT][STICK_MODE_PUSH_FIRE] 			= POS_STICK_L_PUSH_FIRE,
	[STICK_TYPE_LEFT][STICK_MODE_PUSH_TORCH_FIRE]	= POS_STICK_L_PUSH_TORCH_FIRE,
	[STICK_TYPE_LEFT][STICK_MODE_CLEAN_FLOOR] 		= POS_STICK_L_CLEAN_FLOOR,
	[STICK_TYPE_LEFT][STICK_MODE_CLEAN_HEART] 		= POS_STICK_L_CLEAN_HEART,
};


/* set finger position depends on mode */
uint8_t stick_set_mode(stick_t *stick, uint8_t mode, int16_t pos_offset)
{
	uint8_t ax12_id, err;

	/* set ax12 ids */
	if(stick->type == STICK_TYPE_RIGHT)
		ax12_id = AX12_ID_STICK_R;
	else
		ax12_id = AX12_ID_STICK_L;
		
	/* set ax12 possitions depends on mode and type */
	if(mode >= STICK_MODE_MAX) {
		ACTUATORS_ERROR("Unknow %s STICK MODE", stick->type == STICK_TYPE_RIGHT? "RIGHT":"LEFT");
		return -1;
	}

	stick->mode = mode;
	if(stick->type == STICK_TYPE_RIGHT)
		stick->ax12_pos = stick_ax12_pos[stick->type][stick->mode] + pos_offset;
	else
		stick->ax12_pos = stick_ax12_pos[stick->type][stick->mode] - pos_offset;
	
	/* saturate to position range */
	if(stick->ax12_pos_l > stick_ax12_pos_l[STICK_MODE_L_POS_MAX])
		stick->ax12_pos_l = stick_ax12_pos_l[STICK_MODE_L_POS_MAX];
	if(stick->ax12_pos_l < stick_ax12_pos_l[STICK_MODE_L_POS_MIN])
		stick->ax12_pos_l = stick_ax12_pos_l[STICK_MODE_L_POS_MIN];

	if(stick->ax12_pos_r > stick_ax12_pos_r[STICK_MODE_R_POS_MAX])
		stick->ax12_pos_r = stick_ax12_pos_r[STICK_MODE_R_POS_MAX];
	if(stick->ax12_pos_r < stick_ax12_pos_r[STICK_MODE_R_POS_MIN])
		stick->ax12_pos_r = stick_ax12_pos_r[STICK_MODE_R_POS_MIN];

	/* apply to ax12 */
	err = ax12_user_write_int(&gen.ax12, ax12_id, AA_GOAL_POSITION_L, stick->ax12_pos);

	/* update time for timeout and reset blocking */
	stick->time_us = time_get_us2();
	stick->blocking = 0;

	if(err) 	return err;
	return 0;
}

/* return END_TRAJ or END_BLOCKING */
int8_t stick_check_mode_done(stick_t *stick)
{
	static microseconds us = 0;
	uint16_t ax12_pos;
	uint8_t ax12_id;

	/* ax12 position pulling */
	if(time_get_us2() - us < AX12_PULLING_TIME_us)
		return 0;

	/* update time */
	us = time_get_us2();

	/* set ax12 ids */
	if(stick->type == STICK_TYPE_RIGHT) 
		ax12_id = AX12_ID_STICK_R;
	else
		ax12_id = AX12_ID_STICK_L;

	/* read positions */
	if(ax12_user_read_int(&gen.ax12, ax12_id, AA_PRESENT_POSITION_L, &ax12_pos))
		return 0;

	/* check if position is inside window */
	if(ABS(stick->ax12_pos - ax12_pos) < AX12_WINDOW_POSITION)	
		return END_TRAJ;
	
	/* ax12 blocking timeout */
	if(time_get_us2() - stick->time_us > AX12_BLOCKING_TIMEOUT_us) {
		//XXX ax12_user_write_int(&gen.ax12, ax12_id, AA_GOAL_POSITION_L, ax12_pos);
		stick->blocking = 1;
		return END_BLOCKING;
	}

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t stick_wait_end(stick_t *stick)
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = stick_check_mode_done(stick);

	return ret;
}

/**** boot funcions *********************************************************/

/* set boot position depends on mode */
uint8_t boot_door_set_mode(boot_t *boot, uint8_t door_mode)
{
	uint8_t pos_saturated = 0;

	switch(door_mode) {
		case BOOT_MODE_OPEN:
			boot->door_servo_pos = pwm_servo_set(PWM_SERVO_BOOT_DOOR, POS_BOOT_DOOR_OPEN);
			boot->door_mode = door_mode;
			pos_saturated = boot->door_servo_pos != POS_BOOT_DOOR_OPEN? 1:0;
			break;
		case BOOT_MODE_CLOSE:
			boot->door_servo_pos = pwm_servo_set(PWM_SERVO_BOOT_DOOR, POS_BOOT_DOOR_CLOSE);
			boot->door_mode = door_mode;
			pos_saturated = boot->door_servo_pos != POS_BOOT_DOOR_CLOSE? 1:0;
			break;
		default:
			ACTUATORS_ERROR("Unknown BOOT MODE");
			break;
	}
	if(pos_saturated)
		ACTUATORS_ERROR("Boot position saturated");
}

void boot_tray_set_mode(boot_t *boot, uint8_t tray_mode)
{
	uint8_t pos_saturated = 0;

	switch(tray_mode) {
		case BOOT_TRAY_MODE_DOWN:
			pwm_mc_set(PWM_MC_BOOT_TRAY, 0);
			break;
		case BOOT_TRAY_MODE_VIBRATE:
			pwm_mc_set(PWM_MC_BOOT_TRAY, BOOT_TRAY_VIBRATE_PWM);
			break;
		default:
			ACTUATORS_ERROR("Unknown BOOT TRAY MODE");
			break;
	}

}


/**** tree tray funcions *********************************************************/
uint16_t tree_tray_ax12_pos [TREE_TRAY_MODE_MAX] = {
	[TREE_TRAY_MODE_OPEN] 		= POS_TREE_TRAY_OPEN,
	[TREE_TRAY_MODE_CLOSE] 		= POS_TREE_TRAY_CLOSE,
	[TREE_TRAY_MODE_HARVEST]	= POS_TREE_TRAY_HARVEST,
};

/* set tree_tray position depends on mode */
uint8_t tree_tray_set_mode(tree_tray_t *tree_tray, uint8_t mode, int16_t pos_offset)
{
	uint8_t ax12_id, err;

	/* set ax12 ids */
	ax12_id = AX12_ID_TREE_TRAY;

	/* set ax12 possitions depends on mode and type */
	if(mode >= TRAY_TREE_MODE_MAX) {
		ACTUATORS_ERROR("Unknow %s STICK MODE", tree_tray->type == TRAY_TREE_TYPE_RIGHT? "RIGHT":"LEFT");
		return -1;
	}

	tree_tray->mode = mode;
	tree_tray->ax12_pos = tree_tray_ax12_pos[tree_tray->mode] + pos_offset;

	
	/* saturate to position range */
	if(tree_tray->ax12_pos > tree_tray_ax12_pos[TRAY_TREE_MODE_POS_MAX])
		tree_tray->ax12_pos = tree_tray_ax12_pos[TRAY_TREE_MODE_POS_MAX];
	if(tree_tray->ax12_pos < tree_tray_ax12_pos[TRAY_TREE_MODE_POS_MIN])
		tree_tray->ax12_pos = tree_tray_ax12_pos[TRAY_TREE_MODE_POS_MIN];


	/* apply to ax12 */
	err = ax12_user_write_int(&gen.ax12, ax12_id, AA_GOAL_POSITION_L, tree_tray->ax12_pos);

	/* update time for timeout and reset blocking */
	tree_tray->time_us = time_get_us2();
	tree_tray->blocking = 0;

	if(err) 	return err;
	return 0;
}

/* return END_TRAJ or END_BLOCKING */
int8_t tree_tray_check_mode_done(tree_tray_t *tree_tray)
{
	static microseconds us = 0;
	uint16_t ax12_pos;
	uint8_t ax12_id;

	/* ax12 position pulling */
	if(time_get_us2() - us < AX12_PULLING_TIME_us)
		return 0;

	/* update time */
	us = time_get_us2();

	/* set ax12 ids */
	ax12_id = AX12_ID_TREE_TRAY;


	/* read positions */
	if(ax12_user_read_int(&gen.ax12, ax12_id, AA_PRESENT_POSITION_L, &ax12_pos))
		return 0;

	/* check if position is inside window */
	if(ABS(tree_tray->ax12_pos - ax12_pos) < AX12_WINDOW_POSITION)	
		return END_TRAJ;
	
	/* ax12 blocking timeout */
	if(time_get_us2() - tree_tray->time_us > AX12_BLOCKING_TIMEOUT_us) {
		ax12_user_write_int(&gen.ax12, ax12_id, AA_GOAL_POSITION_L, ax12_pos);
		tree_tray->blocking = 1;
		return END_BLOCKING;
	}

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t tree_tray_wait_end(tree_tray_t *tree_tray)
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = tree_tray_check_mode_done(tree_tray);

	return ret;
}

/* init all actuators */
void actuator_init(void)
{
//#define PROGRAM_AX12
#ifdef PROGRAM_AX12
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_SHUTDOWN, 0x24);	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_LED, 0x24);

	/* specific config for mirror ax12, angle is limited */ 
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0xFF);	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CW_ANGLE_LIMIT_L, 0x00);	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CCW_ANGLE_LIMIT_L, 0x3FF);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_MOVING_SPEED_L, 0x3FF);#endif


	/* init structures */
	slavedspic.stick_l.type = STICK_TYPE_LEFT;
	slavedspic.stick_r.type = STICK_TYPE_RIGHT;

}

#endif
