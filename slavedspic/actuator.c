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
void lift_hard_stop(void)
{
	cs_set_consign(&slavedspic.lift.cs, encoders_dspic_get_value(LIFT_ENCODER));
	slavedspic.lift.qr.previous_var = 0;
	slavedspic.lift.qr.previous_out = encoders_dspic_get_value(LIFT_ENCODER);
}

/* calibrate initial position */
void lift_calibrate(void)
{
#define AUTOPOS_SPEED			50
#define AUTOPOS_ACCEL			1
#define AUTOPOS_BIG_DIST_mm		60000
#define AUTOPOS_BD_TIMEOUT_ms	5000

	int8_t ret;
	int16_t kp, ki, kd;

	/* save and set PID constants */
	kp = pid_get_gain_P(&slavedspic.lift.pid);
	ki = pid_get_gain_I(&slavedspic.lift.pid);
	kd = pid_get_gain_D(&slavedspic.lift.pid);
	pid_set_gains(&slavedspic.lift.pid,  5000, 0, 0);

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
	bd_reset(&slavedspic.lift.bd);
	ACTUATORS_DEBUG("End blocking, encoder is reset to zero");	

	/* restore speed, acceleration and PID */
	quadramp_set_1st_order_vars(&slavedspic.lift.qr, LIFT_SPEED, LIFT_SPEED);
	quadramp_set_2nd_order_vars(&slavedspic.lift.qr, LIFT_ACCEL, LIFT_ACCEL);
	pid_set_gains(&slavedspic.lift.pid, kp, ki, kd);

	ACTUATORS_DEBUG("Restored speed, acceleration and PID");
	ACTUATORS_DEBUG("%s ends", __FUNCTION__);

	/* set calibration flag */
	slavedspic.lift.calibrated = 1;
	return;
}

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
	cs_set_consign(&slavedspic.lift.cs, (int32_t)(((int32_t)LIFT_HEIGHT_MAX_mm-height_mm) * LIFT_K_IMP_mm));
	slavedspic.lift.blocking = 0;
}

/* return heigh in mm */
int32_t lift_get_height(void)
{
	return (int32_t)((int32_t)LIFT_HEIGHT_MAX_mm - (encoders_dspic_get_value(LIFT_ENCODER)/LIFT_K_IMP_mm));
}

/* return 1 if height reached, -1 if blocking and zero if no ends yet */
int8_t lift_check_height_reached(void)
{
	/* test consign end */
	if(cs_get_consign(&slavedspic.lift.cs) == cs_get_filtered_consign(&slavedspic.lift.cs)){
		return END_TRAJ;
	}

	/* test blocking */
	if(bd_get(&slavedspic.lift.bd)) {
		lift_hard_stop();
		pid_reset(&slavedspic.lift.pid);
		bd_reset(&slavedspic.lift.bd);
		slavedspic.lift.blocking = 1;
		ACTUATORS_ERROR("Lift ENDS BLOCKING");
		return END_BLOCKING;
	}

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t lift_wait_end()
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = lift_check_height_reached();

	return ret;
}



/**** combs funcions *********************************************************/

uint16_t combs_ax12_pos_l [COMBS_MODE_MAX] = {
	[COMBS_MODE_HIDE] 			= POS_COMB_L_HIDE, 
	[COMBS_MODE_OPEN] 			= POS_COMB_L_OPEN, 
	[COMBS_MODE_HARVEST_CLOSE]  = POS_COMB_L_HARVEST_CLOSE, 
	[COMBS_MODE_HARVEST_OPEN] 	= POS_COMB_L_HARVEST_OPEN, 
};

uint16_t combs_ax12_pos_r [COMBS_MODE_MAX] = {
	[COMBS_MODE_HIDE] 			= POS_COMB_R_HIDE, 
	[COMBS_MODE_OPEN] 			= POS_COMB_R_OPEN, 
	[COMBS_MODE_HARVEST_CLOSE]  = POS_COMB_R_HARVEST_CLOSE, 
	[COMBS_MODE_HARVEST_OPEN] 	= POS_COMB_R_HARVEST_OPEN, 
};

#ifndef old_version
struct ax12_traj ax12_comb_l = { .id = AX12_ID_COMB_L; .zero_offset_pos = 0);
struct ax12_traj ax12_comb_r = { .id = AX12_ID_COMB_R; .zero_offset_pos = 0);
#endif

/* set finger position depends on mode */
int8_t combs_set_mode(combs_t *combs, uint8_t mode, int16_t pos_offset)
{
#ifdef old_version
	uint8_t ax12_left_id, ax12_right_id, err1=0, err2=0;

	/* set ax12 ids */
	ax12_left_id = AX12_ID_COMB_L;
	ax12_right_id = AX12_ID_COMB_R;
#endif

	/* set ax12 possitions depends on mode and type */
	if(mode >= COMBS_MODE_MAX) {
		ACTUATORS_ERROR("Unknow COMBS MODE");
		return -1;
	}

   /* ax12 possitions */
	combs->mode_old = combs->mode;
	combs->mode = mode;
	combs->ax12_pos_l = combs_ax12_pos_l[combs->mode] + pos_offset;
	combs->ax12_pos_r = combs_ax12_pos_r[combs->mode] - pos_offset;

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
#ifdef old_version
	err1 = ax12_user_write_int(&gen.ax12, ax12_left_id, AA_GOAL_POSITION_L, combs->ax12_pos_l);
    err2 = ax12_user_write_int(&gen.ax12, ax12_right_id, AA_GOAL_POSITION_L, combs->ax12_pos_r);

	/* update time for timeout and reset blocking */
	combs->blocking = 0;
	combs->time_us = time_get_us2();

	if(err1) return err1;
	if(err2) return err2;
#else
    ax12_set_pos (&ax12_comb_l, combs->ax12_pos_l);
    ax12_set_pos (&ax12_comb_r, combs->ax12_pos_r);
#endif
	return 0;
}

#ifdef old_version
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
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_GOAL_POSITION_L, ax12_pos_l);
		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_GOAL_POSITION_L, ax12_pos_r);
		combs->blocking = 1;
		return END_BLOCKING;
	}

	return 0;

}
#endif

/* return END_TRAJ or END_BLOCKING */

uint8_t combs_wait_end(combs_t *combs)
{
#ifdef old_version
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = combs_check_mode_done(combs);

	return ret;
#else
    uint8_t ret_l, ret_l; 
   
    ret_l = ax12_wait_traj_end (&ax12_comb_l, AX12_END_TRAJ);
    ret_r = ax12_wait_traj_end (&ax12_comb_r, AX12_END_TRAJ);

    return (ret_l | ret_r);
#endif
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

#ifndef old_version
struct ax12_traj ax12_stick_l = { .id = AX12_ID_STICK_L; .zero_offset_pos = 0);
struct ax12_traj ax12_stick_r = { .id = AX12_ID_STICK_R; .zero_offset_pos = 0);
#endif

/* set finger position depends on mode */
uint8_t stick_set_mode(stick_t *stick, uint8_t mode, int16_t pos_offset)
{
#ifdef old_version
	uint8_t ax12_id, err;

	/* set ax12 ids */
	if(stick->type == STICK_TYPE_RIGHT)
		ax12_id = AX12_ID_STICK_R;
	else
		ax12_id = AX12_ID_STICK_L;
#endif
		
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
	if(stick->type == STICK_TYPE_LEFT) {
		if(stick->ax12_pos > stick_ax12_pos[STICK_TYPE_LEFT][STICK_MODE_L_POS_MAX])
			stick->ax12_pos = stick_ax12_pos[STICK_TYPE_LEFT][STICK_MODE_L_POS_MAX];
		if(stick->ax12_pos < stick_ax12_pos[STICK_TYPE_LEFT][STICK_MODE_L_POS_MIN])
			stick->ax12_pos = stick_ax12_pos[STICK_TYPE_LEFT][STICK_MODE_L_POS_MIN];
	} 
	else {
		if(stick->ax12_pos > stick_ax12_pos[STICK_TYPE_RIGHT][STICK_MODE_R_POS_MAX])
			stick->ax12_pos = stick_ax12_pos[STICK_TYPE_RIGHT][STICK_MODE_R_POS_MAX];
		if(stick->ax12_pos < stick_ax12_pos[STICK_TYPE_RIGHT][STICK_MODE_R_POS_MIN])
			stick->ax12_pos = stick_ax12_pos[STICK_TYPE_RIGHT][STICK_MODE_R_POS_MIN];
	}

#ifdef old_version
	/* apply to ax12 */
	err = ax12_user_write_int(&gen.ax12, ax12_id, AA_GOAL_POSITION_L, stick->ax12_pos);

	/* update time for timeout and reset blocking */
	stick->time_us = time_get_us2();
	stick->blocking = 0;

	if(err) 	return err;
#else

    if(stick->type == STICK_TYPE_LEFT)
        ax12_set_pos (&ax12_stick_l, stick->ax12_pos);
    else
        ax12_set_pos (&ax12_stick_r, stick->ax12_pos);
#endif
	return 0;
}

#ifdef old_version
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
#endif


/* return END_TRAJ or END_BLOCKING */
uint8_t stick_wait_end(stick_t *stick)
{
#ifdef old_version
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = stick_check_mode_done(stick);

	return ret;
#else
    if(stick->type == STICK_TYPE_LEFT)
        return ax12_wait_traj_end (&ax12_stick_l, AX12_END_TRAJ);
    else
        return ax12_wait_traj_end (&ax12_stick_r, AX12_END_TRAJ);
#endif
}


/**** boot funcions *********************************************************/

/* set boot position depends on mode */
void boot_door_set_mode(boot_t *boot, uint8_t door_mode)
{
	uint8_t pos_saturated = 0;

	switch(door_mode) {
		case BOOT_DOOR_MODE_OPEN:
			boot->door_servo_pos = pwm_servo_set(PWM_SERVO_BOOT_DOOR, POS_BOOT_DOOR_OPEN);
			boot->door_mode = door_mode;
			pos_saturated = boot->door_servo_pos != POS_BOOT_DOOR_OPEN? 1:0;
			break;
		case BOOT_DOOR_MODE_CLOSE:
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
	if(pos_saturated)
		ACTUATORS_ERROR("Boot tray position saturated");

}


/**** tree tray funcions *********************************************************/
uint16_t tree_tray_ax12_pos [TREE_TRAY_MODE_MAX] = {
	[TREE_TRAY_MODE_OPEN] 		= POS_TREE_TRAY_OPEN,
	[TREE_TRAY_MODE_CLOSE] 		= POS_TREE_TRAY_CLOSE,
	[TREE_TRAY_MODE_HARVEST]	= POS_TREE_TRAY_HARVEST,
};

#ifndef old_version
struct ax12_traj ax12_tree_tray = { .id = AX12_ID_TREE_TRAY; .zero_offset_pos = 0);
#endif

/* set tree_tray position depends on mode */
uint8_t tree_tray_set_mode(tree_tray_t *tree_tray, uint8_t mode, int16_t pos_offset)
{
#ifdef old_version
	uint8_t ax12_id, err;

	/* set ax12 ids */
	ax12_id = AX12_ID_TREE_TRAY;
#endif
	/* set ax12 possitions depends on mode and type */
	if(mode >= TREE_TRAY_MODE_MAX) {
		ACTUATORS_ERROR("Unknow TREE TRAY MODE");
		return -1;
	}

	tree_tray->mode = mode;
	tree_tray->ax12_pos = tree_tray_ax12_pos[tree_tray->mode] + pos_offset;

	
	/* saturate to position range */
	if(tree_tray->ax12_pos > tree_tray_ax12_pos[TREE_TRAY_MODE_POS_MAX])
		tree_tray->ax12_pos = tree_tray_ax12_pos[TREE_TRAY_MODE_POS_MAX];
	if(tree_tray->ax12_pos < tree_tray_ax12_pos[TREE_TRAY_MODE_POS_MIN])
		tree_tray->ax12_pos = tree_tray_ax12_pos[TREE_TRAY_MODE_POS_MIN];

#ifdef old_version
	/* apply to ax12 */
	err = ax12_user_write_int(&gen.ax12, ax12_id, AA_GOAL_POSITION_L, tree_tray->ax12_pos);

	/* update time for timeout and reset blocking */
	tree_tray->time_us = time_get_us2();
	tree_tray->blocking = 0;

	if(err) 	return err;
#else
    ax12_set_pos (&ax12_tree_tray, tree_tray->ax12_pos);
#endif
	return 0;
}

#ifdef old_version
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
#endif

/* return END_TRAJ or END_BLOCKING */
uint8_t tree_tray_wait_end(tree_tray_t *tree_tray)
{
#ifdef old_version
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = tree_tray_check_mode_done(tree_tray);

	return ret;
#else
    return ax12_wait_traj_end (&ax12_tree_tray, AX12_END_TRAJ);
#endif
}

/**** vacuum funcions *********************************************************/

void vacuum_motor_set (uint8_t num, uint8_t on) {

	if (num == 1)
		_LATC1 = !on;
	else if (num == 2)
		_LATB3 = !on;
}

void vacuum_ev_set (uint8_t num, uint8_t on) 
{
	if (num == 1)
		_LATB10 = on;
	else if (num == 2)
		_LATB11 = on;
}

void vacuum_system_enable (uint8_t num) {
	vacuum_ev_set (num, 1);
	time_wait_ms (500);
	vacuum_motor_set (num, 1);
}

void vacuum_system_disable (uint8_t num) {
	vacuum_ev_set (num, 0);
	vacuum_motor_set (num, 0);
}



#if 1

/**************************  AX12 MANAGE FUNCTIONS ****************************/
struct ax12_traj 
{
#define AX12_K_IMP_DEG			(1024.0/300.0)
#define AX12_K_MS_DEG           (200.0/60.0)

	uint8_t id;
	int16_t zero_offset_pos;

	int16_t goal_angle_deg;
	int16_t goal_pos;
	microseconds goal_time_us;

    int16_t pos;
	int16_t angle_deg;
};


/* set position */
void ax12_set_pos (struct ax12_traj *ax12, int16_t pos)
{
     /* update current position/angle */
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12->pos);
 	ax12->angle_deg = (uint16_t)((ax12->pos - ax12->zero_offset_pos) / AX12_K_IMP_DEG);

    /* set goal angle */
    ax12->goal_pos = pos;
    ax12->goal_angle_deg = (uint16_t)((pos - ax12->zero_offset_pos) / AX12_K_IMP_DEG);
	ax12_user_write_int(&gen.ax12, ax12->id , AA_GOAL_POSITION_L, ax12->goal_pos);

    /* update goal time */
	ax12->goal_time_us = (microseconds)(ABS(ax12->angle_deg - ax12->goal_angle_deg) * AX12_K_MS_DEG * 1000);
	ax12->time_us = time_get_us2();
}

/* set angle */
void ax12_set_a (struct ax12_traj *ax12, int16_t a)
{
    /* update current position/angle */
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12->pos);
 	ax12->angle_deg = (uint16_t)((ax12->pos - ax12->zero_offset_pos) / AX12_K_IMP_DEG);

    /* set goal angle */
    ax12->goal_angle_deg = a;
    ax12->goal_pos = ax12->zero_offset_pos + (uint16_t)(ax12->goal_angle_deg * AX12_K_IMP_DEG);
	ax12_user_write_int(&gen.ax12, ax12->id , AA_GOAL_POSITION_L, ax12->goal_pos);

    /* update goal time */
	ax12->goal_time_us = (microseconds)(ABS(ax12->angle_deg - ax12->goal_angle_deg) * AX12_K_MS_DEG * 1000);
	ax12->time_us = time_get_us2();
}

/* get angle */
int8_t ax12_get_a (struct ax12_traj *ax12)
{
	uint16_t pos;

    /* update current position/angle */
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &pos);
	ax12->angle_deg = (uint16_t)((pos - ax12->zero_offset_pos) / AX12_K_IMP_DEG);

    return ax12->angle_deg;
}


/* test end traj */
#define AX12_END_TRAJ   1
#define AX12_END_NEAR   2
#define AX12_END_TIME   3
uint8_t ax12_test_traj_end (struct ax12_traj *ax12)
{
	uint16_t pos;
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12->pos);
    uint8_t ret = 0;

	if (ABS(ax12->goal_pos - ax12->pos) < AX12_WINDOW_NO_NEAR)
		ret |= AX12_END_TRAJ;

	if (ABS(ax12->goal_pos - ax12->pos) < AX12_WINDOW_NEAR)
		ret |=  AX12_END_NEAR;

	if (time_get_us2() - ax12->time_us > ax12->goal_time_us)
		ret |=  AX12_END_TIME;

    return ret;
}

/* wait traj end */
uint8_t ax12_wait_traj_end (struct ax12_traj *ax12, uint8_t flags) 
{
    microseconds us = time_get_us2();
    uint8_t ret = 0, __ret = 0;

    while (ret == 0) {
        /* check end traj periodicaly (T = 5ms) */
        if (time_get_us2() - us >= 5000L) {
            __ret = ax12_test_traj_end (ax12);
            us = time_get_us2();


            if ((flags & AX12_END_TRAJ) & __ret)
                ret = __ret;

            if ((flags & AX12_END_NEAR) & __ret)
                ret = __ret;

            if ((flags & AX12_END_TIME) & __ret)
                ret = __ret;
        }
    }
}


/****************************  ARM  *******************************************/

#define SHOULDER_JOIN_X   150
#define SHOULDER_JOIN_Y   162

#define ARM_LENGTH        200

#define ARM_X_MAX   (SHOULDER_JOIN_X + ARM_LENGTH)
#define ARM_X_MIN   (SHOULDER_JOIN_X - ARM_LENGTH)

#define ARM_Y_MAX   (SHOULDER_JOIN_Y + ARM_LENGTH)
#define ARM_Y_MIN   (SHOULDER_JOIN_Y)

#define ARM_H_MAX   (LIFT_HEIGHT_MAX_mm)
#define ARM_H_MIN   (LIFT_HEIGHT_MAX_mm)

#define ARM_A_MAX   (+90)
#define ARM_A_MIN   (-90)

#define ARM_ELBOW_A_MAX (180)
#define ARM_ELBOW_A_MIN (0)

#define ARM_WRIST_A_MAX (+90)
#define ARM_WRIST_A_MIN (-90)

struct ax12_traj ax12_shoulder = { .id = AX12_ID_SHOULDER; .zero_offset_pos = 512);
struct ax12_traj ax12_elbow    = { .id = AX12_ID_ELBOW;    .zero_offset_pos = 0);
struct ax12_traj ax12_wrist    = { .id = AX12_ID_WRIST;    .zero_offset_pos = 512);



void arm_a_to_xy (uint16_t a, uint16_t *x, uint16_t *y)
{
    double x, y;

    x = ARM_LENGTH * cos(RAD(a));
    y = ARM_LENGTH * sen(RAD(a));

    *x = (uint16_t)x + SHOULDER_JOIN_X;
    *y = (uint16_t)y + SHOULDER_JOIN_Y;
}

void arm_x_to_ay (uint16_t x, uint16_t *a, uint16_t *y)
{
    double a, y;

    a = acos (x / ARM_LENGTH);
    y = ARM_LENGTH * sen(a);

    *a = (uint16_t)DEG(a);
    *y = (uint16_t)y + SHOULDER_JOIN_Y;   
}

void arm_y_to_ax (uint16_t y, uint16_t *a, uint16_t *x)
{
    double a, x;

    a = asin (y / ARM_LENGTH);
    x = ARM_LENGTH * cos(a);

    *a = (uint16_t)DEG(a);
    *x = (uint16_t)x + SHOULDER_JOIN_X;   
}

struct arm 
{
    int16_t h;
    int16_t x;
    int16_t y;
    int16_t a;

    int16_t elbow_a;
    int16_t wrist_a;
};

/* set sucker heigh 
 * XXX elbow angle is taken in account */
void arm_goto_h (int16_t h)
{
    int8_t elbow_a;
    int16_t sucker_offset;
    int8_t h_sucker;
    
    /* calculate sucker height */
    elbow_a = ABS(ax12_get_a(&ax12_elbow));
    sucker_offset = SUCKER_LENGTH - (SUCKER_LENGHT * cos(RAD(a)));
    h_sucker = h-sucker_offset;

    /* check limints */
    if (h_sucker > ARM_H_MAX)   h_sucker = ARM_H_MAX;
    if (h_sucker < ARM_H_MIN)   h_sucker = ARM_H_MIN;   

    lift_set_height (h_sucker);
}

uint8_t arm_h_wait_traj_end (void)
{
    return lift_wait_end();
}


/* goto x coordinate, relative to robot zero coordinates.
 * XXX suposes elbow angle of 0 deg (sucker in parallel with ground) */
void arm_goto_x (int16_t x)
{
    int16_t a,y;

    /* calculate angle pos */
    arm_x_to_ay (x, &a, &y); 

    /* check limints */
    if (a > ARM_SHOULDER_A_MAX)   a = ARM_SHOULDER_A_MAX;
    if (a < ARM_SHOULDER_A_MIN)   a = ARM_SHOULDER_A_MIN;

    /* set pos */
    ax12_set_a_abs (&ax12_shoulder, a);  
}

/* goto y coordinate, relative to robot zero coordinates.
 * XXX suposes elbow angle of 0 deg (sucker in parallel with ground) */
void arm_goto_y (int16_t y)
{
    int16_t a,x;

    /* calculate angle pos */
    arm_y_to_ax (x, &a, &x); 

    /* check limints */
    if (a > ARM_SHOULDER_A_MAX)   a = ARM_SHOULDER_A_MAX;
    if (a < ARM_SHOULDER_A_MIN)   a = ARM_SHOULDER_A_MIN;

    /* set pos */
    ax12_set_a_abs (&ax12_shoulder, a); 
}


void arm_elbow_goto_a_abs (int8_t a)
{
    /* check limints */
    if (a > ARM_ELBOW_A_MAX)   a = ARM_ELBOW_A_MAX;
    if (a < ARM_ELBOW_A_MIN)   a = ARM_ELBOW_A_MIN;

    /* set pos */
    ax12_set_a_abs (&ax12_elbow, a);
}

void arm_elbow_goto_a_rel (int8_t a)
{
    /* calculate a abs */
    a = ax12_get_a (&ax12_elbow) + a;

    /* check limints */
    if (a > ARM_ELBOW_A_MAX)   a = ARM_ELBOW_A_MAX;
    if (a < ARM_ELBOW_A_MIN)   a = ARM_ELBOW_A_MIN;
  
    /* set pos */
    ax12_set_a_abs (&ax12_elbow, a);
}

void arm_elbow_wait_traj_end (void) {
    return ax12_wait_traj_end(&ax12_elbow);
}

void arm_wrist_goto_a_abs (int8_t a)
{
    /* check limints */
    if (a > ARM_WRIST_A_MAX)   a = ARM_WRIST_A_MAX;
    if (a < ARM_WRIST_A_MIN)   a = ARM_WRIST_A_MIN;

    /* set pos */
    ax12_set_a_abs (&ax12_wrist, a);
}

void arm_wrist_goto_a_rel (int8_t a)
{
    /* calculate a abs */
    a = ax12_get_a (&ax12_wrist) + a;

    /* check limints */
    if (a > ARM_WRIST_A_MAX)   a = ARM_WRIST_A_MAX;
    if (a < ARM_WRIST_A_MIN)   a = ARM_WRIST_A_MIN;

    /* set pos */
    ax12_set_a_abs (&ax12_wrist, a);
}

void arm_wrist_wait_traj_end (void) {
    return ax12_wait_traj_end(&ax12_wrist);
}





#endif


/* init all actuators */
void actuator_init(void)
{
//#define PROGRAM_AX12
#ifdef PROGRAM_AX12
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_SHUTDOWN, 0x24);
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_LED, 0x24);

	/* specific config for mirror ax12, angle is limited */ 
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0xFF);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CW_ANGLE_LIMIT_L, 0x00);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CCW_ANGLE_LIMIT_L, 0x3FF);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_MOVING_SPEED_L, 0x3FF);
#endif

	/* init structures */
	slavedspic.stick_l.type = STICK_TYPE_LEFT;
	slavedspic.stick_r.type = STICK_TYPE_RIGHT;
}

