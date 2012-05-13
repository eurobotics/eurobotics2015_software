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

/* AX12 stuff */
#define AX12_PULLING_TIME_us		5000L
#define AX12_WINDOW_POSITION		15
#define AX12_BLOCKING_TIMEOUT_us	1300000L


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
/**** turbine funcions *********************************************************/

/* power on the turbine controllers */
void turbine_power_on(turbine_t *turbine) {
	turbine->power = TURBINE_POWER_ON;
	TURBINE_POWER_PIN = turbine->power;
}

/* power off the turbine controllers */
void turbine_power_off(turbine_t *turbine) {
	turbine->power = TURBINE_POWER_OFF;
	TURBINE_POWER_PIN = turbine->power;
}

/* move servo with angle speed control */
void turbine_angle_speed_control(void * data)
{
	turbine_t *turbine = (turbine_t *)data;
	int16_t angle_pos;
	uint8_t flags;

	/* next angle position */
	if(turbine->angle_consign >= turbine->angle_pos) {
		angle_pos = turbine->angle_pos + turbine->angle_speed;
		if(angle_pos >= turbine->angle_consign)
			angle_pos = turbine->angle_consign;
	}
	else {
		angle_pos = turbine->angle_pos - turbine->angle_speed;
		if(angle_pos <= (int16_t)turbine->angle_consign)
			angle_pos = turbine->angle_consign;
	}

	/* apply at servo */
	turbine->angle_pos = pwm_servo_set(TURBINE_ANGLE_PWM_SERVO, (uint16_t)angle_pos);

	/* kill event if angle is equal to consign */
	if(angle_pos == turbine->angle_consign) {
		IRQ_LOCK(flags);
		scheduler_del_event(turbine->angle_event_handler);
		IRQ_UNLOCK(flags);
	}
}

/* set angle consing */
void turbine_set_angle(turbine_t *turbine, int16_t angle_deg, uint16_t angle_speed)
{
	uint16_t angle_pos;
	uint8_t flags;

	/* delete event */
	IRQ_LOCK(flags);
	scheduler_del_event(turbine->angle_event_handler);
	IRQ_UNLOCK(flags);

	/* position consing */
	angle_pos = TURBINE_POS_ANGLE_ZERO + (angle_deg * TURBINE_K_POS_DEG);

	/* saturate to range */
	if(angle_pos > TURBINE_POS_ANGLE_MAX)
		angle_pos = TURBINE_POS_ANGLE_MAX;
	if(angle_pos < TURBINE_POS_ANGLE_MIN)
		angle_pos = TURBINE_POS_ANGLE_MIN;

#ifdef OLD_VERSION
	/* set and save consign */
	if(angle_pos >= turbine->angle_pos)
		for(i=turbine->angle_pos; i<=angle_pos; i=i+5) {
			turbine->angle_pos = pwm_servo_set(TURBINE_ANGLE_PWM_SERVO, i);
			time_wait_ms(20);
		}
	else
		//for(i=turbine->angle_pos; i>=angle_pos; i=i-5) {
			//turbine->angle_pos = pwm_servo_set(TURBINE_ANGLE_PWM_SERVO, i);
			turbine->angle_pos = pwm_servo_set(TURBINE_ANGLE_PWM_SERVO, angle_pos);
		//	time_wait_ms(20);
		//}



	/* wait to reach the position */
	if(wait_ms)
		time_wait_ms(wait_ms);
#endif

	/* manage servo on event */
	IRQ_LOCK(flags);
	turbine->angle_consign = angle_pos;
	turbine->angle_speed = angle_speed;
	IRQ_UNLOCK(flags);
	turbine->angle_event_handler = scheduler_add_periodical_event(turbine_angle_speed_control,
									 		(void *)(turbine), TURBINE_ANGLE_SPEED_CONTROL_us / SCHEDULER_UNIT);

}

/* return true if angle consign is reached */
int8_t turbine_check_angle_reached(turbine_t *turbine) {
	return (turbine->angle_pos == turbine->angle_consign);
}

/* set speed of blow speed of turbines */
void turbine_set_blow_speed(turbine_t *turbine, uint16_t speed) {
	turbine->blow_speed = pwm_servo_set(TURBINE_SPEED_PWM_SERVO, speed);
}

/* return the blow speed of turbines */
int16_t turbine_get_angle(turbine_t *turbine) {
	return (int16_t)(((int16_t)turbine->angle_pos - (int16_t)TURBINE_POS_ANGLE_ZERO) / TURBINE_K_POS_DEG);
}

/* return the angle in deg of turbines */
uint16_t turbine_get_blow_speed(turbine_t *turbine) {
	return turbine->blow_speed;
}



/**** fingers funcions *********************************************************/

uint16_t fingers_ax12_pos_l[FINGERS_TYPE_MAX][FINGERS_MODE_MAX] = {
	[FINGERS_TYPE_FLOOR][FINGERS_MODE_HUG] 	      	= POS_FINGER_FLOOR_L_HUG, 
	[FINGERS_TYPE_FLOOR][FINGERS_MODE_OPEN] 	         = POS_FINGER_FLOOR_L_OPEN,
	[FINGERS_TYPE_FLOOR][FINGERS_MODE_HOLD] 		      = POS_FINGER_FLOOR_L_HOLD,
	[FINGERS_TYPE_FLOOR][FINGERS_MODE_CLOSE] 		      = POS_FINGER_FLOOR_L_CLOSE,
	[FINGERS_TYPE_FLOOR][FINGERS_MODE_PUSHIN]       	= POS_FINGER_FLOOR_L_PUSHIN,

	[FINGERS_TYPE_TOTEM][FINGERS_MODE_HUG] 	       	= POS_FINGER_TOTEM_L_HUG,
	[FINGERS_TYPE_TOTEM][FINGERS_MODE_OPEN] 	       	= POS_FINGER_TOTEM_L_OPEN,
	[FINGERS_TYPE_TOTEM][FINGERS_MODE_HOLD] 		      = POS_FINGER_TOTEM_L_HOLD,
	[FINGERS_TYPE_TOTEM][FINGERS_MODE_CLOSE] 		      = POS_FINGER_TOTEM_L_CLOSE,
	[FINGERS_TYPE_TOTEM][FINGERS_MODE_PUSHIN] 	      = POS_FINGER_TOTEM_L_PUSHIN,
};

uint16_t fingers_ax12_pos_r[FINGERS_TYPE_MAX][FINGERS_MODE_MAX] = {
	[FINGERS_TYPE_FLOOR][FINGERS_MODE_HUG] 	      	= POS_FINGER_FLOOR_R_HUG,
	[FINGERS_TYPE_FLOOR][FINGERS_MODE_OPEN] 	       	= POS_FINGER_FLOOR_R_OPEN,
	[FINGERS_TYPE_FLOOR][FINGERS_MODE_HOLD] 		      = POS_FINGER_FLOOR_R_HOLD,
	[FINGERS_TYPE_FLOOR][FINGERS_MODE_CLOSE] 		      = POS_FINGER_FLOOR_R_CLOSE,
	[FINGERS_TYPE_FLOOR][FINGERS_MODE_PUSHIN] 	      = POS_FINGER_FLOOR_R_PUSHIN,

	[FINGERS_TYPE_TOTEM][FINGERS_MODE_HUG] 	      	= POS_FINGER_TOTEM_R_HUG,
	[FINGERS_TYPE_TOTEM][FINGERS_MODE_OPEN] 	      	= POS_FINGER_TOTEM_R_OPEN,
	[FINGERS_TYPE_TOTEM][FINGERS_MODE_HOLD] 	      	= POS_FINGER_TOTEM_R_HOLD,
	[FINGERS_TYPE_TOTEM][FINGERS_MODE_CLOSE] 		      = POS_FINGER_TOTEM_R_CLOSE,
	[FINGERS_TYPE_TOTEM][FINGERS_MODE_PUSHIN]       	= POS_FINGER_TOTEM_R_PUSHIN,
};


/* set finger position depends on mode */
int8_t fingers_set_mode(fingers_t *fingers, uint8_t mode, int16_t pos_offset)
{
	uint8_t ax12_left_id, ax12_right_id, err1=0, err2=0;
   uint8_t fingers_type;

 	if(fingers->type == FINGERS_TYPE_TOTEM_LEFT || fingers->type == FINGERS_TYPE_TOTEM_RIGHT || fingers->type == FINGERS_TYPE_TOTEM) 
      fingers_type = FINGERS_TYPE_TOTEM;
	else
	   fingers_type = FINGERS_TYPE_FLOOR;


	/* set ax12 ids */
	if(fingers_type == FINGERS_TYPE_TOTEM) {
		ax12_left_id = AX12_ID_FINGERS_TOTEM_L;
		ax12_right_id = AX12_ID_FINGERS_TOTEM_R;
	}
	else {
		ax12_left_id = AX12_ID_FINGERS_FLOOR_L;
		ax12_right_id = AX12_ID_FINGERS_FLOOR_R;
   }   
		
	/* set ax12 possitions depends on mode and type */
	if(mode >= FINGERS_MODE_MAX) {
		ACTUATORS_ERROR("Unknow %s FINGERS MODE", fingers->type == FINGERS_TYPE_FLOOR? "FLOOR":"TOTEM");
		return -1;
	}

   /* ax12 possitions */
	fingers->mode = mode;
	if(fingers->type == FINGERS_TYPE_TOTEM_LEFT || fingers->type == FINGERS_TYPE_TOTEM)
	   fingers->ax12_pos_l = fingers_ax12_pos_l[FINGERS_TYPE_TOTEM][fingers->mode] + pos_offset;
	else if(fingers->type == FINGERS_TYPE_FLOOR_LEFT || fingers->type == FINGERS_TYPE_FLOOR)
	   fingers->ax12_pos_l = fingers_ax12_pos_l[FINGERS_TYPE_FLOOR][fingers->mode] + pos_offset;

	if(fingers->type == FINGERS_TYPE_TOTEM_RIGHT || fingers->type == FINGERS_TYPE_TOTEM)
	   fingers->ax12_pos_r = fingers_ax12_pos_r[FINGERS_TYPE_TOTEM][fingers->mode] - pos_offset;
	else if(fingers->type == FINGERS_TYPE_FLOOR_RIGHT || fingers->type == FINGERS_TYPE_FLOOR)
	   fingers->ax12_pos_r = fingers_ax12_pos_r[FINGERS_TYPE_FLOOR][fingers->mode] - pos_offset;

   /* set general type */
   fingers->type = fingers_type;

	/* set speed */
#if 1
   if((fingers->type == FINGERS_TYPE_TOTEM)&& (fingers->mode == FINGERS_MODE_CLOSE || fingers->mode == FINGERS_MODE_HOLD)) {
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_MOVING_SPEED_L, 300);
		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_MOVING_SPEED_L, 300);
	}
	else if(fingers->type == FINGERS_TYPE_TOTEM) {
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_MOVING_SPEED_L, 0x3ff);
		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_MOVING_SPEED_L, 0x3ff);
	}
#endif

	/* saturate to position range */

	if(fingers->ax12_pos_l > fingers_ax12_pos_l[fingers->type][FINGERS_MODE_L_POS_MAX])
		fingers->ax12_pos_l = fingers_ax12_pos_l[fingers->type][FINGERS_MODE_L_POS_MAX];
	if(fingers->ax12_pos_l < fingers_ax12_pos_l[fingers->type][FINGERS_MODE_L_POS_MIN])
		fingers->ax12_pos_l = fingers_ax12_pos_l[fingers->type][FINGERS_MODE_L_POS_MIN];

	if(fingers->ax12_pos_r > fingers_ax12_pos_r[fingers->type][FINGERS_MODE_R_POS_MAX])
		fingers->ax12_pos_r = fingers_ax12_pos_r[fingers->type][FINGERS_MODE_R_POS_MAX];
	if(fingers->ax12_pos_r < fingers_ax12_pos_r[fingers->type][FINGERS_MODE_R_POS_MIN])
		fingers->ax12_pos_r = fingers_ax12_pos_r[fingers->type][FINGERS_MODE_R_POS_MIN];
 
	/* apply to ax12 */
	err1 = ax12_user_write_int(&gen.ax12, ax12_left_id, AA_GOAL_POSITION_L, fingers->ax12_pos_l);
   err2 = ax12_user_write_int(&gen.ax12, ax12_right_id, AA_GOAL_POSITION_L, fingers->ax12_pos_r);

	if(err1) return err1;
	if(err2) return err2;

	/* update time for timeout and reset blocking */
	fingers->time_us = time_get_us2();
	fingers->blocking = 0;

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
int8_t fingers_check_mode_done(fingers_t *fingers)
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
	if(fingers->type == FINGERS_TYPE_FLOOR) {
		ax12_left_id = AX12_ID_FINGERS_FLOOR_L;
		ax12_right_id = AX12_ID_FINGERS_FLOOR_R;
	}
	else {
		ax12_left_id = AX12_ID_FINGERS_TOTEM_L;
		ax12_right_id = AX12_ID_FINGERS_TOTEM_R;
	}

	/* read positions */
	if(ax12_user_read_int(&gen.ax12, ax12_left_id, AA_PRESENT_POSITION_L, &ax12_pos_l))
		return 0;

	if(ax12_user_read_int(&gen.ax12, ax12_right_id, AA_PRESENT_POSITION_L, &ax12_pos_r))
		return 0;

	/* check if positions are inside window */
	if(ABS(fingers->ax12_pos_l - ax12_pos_l) < AX12_WINDOW_POSITION 
		&& ABS(fingers->ax12_pos_r - ax12_pos_r) < AX12_WINDOW_POSITION)	
		return END_TRAJ;
	
	/* ax12 blocking timeout */
	if(time_get_us2() - fingers->time_us > AX12_BLOCKING_TIMEOUT_us) {
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_GOAL_POSITION_L, ax12_pos_l);		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_GOAL_POSITION_L, ax12_pos_r);
		fingers->blocking = 1;
		return END_BLOCKING;
	}

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t fingers_wait_end(fingers_t *fingers)
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = fingers_check_mode_done(fingers);

	return ret;
}

/**** arms funcions *********************************************************/
uint16_t arm_ax12_pos[ARM_TYPE_MAX][ARM_MODE_MAX] = {
	[ARM_TYPE_RIGHT][ARM_MODE_HIDE] 			= POS_ARM_R_HIDE,
	[ARM_TYPE_RIGHT][ARM_MODE_SHOW] 			= POS_ARM_R_SHOW,
	[ARM_TYPE_RIGHT][ARM_MODE_PUSH_GOLDBAR]= POS_ARM_R_PUSH_GOLDBAR,
	[ARM_TYPE_RIGHT][ARM_MODE_PUSH_FLOOR] 	= POS_ARM_R_PUSH_FLOOR,

	[ARM_TYPE_LEFT][ARM_MODE_HIDE] 			= POS_ARM_L_HIDE,
	[ARM_TYPE_LEFT][ARM_MODE_SHOW] 			= POS_ARM_L_SHOW,
	[ARM_TYPE_LEFT][ARM_MODE_PUSH_GOLDBAR]	= POS_ARM_L_PUSH_GOLDBAR,
	[ARM_TYPE_LEFT][ARM_MODE_PUSH_FLOOR] 	= POS_ARM_L_PUSH_FLOOR,
};

/* set finger position depends on mode */
uint8_t arm_set_mode(arm_t *arm, uint8_t mode, int16_t pos_offset)
{
	uint8_t ax12_id, err;

	/* set ax12 ids */
	if(arm->type == ARM_TYPE_RIGHT)
		ax12_id = AX12_ID_ARM_R;
	else
		ax12_id = AX12_ID_ARM_L;
		
	/* set ax12 possitions depends on mode and type */
	if(mode >= ARM_MODE_MAX) {
		ACTUATORS_ERROR("Unknow %s ARM MODE", arm->type == ARM_TYPE_RIGHT? "RIGHT":"LEFT");
		return -1;
	}

	arm->mode = mode;
	if(arm->type == ARM_TYPE_RIGHT)
		arm->ax12_pos = arm_ax12_pos[arm->type][arm->mode] + pos_offset;
	else
		arm->ax12_pos = arm_ax12_pos[arm->type][arm->mode] - pos_offset;
	
	/* apply to ax12 */
	err = ax12_user_write_int(&gen.ax12, ax12_id, AA_GOAL_POSITION_L, arm->ax12_pos);
	if(err) 	return err;

	/* update time for timeout and reset blocking */
	arm->time_us = time_get_us2();
	arm->blocking = 0;

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
int8_t arm_check_mode_done(arm_t *arm)
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
	if(arm->type == ARM_TYPE_RIGHT) 
		ax12_id = AX12_ID_ARM_R;
	else
		ax12_id = AX12_ID_ARM_L;

	/* read positions */
	if(ax12_user_read_int(&gen.ax12, ax12_id, AA_PRESENT_POSITION_L, &ax12_pos))
		return 0;

	/* check if position is inside window */
	if(ABS(arm->ax12_pos - ax12_pos) < AX12_WINDOW_POSITION)	
		return END_TRAJ;
	
	/* ax12 blocking timeout */
	if(time_get_us2() - arm->time_us > AX12_BLOCKING_TIMEOUT_us) {
		ax12_user_write_int(&gen.ax12, ax12_id, AA_GOAL_POSITION_L, ax12_pos);
		arm->blocking = 1;
		return END_BLOCKING;
	}

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t arm_wait_end(arm_t *arm)
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = arm_check_mode_done(arm);

	return ret;
}

/**** boot funcions *********************************************************/
uint16_t boot_ax12_pos[BOOT_MODE_MAX] = {
	[BOOT_MODE_OPEN_FULL] 	= POS_BOOT_OPEN_FULL,
	[BOOT_MODE_OPEN_HOLD] 	= POS_BOOT_OPEN_HOLD,
	[BOOT_MODE_CLOSE]			= POS_BOOT_CLOSE,
};

/* set finger position depends on mode */
uint8_t boot_set_mode(boot_t *boot, uint8_t mode)
{
	uint8_t err;
		
	/* set ax12 possitions depends on mode and type */
	if(mode >= BOOT_MODE_MAX) {
		ACTUATORS_ERROR("Unknow BOOT MODE");
		return -1;
	}

	boot->mode = mode;
	boot->ax12_pos = boot_ax12_pos[boot->mode];
	
	/* apply to ax12 */
	err = ax12_user_write_int(&gen.ax12, AX12_ID_BOOT, AA_GOAL_POSITION_L, boot->ax12_pos);
	if(err) 	
		return err;

	return 0;
}

/* return 1 if mode is done */
uint8_t boot_check_mode_done(boot_t *boot)
{
	static microseconds us = 0;
	uint16_t ax12_pos;

	/* ax12 position pulling */
	if(time_get_us2() - us < AX12_PULLING_TIME_us)
		return 0;

	/* update time */
	us = time_get_us2();

	/* read positions */
	if(ax12_user_read_int(&gen.ax12, AX12_ID_BOOT, AA_PRESENT_POSITION_L, &ax12_pos))
		return 0;

	/* check if position is inside window */
	if(ABS(boot->ax12_pos - ax12_pos) < AX12_WINDOW_POSITION)	
		return END_TRAJ;
	
	return 0;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t boot_wait_end(boot_t *boot)
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = boot_check_mode_done(boot);

	return ret;
}

/**** hook funcions *********************************************************/

uint16_t hook_ax12_pos[HOOK_MODE_MAX] = {
	[HOOK_MODE_HIDE] 		= POS_HOOK_HIDE,
	[HOOK_MODE_SHOW] 		= POS_HOOK_SHOW,
	[HOOK_MODE_FUCKYOU]	= POS_HOOK_FUCKYOU,
	[HOOK_MODE_OPEN_HOLD]= POS_HOOK_OPEN_HOLD,
};

/* set finger position depends on mode */
uint8_t hook_set_mode(hook_t *hook, uint8_t mode)
{
	uint8_t err;
		
	/* set ax12 possitions depends on mode and type */
	if(mode >= HOOK_MODE_MAX) {
		ACTUATORS_ERROR("Unknow HOOK MODE");
		return -1;
	}

	hook->mode = mode;
	hook->ax12_pos = hook_ax12_pos[hook->mode];
	
	/* apply to ax12 */
	err = ax12_user_write_int(&gen.ax12, AX12_ID_HOOK, AA_GOAL_POSITION_L, hook->ax12_pos);
	if(err) 	return err;

	return 0;
}

/* return 1 if mode is done */
uint8_t hook_check_mode_done(hook_t *hook)
{
	static microseconds us = 0;
	uint16_t ax12_pos;

	/* ax12 position pulling */
	if(time_get_us2() - us < AX12_PULLING_TIME_us)
		return 0;

	/* update time */
	us = time_get_us2();

	/* read positions */
	if(ax12_user_read_int(&gen.ax12, AX12_ID_HOOK, AA_PRESENT_POSITION_L, &ax12_pos))
		return 0;

	/* check if position is inside window */
	if(ABS(hook->ax12_pos - ax12_pos) < AX12_WINDOW_POSITION)	
		return 1;
	
	return 0;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t hook_wait_end(hook_t *hook)
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = hook_check_mode_done(hook);

	return ret;
}

/**** trays funcions *********************************************************/

void tray_store_vibrate(void * data)
{
	tray_t *tray = (tray_t *)data;

	if(tray->servo_pos == POS_TRAY_STORE_UP)
		tray->servo_pos = pwm_servo_set(PWM_SERVO_TRAY_STORE, POS_TRAY_STORE_DOWN);
	else
		tray->servo_pos = pwm_servo_set(PWM_SERVO_TRAY_STORE, POS_TRAY_STORE_UP);
}

void tray_set_mode(tray_t *tray, uint8_t mode)
{
	uint8_t pos_saturated = 0;

	if(tray->type == TRAY_TYPE_RECEPTION) {
#if notyet
		switch(mode) {
			case TRAY_MODE_DOWN:
				tray->servo_pos = pwm_servo_set(PWM_SERVO_TRAY_RECEPTION, POS_TRAY_RECEPTION_DOWN);
				tray->mode = mode;
				pos_saturated = tray->servo_pos != POS_TRAY_RECEPTION_DOWN? 1:0;
				break;
			case TRAY_MODE_UP:
				tray->servo_pos = pwm_servo_set(PWM_SERVO_TRAY_RECEPTION, POS_TRAY_RECEPTION_UP);
				tray->mode = mode;
				pos_saturated = tray->servo_pos != POS_TRAY_RECEPTION_UP? 1:0;
				break;
			case TRAY_MODE_VIBRATE:
				ACTUATORS_ERROR("RECEPTION TRAY MODE does not exist");
				break;
			default:
				ACTUATORS_ERROR("Unknown RECEPTION TRAY MODE");
				break;
		}
		if(pos_saturated)
			ACTUATORS_ERROR("Reception Tray position saturated");
#endif
	}
	else if(tray->type == TRAY_TYPE_STORE) {

		scheduler_del_event(tray->event_handler);

		switch(mode) {
			case TRAY_MODE_DOWN:
				tray->servo_pos = pwm_servo_set(PWM_SERVO_TRAY_STORE, POS_TRAY_STORE_DOWN);
				tray->mode = mode;
				pos_saturated = tray->servo_pos != POS_TRAY_STORE_DOWN? 1:0;
				break;
			case TRAY_MODE_UP:
				tray->servo_pos = pwm_servo_set(PWM_SERVO_TRAY_STORE, POS_TRAY_STORE_UP);
				tray->mode = mode;
				pos_saturated = tray->servo_pos != POS_TRAY_STORE_UP? 1:0;
				break;
			case TRAY_MODE_VIBRATE:
				tray->event_handler = scheduler_add_periodical_event(tray_store_vibrate,
									 		(void *)(&slavedspic.tray_store), 
											TRAY_STORE_VIBRATE_PERIOD_us / SCHEDULER_UNIT);
				break;
			default:
				ACTUATORS_ERROR("Unknown STORE TRAY MODE");
				break;
		}
		if(pos_saturated)
			ACTUATORS_ERROR("Store Tray position saturated");
	}
	else if(tray->type == TRAY_TYPE_BOOT) {
		switch(mode) {
			case TRAY_MODE_DOWN:
				pwm_mc_set(PWM_MC_TRAY_BOOT, 0);
				break;
			case TRAY_MODE_UP:
				ACTUATORS_ERROR("BOOT TRAY MODE does not exist");
				break;
			case TRAY_MODE_VIBRATE:
				pwm_mc_set(PWM_MC_TRAY_BOOT, TRAY_BOOT_VIBRATE_PWM);
				break;
			default:
				ACTUATORS_ERROR("Unknown BOOT TRAY MODE");
				break;
		}
	}
}



/* init all actuators */
void actuator_init(void)
{
#ifdef PROGRAM_AX12
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_SHUTDOWN, 0x24);	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_LED, 0x24);

	/* specific config for mirror ax12, angle is limited */ 
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0xFF);	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CW_ANGLE_LIMIT_L, 0x00);	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CCW_ANGLE_LIMIT_L, 0x3FF);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_MOVING_SPEED_L, 0x3FF);#endif

	/* init ax12 */
	//ax12_user_write_int(&gen.ax12, AX12_ID_FINGERS_FLOOR_L, AA_MOVING_SPEED_L, 350);
	ax12_user_write_int(&gen.ax12, AX12_ID_BOOT, AA_MOVING_SPEED_L, 300);
	ax12_user_write_int(&gen.ax12, AX12_ID_HOOK, AA_MOVING_SPEED_L, 300);

   //ax12_user_write_int(&gen.ax12, 254, AA_MOVING_SPEED_L, 300);

	//ax12_user_write_int(&gen.ax12, AX12_ID_ARM_R, AA_MOVING_SPEED_L, 0x3FF);
	//ax12_user_write_int(&gen.ax12, AX12_ID_ARM_L, AA_MOVING_SPEED_L, 0x3FF);

	/* init structures */
	slavedspic.fingers_totem.type = FINGERS_TYPE_TOTEM;
	slavedspic.fingers_floor.type = FINGERS_TYPE_FLOOR;

	slavedspic.arm_left.type = ARM_TYPE_LEFT;
	slavedspic.arm_right.type = ARM_TYPE_RIGHT;

	slavedspic.tray_reception.type = TRAY_TYPE_RECEPTION;
	slavedspic.tray_store.type = TRAY_TYPE_STORE;
	slavedspic.tray_boot.type = TRAY_TYPE_BOOT;

}
