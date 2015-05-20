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
#include <clock_time.h>
#include <rdline.h>

#include "../common/i2c_commands.h"
#include "actuator.h"
#include "ax12_user.h"
#include "state.h"
#include "main.h"
#include "sensor.h"

#define ACTUATORS_DEBUG(args...) 	DEBUG(E_USER_ACTUATORS, args)
#define ACTUATORS_NOTICE(args...) 	NOTICE(E_USER_ACTUATORS, args)
#define ACTUATORS_ERROR(args...) 	ERROR(E_USER_ACTUATORS, args)

/* AX12 stuff */
#define AX12_PULLING_TIME_us		5000L
#define AX12_WINDOW_POSITION		15
#define AX12_BLOCKING_TIMEOUT_us	600000L


/**************************  AX12 MANAGE FUNCTIONS ****************************/
struct ax12_traj 
{
#define AX12_K_IMP_DEG		(3.413) //(1024.0/300.0)  // 3.413 imp/deg
#define AX12_K_MS_DEG		(3.333) //(200.0/60.0)	// 3.333 ms/deg	
#define AX12_WINDOW_NO_NEAR	(6.0*AX12_K_IMP_DEG)  // 6 deg
#define AX12_WINDOW_NEAR	(10.0*AX12_K_IMP_DEG) // 20 deg

	uint8_t id;
	int16_t zero_offset_pos;

	int16_t goal_angle_deg;
	uint16_t goal_pos;

	uint16_t goal_time_ms;
    microseconds time_us;
    
    uint16_t pos;
	int16_t angle_deg;
    uint16_t speed;
};


/* set position */
void ax12_set_pos (struct ax12_traj *ax12, int16_t pos)
{
    float k_ms_deg = 0.0;

    /* update current position/angle */
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12->pos);
 	ax12->angle_deg = (int16_t)(ax12->pos - ax12->zero_offset_pos);
 	ax12->angle_deg = (int16_t)(ax12->angle_deg / AX12_K_IMP_DEG);

    /* set goal angle */
    ax12->goal_pos = pos;
    ax12->goal_angle_deg = (int16_t)((pos - ax12->zero_offset_pos) / AX12_K_IMP_DEG);
	ax12_user_write_int(&gen.ax12, ax12->id , AA_GOAL_POSITION_L, ax12->goal_pos);

    /* update goal time */
    ax12_user_read_int(&gen.ax12, ax12->id, AA_MOVING_SPEED_L, &ax12->speed);
    k_ms_deg = AX12_K_MS_DEG * 0x3ff;
    k_ms_deg /= (ax12->speed==0? 0x3ff:ax12->speed);

	//ax12->goal_time_us = (microseconds)(ABS(ax12->angle_deg - ax12->goal_angle_deg) * AX12_K_MS_DEG * 1000);
	ax12->goal_time_ms = (ABS(ax12->angle_deg - ax12->goal_angle_deg) * k_ms_deg);
	ax12->time_us = time_get_us2();
}

/* set angle */
void ax12_set_a (struct ax12_traj *ax12, int16_t a)
{
    double k_ms_deg = 0.0;
	//printf ("%s, a = %d\n\r", __FUNCTION__, a);

    /* update current position/angle */
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12->pos);
 	ax12->angle_deg = (int16_t)(ax12->pos - ax12->zero_offset_pos);
 	ax12->angle_deg = (int16_t)(ax12->angle_deg / AX12_K_IMP_DEG);

    /* set goal angle */
    ax12->goal_angle_deg = a;
    ax12->goal_pos = ax12->zero_offset_pos + (int16_t)(ax12->goal_angle_deg * AX12_K_IMP_DEG);

	//printf ("%s, goal pos = %d\n\r", __FUNCTION__, ax12->goal_pos);
	ax12_user_write_int(&gen.ax12, ax12->id , AA_GOAL_POSITION_L, ax12->goal_pos);

    /* update goal time */
    ax12_user_read_int(&gen.ax12, ax12->id, AA_MOVING_SPEED_L, &ax12->speed);
    k_ms_deg = AX12_K_MS_DEG * 0x3ff;
    k_ms_deg /= (ax12->speed==0? 0x3ff:ax12->speed);

	//printf("k_ms_deg = %f\n\r", k_ms_deg);

	//ax12->goal_time_us = (ABS(ax12->angle_deg - ax12wri	->goal_angle_deg) * AX12_K_MS_DEG * 1000);
	ax12->goal_time_ms = (ABS(ax12->angle_deg - ax12->goal_angle_deg) * k_ms_deg);
	ax12->time_us = time_get_us2();

	//printf ("da %d, goal_us %ld\n\r", (int16_t)ABS(ax12->angle_deg - ax12->goal_angle_deg), (int32_t)ax12->goal_time_ms);
	//printf ("a=%d, goal_a =%d\n\r", ax12->angle_deg, ax12->goal_angle_deg);
}

/* get angle */
int16_t ax12_get_a (struct ax12_traj *ax12)
{
    /* update current position/angle */
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12->pos);
	ax12->angle_deg = ((ax12->pos - ax12->zero_offset_pos)); 
	ax12->angle_deg = (int16_t)(ax12->angle_deg  * (1.0/AX12_K_IMP_DEG));

	//printf ("%s, a = %d\n\r", __FUNCTION__, ax12->angle_deg);
    return ax12->angle_deg;
}


const char *err_tab []= {
	"END_TRAJ",
	"END_NEAR",
	"END_TIME",
	"END_BLOCKING",
	"END_RESERVED",
};

/* return string from end traj type num */
const char *get_err (uint8_t err)
{
	uint8_t i;
	if (err == 0)
		return "SUCCESS";
	for (i=0 ; i<4; i++) {
		if (err & (1 <<i))
			return err_tab[i];
	}
	return "END_UNKNOWN";
}


uint8_t ax12_test_traj_end (struct ax12_traj *ax12, uint8_t flags)
{
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12->pos);
    uint8_t ret = 0;

	if (flags & END_TRAJ) {
		if (ABS((int16_t)ax12->goal_pos - (int16_t)ax12->pos) < AX12_WINDOW_NO_NEAR)
			ret = END_TRAJ;
	}
	else if (flags & END_NEAR) {
		if (ABS((int16_t)ax12->goal_pos - (int16_t)ax12->pos) < AX12_WINDOW_NEAR)
			ret = END_NEAR;
	}
	//if (flags & END_TIME)
	//	if ((uint32_t)(time_get_us2() - ax12->time_us)/1000 > ax12->goal_time_ms)
	//		ret |= END_TIME;

	//if (flags & END_BLOCKING)
    else if ((uint32_t)(time_get_us2() - ax12->time_us)/1000 > (3*ax12->goal_time_ms)) {
            ax12_user_write_int(&gen.ax12, ax12->id , AA_GOAL_POSITION_L, ax12->pos);                       
		    ret = END_BLOCKING;
        }

	if (ret) {
		//DEBUG(E_USER_ACTUATORS, "goal_pos = %d, read_pos = %d",  ax12->goal_pos, ax12->pos);
		DEBUG(E_USER_ACTUATORS, "Got %s",  get_err (ret));
	}
    return ret;
}

/* wait traj end */
uint8_t ax12_wait_traj_end (struct ax12_traj *ax12, uint8_t flags) 
{
    microseconds us = time_get_us2();
    uint8_t ret = 0;

    while (ret == 0) {
        /* check end traj periodicaly (T = 5ms) */
        if ((uint32_t)(time_get_us2() - us) >= 5000L) {
            ret = ax12_test_traj_end (ax12, flags);
            us = time_get_us2();
        }

    }

	//DEBUG(E_USER_ACTUATORS, "Got %s",  get_err (ret));
    return ret;
}



/**** stands_exchanger functions ********************************************************/

/* stop without rampe */
void stands_exchanger_hard_stop(void)
{
	cs_set_consign(&slavedspic.stands_exchanger.cs, encoders_dspic_get_value(STANDS_EXCHANGER_ENCODER));
	slavedspic.stands_exchanger.qr.previous_var = 0;
	slavedspic.stands_exchanger.qr.previous_out = encoders_dspic_get_value(STANDS_EXCHANGER_ENCODER);
}

/* calibrate initial position */
void stands_exchanger_calibrate(void)
{
#define AUTOPOS_SPEED			20
#define AUTOPOS_ACCEL			10
#define AUTOPOS_BIG_DIST_mm		-350
#define AUTOPOS_BD_TIMEOUT_ms	10000

	int8_t ret;
	int16_t kp, ki, kd;

	/* save and set PID constants */
	kp = pid_get_gain_P(&slavedspic.stands_exchanger.pid);
	ki = pid_get_gain_I(&slavedspic.stands_exchanger.pid);
	kd = pid_get_gain_D(&slavedspic.stands_exchanger.pid);
	pid_set_gains(&slavedspic.stands_exchanger.pid,  50, 0, 0);

	/* set low speed, and hi acceleration for fast response */
	quadramp_set_1st_order_vars(&slavedspic.stands_exchanger.qr, AUTOPOS_SPEED, AUTOPOS_SPEED);
	quadramp_set_2nd_order_vars(&slavedspic.stands_exchanger.qr, AUTOPOS_ACCEL, AUTOPOS_ACCEL);
	ACTUATORS_DEBUG("Down speed and acceleration");

	/* goto zero with cs */
	slavedspic.flags |= DO_CS;
	cs_set_consign(&slavedspic.stands_exchanger.cs, (int32_t)(AUTOPOS_BIG_DIST_mm * STANDS_EXCHANGER_K_IMP_mm));
	ACTUATORS_DEBUG("Goto zero");	

	/* wait end initial positioning */
	ret = WAIT_COND_OR_TIMEOUT(bd_get(&slavedspic.stands_exchanger.bd), AUTOPOS_BD_TIMEOUT_ms);
	if(!ret) {
		ACTUATORS_DEBUG("Initial positioning not reached");
		return;
	}

	/* reset encoder */
	encoders_dspic_set_value(STANDS_EXCHANGER_ENCODER, STANDS_EXCHANGER_CALIB_IMP_MAX);
	stands_exchanger_hard_stop();
	pid_reset(&slavedspic.stands_exchanger.pid);
	bd_reset(&slavedspic.stands_exchanger.bd);
	ACTUATORS_DEBUG("End initial positioning, encoder is reset to zero");	

	/* restore speed, acceleration and PID */
	quadramp_set_1st_order_vars(&slavedspic.stands_exchanger.qr, STANDS_EXCHANGER_SPEED, STANDS_EXCHANGER_SPEED);
	quadramp_set_2nd_order_vars(&slavedspic.stands_exchanger.qr, STANDS_EXCHANGER_ACCEL, STANDS_EXCHANGER_ACCEL);
	pid_set_gains(&slavedspic.stands_exchanger.pid, kp, ki, kd);

	ACTUATORS_DEBUG("Restored speed, acceleration and PID");
	ACTUATORS_DEBUG("%s ends", __FUNCTION__);

	/* set calibration flag */
	slavedspic.stands_exchanger.calibrated = 1;
	stands_exchanger_wait_end();

	return;
}

/* set position in mm */
void stands_exchanger_set_position(int32_t position_mm)
{
	/* check calibration flag */
//	if(!slavedspic.stands_exchanger.calibrated) {
//		ACTUATORS_ERROR("Can't set position, stands_exchanger is NOT CALIBRATED");
//		return;
//	}

	/* saturate position */
	if(position_mm > STANDS_EXCHANGER_POSITION_MAX_mm)
		position_mm = STANDS_EXCHANGER_POSITION_MAX_mm;
	else if(position_mm < STANDS_EXCHANGER_POSITION_MIN_mm)
		position_mm = STANDS_EXCHANGER_POSITION_MIN_mm;

	/* apply consign */
//	cs_set_consign(&slavedspic.stands_exchanger.cs, (int32_t)(((int32_t)STANDS_EXCHANGER_POSITION_MAX_mm-position_mm) * STANDS_EXCHANGER_K_IMP_mm));
	cs_set_consign(&slavedspic.stands_exchanger.cs, (int32_t)(position_mm * STANDS_EXCHANGER_K_IMP_mm));
	slavedspic.stands_exchanger.blocking = 0;
}

/* return position in mm */
int32_t stands_exchanger_get_position(void)
{
	return (int32_t)((int32_t)STANDS_EXCHANGER_POSITION_MAX_mm - (encoders_dspic_get_value(STANDS_EXCHANGER_ENCODER)/STANDS_EXCHANGER_K_IMP_mm));
}

/* return 1 if position reached, -1 if blocking and zero if no ends yet */
int8_t stands_exchanger_check_position_reached(void)
{

	/* test consign end */
	if(cs_get_consign(&slavedspic.stands_exchanger.cs) == cs_get_filtered_consign(&slavedspic.stands_exchanger.cs) && 
	   ABS(cs_get_error(&slavedspic.stands_exchanger.cs)) < 100) {
		return END_TRAJ;
	}

	/* test blocking */
	if(bd_get(&slavedspic.stands_exchanger.bd)) {
		stands_exchanger_hard_stop();
		pid_reset(&slavedspic.stands_exchanger.pid);
		bd_reset(&slavedspic.stands_exchanger.bd);
		slavedspic.stands_exchanger.blocking = 1;
		return END_BLOCKING;
	}

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t stands_exchanger_test_traj_end()
{
	uint8_t ret = 0;

	ret = stands_exchanger_check_position_reached();

	if (ret) {
		DEBUG(E_USER_ACTUATORS, "Got %s",  get_err (ret));
	}
	return ret;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t stands_exchanger_wait_end()
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = stands_exchanger_check_position_reached();

	return ret;
}



/**** popcorn tray functions *********************************************************/
uint16_t popcorn_tray_servo_pos [POPCORN_TRAY_MODE_MAX] = {
	[POPCORN_TRAY_MODE_OPEN] 	= POS_POPCORN_TRAY_OPEN,
	[POPCORN_TRAY_MODE_CLOSE] 	= POS_POPCORN_TRAY_CLOSE
};

/* set popcorn_tray position depends on mode */
int8_t popcorn_tray_set_mode(popcorn_tray_t *popcorn_tray, uint8_t mode, int16_t pos_offset)
{
	/* set futaba servo position depends on mode */
	if(mode >= POPCORN_TRAY_MODE_MAX) {
		ACTUATORS_ERROR("Unknown POPCORN TRAY MODE");
		return -1;
	}

	popcorn_tray->mode = mode;
	popcorn_tray->servo_pos = popcorn_tray_servo_pos[popcorn_tray->mode] + pos_offset;
	
	/* saturate to position range */
	if(popcorn_tray->servo_pos > popcorn_tray_servo_pos[POPCORN_TRAY_MODE_POS_MAX])
		popcorn_tray->servo_pos = popcorn_tray_servo_pos[POPCORN_TRAY_MODE_POS_MAX];
	else if(popcorn_tray->servo_pos < popcorn_tray_servo_pos[POPCORN_TRAY_MODE_POS_MIN])
		popcorn_tray->servo_pos = popcorn_tray_servo_pos[POPCORN_TRAY_MODE_POS_MIN];

	/* apply to futaba servo */
#ifndef HOST_VERSION
	pwm_servo_set(PWM_SERVO_POPCORN_TRAY, popcorn_tray->servo_pos);
#endif

	return 0;
}



/**** stands_clamp functions *********************************************************/
uint16_t stands_clamp_servo_pos[STANDS_CLAMP_TYPE_MAX][STANDS_CLAMP_MODE_MAX] = {
	[STANDS_CLAMP_TYPE_LEFT][STANDS_CLAMP_MODE_FULL_OPEN] 	= POS_STANDS_CLAMP_L_FULL_OPEN,
	[STANDS_CLAMP_TYPE_LEFT][STANDS_CLAMP_MODE_OPEN] 		= POS_STANDS_CLAMP_L_OPEN,
	[STANDS_CLAMP_TYPE_LEFT][STANDS_CLAMP_MODE_CLOSE] 		= POS_STANDS_CLAMP_L_CLOSE,

	[STANDS_CLAMP_TYPE_RIGHT][STANDS_CLAMP_MODE_FULL_OPEN] 	= POS_STANDS_CLAMP_R_FULL_OPEN,
	[STANDS_CLAMP_TYPE_RIGHT][STANDS_CLAMP_MODE_OPEN] 		= POS_STANDS_CLAMP_R_OPEN,
	[STANDS_CLAMP_TYPE_RIGHT][STANDS_CLAMP_MODE_CLOSE] 		= POS_STANDS_CLAMP_R_CLOSE,

};

/* set stands_clamp position depends on mode */
int8_t stands_clamp_set_mode(stands_clamp_t *stands_clamp, uint8_t mode, int16_t pos_offset)
{	
	/* set futaba servo position depends on mode and type */
	if(mode >= STANDS_CLAMP_MODE_MAX) {
		ACTUATORS_ERROR("Unknown %s STANDS_CLAMP MODE", stands_clamp->type == STANDS_CLAMP_TYPE_RIGHT? "RIGHT":"LEFT");
		return -1;
	}

	stands_clamp->mode = mode;
	stands_clamp->servo_pos = stands_clamp_servo_pos[stands_clamp->type][stands_clamp->mode] + pos_offset;
	
	/* saturate to position range */
	if(stands_clamp->type == STANDS_CLAMP_TYPE_LEFT) {
		if(stands_clamp->servo_pos > stands_clamp_servo_pos[STANDS_CLAMP_TYPE_LEFT][STANDS_CLAMP_MODE_L_POS_MAX])
			stands_clamp->servo_pos = stands_clamp_servo_pos[STANDS_CLAMP_TYPE_LEFT][STANDS_CLAMP_MODE_L_POS_MAX];
		else if(stands_clamp->servo_pos < stands_clamp_servo_pos[STANDS_CLAMP_TYPE_LEFT][STANDS_CLAMP_MODE_L_POS_MIN])
			stands_clamp->servo_pos = stands_clamp_servo_pos[STANDS_CLAMP_TYPE_LEFT][STANDS_CLAMP_MODE_L_POS_MIN];
	} 
	else {
		if(stands_clamp->servo_pos > stands_clamp_servo_pos[STANDS_CLAMP_TYPE_RIGHT][STANDS_CLAMP_MODE_R_POS_MAX])
			stands_clamp->servo_pos = stands_clamp_servo_pos[STANDS_CLAMP_TYPE_RIGHT][STANDS_CLAMP_MODE_R_POS_MAX];
		else if(stands_clamp->servo_pos < stands_clamp_servo_pos[STANDS_CLAMP_TYPE_RIGHT][STANDS_CLAMP_MODE_R_POS_MIN])
			stands_clamp->servo_pos = stands_clamp_servo_pos[STANDS_CLAMP_TYPE_RIGHT][STANDS_CLAMP_MODE_R_POS_MIN];
	}

#ifndef HOST_VERSION
	/* apply to futaba servo */
	if(stands_clamp->type == STANDS_CLAMP_TYPE_LEFT)
		pwm_servo_set(PWM_SERVO_STANDS_CLAMP_L, stands_clamp->servo_pos);
	else
		pwm_servo_set(PWM_SERVO_STANDS_CLAMP_R, stands_clamp->servo_pos);
#endif

	return 0;
}



/**** stands_tower_clamps functions *********************************************************/
uint16_t stands_tower_clamps_up_ax12_pos [STANDS_TOWER_CLAMPS_MODE_MAX] = {
	[STANDS_TOWER_CLAMPS_MODE_UNLOCK_LEFT] 	= POS_STANDS_TOWER_CLAMPS_UP_UNLOCK_LEFT, 
	[STANDS_TOWER_CLAMPS_MODE_LOCK] 		= POS_STANDS_TOWER_CLAMPS_UP_LOCK, 
	[STANDS_TOWER_CLAMPS_MODE_UNLOCK_RIGHT] = POS_STANDS_TOWER_CLAMPS_UP_UNLOCK_RIGHT
}; 

uint16_t stands_tower_clamps_down_ax12_pos [STANDS_TOWER_CLAMPS_MODE_MAX] = {
	[STANDS_TOWER_CLAMPS_MODE_UNLOCK_LEFT] 	= POS_STANDS_TOWER_CLAMPS_DOWN_UNLOCK_LEFT, 
	[STANDS_TOWER_CLAMPS_MODE_LOCK] 		= POS_STANDS_TOWER_CLAMPS_DOWN_LOCK, 
	[STANDS_TOWER_CLAMPS_MODE_UNLOCK_RIGHT] = POS_STANDS_TOWER_CLAMPS_DOWN_UNLOCK_RIGHT 
};

struct ax12_traj ax12_stands_tower_clamps_up = { .id = AX12_ID_STANDS_TOWER_CLAMPS_UP, .zero_offset_pos = 0 };
struct ax12_traj ax12_stands_tower_clamps_down = { .id = AX12_ID_STANDS_TOWER_CLAMPS_DOWN, .zero_offset_pos = 0 };

/* set stands_tower_clamps position depends on mode */
int8_t stands_tower_clamps_set_mode(stands_tower_clamps_t *stands_tower_clamps, uint8_t mode, int16_t pos_offset)
{
	/* set ax12 positions depends on mode */
	if(mode >= STANDS_TOWER_CLAMPS_MODE_MAX) {
		ACTUATORS_ERROR("Unknown STANDS_TOWER_CLAMPS MODE");
		return -1;
	}

	/* ax12 positions */
	stands_tower_clamps->mode = mode;
	stands_tower_clamps->ax12_pos_up = stands_tower_clamps_up_ax12_pos[stands_tower_clamps->mode] + pos_offset;
	stands_tower_clamps->ax12_pos_down = stands_tower_clamps_down_ax12_pos[stands_tower_clamps->mode] + pos_offset;

	/* saturate to position range */
	if(stands_tower_clamps->ax12_pos_up > stands_tower_clamps_up_ax12_pos[STANDS_TOWER_CLAMPS_MODE_UP_POS_MAX])
		stands_tower_clamps->ax12_pos_up = stands_tower_clamps_up_ax12_pos[STANDS_TOWER_CLAMPS_MODE_UP_POS_MAX];
	else if(stands_tower_clamps->ax12_pos_up < stands_tower_clamps_up_ax12_pos[STANDS_TOWER_CLAMPS_MODE_UP_POS_MIN])
		stands_tower_clamps->ax12_pos_up = stands_tower_clamps_up_ax12_pos[STANDS_TOWER_CLAMPS_MODE_UP_POS_MIN];

	if(stands_tower_clamps->ax12_pos_down > stands_tower_clamps_down_ax12_pos[STANDS_TOWER_CLAMPS_MODE_DOWN_POS_MAX])
		stands_tower_clamps->ax12_pos_down = stands_tower_clamps_down_ax12_pos[STANDS_TOWER_CLAMPS_MODE_DOWN_POS_MAX];
	else if(stands_tower_clamps->ax12_pos_down < stands_tower_clamps_down_ax12_pos[STANDS_TOWER_CLAMPS_MODE_DOWN_POS_MIN])
		stands_tower_clamps->ax12_pos_down = stands_tower_clamps_down_ax12_pos[STANDS_TOWER_CLAMPS_MODE_DOWN_POS_MIN];

	/* apply to ax12 */
    ax12_set_pos(&ax12_stands_tower_clamps_up, stands_tower_clamps->ax12_pos_up);
    ax12_set_pos(&ax12_stands_tower_clamps_down, stands_tower_clamps->ax12_pos_down);

	return 0;
}

/* return stands_tower_clamps traj flag */
uint8_t stands_tower_clamps_test_traj_end(stands_tower_clamps_t *stands_tower_clamps)
{
    uint8_t ret_up, ret_down;
   
   	ret_up = ax12_test_traj_end (&ax12_stands_tower_clamps_up, END_NEAR|END_TRAJ);
   	ret_down = ax12_test_traj_end (&ax12_stands_tower_clamps_down, END_NEAR|END_TRAJ);

    return (ret_up | ret_down);
}

/* return END_TRAJ or END_TIMER */
uint8_t stands_tower_clamps_wait_end(stands_tower_clamps_t *stands_tower_clamps)
{
    uint8_t ret_up, ret_down;
   
   	ret_up = ax12_wait_traj_end (&ax12_stands_tower_clamps_up, END_NEAR|END_TRAJ);
   	ret_down = ax12_wait_traj_end (&ax12_stands_tower_clamps_down, END_NEAR|END_TRAJ);

    return (ret_up | ret_down);
}



/**** stands_elevators functions *********************************************************/
uint16_t stands_elevator_ax12_pos[STANDS_ELEVATOR_TYPE_MAX][STANDS_ELEVATOR_MODE_MAX] = {
	[STANDS_ELEVATOR_TYPE_LEFT][STANDS_ELEVATOR_MODE_UP] 	= POS_STANDS_ELEVATOR_L_UP,
	[STANDS_ELEVATOR_TYPE_LEFT][STANDS_ELEVATOR_MODE_DOWN] 	= POS_STANDS_ELEVATOR_L_DOWN,

	[STANDS_ELEVATOR_TYPE_RIGHT][STANDS_ELEVATOR_MODE_UP] 	= POS_STANDS_ELEVATOR_R_UP,
	[STANDS_ELEVATOR_TYPE_RIGHT][STANDS_ELEVATOR_MODE_DOWN]	= POS_STANDS_ELEVATOR_R_DOWN
};

struct ax12_traj ax12_stands_elevator_l = { .id = AX12_ID_STANDS_ELEVATOR_L, .zero_offset_pos = 0 };
struct ax12_traj ax12_stands_elevator_r = { .id = AX12_ID_STANDS_ELEVATOR_R, .zero_offset_pos = 0 };

/* set stands_elevators position depends on mode*/
int8_t stands_elevator_set_mode(stands_elevator_t *stands_elevator, uint8_t mode, int16_t pos_offset)
{	
	/* set ax12 position depends on mode and type */
	if(mode >= STANDS_ELEVATOR_MODE_MAX) {
		ACTUATORS_ERROR("Unknown %s STANDS_ELEVATOR MODE", stands_elevator->type == STANDS_ELEVATOR_TYPE_RIGHT? "RIGHT":"LEFT");
		return -1;
	}

	stands_elevator->mode = mode;
	stands_elevator->ax12_pos = stands_elevator_ax12_pos[stands_elevator->type][stands_elevator->mode] + pos_offset;

	/* saturate to position range */
	if(stands_elevator->type == STANDS_ELEVATOR_TYPE_LEFT) {
		if(stands_elevator->ax12_pos > stands_elevator_ax12_pos[STANDS_ELEVATOR_TYPE_LEFT][STANDS_ELEVATOR_MODE_L_POS_MAX])
			stands_elevator->ax12_pos = stands_elevator_ax12_pos[STANDS_ELEVATOR_TYPE_LEFT][STANDS_ELEVATOR_MODE_L_POS_MAX];
		else if(stands_elevator->ax12_pos < stands_elevator_ax12_pos[STANDS_ELEVATOR_TYPE_LEFT][STANDS_ELEVATOR_MODE_L_POS_MIN])
			stands_elevator->ax12_pos = stands_elevator_ax12_pos[STANDS_ELEVATOR_TYPE_LEFT][STANDS_ELEVATOR_MODE_L_POS_MIN];
	} 
	else if(stands_elevator->type == STANDS_ELEVATOR_TYPE_RIGHT) {
		if(stands_elevator->ax12_pos > stands_elevator_ax12_pos[STANDS_ELEVATOR_TYPE_RIGHT][STANDS_ELEVATOR_MODE_R_POS_MAX])
			stands_elevator->ax12_pos = stands_elevator_ax12_pos[STANDS_ELEVATOR_TYPE_RIGHT][STANDS_ELEVATOR_MODE_R_POS_MAX];
		else if(stands_elevator->ax12_pos < stands_elevator_ax12_pos[STANDS_ELEVATOR_TYPE_RIGHT][STANDS_ELEVATOR_MODE_R_POS_MIN])
			stands_elevator->ax12_pos = stands_elevator_ax12_pos[STANDS_ELEVATOR_TYPE_RIGHT][STANDS_ELEVATOR_MODE_R_POS_MIN];
	} 

	/* apply to ax12 */
	if(stands_elevator->type == STANDS_ELEVATOR_TYPE_LEFT)
    	ax12_set_pos(&ax12_stands_elevator_l, stands_elevator->ax12_pos);
	else if(stands_elevator->type == STANDS_ELEVATOR_TYPE_RIGHT)
    	ax12_set_pos(&ax12_stands_elevator_r, stands_elevator->ax12_pos);

	return 0;
}

/* return stands_elevator traj flag */
uint8_t stands_elevator_test_traj_end(stands_elevator_t *stands_elevator)
{
    uint8_t ret = 0;

	if(stands_elevator->type == STANDS_ELEVATOR_TYPE_LEFT) {
    	ret = ax12_test_traj_end (&ax12_stands_elevator_l, END_NEAR|END_TRAJ);
	} 
	else if(stands_elevator->type == STANDS_ELEVATOR_TYPE_RIGHT) {
    	ret = ax12_test_traj_end (&ax12_stands_elevator_r, END_NEAR|END_TRAJ);
	} 
   
    return ret;
}

/* return END_TRAJ or END_TIMER */
uint8_t stands_elevator_wait_end(stands_elevator_t *stands_elevator)
{
    uint8_t ret = 0;

	if(stands_elevator->type == STANDS_ELEVATOR_TYPE_LEFT) {
    	ret = ax12_wait_traj_end (&ax12_stands_elevator_l, END_NEAR|END_TRAJ);
	} 
	else if(stands_elevator->type == STANDS_ELEVATOR_TYPE_RIGHT) {
    	ret = ax12_wait_traj_end (&ax12_stands_elevator_r, END_NEAR|END_TRAJ);
	} 
   
    return ret;
}



/**** stands_blade functions *********************************************************/
int8_t stands_blade_ax12_ang[STANDS_BLADE_TYPE_MAX][STANDS_BLADE_MODE_MAX] = {
	[STANDS_BLADE_TYPE_LEFT][STANDS_BLADE_MODE_HIDE_LEFT] 			= ANG_STANDS_BLADE_L_HIDE_LEFT,
	[STANDS_BLADE_TYPE_LEFT][STANDS_BLADE_MODE_PUSH_STAND_LEFT] 	= ANG_STANDS_BLADE_L_PUSH_STAND_LEFT,
	[STANDS_BLADE_TYPE_LEFT][STANDS_BLADE_MODE_CENTER] 				= ANG_STANDS_BLADE_L_CENTER,
	[STANDS_BLADE_TYPE_LEFT][STANDS_BLADE_MODE_PUSH_STAND_RIGHT] 	= ANG_STANDS_BLADE_L_PUSH_STAND_RIGHT,
	[STANDS_BLADE_TYPE_LEFT][STANDS_BLADE_MODE_HIDE_RIGHT] 			= ANG_STANDS_BLADE_L_HIDE_RIGHT,

	[STANDS_BLADE_TYPE_RIGHT][STANDS_BLADE_MODE_HIDE_LEFT] 			= ANG_STANDS_BLADE_R_HIDE_LEFT,
	[STANDS_BLADE_TYPE_RIGHT][STANDS_BLADE_MODE_PUSH_STAND_LEFT] 	= ANG_STANDS_BLADE_R_PUSH_STAND_LEFT,
	[STANDS_BLADE_TYPE_RIGHT][STANDS_BLADE_MODE_CENTER] 			= ANG_STANDS_BLADE_R_CENTER,
	[STANDS_BLADE_TYPE_RIGHT][STANDS_BLADE_MODE_PUSH_STAND_RIGHT] 	= ANG_STANDS_BLADE_R_PUSH_STAND_RIGHT,
	[STANDS_BLADE_TYPE_RIGHT][STANDS_BLADE_MODE_HIDE_RIGHT] 		= ANG_STANDS_BLADE_R_HIDE_RIGHT
};

struct ax12_traj ax12_stands_blade_l = { .id = AX12_ID_STANDS_BLADE_L, .zero_offset_pos = POS_OFFSET_STANDS_BLADE_L };
struct ax12_traj ax12_stands_blade_r = { .id = AX12_ID_STANDS_BLADE_R, .zero_offset_pos = POS_OFFSET_STANDS_BLADE_R };

/* set stands_blade angle depends on mode*/
int8_t stands_blade_set_mode(stands_blade_t *stands_blade, uint8_t mode, int8_t ang_offset)
{
	/* set ax12 position depends on mode and type */
	if(mode >= STANDS_BLADE_MODE_MAX) {
		ACTUATORS_ERROR("Unknown %s STANDS_BLADE MODE", stands_blade->type == STANDS_BLADE_TYPE_RIGHT? "RIGHT":"LEFT");
		return -1;
	}

	stands_blade->mode = mode;
	if(stands_blade->mode == I2C_STANDS_BLADE_MODE_SET_ANGLE)
		stands_blade->ax12_ang = ang_offset;
	else
		stands_blade->ax12_ang = stands_blade_ax12_ang[stands_blade->type][stands_blade->mode] + ang_offset;

	/* saturate to angle range */
	if(stands_blade->type == STANDS_BLADE_TYPE_LEFT) {
		if(stands_blade->ax12_ang > stands_blade_ax12_ang[STANDS_BLADE_TYPE_LEFT][STANDS_BLADE_MODE_L_ANG_MAX])
			stands_blade->ax12_ang = stands_blade_ax12_ang[STANDS_BLADE_TYPE_LEFT][STANDS_BLADE_MODE_L_ANG_MAX];
		else if(stands_blade->ax12_ang < stands_blade_ax12_ang[STANDS_BLADE_TYPE_LEFT][STANDS_BLADE_MODE_L_ANG_MIN])
			stands_blade->ax12_ang = stands_blade_ax12_ang[STANDS_BLADE_TYPE_LEFT][STANDS_BLADE_MODE_L_ANG_MIN];
	} 
	else if(stands_blade->type == STANDS_BLADE_TYPE_RIGHT) {
		if(stands_blade->ax12_ang > stands_blade_ax12_ang[STANDS_BLADE_TYPE_RIGHT][STANDS_BLADE_MODE_R_ANG_MAX])
			stands_blade->ax12_ang = stands_blade_ax12_ang[STANDS_BLADE_TYPE_RIGHT][STANDS_BLADE_MODE_R_ANG_MAX];
		else if(stands_blade->ax12_ang < stands_blade_ax12_ang[STANDS_BLADE_TYPE_RIGHT][STANDS_BLADE_MODE_R_ANG_MIN])
			stands_blade->ax12_ang = stands_blade_ax12_ang[STANDS_BLADE_TYPE_RIGHT][STANDS_BLADE_MODE_R_ANG_MIN];
	} 

	/* apply to ax12 */
	if(stands_blade->type == STANDS_BLADE_TYPE_LEFT)
    	ax12_set_a(&ax12_stands_blade_l, stands_blade->ax12_ang);
	else if(stands_blade->type == STANDS_BLADE_TYPE_RIGHT)
    	ax12_set_a(&ax12_stands_blade_r, stands_blade->ax12_ang);

	return 0;
}

/* return stands_blade traj flag */
uint8_t stands_blade_test_traj_end(stands_blade_t *stands_blade)
{
    uint8_t ret = 0;
   
	if(stands_blade->type == STANDS_BLADE_TYPE_LEFT) {
    	ret = ax12_test_traj_end (&ax12_stands_blade_l, END_NEAR|END_TRAJ);
	} 
	else if(stands_blade->type == STANDS_BLADE_TYPE_RIGHT) {
    	ret = ax12_test_traj_end (&ax12_stands_blade_r, END_NEAR|END_TRAJ);
	} 

    return ret;
}

/* return END_TRAJ or END_TIMER */
uint8_t stands_blade_wait_end(stands_blade_t *stands_blade)
{
    uint8_t ret = 0;
   
	if(stands_blade->type == STANDS_BLADE_TYPE_LEFT) {
    	ret = ax12_wait_traj_end (&ax12_stands_blade_l, END_NEAR|END_TRAJ);
	} 
	else if(stands_blade->type == STANDS_BLADE_TYPE_RIGHT) {
    	ret = ax12_wait_traj_end (&ax12_stands_blade_r, END_NEAR|END_TRAJ);
	} 

    return ret;
}



/**** cup_clamp_popcorn_door functions *********************************************************/
uint16_t cup_clamp_popcorn_door_ax12_pos[CUP_CLAMP_POPCORN_DOOR_TYPE_MAX][CUP_CLAMP_POPCORN_DOOR_MODE_MAX] = {
	[CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT][CUP_CLAMP_MODE_HIDE] 		= POS_CUP_CLAMP_L_HIDE,
	[CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT][CUP_CLAMP_MODE_LOCKED] 		= POS_CUP_CLAMP_L_LOCKED,
	[CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT][CUP_CLAMP_MODE_OPEN] 		= POS_CUP_CLAMP_L_OPEN,
	[CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT][POPCORN_DOOR_MODE_OPEN] 		= POS_POPCORN_DOOR_L_OPEN,

	[CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT][CUP_CLAMP_MODE_HIDE] 		= POS_CUP_CLAMP_R_HIDE,
	[CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT][CUP_CLAMP_MODE_LOCKED]		= POS_CUP_CLAMP_R_LOCKED,
	[CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT][CUP_CLAMP_MODE_OPEN] 		= POS_CUP_CLAMP_R_OPEN,
	[CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT][POPCORN_DOOR_MODE_OPEN]		= POS_POPCORN_DOOR_R_OPEN
};

struct ax12_traj ax12_cup_clamp_popcorn_door_l = { .id = AX12_ID_CUP_CLAMP_POPCORN_DOOR_L, .zero_offset_pos = 0 };
struct ax12_traj ax12_cup_clamp_popcorn_door_r = { .id = AX12_ID_CUP_CLAMP_POPCORN_DOOR_R, .zero_offset_pos = 0 };

/* set cup_clamp_popcorn_door position depends on mode*/
int8_t cup_clamp_popcorn_door_set_mode(cup_clamp_popcorn_door_t *cup_clamp_popcorn_door, uint8_t mode, int16_t pos_offset)
{	
	/* set ax12 position depends on mode and type */
	if(mode >= CUP_CLAMP_POPCORN_DOOR_MODE_MAX) {
		ACTUATORS_ERROR("Unknown CUP_CLAMP_POPCORN_DOOR MODE", cup_clamp_popcorn_door->type == CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT? "RIGHT":"LEFT");
		return -1;
	}

	/* ax12 positions */
	cup_clamp_popcorn_door->mode = mode;
	cup_clamp_popcorn_door->ax12_pos = cup_clamp_popcorn_door_ax12_pos[cup_clamp_popcorn_door->type][cup_clamp_popcorn_door->mode] + pos_offset;

	/* saturate to position range */
	if(cup_clamp_popcorn_door->type == CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT) {
		if(cup_clamp_popcorn_door->ax12_pos > cup_clamp_popcorn_door_ax12_pos[CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT][CUP_CLAMP_POPCORN_DOOR_MODE_L_POS_MAX])
			cup_clamp_popcorn_door->ax12_pos = cup_clamp_popcorn_door_ax12_pos[CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT][CUP_CLAMP_POPCORN_DOOR_MODE_L_POS_MAX];
		else if(cup_clamp_popcorn_door->ax12_pos < cup_clamp_popcorn_door_ax12_pos[CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT][CUP_CLAMP_POPCORN_DOOR_MODE_L_POS_MIN])
			cup_clamp_popcorn_door->ax12_pos = cup_clamp_popcorn_door_ax12_pos[CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT][CUP_CLAMP_POPCORN_DOOR_MODE_L_POS_MIN];
	}
	else if(cup_clamp_popcorn_door->type == CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT) {
		if(cup_clamp_popcorn_door->ax12_pos > cup_clamp_popcorn_door_ax12_pos[CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT][CUP_CLAMP_POPCORN_DOOR_MODE_R_POS_MAX])
			cup_clamp_popcorn_door->ax12_pos = cup_clamp_popcorn_door_ax12_pos[CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT][CUP_CLAMP_POPCORN_DOOR_MODE_R_POS_MAX];
		else if(cup_clamp_popcorn_door->ax12_pos < cup_clamp_popcorn_door_ax12_pos[CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT][CUP_CLAMP_POPCORN_DOOR_MODE_R_POS_MIN])
			cup_clamp_popcorn_door->ax12_pos = cup_clamp_popcorn_door_ax12_pos[CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT][CUP_CLAMP_POPCORN_DOOR_MODE_R_POS_MIN];
	}

	/* apply to ax12 */
	if(cup_clamp_popcorn_door->type == CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT)
	   	ax12_set_pos(&ax12_cup_clamp_popcorn_door_l, cup_clamp_popcorn_door->ax12_pos);
	else if(cup_clamp_popcorn_door->type == CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT)
	   	ax12_set_pos(&ax12_cup_clamp_popcorn_door_r, cup_clamp_popcorn_door->ax12_pos);

	return 0;
}

/* set cup_clamp_position depends on mode */
int8_t cup_clamp_set_mode(cup_clamp_popcorn_door_t *cup_clamp_popcorn_door, uint8_t mode, int16_t pos_offset)
{
	return cup_clamp_popcorn_door_set_mode(cup_clamp_popcorn_door, mode, pos_offset);
}

/* set popcorn_door position depends on mode */
int8_t popcorn_door_set_mode(cup_clamp_popcorn_door_t *cup_clamp_popcorn_door, uint8_t mode, int16_t pos_offset)
{
	return cup_clamp_popcorn_door_set_mode(cup_clamp_popcorn_door, mode, pos_offset);
}

/* return cup_clamp_popcorn_door traj flag */
uint8_t cup_clamp_popcorn_door_test_traj_end(cup_clamp_popcorn_door_t *cup_clamp_popcorn_door)
{
    uint8_t ret = 0;
 
	if(cup_clamp_popcorn_door->type == CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT)
		ret = ax12_test_traj_end (&ax12_cup_clamp_popcorn_door_l, END_NEAR|END_TRAJ);
	else if(cup_clamp_popcorn_door->type == CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT)
		ret = ax12_test_traj_end (&ax12_cup_clamp_popcorn_door_r, END_NEAR|END_TRAJ);

    return ret;
}

/* return END_TRAJ or END_TIMER */
uint8_t cup_clamp_popcorn_door_wait_end(cup_clamp_popcorn_door_t *cup_clamp_popcorn_door)
{
    uint8_t ret = 0;
   
	if(cup_clamp_popcorn_door->type == CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT)
		ret = ax12_wait_traj_end (&ax12_cup_clamp_popcorn_door_l, END_NEAR|END_TRAJ);
	else if(cup_clamp_popcorn_door->type == CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT)
		ret = ax12_wait_traj_end (&ax12_cup_clamp_popcorn_door_r, END_NEAR|END_TRAJ);

    return ret;
}



/**** popcorn_ramps functions *********************************************************/
uint16_t popcorn_ramp_l_ax12_pos [POPCORN_RAMPS_MODE_MAX] = {
	[POPCORN_RAMPS_MODE_HIDE] 		= POS_POPCORN_RAMP_L_HIDE, 
	[POPCORN_RAMPS_MODE_HARVEST] 	= POS_POPCORN_RAMP_L_HARVEST, 
	[POPCORN_RAMPS_MODE_OPEN]		= POS_POPCORN_RAMP_L_OPEN 
};

uint16_t popcorn_ramp_r_ax12_pos [POPCORN_RAMPS_MODE_MAX] = {
	[POPCORN_RAMPS_MODE_HIDE] 		= POS_POPCORN_RAMP_R_HIDE, 
	[POPCORN_RAMPS_MODE_HARVEST] 	= POS_POPCORN_RAMP_R_HARVEST, 
	[POPCORN_RAMPS_MODE_OPEN]		= POS_POPCORN_RAMP_R_OPEN 
};

struct ax12_traj ax12_popcorn_ramp_l = { .id = AX12_ID_POPCORN_RAMP_L, .zero_offset_pos = 0 };
struct ax12_traj ax12_popcorn_ramp_r = { .id = AX12_ID_POPCORN_RAMP_R, .zero_offset_pos = 0 };

/* set popcorn_ramps position depends on mode */
int8_t popcorn_ramps_set_mode(popcorn_ramps_t *popcorn_ramps, uint8_t mode, int16_t pos_offset)
{
	/* set ax12 positions depends on mode and type */
	if(mode >= POPCORN_RAMPS_MODE_MAX) {
		ACTUATORS_ERROR("Unknown POPCORN_RAMPS MODE");
		return -1;
	}

   /* ax12 positions */
	popcorn_ramps->mode = mode;
	popcorn_ramps->ax12_pos_l = popcorn_ramp_l_ax12_pos[popcorn_ramps->mode] + pos_offset;
	popcorn_ramps->ax12_pos_r = popcorn_ramp_r_ax12_pos[popcorn_ramps->mode] - pos_offset;

	/* saturate to position range */
	if(popcorn_ramps->ax12_pos_l > popcorn_ramp_l_ax12_pos[POPCORN_RAMPS_MODE_L_POS_MAX])
		popcorn_ramps->ax12_pos_l = popcorn_ramp_l_ax12_pos[POPCORN_RAMPS_MODE_L_POS_MAX];
	else if(popcorn_ramps->ax12_pos_l < popcorn_ramp_l_ax12_pos[POPCORN_RAMPS_MODE_L_POS_MIN])
		popcorn_ramps->ax12_pos_l = popcorn_ramp_l_ax12_pos[POPCORN_RAMPS_MODE_L_POS_MIN];

	if(popcorn_ramps->ax12_pos_r > popcorn_ramp_r_ax12_pos[POPCORN_RAMPS_MODE_R_POS_MAX])
		popcorn_ramps->ax12_pos_r = popcorn_ramp_r_ax12_pos[POPCORN_RAMPS_MODE_R_POS_MAX];
	else if(popcorn_ramps->ax12_pos_r < popcorn_ramp_r_ax12_pos[POPCORN_RAMPS_MODE_R_POS_MIN])
		popcorn_ramps->ax12_pos_r = popcorn_ramp_r_ax12_pos[POPCORN_RAMPS_MODE_R_POS_MIN];
 
	/* apply to ax12 */
    ax12_set_pos (&ax12_popcorn_ramp_l, popcorn_ramps->ax12_pos_l);
    ax12_set_pos (&ax12_popcorn_ramp_r, popcorn_ramps->ax12_pos_r);

	return 0;
}

/* return popcorn_ramps traj flag */
uint8_t popcorn_ramps_test_traj_end(popcorn_ramps_t *popcorn_ramps)
{
    uint8_t ret_l, ret_r;
   
    ret_l = ax12_test_traj_end (&ax12_popcorn_ramp_l, END_NEAR|END_TRAJ);
    ret_r = ax12_test_traj_end (&ax12_popcorn_ramp_r, END_NEAR|END_TRAJ);

    return (ret_l | ret_r);
}

/* return END_TRAJ or END_TIMER */
uint8_t popcorn_ramps_wait_end(popcorn_ramps_t *popcorn_ramps)
{
    uint8_t ret_l, ret_r;
   
    ret_l = ax12_wait_traj_end (&ax12_popcorn_ramp_l, END_NEAR|END_TRAJ);
    ret_r = ax12_wait_traj_end (&ax12_popcorn_ramp_r, END_NEAR|END_TRAJ);

    return (ret_l | ret_r);
}



/**** cup_clamp_front functions *********************************************************/
uint16_t cup_clamp_front_ax12_pos [CUP_CLAMP_FRONT_MODE_MAX] = {
	[CUP_CLAMP_FRONT_MODE_OPEN] 		= POS_CUP_CLAMP_FRONT_OPEN, 
	[CUP_CLAMP_FRONT_MODE_CUP_LOCKED] 	= POS_CUP_CLAMP_FRONT_CUP_LOCKED 
};

struct ax12_traj ax12_cup_clamp_front = { .id = AX12_ID_CUP_CLAMP_FRONT, .zero_offset_pos = 0 };

/* set cup_clamp_front position depends on mode */
int8_t cup_clamp_front_set_mode(cup_clamp_front_t *cup_clamp_front, uint8_t mode, int16_t pos_offset)
{
	/* set ax12 positions depends on mode */
	if(mode >= CUP_CLAMP_FRONT_MODE_MAX) {
		ACTUATORS_ERROR("Unknown CUP_CLAMP_FRONT MODE");
		return -1;
	}

	/* ax12 positions */
	cup_clamp_front->mode = mode;
	cup_clamp_front->ax12_pos = cup_clamp_front_ax12_pos[cup_clamp_front->mode] + pos_offset;

	/* saturate to position range */
	if(cup_clamp_front->ax12_pos > cup_clamp_front_ax12_pos[CUP_CLAMP_FRONT_MODE_POS_MAX])
		cup_clamp_front->ax12_pos = cup_clamp_front_ax12_pos[CUP_CLAMP_FRONT_MODE_POS_MAX];
	else if(cup_clamp_front->ax12_pos < cup_clamp_front_ax12_pos[CUP_CLAMP_FRONT_MODE_POS_MIN])
		cup_clamp_front->ax12_pos = cup_clamp_front_ax12_pos[CUP_CLAMP_FRONT_MODE_POS_MIN];

	/* apply to ax12 */
    ax12_set_pos(&ax12_cup_clamp_front, cup_clamp_front->ax12_pos);

	return 0;
}

/* return cup_clamp_front traj flag */
uint8_t cup_clamp_front_test_traj_end(cup_clamp_front_t *cup_clamp_front)
{
    uint8_t ret;
   
    ret = ax12_test_traj_end (&ax12_cup_clamp_front, END_NEAR|END_TRAJ);

    return ret;
}

/* return END_TRAJ or END_TIMER */
uint8_t cup_clamp_front_wait_end(cup_clamp_front_t *cup_clamp_front)
{
    uint8_t ret;
   
    ret = ax12_wait_traj_end (&ax12_cup_clamp_front, END_NEAR|END_TRAJ);

    return ret;
}



/**** cup_holder_front functions *********************************************************/
uint16_t cup_holder_front_ax12_pos [CUP_HOLDER_FRONT_MODE_MAX] = {
	[CUP_HOLDER_FRONT_MODE_CUP_HOLD] 	= POS_CUP_HOLDER_FRONT_CUP_HOLD,
	[CUP_HOLDER_FRONT_MODE_READY] 		= POS_CUP_HOLDER_FRONT_READY,
	[CUP_HOLDER_FRONT_MODE_HIDE] 		= POS_CUP_HOLDER_FRONT_HIDE
};

struct ax12_traj ax12_cup_holder_front = { .id = AX12_ID_CUP_HOLDER_FRONT, .zero_offset_pos = 0 };

/* set cup_holder_front position depends on mode */
int8_t cup_holder_front_set_mode(cup_holder_front_t *cup_holder_front, uint8_t mode, int16_t pos_offset)
{
	/* set ax12 positions depends on mode */
	if(mode >= CUP_HOLDER_FRONT_MODE_MAX) {
		ACTUATORS_ERROR("Unknown CUP_HOLDER_FRONT MODE");
		return -1;
	}

	/* ax12 positions */
	cup_holder_front->mode = mode;
	cup_holder_front->ax12_pos = cup_holder_front_ax12_pos[cup_holder_front->mode] + pos_offset;

	/* saturate to position range */
	if(cup_holder_front->ax12_pos > cup_holder_front_ax12_pos[CUP_HOLDER_FRONT_MODE_POS_MAX])
		cup_holder_front->ax12_pos = cup_holder_front_ax12_pos[CUP_HOLDER_FRONT_MODE_POS_MAX];
	else if(cup_holder_front->ax12_pos < cup_holder_front_ax12_pos[CUP_HOLDER_FRONT_MODE_POS_MIN])
		cup_holder_front->ax12_pos = cup_holder_front_ax12_pos[CUP_HOLDER_FRONT_MODE_POS_MIN];

	/* apply to ax12 */
    ax12_set_pos(&ax12_cup_holder_front, cup_holder_front->ax12_pos);

	return 0;
}

/* return cup_holder_front traj flag */
uint8_t cup_holder_front_test_traj_end(cup_holder_front_t *cup_holder_front)
{
    uint8_t ret;
   
    ret = ax12_test_traj_end (&ax12_cup_holder_front, END_NEAR|END_TRAJ);

    return ret;
}

/* return END_TRAJ or END_TIMER */
uint8_t cup_holder_front_wait_end(cup_holder_front_t *cup_holder_front)
{
    uint8_t ret;
   
    ret = ax12_wait_traj_end (&ax12_cup_holder_front, END_NEAR|END_TRAJ);

    return ret;
}



/* init all actuators */
void actuator_init(void)
{
#define PROGRAM_AX12
#ifdef PROGRAM_AX12
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_SHUTDOWN, 0x24);
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_LED, 0x24);

	/* specific config for mirror ax12, angle is limited */ 
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0xFF);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CW_ANGLE_LIMIT_L, 0x00);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CCW_ANGLE_LIMIT_L, 0x3FF);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_MOVING_SPEED_L, 0x3FF);
#endif

	slavedspic.stands_clamp_l.type = STANDS_CLAMP_TYPE_LEFT;
	slavedspic.stands_clamp_r.type = STANDS_CLAMP_TYPE_RIGHT;

	slavedspic.stands_elevator_l.type = STANDS_ELEVATOR_TYPE_LEFT;
	slavedspic.stands_elevator_r.type = STANDS_ELEVATOR_TYPE_RIGHT;

	slavedspic.stands_blade_l.type = STANDS_BLADE_TYPE_LEFT;
	slavedspic.stands_blade_r.type = STANDS_BLADE_TYPE_RIGHT;

	slavedspic.cup_clamp_popcorn_door_l.type = STANDS_BLADE_TYPE_LEFT;
	slavedspic.cup_clamp_popcorn_door_r.type = STANDS_BLADE_TYPE_RIGHT;
}

