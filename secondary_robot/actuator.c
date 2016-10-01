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
 *  Javier Bali?as Santos <javier@arc-robots.org>
 */

#include <aversive.h>
#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <ax12.h>

#include "i2c_protocol.h"
#include "actuator.h"
#include "main.h"
#include "robotsim.h"
#include "beacon.h"
#include "ax12_user.h"

void dac_set_and_save(void *pwm_mc, int32_t val)
{
	/* save value */	
	if (pwm_mc == LEFT_MOTOR)
		mainboard.pwm_l = val;
	else if (pwm_mc == RIGHT_MOTOR)
		mainboard.pwm_r = val;


	/* set value */
#ifdef HOST_VERSION
	robotsim_pwm(pwm_mc, val);
#else
	pwm_mc_set(pwm_mc, val);
#endif
}



/* actual beacon position and speed */
static volatile int32_t beacon_pos;
static volatile int32_t beacon_speed = 0;

/* beacon speed calculation based on encoder position,
 * used by cs as feedback. Must be compatible format with cs */
int32_t encoders_update_beacon_speed(void * dummy)
{
	int32_t ret = 0;
	uint8_t flags;



	/* critical section */
	IRQ_LOCK(flags);

#ifndef HOST_VERSION

	/* get encoder position */
	ret = (int32_t)beacon_encoder_get_value();
#endif
	/* calulate speed */
	beacon_speed = ret - beacon_pos;
	beacon_pos = ret;

	IRQ_UNLOCK(flags);
	
	return beacon_speed;
}

/* read actual beacon speed */
int32_t encoders_get_beacon_speed(void * dummy)
{
	return beacon_speed;
}


/* set arm position */

static uint16_t arm_position[ARM_TYPE_MAX][ARM_MODE_MAX] = {
	[ARM_TYPE_LEFT][ARM_MODE_HIDE] 		= ARM_LEFT_POS_HIDE,
	[ARM_TYPE_LEFT][ARM_MODE_CARPET] 	= ARM_LEFT_POS_CARPET,
	[ARM_TYPE_LEFT][ARM_MODE_CLAPPER] 	= ARM_LEFT_POS_CLAPPER,
	[ARM_TYPE_LEFT][ARM_MODE_CLEAN] 	= ARM_LEFT_POS_CLEAN,

	[ARM_TYPE_RIGHT][ARM_MODE_HIDE] 	= ARM_RIGHT_POS_HIDE,
	[ARM_TYPE_RIGHT][ARM_MODE_CARPET] 	= ARM_RIGHT_POS_CARPET,
	[ARM_TYPE_RIGHT][ARM_MODE_CLAPPER] 	= ARM_RIGHT_POS_CLAPPER,
	[ARM_TYPE_RIGHT][ARM_MODE_CLEAN] 	= ARM_RIGHT_POS_CLEAN,
};

static uint8_t arm_mode[ARM_TYPE_MAX] = { ARM_MODE_HIDE, ARM_MODE_HIDE};

void arm_set_mode (uint8_t type, uint8_t mode)
{
#ifndef HOST_VERSION

	struct pwm_servo *servo;
	
	/* be sure the other arm is hidden, and update this mode */
	if (mode != ARM_MODE_HIDE && arm_mode[!type] != ARM_MODE_HIDE) {
		servo = (!type==ARM_TYPE_LEFT? &gen.pwm_servo_oc3 : &gen.pwm_servo_oc1);
	   	pwm_servo_set(servo, arm_position[!type][ARM_MODE_HIDE]);
		arm_mode[!type] = ARM_MODE_HIDE;
	}

	/* set position and update mode */
	servo = (type==ARM_TYPE_LEFT? &gen.pwm_servo_oc3 : &gen.pwm_servo_oc1);
	pwm_servo_set(servo, arm_position[type][mode]);
	arm_mode[type] = ARM_MODE_HIDE;

#endif
}

/* set cup front clamp position */
void cup_front_clamp_set_position (uint16_t pos)
{
#ifndef HOST_VERSION
	pwm_servo_set(&gen.pwm_servo_oc2, pos);
#endif
}

/* set cup rear clamp position */
void cup_rear_clamp_set_position (uint16_t pos)
{
#ifndef HOST_VERSION
	pwm_servo_set(&gen.pwm_servo_oc4, pos);
#endif
}


/* TODO: set auxiliary wheels */
void aux_wheels_set_position (uint16_t pos)
{
#ifndef HOST_VERSION
	pwm_servo_set(&gen.pwm_servo_oc4, pos);
#endif
}

