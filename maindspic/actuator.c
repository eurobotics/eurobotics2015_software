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
#include <dac_mc.h>

#include "i2c_protocol.h"
#include "actuator.h"
#include "main.h"
#include "robotsim.h"

void dac_set_and_save(void *dac, int32_t val)
{
#define RIGHT_MOTOR_OFFSET	0 
#define LEFT_MOTOR_OFFSET   3500

#define RIGHT_MOTOR_MAX		(65535-LEFT_MOTOR_OFFSET)
#define LEFT_MOTOR_MAX		(65535-RIGHT_MOTOR_OFFSET)
	
	if (dac == LEFT_MOTOR) {
		/* apply offset */
		val = val > 0? (val + LEFT_MOTOR_OFFSET):(val - LEFT_MOTOR_OFFSET);

		/* we need to do the saturation here, before saving the value */
		if (val > LEFT_MOTOR_MAX)
			val = LEFT_MOTOR_MAX;
		if (val < -LEFT_MOTOR_MAX)
			val = -LEFT_MOTOR_MAX;

		/* save value */
		mainboard.dac_l = val;
	}
	else if (dac == RIGHT_MOTOR){
		/* apply offset */
		val = val > 0? (val + RIGHT_MOTOR_OFFSET):(val - RIGHT_MOTOR_OFFSET);

		/* we need to do the saturation here, before saving the value */
		if (val > RIGHT_MOTOR_MAX)
			val = RIGHT_MOTOR_MAX;
		if (val < -RIGHT_MOTOR_MAX)
			val = -RIGHT_MOTOR_MAX;

		/* save value */
		mainboard.dac_r = val;
	}

	/* set value */
#ifdef HOST_VERSION
	robotsim_pwm(dac, val);
#else
	dac_mc_set(dac, val);
#endif
}



