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

#include "i2c_protocol.h"
#include "actuator.h"
#include "main.h"
#include "robotsim.h"

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



