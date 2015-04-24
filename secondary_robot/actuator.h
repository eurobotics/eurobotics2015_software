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

#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include <stdint.h>

/* used by cs, correct offset and save values */
void dac_set_and_save(void *dac, int32_t val);

/* beacon speed calculation based on encoder position,
 * used by cs as feedback. Must be compatible format with cs */
int32_t encoders_update_beacon_speed(void * number);

/* read actual beacon speed */
int32_t encoders_get_beacon_speed(void * dummy);


/* set arm mode */

/* arm side */
#define ARM_TYPE_LEFT	0
#define ARM_TYPE_RIGHT	1
#define ARM_TYPE_MAX	2

/* arm modes */
#define ARM_MODE_HIDE		0
#define ARM_MODE_CARPET		1
#define ARM_MODE_CLAPPER	2
#define ARM_MODE_MAX		3

/* left arm positions */
#define ARM_LEFT_POS_HIDE		530
#define ARM_LEFT_POS_CARPET		820
#define ARM_LEFT_POS_CLAPPER	820

/* right arm positions */
#define ARM_RIGHT_POS_HIDE		700 //730
#define ARM_RIGHT_POS_CARPET	400
#define ARM_RIGHT_POS_CLAPPER	400

void arm_set_mode (uint8_t type, uint8_t mode);

/* set cup clamp position */
#define CUP_CLAMP_POS_OPEN		700 //650
#define CUP_CLAMP_POS_CLOSE		350

void cup_clamp_set_position (uint16_t pos);

/* TODO: set auxiliary mode */
#define AUX_WHEELS_POS_HIDDEN		500
#define AUX_WHEELS_POS_ROLLER		500
#define AUX_WHEELS_POS_CLIMB_STAIRS	500

void aux_wheels_set_position (uint16_t pos);


#endif

