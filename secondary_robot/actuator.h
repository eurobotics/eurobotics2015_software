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

/* actuators 2012 */
#define END_TRAJ		1
#define END_BLOCKING	2

#define ARM_POS_HIDE			792
#define ARM_POS_HOLD_MAP	680
#define ARM_POS_PICKUP_MAP	530

#define TEETH_POS_OPEN	737
#define TEETH_POS_CLOSE 515

void actuators_init(void);

inline uint8_t arm_set_pos(uint16_t pos);
inline uint8_t arm_wait_end(void);
inline uint8_t arm_disable_torque(void);
inline uint8_t arm_enable_torque(void);

inline uint8_t teeth_set_pos(uint16_t pos);
inline uint8_t teeth_wait_end(void);

#endif

