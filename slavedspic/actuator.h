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

#define POS_FINGER_TOTEM_R_OPEN	738
#define POS_FINGER_TOTEM_R_HOLD	492
#define POS_FINGER_TOTEM_R_CLOSE	384

#define POS_FINGER_TOTEM_L_OPEN	463
#define POS_FINGER_TOTEM_L_HOLD	704
#define POS_FINGER_TOTEM_L_CLOSE	809

#define POS_FINGER_FLOOR_R_OPEN	807
#define POS_FINGER_FLOOR_R_HOLD	387
#define POS_FINGER_FLOOR_R_CLOSE	269

#define POS_FINGER_FLOOR_L_OPEN 	254
#define POS_FINGER_FLOOR_L_HOLD	700
#define POS_FINGER_FLOOR_L_CLOSE	805

#define POS_BOOT_OPEN			639
#define POS_BOOT_HOLD			759
#define POS_BOOT_CLOSE			814

#define POS_ARM_R_HIDE			533
#define POS_ARM_R_BELOW_HOLD
#define POS_ARM_R_PUSH_FLOOR

#define POS_ARM_L_HIDE			238
#define POS_ARM_L_BELOW_HOLD
#define POS_ARM_L_PUSH_FLOOR

#define POS_HOOK_HIDE
#define POS_HOOK_DOWN
#define POS_HOOK_OPEN_HOLD




#ifdef notyet

/* turbine structure */
struct {
	uint8_t power;
#define TURBINE_POWER_ON 	0
#define TURBINE_POWER_OFF 	1

	uint16_t angle_deg;
	uint16_t angle_pos;
#define TURBINE_POS_ANGLE_ZERO 	0
#define TURBINE_K_POS_DEG		 	1

	uint16_t blow_speed;

} turbine_t;

/* init actuators */
void actuator_init(void);

/**** lift functions */
void lift_hard_stop(struct cs_block *lift);
void lift_autopos(struct cs_block_t *lift);
void lift_set_height(struct cs_block_t *lift,  int16_t height_mm);
int16_t lift_get_height(struct cs_block_t *lift);

#define END_LIFT			1
#define END_BLOCKING		2
uint8_t lift_wait_heigh_reached(struct cs_block *lift);

/**** turbine funcions */
void turbine_power_on(turbine_t *turbine);
void turbine_power_off(void);
void turbine_set_angle(turbine_t *turbine, uint16_t angle_deg, uint16_t wait_ms);
void turbine_set_speed(turbine_t *turbine, uint16_t speed);

uint16_t turgine_get_angle(turbine_t *turbine);
uint16_t turbine_get_speed(turbine_t *turbine);

#endif

#endif
