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
