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

/* belt systems managemement */
#define BELTS_SIDE_REAR		0
#define BELTS_SIDE_FRONT	1

#define BELTS_MODE_IN		0
#define BELTS_MODE_OUT		1
#define BELTS_MODE_LEFT		2
#define BELTS_MODE_RIGHT	3

/* belts */
void belts_mode_set(uint8_t side, uint8_t mode, uint16_t speed);
uint16_t belts_load_get(uint8_t side);

/* mirrors */
void mirror_pos_set(uint8_t side, uint16_t pos);
#endif

/* init actuators */
void actuator_init(void);

#endif

