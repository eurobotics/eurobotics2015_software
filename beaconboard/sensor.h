/*  
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
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
 *  Revision : $Id: sensor.h,v 1.1 2009/03/29 18:44:54 zer0 Exp $
 *
 */

/*   *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011) *  Javier Baliñas Santos <javier@arc-robots.org> * *  Code ported to family of microcontrollers dsPIC from *  sensor.h,v 1.1 2009/03/29 18:44:54 zer0 Exp. */

/* synchronize with sensor.c */
#define S_CAP1         0
#define S_CAP2         1
#define S_CAP3         2
#define S_CAP4         3
#define S_CAP5         4
#define SENSOR_MAX     5

/* init sensors */
void sensor_init(void);

/* get filtered values of boolean sensors */
uint16_t sensor_get_all(void);
uint8_t sensor_get(uint8_t i);
