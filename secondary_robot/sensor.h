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
 *  Revision : $Id: sensor.h,v 1.5 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  sensor.h,v 1.5 2009/05/27 20:04:07 zer0 Exp.
 */

/* synchronize with sensor.c */
#define S_START_SWITCH 0
#define S_COLOR_SWITCH 1
#define S_OPP_FRONT_L  2
#define S_OPP_FRONT_R  3
#define S_OPP_REAR  	  4

#define S_GP0_0		 8
#define S_GP0_1		 9
#define S_GP0_2		 10
#define S_GP0_3		 11
#define S_GP0_4		 12
#define S_GP0_5		 13
#define S_GP0_6		 14
#define S_GP0_7		 15

#define S_GP1_0		 16
#define S_GP1_1		 17
#define S_GP1_2		 18
#define S_GP1_3		 19
#define S_GP1_4		 20
#define S_GP1_5		 21
#define S_GP1_6		 22
#define S_GP1_7		 23

#define S_GP2_0		 24
#define S_GP2_1		 25
#define S_GP2_2		 26
#define S_GP2_3		 27
#define S_GP2_4		 28
#define S_GP2_5		 29
#define S_GP2_6		 30
#define S_GP2_7		 31

#define S_GP3_0		 32
#define S_GP3_1		 33
#define S_GP3_2		 34
#define S_GP3_3		 35
#define S_GP3_4		 36
#define S_GP3_5		 37
#define S_GP3_6		 38
#define S_GP3_7		 39
#define SENSOR_MAX    5

/* digital and analog */
void sensor_init(void);

/* get filtered values for adc */
int16_t sensor_get_adc(uint8_t i);

/* get filtered values of boolean sensors */
uint64_t sensor_get_all(void);
uint8_t sensor_get(uint8_t i);

/* manage obtacles with sensors */
/* XXX take care with disable obstacle detection */
void sensor_obstacle_disable(void);
void sensor_obstacle_enable(void);
uint8_t sensor_obstacle_is_disabled(void);


