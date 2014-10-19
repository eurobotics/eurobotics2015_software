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
#define ADC_LASER_1   0
#define ADC_LASER_2   1
#define ADC_MAX       2

/* synchronize with sensor.c */
#define S_SENSOR_1     0
#define S_SENSOR_2     1
#define S_SENSOR_3     2
#define S_RESERVED4    3
#define S_RESERVED5    4
#define S_RESERVED6    5
#define S_RESERVED7    6
#define S_RESERVED8    7

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

#define SENSOR_MAX    24


/* sensors map */
#define S_START			S_GP0_4
#define S_COLOR			S_SENSOR_3
#define S_TURN				S_SENSOR_2
#define S_BEACON			S_SENSOR_1
#define S_OBS_FRONT_L	S_GP0_2
#define S_OBS_FRONT_R	S_GP0_3
#define S_OBS_REAR_L		S_GP0_0
#define S_OBS_REAR_R		S_GP0_1

#ifdef notyet
/* lasers map */
#define ADC_LASER_R		ADC_LASER_1
#define ADC_LASER_L		ADC_LASER_2
#define ADC_LASER_MAX	2
#endif


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


