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
 *  Revision : $Id: sensor.c,v 1.3 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*   *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011) *  Javier Baliñas Santos <javier@arc-robots.org> * *  Code ported to family of microcontrollers dsPIC from *  sensor.c,v 1.3 2009/05/27 20:04:07 zer0 Exp. */

#include <stdlib.h>

#include <aversive.h>
#include <aversive/error.h>

#include <scheduler.h>
#include <pwm_mc.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "sensor.h"


/***********************************************************
 * boolean sensors 
 ***********************************************************/

/* value of filtered sensors */
static uint16_t sensor_filtered = 0;

/* sensor filter data structure */
struct sensor_filter {
	uint8_t filter;
	uint8_t prev;
	uint8_t thres_off;
	uint8_t thres_on;
	uint8_t cpt;
	uint8_t invert;
};

/* setup of sensor adquisition */
static struct sensor_filter sensor_filter[SENSOR_MAX] = {
	[S_CAP1] = { 1, 0, 0, 1, 0, 1 }, /* 0 */
	[S_CAP2] = { 1, 0, 0, 1, 0, 1 }, /* 1 */
	[S_CAP3] = { 1, 0, 0, 1, 0, 1 }, /* 2 */
	[S_CAP4] = { 1, 0, 0, 1, 0, 1 }, /* 3 */
	[S_CAP5] = { 1, 0, 0, 1, 0, 1 }, /* 4 */
};


/* sensor mapping : 
 * 0  :  PORTB 9 (cap1: BUMPER)
 * 1-2:  PORTC 4->5 (cap2 -> cap3: PZ2_1 -> PZ2_2)
 * 3-4:  PORTC 8->9 (cap4 -> cap5: HERR2 -> HERR1)
 * 5-15: reserved
 */

/* get the physical value of pins */
static uint16_t sensor_read(void)
{
	uint16_t tmp = 0;
	tmp |= (uint16_t) ((PORTB & _BV(9)) >> 9) << 0;
	tmp |= (uint16_t) ((PORTC & (_BV(4)|_BV(5))) >> 4) << 1;
	tmp |= (uint16_t) ((PORTC & (_BV(8)|_BV(9))) >> 8) << 3;
	
	/* add reserved sensors here */
	return tmp;
}

/* boolean sensors processing, called periodically, see init below */
static void do_boolean_sensors(void *dummy)
{
	uint8_t i;
	uint8_t flags;
	uint16_t sensor = sensor_read();
	uint16_t tmp = 0;

	for (i=0; i<SENSOR_MAX; i++) {
		if ((1 << i) & sensor) {
			if (sensor_filter[i].cpt < sensor_filter[i].filter)
				sensor_filter[i].cpt++;
			if (sensor_filter[i].cpt >= sensor_filter[i].thres_on)
				sensor_filter[i].prev = 1;
		}
		else {
			if (sensor_filter[i].cpt > 0)
				sensor_filter[i].cpt--;
			if (sensor_filter[i].cpt <= sensor_filter[i].thres_off)
				sensor_filter[i].prev = 0;
		}
		
		if (sensor_filter[i].prev) {
			tmp |= (1UL << i);
		}
	}
	IRQ_LOCK(flags);
	sensor_filtered = tmp;
	IRQ_UNLOCK(flags);
}

/* get filtered value of boolean sensors */
uint16_t sensor_get_all(void)
{
	uint16_t tmp;
	uint8_t flags;
	IRQ_LOCK(flags);
	tmp = sensor_filtered;
	IRQ_UNLOCK(flags);
	return tmp;
}

/* get filtered value of one sensor */
uint8_t sensor_get(uint8_t i)
{
	uint16_t tmp = sensor_get_all();
	return (tmp & _BV(i));
}

/***************************************************************
 * common sensors functions 
 ***************************************************************/

/* all sensors processing, called periodically, see init below */
static void do_sensors(void *dummy)
{
	do_boolean_sensors(NULL);

	/* add other kind of sensors here */
}

/* initilize all sensors */
void sensor_init(void)
{
	/* add inits here */

	/* CS EVENT */
	scheduler_add_periodical_event_priority(do_sensors, NULL, 
						EVENT_PERIOD_SENSOR / SCHEDULER_UNIT, EVENT_PRIO_SENSOR);
}

