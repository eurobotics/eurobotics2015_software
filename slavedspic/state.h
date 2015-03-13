/*  
 *  Copyright Droids Corporation (2009)
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
 *  Revision : $Id: state.h,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*   *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011) *  Javier Baliñas Santos <javier@arc-robots.org> * *  Code ported to family of microcontrollers dsPIC from *  state.h,v 1.4 2009/05/27 20:04:07 zer0 Exp. */

#ifndef _STATE_H_
#define _STATE_H_


/* popcorn_system */
typedef struct {
#define PS_STATE_IDLE							0

#define PS_STATE_CUP_FRONT_CATCH_AND_DROP		10
#define PS_STATE_CUP_FRONT_RELEASE				11

#define PS_STATE_CUP_REAR_CATCH					20
#define PS_STATE_CUP_REAR_RELEASE				21

#define PS_STATE_MACHINES_READY					30
#define PS_STATE_MACHINES_HARVEST				31
#define PS_STATE_MACHINES_END					32

#define PS_STATE_STOCK_DROP						40
#define PS_STATE_STOCK_END						41

	uint8_t state;
	uint8_t state_rqst;
	uint8_t state_changed;

	/* info */
	uint8_t cup_front_catched;
	uint8_t cup_rear_catched;
	uint8_t machine_popcorns_catched;

	/* conf */
	uint8_t sensor_front;
	uint8_t sensor_rear;

}popcorn_system_t;

/* stands_system */
typedef struct {
#define SS_STATE_IDLE					0

#define SS_STATE_HIDE					10
#define SS_STATE_HARVESTING				11
#define SS_STATE_BUILD_SPOTLIGHT		12
#define SS_STATE_RELEASE_SPOTLIGHT		13

	uint8_t state;
	uint8_t state_rqst;
	uint8_t state_changed;

	/* info */
	uint8_t stored_stands;

	/* conf */
	uint8_t stand_sensor;

}stands_system_t;


/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_slavedspic_set_mode *cmd);

/* get mode */
uint8_t state_get_mode(void);

/* run state machines */
void state_machines(void);

/* init state machines */
void state_init(void);

#endif
