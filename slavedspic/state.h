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

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Balias Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  state.h,v 1.4 2009/05/27 20:04:07 zer0 Exp.
 */

#ifndef _STATE_H_
#define _STATE_H_

#include "../common/i2c_commands.h"
#include "actuator.h"

#define SIMPLE_ACTUATORS
#define ACTUATOR_SYSTEMS
//#define INFOS


/* popcorn_system */
typedef struct {

	/* working mode */
	uint8_t mode;
	uint8_t mode_rqst;
	uint8_t mode_changed;

	/* substate of modes, XXX variable shared between them */
	uint8_t substate;
#define SAVE							0

#define CATCH_CUP_FRONT					1
#define WAITING_CUP_FRONT_CAUGHT		2
#define RAISE_CUP_FRONT					3
#define WAITING_CUP_FRONT_RAISED		4
#define WAITING_POPCORNS_DROPPED		5

#define DOWN_CUP_FRONT					11
#define WAITING_CUP_FRONT_DOWNED		12
#define RELEASE_CUP_FRONT				13
#define WAITING_CUP_FRONT_RELEASED		14

#define HIDE_CUP_FRONT					18
#define WAITING_CUP_FRONT_HIDDEN		19

#define OPEN_RIGHT_CUP_REAR				21
#define WAITING_CUP_REAR_OPENED_RIGHT	22
#define OPEN_LEFT_CUP_REAR				23
#define WAITING_CUP_REAR_OPENED_LEFT	24

#define CATCH_CUP_REAR					31
#define WAITING_CUP_REAR_CAUGHT			32
#define RELEASE_CUP_REAR				33
#define WAITING_CUP_REAR_RELEASED		34

#define OPEN_TRAY						41
#define WAITING_TRAY_OPENED				42
#define OPEN_RAMPS						43
#define WAITING_RAMPS_OPENED			44

#define MOVE_RAMPS						51
#define WAITING_RAMPS_MOVED				52
#define WAITING_POPCORNS_HARVESTED		53

#define CLOSE_RAMPS						61
#define WAITING_RAMPS_CLOSED			62
#define CLOSE_TRAY						63
#define WAITING_TRAY_CLOSED				64

#define OPEN_DOORS						71
#define WAITING_DOORS_OPENED			72
#define WAITING_STOCK_DROPPED			73

#define CLOSE_LEFT_CLAMP				81
#define WAITING_LEFT_CLAMP_CLOSED		82
#define CLOSE_RIGHT_CLAMP				83
#define WAITING_RIGHT_CLAMP_CLOSED		84

	/* system status */
	uint8_t status;
#define STATUS_READY		I2C_SLAVEDSPIC_STATUS_READY
#define STATUS_BUSY			I2C_SLAVEDSPIC_STATUS_BUSY
#define STATUS_DONE			I2C_SLAVEDSPIC_DONE	
#define STATUS_BLOCKED		I2C_SLAVEDSPIC_BLOCKED
#define STATUS_ERROR		I2C_SLAVEDSPIC_ERROR

	/* infos */
	uint8_t cup_front_catched;
	uint8_t cup_front_popcorns_harvested;
	uint8_t cup_rear_catched;
	uint8_t machine_popcorns_catched;
}popcorn_system_t;

/* stands_system */
typedef struct {

	/* working mode */
	uint8_t mode;
	uint8_t mode_rqst;
	uint8_t mode_changed;

	/* substate of each working mode, XXX variable shared between them */
	uint8_t substate;
#define SAVE						0

#define CLOSE_CLAMPS				1
#define WAITING_CLAMPS_CLOSED		2
#define LIFT_ELEVATOR				3
#define WAITING_ELEVATOR_LIFTED		4
#define HIDE_BLADE					5
#define WAITING_BLADE_HIDDEN		6

#define READY_BLADE					11
#define WAITING_BLADE_READY			12
#define PUSH_STAND					13
#define WAITING_STAND_PUSHED		14
#define DESCEND_TOWER				15
#define WAITING_TOWER_DESCENDED		16
#define OPEN_CLAMP					17
#define WAITING_CLAMP_OPENED		18
#define DESCEND_ELEVATOR			19
#define WAITING_ELEVATOR_DESCENDED	20

#define GO_CENTER					21
#define WAITING_CENTERED			22
#define RETURN_HOME					23
#define WAITING_RETURNED			24

#define OPEN_ALL					31
#define WAITING_ALL_OPENED			32

	/* status */
	uint8_t status;

	/* XXX pecific for spotlight building */
	uint8_t spotlight_substate;
#define IDLE				0
#define HIDE_TOWER			1
#define HARVEST_STAND		2
#define RELEASE_STAND		3
#define CENTER_STAND		4

	uint8_t spotlight_status;
	uint8_t spotlight_mode;
#define SM_PRINCIPAL		1
#define SM_SECONDARY		2

	/* blade angle consign */
	int8_t blade_angle;

	/* time counters */
	microseconds us;
	microseconds us_system;

	/* infos */
	uint8_t stored_stands;

	/* sensors */
	uint8_t stand_sensor;

	/* actuators */
	stands_blade_t *blade;
	stands_clamp_t *clamp;
	stands_elevator_t *elevator;

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
