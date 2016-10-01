/*
 *  Copyright Droids Corporation (2007)
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
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  i2c_commands.h,v 1.9 2009/05/27 20:04:06 zer0 Exp.
 */

#ifndef _I2C_COMMANDS_H_
#define _I2C_COMMANDS_H_

#define I2C_SLAVEDSPIC_ADDR 	0x10
#define I2C_GPIOS_01_ADDR 	  	0x20
#define I2C_GPIOS_23_ADDR 	  	0x21

#define I2C_COLOR_GREEN  	0
#define I2C_COLOR_YELLOW	1
#define I2C_COLOR_MAX		2

#define I2C_OPPONENT_NOT_THERE (-1000)

#define I2C_SIDE_LEFT   0
#define I2C_SIDE_RIGHT  1
#define I2C_SIDE_ALL    2

/* XXX keep synchronized with bt_commands.h */
#define BT_SIDE_FRONT 0
#define BT_SIDE_REAR  1
#define BT_SIDE_ALL   2



struct i2c_cmd_hdr {
	uint8_t cmd;
};


/* shared between robots */
#define WAIT_FOR_MAIN_ROBOT_TIMEOUT 60

/****/
/* request status of boards and gpios (read data) */
/****/

#define I2C_REQ_GPIOS_STATUS 	0x00
#define I2C_ANS_GPIOS_STATUS 	0x00
struct i2c_gpios_status {
	int8_t gpio0;	
	int8_t gpio1;	
};

#define I2C_REQ_SLAVEDSPIC_STATUS 0x01
#define I2C_ANS_SLAVEDSPIC_STATUS 0x01
struct i2c_slavedspic_status{
	struct i2c_cmd_hdr hdr;

	/* infos */
	uint8_t status;
/* implicit priority don't changed */
#define I2C_SLAVEDSPIC_STATUS_BLOCKED	1
#define I2C_SLAVEDSPIC_STATUS_ERROR		2
#define I2C_SLAVEDSPIC_STATUS_READY		4
#define I2C_SLAVEDSPIC_STATUS_BUSY		8
#define I2C_SLAVEDSPIC_STATUS_WAITING	16
#define I2C_SLAVEDSPIC_STATUS_STORING	32
#define I2C_SLAVEDSPIC_STATUS_DONE		64
#define I2C_SLAVEDSPIC_STATUS_RESERVED  128

    
	/* systems */
	struct {
		uint8_t mode;
		uint8_t status;
		uint8_t cup_front_catched;
		uint8_t cup_rear_catched;
		uint8_t machine_popcorns_catched;
	}popcorn_system;

	struct {
		uint8_t mode;
		uint8_t status;
		uint8_t stored_stands;
	}stands_system[I2C_SIDE_ALL];
};


/****/
/* commands to boards (write data) */
/****/

#define I2C_CMD_GENERIC			0x00
#define I2C_CMD_LED_CONTROL		0x01
struct i2c_cmd_led_control{
	struct i2c_cmd_hdr hdr;
	uint8_t led_num:7;
	uint8_t state:1;	
};

#define I2C_CMD_SLAVEDSPIC_SET_MODE 0x02
struct i2c_cmd_slavedspic_set_mode {
	struct i2c_cmd_hdr hdr;
	
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_SLAVEDSPIC_MODE_INIT_LEFT					0x01
#define I2C_SLAVEDSPIC_MODE_INIT_RIGHT					0x10
#define I2C_SLAVEDSPIC_MODE_POWER_OFF					0x02

/* simple actuator modes */
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_SLAVEDSPIC_MODE_STANDS_BLADE				0x03
#define I2C_SLAVEDSPIC_MODE_STANDS_CLAMP				0x04
#define I2C_SLAVEDSPIC_MODE_STANDS_ELEVATOR				0x05
#define I2C_SLAVEDSPIC_MODE_STANDS_TOWER_CLAMPS			0x06
#define I2C_SLAVEDSPIC_MODE_CUP_CLAMP_POPCORN_DOOR		0x07
#define I2C_SLAVEDSPIC_MODE_POPCORN_TRAY				0x08
#define I2C_SLAVEDSPIC_MODE_POPCORN_RAMPS				0x09
#define I2C_SLAVEDSPIC_MODE_CUP_CLAMP_FRONT				0x0A
#define I2C_SLAVEDSPIC_MODE_CUP_HOLDER_FRONT			0x0B

/* multiple actuator modes */
#define I2C_SLAVEDSPIC_MODE_POPCORN_SYSTEM   			0x0C
#define I2C_SLAVEDSPIC_MODE_STANDS_SYSTEM				0x0D

/* set infos */
#define I2C_SLAVEDSPIC_MODE_SET_INFOS					0x0E


	uint8_t mode;
	union{

		struct {
			uint8_t type;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_STANDS_BLADE_TYPE_LEFT		0
#define I2C_STANDS_BLADE_TYPE_RIGHT		1	

			uint8_t mode;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_STANDS_BLADE_MODE_HIDE_LEFT				0
#define I2C_STANDS_BLADE_MODE_PUSH_STAND_LEFT		1
#define I2C_STANDS_BLADE_MODE_CENTER				2
#define I2C_STANDS_BLADE_MODE_PUSH_STAND_RIGHT		3
#define I2C_STANDS_BLADE_MODE_HIDE_RIGHT			4
#define I2C_STANDS_BLADE_MODE_SET_ANGLE				5

			int8_t offset;
		} stands_blade;

		struct {
			uint8_t type;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_STANDS_CLAMP_TYPE_LEFT		0
#define I2C_STANDS_CLAMP_TYPE_RIGHT		1	

			uint8_t mode;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_STANDS_CLAMP_MODE_FULL_OPEN		0
#define I2C_STANDS_CLAMP_MODE_OPEN			1
#define I2C_STANDS_CLAMP_MODE_CLOSE			2

			int8_t offset;
		} stands_clamp;

		struct {
			uint8_t type;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_STANDS_ELEVATOR_TYPE_LEFT		0
#define I2C_STANDS_ELEVATOR_TYPE_RIGHT		1	

			uint8_t mode;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_STANDS_ELEVATOR_MODE_UP		0
#define I2C_STANDS_ELEVATOR_MODE_DOWN	1

			int8_t offset;
		} stands_elevator;

		struct {
			uint8_t mode;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_STANDS_TOWER_CLAMPS_MODE_UNLOCK_LEFT 	0	
#define I2C_STANDS_TOWER_CLAMPS_MODE_LOCK 			1		
#define I2C_STANDS_TOWER_CLAMPS_MODE_UNLOCK_RIGHT	2	

			int8_t offset;
		} stands_tower_clamps;

		struct {
			uint8_t type;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT		0
#define I2C_CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT		1	

			uint8_t mode;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_CUP_CLAMP_MODE_HIDE			0
#define I2C_CUP_CLAMP_MODE_LOCKED		1
#define I2C_CUP_CLAMP_MODE_OPEN			2
#define I2C_POPCORN_DOOR_MODE_CLOSE		2
#define I2C_POPCORN_DOOR_MODE_OPEN		3

			int8_t offset;
		} cup_clamp_popcorn_door;

		struct {
			uint8_t mode;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_POPCORN_TRAY_MODE_OPEN		0
#define I2C_POPCORN_TRAY_MODE_CLOSE		1

			int8_t offset;
		} popcorn_tray;

		struct {
			uint8_t mode;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_POPCORN_RAMPS_MODE_HIDE		0
#define I2C_POPCORN_RAMPS_MODE_HARVEST	1
#define I2C_POPCORN_RAMPS_MODE_OPEN		2

			int8_t offset;
		} popcorn_ramps;

		struct {
			uint8_t mode;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_CUP_CLAMP_FRONT_MODE_OPEN			0
#define I2C_CUP_CLAMP_FRONT_MODE_CUP_LOCKED		1

			int8_t offset;
		} cup_clamp_front;

		struct {
			uint8_t mode;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_CUP_HOLDER_FRONT_MODE_CUP_HOLD		0
#define I2C_CUP_HOLDER_FRONT_MODE_READY			1
#define I2C_CUP_HOLDER_FRONT_MODE_HIDE			2

			int8_t offset;
		} cup_holder_front;

		struct {
			uint8_t mode;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_SLAVEDSPIC_MODE_PS_IDLE							0

#define I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_READY				10
#define I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_CATCH_AND_DROP		11
#define I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_RELEASE			12
#define I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_HIDE				13

#define I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_OPEN				20
#define I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_CATCH				21
#define I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_RELEASE				22

#define I2C_SLAVEDSPIC_MODE_PS_MACHINES_READY				30
#define I2C_SLAVEDSPIC_MODE_PS_MACHINES_HARVEST				31
#define I2C_SLAVEDSPIC_MODE_PS_MACHINES_END					32

#define I2C_SLAVEDSPIC_MODE_PS_STOCK_DROP					40
#define I2C_SLAVEDSPIC_MODE_PS_STOCK_END					41

		} popcorn_system;

		struct {
			uint8_t side;

			uint8_t mode;
/* XXX syncronized with slavedispic/actuators.h */
#define I2C_SLAVEDSPIC_MODE_SS_IDLE					0

#define I2C_SLAVEDSPIC_MODE_SS_HIDE_TOWER			10
#define I2C_SLAVEDSPIC_MODE_SS_HARVEST_STAND_DO		11
#define I2C_SLAVEDSPIC_MODE_SS_HARVEST_STAND_READY	12
#define I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT		13
#define I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT	14

			int8_t blade_angle;

		} stands_system;

		struct {
			int8_t cup_front_catched;
			int8_t cup_rear_catched;
			int8_t machine_popcorns_catched;
			int8_t stored_stands_l;
			int8_t stored_stands_r;

		} set_infos;

		/* add more here */
	};
};

#endif
