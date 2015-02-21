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

#define I2C_SIDE_LEFT   1
#define I2C_SIDE_RIGHT  2
#define I2C_SIDE_ALL    3



struct i2c_cmd_hdr {
	uint8_t cmd;
};


/* shared between robots */
#define WAIT_FOR_MAIN_ROBOT_TIMEOUT 60

/****/
/* request status of boards and gpios (read data) */
/****/

#define I2C_REQ_GPIOS_STATUS 			0x00
#define I2C_ANS_GPIOS_STATUS 			0x00
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
#define I2C_SLAVEDSPIC_STATUS_BUSY		1
#define I2C_SLAVEDSPIC_STATUS_READY		0
    
	uint8_t nb_stored_stands_l, nb_stored_stands_r;
};


/****/
/* commands to boards (write data) */
/****/

#define I2C_CMD_GENERIC				0x00

#define I2C_CMD_LED_CONTROL		0x01
struct i2c_cmd_led_control{
	struct i2c_cmd_hdr hdr;
	uint8_t led_num:7;
	uint8_t state:1;	
};

#define I2C_CMD_SLAVEDSPIC_SET_MODE 0x02
struct i2c_cmd_slavedspic_set_mode {
	struct i2c_cmd_hdr hdr;
	
#define I2C_SLAVEDSPIC_MODE_INIT						0x01
#define I2C_SLAVEDSPIC_MODE_POWER_OFF					0x02

/* simple actuator modes */
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
#define I2C_SLAVEDSPIC_MODE_HARVEST_POPCORNS   			0x0C
#define I2C_SLAVEDSPIC_MODE_DUMP_POPCORNS				0x0D
#define I2C_SLAVEDSPIC_MODE_DUMP_FRONT_CUP				0x0E


	uint8_t mode;
	union{

		struct {
			uint8_t type;
#define I2C_STANDS_BLADE_TYPE_LEFT		0
#define I2C_STANDS_BLADE_TYPE_RIGHT		1	

			uint8_t mode;
#define I2C_STANDS_BLADE_MODE_HIDE_LEFT				0
#define I2C_STANDS_BLADE_MODE_PUSH_STAND_LEFT		1
#define I2C_STANDS_BLADE_MODE_CENTER				2
#define I2C_STANDS_BLADE_MODE_PUSH_STAND_RIGHT		3
#define I2C_STANDS_BLADE_MODE_HIDE_RIGHT			4

			int8_t offset;
		} stands_blade;

		struct {
			uint8_t type;
#define I2C_STANDS_CLAMP_TYPE_LEFT		0
#define I2C_STANDS_CLAMP_TYPE_RIGHT		1	

			uint8_t mode;
#define I2C_STANDS_CLAMP_MODE_OPEN		0
#define I2C_STANDS_CLAMP_MODE_CLOSE		1

			int8_t offset;
		} stands_clamp;

		struct {
			uint8_t type;
#define I2C_STANDS_ELEVATOR_TYPE_LEFT		0
#define I2C_STANDS_ELEVATOR_TYPE_RIGHT		1	

			uint8_t mode;
#define I2C_STANDS_ELEVATOR_MODE_UP		0
#define I2C_STANDS_ELEVATOR_MODE_DOWN	1

			int8_t offset;
		} stands_elevator;

		struct {
			uint8_t mode;
#define I2C_STANDS_TOWER_CLAMPS_MODE_UNLOCK_LEFT 	0	
#define I2C_STANDS_TOWER_CLAMPS_MODE_LOCK 			1		
#define I2C_STANDS_TOWER_CLAMPS_MODE_UNLOCK_RIGHT	2	

			int8_t offset;
		} stands_tower_clamps;

		struct {
			uint8_t mode;
#define I2C_CUP_CLAMP_MODE_HIDE			0
#define I2C_CUP_CLAMP_MODE_LOCKED		1
#define I2C_CUP_CLAMP_MODE_OPEN			2
#define I2C_POPCORN_DOOR_MODE_CLOSE		2
#define I2C_POPCORN_DOOR_MODE_OPEN		3

			int8_t offset;
		} cup_clamp_popcorn_door;

		struct {
			uint8_t mode;
#define I2C_POPCORN_TRAY_MODE_OPEN		0
#define I2C_POPCORN_TRAY_MODE_CLOSE		1

			int8_t offset;
		} popcorn_tray;

		struct {
			uint8_t mode;
#define I2C_POPCORN_RAMPS_MODE_HIDE		0
#define I2C_POPCORN_RAMPS_MODE_HARVEST	1
#define I2C_POPCORN_RAMPS_MODE_OPEN		2

			int8_t offset;
		} popcorn_ramps;

		struct {
			uint8_t mode;
#define I2C_CUP_CLAMP_FRONT_MODE_HIDE			0
#define I2C_CUP_CLAMP_FRONT_MODE_CUP_LOCKED		1

			int8_t offset;
		} cup_clamp_front;

		struct {
			uint8_t mode;
#define I2C_CUP_HOLDER_FRONT_MODE_CUP_HOLD		0
#define I2C_CUP_HOLDER_FRONT_MODE_HIDE			1

			int8_t offset;
		} cup_holder_front;

		struct {
			uint8_t mode;
#define I2C_SLAVEDSPIC_MODE_HARVEST_POPCORNS_READY	1
#define I2C_SLAVEDSPIC_MODE_HARVEST_POPCORNS_DO		2	
#define I2C_SLAVEDSPIC_MODE_HARVEST_POPCORNS_END	3
		} harvest_popcorns;

		struct {
			uint8_t mode;
#define I2C_SLAVEDSPIC_MODE_DUMP_POPCORNS_DO		1	
#define I2C_SLAVEDSPIC_MODE_DUMP_POPCORNS_END		2
		} dump_popcorns;

		struct {
			uint8_t mode;
#define I2C_SLAVEDSPIC_MODE_DUMP_FRONT_CUP_CATCH		1
#define I2C_SLAVEDSPIC_MODE_DUMP_FRONT_CUP_PULL_UP		2
#define I2C_SLAVEDSPIC_MODE_DUMP_FRONT_CUP_PULL_DOWN	3
#define I2C_SLAVEDSPIC_MODE_DUMP_FRONT_CUP_DROP			4
		} dump_front_cup;

		/* add more here */
	};
};

#endif
