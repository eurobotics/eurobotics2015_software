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
 *  Revision : $Id: i2c_commands.h,v 1.9 2009/05/27 20:04:06 zer0 Exp $
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

//#include "../slavedspic/actuator.h"

#define I2C_SLAVEDSPIC_ADDR 	0x10
#define I2C_GPIOS_01_ADDR 		0x20
#define I2C_GPIOS_23_ADDR 		0x21

#define I2C_COLOR_PURPLE 	0
#define I2C_COLOR_RED		1
#define I2C_COLOR_MAX		2

#define I2C_OPPONENT_NOT_THERE -1000

struct i2c_cmd_hdr {
	uint8_t cmd;
};


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

#define I2C_SLAVEDSPIC_STATUS_BUSY		1
#define I2C_SLAVEDSPIC_STATUS_READY		0

	/* actuators blocking */
	uint8_t fingers_floor_blocked;
	uint8_t fingers_totem_blocked;
	uint8_t arm_right_blocked;
	uint8_t arm_left_blocked;
	uint8_t lift_blocked;

	/* sensors */
	uint8_t object_catched;

	/* infos */
	uint8_t status;
	uint8_t harvest_mode;
	uint8_t store_mode;
	uint8_t dump_mode;

	uint8_t nb_goldbars_in_boot;
	uint8_t nb_golbarrs_in_mouth;
	uint8_t nb_coins_in_boot;
	uint8_t nb_coins_in_mouth;

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
	
#define I2C_SLAVEDSPIC_MODE_INIT				0x01
#define I2C_SLAVEDSPIC_MODE_POWER_OFF		0x02
#define I2C_SLAVEDSPIC_MODE_FINGERS			0x03
#define I2C_SLAVEDSPIC_MODE_ARM				0x04
#define I2C_SLAVEDSPIC_MODE_HOOK				0x05
#define I2C_SLAVEDSPIC_MODE_BOOT				0x06
#define I2C_SLAVEDSPIC_MODE_TRAY				0x07
#define I2C_SLAVEDSPIC_MODE_TURBINE_ANGLE	0x08
#define I2C_SLAVEDSPIC_MODE_TURBINE_BLOW	0x09
#define I2C_SLAVEDSPIC_MODE_LIFT_HEIGHT	0x0A

#define I2C_SLAVEDSPIC_MODE_HARVEST			0x0B
#define I2C_SLAVEDSPIC_MODE_STORE			0x0C
#define I2C_SLAVEDSPIC_MODE_DUMP				0x0D
	uint8_t mode;
	union{
		struct {
			uint8_t type;
#define I2C_FINGERS_TYPE_FLOOR	FINGERS_TYPE_FLOOR
#define I2C_FINGERS_TYPE_TOTEM	FINGERS_TYPE_TOTEM

			uint8_t mode;
#define I2C_FINGERS_MODE_HUG 		FINGERS_MODE_HUG
#define I2C_FINGERS_MODE_OPEN 	FINGERS_MODE_OPEN
#define I2C_FINGERS_MODE_HOLD 	FINGERS_MODE_HOLD	
#define I2C_FINGERS_MODE_CLOSE	FINGERS_MODE_CLOSE
#define I2C_FINGERS_MODE_PUSHIN	FINGERS_MODE_PUSHIN

			int16_t offset;
		} fingers;

		struct {
			uint8_t type;
#define I2C_ARM_TYPE_RIGHT	ARM_TYPE_RIGHT
#define I2C_ARM_TYPE_LEFT	ARM_TYPE_LEFT

			uint8_t mode;
#define I2C_ARM_MODE_HIDE				ARM_MODE_HIDE
#define I2C_ARM_MODE_SHOW				ARM_MODE_SHOW
#define I2C_ARM_MODE_PUSH_GOLDBAR	ARM_MODE_PUSH_GOLDBAR
#define I2C_ARM_MODE_PUSH_FLOOR		ARM_MODE_PUSH_FLOOR

			int16_t offset;
		} arm;

		struct {
			uint32_t height;
		} lift;

		struct {
			int8_t angle_deg;
			uint16_t angle_speed;
			uint16_t blow_speed;
		} turbine;
		
		struct {
			uint8_t mode;
#define I2C_BOOT_MODE_OPEN_FULL		BOOT_MODE_OPEN_FULL
#define I2C_BOOT_MODE_OPEN_HOLD		BOOT_MODE_OPEN_HOLD
#define I2C_BOOT_MODE_CLOSE			BOOT_MODE_CLOSE

		} boot;

		struct {
			uint8_t mode;
#define I2C_HOOK_MODE_HIDE			HOOK_MODE_HIDE
#define I2C_HOOK_MODE_SHOW			HOOK_MODE_SHOW
#define I2C_HOOK_MODE_FUCKYOU		HOOK_MODE_FUCKYOU
#define I2C_HOOK_MODE_OPEN_HOLD	HOOK_MODE_OPEN_HOLD

		} hook;

		struct {
			uint8_t type;
#define I2C_TRAY_TYPE_RECEPTION	TRAY_TYPE_RECEPTION
#define I2C_TRAY_TYPE_STORE		TRAY_TYPE_STORE
#define I2C_TRAY_TYPE_BOOT			TRAY_TYPE_BOOT

			uint8_t mode;
#define I2C_TRAY_MODE_DOWN			TRAY_MODE_DOWN 	/* it means off in case of boot tray */
#define I2C_TRAY_MODE_UP			TRAY_MODE_UP 		/* only reception and store tray */
#define I2C_TRAY_MODE_VIBRATE		TRAY_MODE_VIBRATE /* only boot and store tray */

		} tray;

		struct {
			uint8_t mode;
#define I2C_HARVEST_MODE_PREPARE_TOTEM				0
#define I2C_HARVEST_MODE_PREPARE_GOLDBAR_TOTEM	1
#define I2C_HARVEST_MODE_PREPARE_GOLDBAR_FLOOR	2
#define I2C_HARVEST_MODE_PREPARE_COINS_TOTEM		3
#define I2C_HARVEST_MODE_PREPARE_COINS_FLOOR		4
#define I2C_HARVEST_MODE_COINS_ISLE					5
#define I2C_HARVEST_MODE_COINS_FLOOR				6
#define I2C_HARVEST_MODE_COINS_TOTEM				7
#define I2C_HARVEST_MODE_GOLDBAR_TOTEM				8
#define I2C_HARVEST_MODE_GOLDBAR_FLOOR				9

		} harvest;

		struct {
			uint8_t mode;
#define I2C_STORE_MODE_GOLDBAR_IN_MOUTH		0
#define I2C_STORE_MODE_GOLDBAR_IN_BOOT			1
#define I2C_STORE_MODE_MOUTH_IN_BOOT			2

			uint8_t times;
		} store;


		struct {
			uint8_t mode;
#define I2C_DUMP_MODE_PREPARE_HOLD			0
#define I2C_DUMP_MODE_PREPARE_MOUTH			1
#define I2C_DUMP_MODE_PREPARE_BOOT			2
#define I2C_DUMP_MODE_BOOT						3
#define I2C_DUMP_MODE_BOOT_BLOWING			4
#define I2C_DUMP_MODE_MOUTH_BLOWING			5
#define I2C_DUMP_MODE_END_BOOT				6
#define I2C_DUMP_MODE_END_MOUTH				7

		} dump;

		/* add more here */
	};
};

#endif
