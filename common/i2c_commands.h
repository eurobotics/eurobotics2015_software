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

#define I2C_SLAVEDSPIC_ADDR 	0x10
#define I2C_GPIOS_01_ADDR 		0x20
#define I2C_GPIOS_23_ADDR 		0x21

#define I2C_COLOR_PURPLE 	0
#define I2C_COLOR_RED		1
#define I2C_COLOR_MAX		2

#define I2C_SIDE_LEFT	0
#define I2C_SIDE_RIGHT 	1
#define I2C_SIDE_MAX		2

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

	struct {
		uint8_t state;
		uint8_t belts_blocked;
		uint8_t token_catched;
	}ts[I2C_SIDE_MAX];
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
#define I2C_SLAVEDSPIC_MODE_TOKEN_TAKE		0x02
#define I2C_SLAVEDSPIC_MODE_TOKEN_EJECT	0x03
#define I2C_SLAVEDSPIC_MODE_TOKEN_STOP		0x04
#define I2C_SLAVEDSPIC_MODE_TOKEN_PUSH_L	0x05
#define I2C_SLAVEDSPIC_MODE_TOKEN_PUSH_R	0x06
#define I2C_SLAVEDSPIC_MODE_TOKEN_OUT		0x07
#define I2C_SLAVEDSPIC_MODE_MIRROR_POS		0x08
#define I2C_SLAVEDSPIC_MODE_TOKEN_SHOW		I2C_SLAVEDSPIC_MODE_TOKEN_PUSH_L
	uint8_t mode;
	union{
		struct {
			uint8_t side;
			uint8_t speed_div4;
		}ts;

		struct {
#define I2C_MIRROR_SIDE_RIGHT	0
#define I2C_MIRROR_SIDE_LEFT	1
			uint8_t side;
			uint8_t pos_h;
			uint8_t pos_l;
		} mirror;		

		/* add more here */
	};
};

#endif
