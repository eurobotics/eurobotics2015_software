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
 *  Revision : $Id$
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  i2c_protocol.c,v 1.5 2009/05/27 20:04:07 zer0 Exp  
 */

#include <string.h>

#include <aversive.h>
#include <aversive/list.h>
#include <aversive/error.h>

#include <i2c_slave_lite.h>
#include <ax12.h>
#include <uart.h>
#include <clock_time.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"
#include "sensor.h"
#include "state.h"
#include "main.h"

uint8_t i2c_watchdog_cnt = 255;

void i2c_protocol_init(void)
{
}

static uint8_t dummy=0;

/*** LED CONTROL ***/
void i2c_led_control(uint8_t l, uint8_t state)
{
	state? LED1_ON():LED1_OFF();
}

static int8_t i2c_set_mode(struct i2c_cmd_slavedspic_set_mode *cmd)
{

	state_set_mode(cmd);
	return 0;
}

void i2c_write_event(uint8_t cmd_byte, uint8_t *buf, int16_t size)
{
	void *void_cmd = buf;
	
	//static uint8_t a = 0;
	
	//a++;
	//if (a & 0x10)
	//	LED1_TOGGLE();
	
	if(cmd_byte == I2C_CMD_GENERIC){

		switch (buf[0]) {
	
			/* Commands */
			case I2C_CMD_LED_CONTROL: 
			{
				struct i2c_cmd_led_control *cmd = void_cmd;
				if (size != sizeof (*cmd))
					break;
				//i2c_led_control(cmd->led_num, cmd->state);
				dummy = cmd->state;
				break;
			}
			
			case I2C_CMD_SLAVEDSPIC_SET_MODE: 
			{
				struct i2c_cmd_slavedspic_set_mode *cmd = void_cmd;
				if (size != sizeof (*cmd))
					break;
			
				LED1_TOGGLE();	
				i2c_set_mode(cmd);
				break;
			}
				
			default:
				break;
		}
	}
	return;
}


void i2c_read_event(uint8_t cmd_byte, uint8_t *buf)
{
	void *void_cmd = buf;
	
	switch(cmd_byte)
	{	
		case I2C_REQ_SLAVEDSPIC_STATUS:
		{
			struct i2c_slavedspic_status *cmd = void_cmd;
			
			(*cmd).hdr.cmd = I2C_ANS_SLAVEDSPIC_STATUS;

         /* XXX syncronized with maindspic */

			/* infos */
			(*cmd).status = slavedspic.status;
			(*cmd).nb_stored_fires = slavedspic.nb_stored_fires;

			/* XXX watchdog time */
			i2c_watchdog_cnt = 5;
			
			break;
		}
		
		default:
			break;
	}
}


