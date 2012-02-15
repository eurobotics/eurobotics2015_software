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
 *  Revision : $Id: i2c_protocol.h,v 1.5 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  i2c_protocol.h,v 1.5 2009/05/27 20:04:07 zer0 Exp.
 */

#ifndef _I2C_PROTOCOL_H_
#define _I2C_PROTOCOL_H_

#include "i2c_mem.h"

/* i2c protocol functions */
void i2c_protocol_init(void);
void i2c_protocol_debug(void);

void i2cproto_wait_update(void);
void i2c_poll_slaves(void *dummy);

void i2c_read_event(uint8_t *rBuff, uint16_t size);
void i2c_write_event(uint16_t size);

/* dummy slavedspic led control */
int8_t i2c_led_control(uint8_t addr, uint8_t led, uint8_t state);

/* token systems control */
int8_t i2c_slavedspic_mode_init(void);
int8_t i2c_slavedspic_mode_token_take(uint8_t side);
int8_t i2c_slavedspic_mode_token_eject(uint8_t side);
int8_t i2c_slavedspic_mode_token_stop(uint8_t side);
int8_t i2c_slavedspic_mode_token_show(uint8_t side);
int8_t i2c_slavedspic_mode_token_out(uint8_t side);
int8_t i2c_slavedspic_mode_token_push_r(uint8_t side);
int8_t i2c_slavedspic_mode_token_push_l(uint8_t side);

/* mirrors control */
int8_t i2c_slavedspic_mode_mirror_pos(uint8_t side, uint16_t pos);

#endif
