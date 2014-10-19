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
 *  i2c_protocol.h,v 1.1 2009/03/05 22:52:35 zer0 Exp 
 */

#ifndef _I2C_PROTOCOL_H_
#define _I2C_PROTOCOL_H_

#include <aversive.h>
#include <stdio.h>
#include <i2c_slave_lite.h>

void i2c_protocol_init(void);
void i2c_read_event(uint8_t cmd_byte, uint8_t *buf);
void i2c_write_event(uint8_t cmd_byte, uint8_t *buf, uint16_t size);

#endif
