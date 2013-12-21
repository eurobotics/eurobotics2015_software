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


/* Actuators mode commands 2012 */
#if 0
int8_t i2c_slavedspic_mode_init(void);
int8_t i2c_slavedspic_mode_power_off(void);
int8_t i2c_slavedspic_mode_fingers(uint8_t type, uint8_t mode, int16_t offset);
int8_t i2c_slavedspic_mode_arm(uint8_t type, uint8_t mode, int16_t offset);
int8_t i2c_slavedspic_mode_lift_height(uint32_t height);
int8_t i2c_slavedspic_mode_hook(uint8_t mode);
int8_t i2c_slavedspic_mode_boot(uint8_t mode);
int8_t i2c_slavedspic_mode_tray(uint8_t type, uint8_t mode);
int8_t i2c_slavedspic_mode_turbine_angle(int8_t angle_deg, uint16_t angle_speed);
int8_t i2c_slavedspic_mode_turbine_blow(int8_t blow_speed);
int8_t i2c_slavedspic_mode_harvest(uint8_t mode);
int8_t i2c_slavedspic_mode_store(uint8_t times, uint8_t mode);
int8_t i2c_slavedspic_mode_dump(uint8_t mode);
int8_t i2c_slavedspic_mode_set_infos_all(int8_t nb_goldbars_in_boot, int8_t nb_goldbars_in_mouth,
                           int8_t nb_coins_in_boot, int8_t nb_coins_in_mouth);
int8_t i2c_slavedspic_mode_set_infos_mouth(int8_t nb_goldbars_in_mouth,int8_t nb_coins_in_mouth);
int8_t i2c_slavedspic_mode_set_infos_boot(int8_t nb_goldbars_in_boot, int8_t nb_coins_in_boot);
void i2c_slavedspic_wait_ready(void);
#endif

#endif
