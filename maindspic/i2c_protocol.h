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
 *  i2c_protocol.h,v 1.5 2009/05/27 20:04:07 zer0 Exp.
 */

#ifndef _I2C_PROTOCOL_H_
#define _I2C_PROTOCOL_H_

#include "i2c_mem.h"
#include "../common/i2c_commands.h"

/* i2c protocol functions */
void i2c_protocol_init(void);
void i2c_protocol_debug(void);

void i2cproto_wait_update(void);
void i2c_poll_slaves(void *dummy);

void i2c_read_event(uint8_t *rBuff, uint16_t size);
void i2c_write_event(uint16_t size);

/* dummy slavedspic led control */
int8_t i2c_led_control(uint8_t addr, uint8_t led, uint8_t state);


/* slavedspic mode commands 2012 */

/****** GENERIC FUNCTIONS */

#define STATUS_READY		I2C_SLAVEDSPIC_STATUS_READY
#define STATUS_BUSY			I2C_SLAVEDSPIC_STATUS_BUSY
#define STATUS_WAITING      I2C_SLAVEDSPIC_STATUS_WAITING
#define STATUS_STORING      I2C_SLAVEDSPIC_STATUS_STORING
#define STATUS_DONE			I2C_SLAVEDSPIC_STATUS_DONE	
#define STATUS_BLOCKED		I2C_SLAVEDSPIC_STATUS_BLOCKED
#define STATUS_ERROR		I2C_SLAVEDSPIC_STATUS_ERROR

/* initialize */
int8_t i2c_slavedspic_mode_init(void);

/* exit */
int8_t i2c_slavedspic_mode_power_off(void);

/* XXX wait for slavedspic is ready */
void i2c_slavedspic_wait_ready(void);

/* get slavedispic status */
uint8_t i2c_slavedspic_get_status(void);

/****** SIMPLE ACTUATORS */

int8_t i2c_slavedspic_mode_blades(uint8_t side, uint8_t mode);
int8_t i2c_slavedspic_mode_tray(uint8_t mode, int8_t offset);

/****** MULTIPLE ACTUATORS */

/* set popcorn system mode */
int8_t i2c_slavedspic_mode_ps(uint8_t mode);

/* get popcorn system status */
uint8_t i2c_slavedspic_get_ps_status(void);

/* return 0 if no status matched, or the status received */
uint8_t i2c_slavedspic_ps_test_status(uint8_t status_flags);
uint8_t i2c_slavedspic_ps_wait_status_or_timeout (uint8_t status_flags, uint16_t timeout);

/* set stands system mode */
/* TODO: update harvest fucntion */
#define i2c_slavedspic_mode_ss_harvest(side,blade_angle_deg)  i2c_slavedspic_mode_ss_harvest_ready(side, blade_angle_deg)
int8_t i2c_slavedspic_mode_ss_harvest_ready(uint8_t side, int8_t blade_angle_deg);
int8_t i2c_slavedspic_mode_ss_harvest_do(uint8_t side, int8_t blade_angle_deg);

/* set stands system mode */
int8_t i2c_slavedspic_mode_ss(uint8_t mode, uint8_t side);

/* get popcorn system status */
uint8_t i2c_slavedspic_get_ss_status(uint8_t side);

/* return 0 if no status matched, or the status received */
uint8_t i2c_slavedspic_ss_test_status(uint8_t side, uint8_t status_flags);
uint8_t i2c_slavedspic_ss_wait_status_or_timeout (uint8_t side, uint8_t status_flags, uint16_t timeout);


#endif
