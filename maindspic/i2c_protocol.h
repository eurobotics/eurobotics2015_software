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

/* initialize */
int8_t i2c_slavedspic_mode_init(void);

/* exit */
int8_t i2c_slavedspic_mode_power_off(void);

/* wait for slavedspic is ready */
void i2c_slavedspic_wait_ready(void);

/****** SIMPLE ACTUATORS */

/* set stick mode */
int8_t i2c_slavedspic_mode_stick(uint8_t type, uint8_t mode, int8_t offset);

/* set comb mode */
int8_t i2c_slavedspic_mode_combs(uint8_t mode, int8_t offset);

/* set tree tray */
int8_t i2c_slavedspic_mode_tree_tray (uint8_t mode, int8_t offset);

/* set boot tray mode */
int8_t i2c_slavedspic_mode_boot_tray(uint8_t mode);

/* set boot door mode*/
int8_t i2c_slavedspic_mode_boot_door(uint8_t mode);


/****** MULTIPLE ACTUATORS */

/* set harvest fruits mode */
int8_t i2c_slavedspic_mode_harvest_fruits(uint8_t mode);

/* set dump fruits mode */
int8_t i2c_slavedspic_mode_dump_fruits(uint8_t mode);


/* fires and torches modes */
int8_t i2c_slavedspic_mode_ready_for_pickup_torch (uint8_t sucker_type);
int8_t i2c_slavedspic_mode_pickup_torch (uint8_t sucker_type);


int8_t i2c_slavedspic_mode_ready_for_pickup_fire (uint8_t sucker_type, uint8_t level);
int8_t i2c_slavedspic_mode_pickup_fire (uint8_t sucker_type, uint8_t level);

int8_t i2c_slavedspic_mode_store_fire (uint8_t sucker_type);
int8_t i2c_slavedspic_mode_load_fire (uint8_t sucker_type);


int8_t i2c_slavedspic_mode_putdown_fire 
        (uint8_t sucker_type, uint8_t level,
         int16_t x, int16_t *y, int16_t *a, int8_t sucker_angle);

int8_t i2c_slavedspic_mode_putdown_fire_inv
        (uint8_t sucker_type, uint8_t level, int16_t x, int16_t *y, int16_t *a);

int8_t i2c_slavedspic_mode_release_fire (uint8_t sucker_type);


#endif
