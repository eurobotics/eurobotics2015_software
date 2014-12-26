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


/* slavedspic mode commands 2012 */

/****** GENERIC FUNCTIONS */

/* initialize */
int8_t i2c_slavedspic_mode_init(void);

/* exit */
int8_t i2c_slavedspic_mode_power_off(void);

/* wait for slavedspic is ready */
void i2c_slavedspic_wait_status(uint8_t status);


/****** LOW LEVEL FUNCTIONS */

/* set arms */
int8_t i2c_slavedspic_set_arm(uint8_t side, uint8_t elbow_height_mm, uint8_t elbow_yaw_deg, int8_t wrist_roll_deg);

/* set stick */
int8_t i2c_slavedspic_set_stick(uint8_t side, uint8_t angle_deg);

/* set comb */
int8_t i2c_slavedspic_set_comb(uint8_t side, uint8_t angle_deg);

/* set comb tray */
int8_t i2c_slavedspic_set_comb_tray(uint8_t angle_deg);

/* set boot tray */
int8_t i2c_slavedspic_set_boot_tray(uint8_t duty);

/* set boot tray */
int8_t i2c_slavedspic_set_boot_door(uint8_t angle_deg);

/* set vacum */
int8_t i2c_slavedspic_set_vacuum_motor(uint8_t duty);

/* set vacum flow */
int8_t i2c_slavedspic_set_vacuum_presure(uint8_t on);



/****** HI LEVEL FUNCTIONS */

/* FIRES STUFF */

/* prepare for pick up fire */
int8_t i2c_slavedspic_mode_prep_pickup_fire(uint8_t side, uint8_t level, uint8_t elbow_yaw_deg, int8_t wrist_roll_deg);

/* pick up fire */
int8_t i2c_slavedspic_mode_pickup_fire(uint8_t side, uint8_t level);

/* store fire */
int8_t i2c_slavedspic_mode_store_fire(uint8_t side);

/* load fires on arms */
int8_t i2c_slavedspic_mode_load_fire(uint8_t nb_left, uint8_t nb_right);

/* prepare for drag fires */
int8_t i2c_slavedspic_mode_prep_drag_fire(uint8_t side, uint8_t level, uint8_t elbow_yaw_deg, int8_t wrist_yaw_deg);

/* drag fires */
int8_t i2c_slavedspic_mode_drag_fire(uint8_t side, uint8_t level);

/* prepare for turn fires */
int8_t i2c_slavedspic_mode_prep_turn_fires(uint8_t side, uint8_t level, uint8_t elbow_yaw_deg);

/* turn fires */
int8_t i2c_slavedspic_mode_turn_fires(uint8_t side, uint8_t level);

/* hide arm */
int8_t i2c_slavedspic_mode_hide_arms(uint8_t side);


/* TREES AND FRUITS STUFF */

/* prepare for harvest tree's fruit */
int8_t i2c_slavedspic_mode_prep_harvest_tree(void);

/* harvest tree's fruit */
int8_t i2c_slavedspic_mode_harvest_tree(void);

/* dump fruit harvested */
int8_t i2c_slavedspic_mode_dump_fruit(uint8_t vibration_duty);

/* ends fruit dumping fruit */
int8_t i2c_slavedspic_mode_end_dump_fruit(void);

/* prepare for scanning toxic fruits */
int8_t i2c_slavedspic_mode_prep_toxic_fruit(uint8_t side, uint8_t level, uint8_t elbow_yaw_deg);

/* pickup toxic fruit */
int8_t i2c_slavedspic_mode_pickup_toxic_fruit(uint8_t side, uint8_t level, uint8_t elbow_yaw_deg);

/* drag toxic fruit */
int8_t i2c_slavedspic_mode_drop_toxic_fruit(uint8_t side);


/* STICKS BROOMS STUFF */

/* sticks are hidden */
int8_t i2c_slavedspic_mode_sticks_hide(void); 

/* use a stick for clean fires */
int8_t i2c_slavedspic_mode_sticks_clean(uint8_t level);



#endif
