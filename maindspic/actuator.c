/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
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
 *  Javier Baliñas Santos <javier@arc-robots.org>
 */

#include <aversive.h>
#include <encoders_dspic.h>
#include <dac_mc.h>

#include "i2c_protocol.h"
#include "actuator.h"
#include "main.h"
#include "robotsim.h"

void dac_set_and_save(void *dac, int32_t val)
{
#ifdef EUROBOT2011_BOARD
//#define RIGHT_MOTOR_OFFSET 2000 
#define LEFT_MOTOR_OFFSET  0
#else
#define RIGHT_MOTOR_OFFSET	0 
//#define LEFT_MOTOR_OFFSET  3500
#define LEFT_MOTOR_OFFSET  0
#endif

#define RIGHT_MOTOR_MAX		(65535-LEFT_MOTOR_OFFSET)
#define LEFT_MOTOR_MAX		(65535-RIGHT_MOTOR_OFFSET)
	
	if (dac == LEFT_MOTOR) {
		/* apply offset */
		//val = val > 0? (val + LEFT_MOTOR_OFFSET):(val - LEFT_MOTOR_OFFSET);

		/* we need to do the saturation here, before saving the value */
		if (val > LEFT_MOTOR_MAX)
			val = LEFT_MOTOR_MAX;
		if (val < -LEFT_MOTOR_MAX)
			val = -LEFT_MOTOR_MAX;

		/* save value */
		mainboard.dac_l = val;
	}
	else if (dac == RIGHT_MOTOR){
		/* apply offset */
		//val = val > 0? (val + RIGHT_MOTOR_OFFSET):(val - RIGHT_MOTOR_OFFSET);

		/* we need to do the saturation here, before saving the value */
		if (val > RIGHT_MOTOR_MAX)
			val = RIGHT_MOTOR_MAX;
		if (val < -RIGHT_MOTOR_MAX)
			val = -RIGHT_MOTOR_MAX;

		/* save value */
		mainboard.dac_r = val;
	}

	/* set value */
#ifdef HOST_VERSION
	robotsim_pwm(dac, val);
#else
	dac_mc_set(dac, val);
#endif
}

#if 0
/* lasers off */
void lasers_set_on(void)
{
	_LATC7 = 1;
}

/* lasers on */
void lasers_set_off(void)
{
	_LATC7 = 0;
}

uint8_t lasers_get_state(void)
{
	return _LATC7;
}



/* manage mirrors position */

#define IDLE							0
#define WAITING_DUMMY_POSSITION	1
#define WAITING_CMD_POSSITION		2

#define DUMMY_POSSITION				300
#define AX12_T_SETUP_US				500000UL

static uint8_t state = IDLE;
static uint8_t mirror_update_pos = 0;
static uint16_t mirror_right_pos;
static uint16_t mirror_left_pos;

void mirrors_state_machine(void)
{
	static microseconds t_setup_us;

	switch(state) {

		case IDLE:
			if(mirror_update_pos) {
				mirror_update_pos = 0;
//				i2c_slavedspic_mode_mirror_pos(I2C_MIRROR_SIDE_RIGHT, DUMMY_POSSITION);
//				time_wait_ms(20); /* little HACK */
//				i2c_slavedspic_mode_mirror_pos(I2C_MIRROR_SIDE_LEFT, DUMMY_POSSITION);
//				t_setup_us = time_get_us2();
				state = WAITING_DUMMY_POSSITION;			
			}
			break;

		case WAITING_DUMMY_POSSITION:

//			if(time_get_us2() - t_setup_us > AX12_T_SETUP_US) {
				i2c_slavedspic_mode_mirror_pos(I2C_MIRROR_SIDE_RIGHT, mirror_right_pos);
				time_wait_ms(50); /* little HACK */
				i2c_slavedspic_mode_mirror_pos(I2C_MIRROR_SIDE_LEFT, mirror_left_pos);
				t_setup_us = time_get_us2();
				state = WAITING_CMD_POSSITION;
//			}
			break;

		case WAITING_CMD_POSSITION:
			if(time_get_us2() - t_setup_us > AX12_T_SETUP_US) {
				state = IDLE;
			}
			break;

		default:
			state = IDLE;
			break;
	}
}

/* set mirrors mode */

#define MIRROR_R_TOWERS_POS	227
#define MIRROR_L_TOWERS_POS	207
#define MIRROR_R_FIGURES_POS	212 //210 //208 //210
#define MIRROR_L_FIGURES_POS	192 //190 //187 //190
#define MIRROR_HIDE_POS			300

void mirrors_set_mode(uint8_t mode)
{
	if(mode == MODE_LOOK_FOR_TOWERS) {
		mirror_right_pos = MIRROR_R_TOWERS_POS;
		mirror_left_pos = MIRROR_L_TOWERS_POS;
	}
	else if(mode == MODE_LOOK_FOR_FIGURES) {
		mirror_right_pos = MIRROR_R_FIGURES_POS;
		mirror_left_pos = MIRROR_L_FIGURES_POS;
	}
	else if(mode == MODE_HIDE_MIRRORS) {
		mirror_right_pos = MIRROR_HIDE_POS;
		mirror_left_pos = MIRROR_HIDE_POS;
	}

	mirror_update_pos = 1;
}

#endif
