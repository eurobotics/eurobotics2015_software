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

#ifndef __MAIN_H__
#define __MAIN_H__

#include <aversive.h>
#include <aversive/error.h>

#include <clock_time.h>
#include <rdline.h>

#include <encoders_dspic.h>
#include <dac_mc.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <ax12.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include "actuator.h"
#include "state.h"

//#define EUROBOT_2012_BOARD

#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		\
			port &= ~_BV(bit);	\
		else				\
			port |= _BV(bit);	\
	} while(0)

#define LED1_ON() 		cbi(LATC, 9)
#define LED1_OFF() 		sbi(LATC, 9)
#define LED1_TOGGLE() 	LED_TOGGLE(LATC, 9)

#define RELE_OUT_PIN			_LATA9
#define RELE_OUT_PIN_ON		0
#define RELE_OUT_PIN_OFF	1

#define DRIVER_OUT_PIN		_LATA4
#define DRIVER_OUT_PIN_ON	0
#define DIVER_OUT_PIN_OFF	1

#define BRAKE_ON()	do{					\
								_LATA7 = 0; 	\
							} while(0)
#define BRAKE_OFF()	do{					\
								_LATA7 = 1;		\
							} while(0)

/* EUROBOT 2012 defines */
#define STANDS_EXCHANGER_ENCODER		((void *)1)
#define PWM_MC_STANDS_EXCHANGER_MOTOR	((void *)&gen.pwm_mc_mod2_ch1)

#define PWM_SERVO_POPCORN_TRAY			&gen.pwm_servo_oc3
#define PWM_SERVO_STANDS_CLAMP_L		&gen.pwm_servo_oc1
#define PWM_SERVO_STANDS_CLAMP_R		&gen.pwm_servo_oc2

#define AX12_ID_STANDS_TOWER_CLAMPS_UP		3
#define AX12_ID_STANDS_TOWER_CLAMPS_DOWN	8
#define AX12_ID_STANDS_ELEVATOR_L			6
#define AX12_ID_STANDS_ELEVATOR_R			0
#define AX12_ID_STANDS_BLADE_L				4
#define AX12_ID_STANDS_BLADE_R				7
#define AX12_ID_CUP_CLAMP_POPCORN_DOOR_L	1
#define AX12_ID_CUP_CLAMP_POPCORN_DOOR_R	10
#define AX12_ID_POPCORN_RAMP_L				11
#define AX12_ID_POPCORN_RAMP_R				5
#define AX12_ID_CUP_CLAMP_FRONT				9
#define AX12_ID_CUP_HOLDER_FRONT			2

#define S_STAND_INSIDE_L			SENSOR1
#define S_STAND_INSIDE_R			SENSOR2
#define S_CUP_REAR					SENSOR4
#define S_STAND_EXCHANGER_ENDSTOP	SENSOR5

/** ERROR NUMS */
#define E_USER_I2C_PROTO   195
#define E_USER_SENSOR      196
#define E_USER_ST_MACH     197
#define E_USER_CS          198
#define E_USER_AX12        199
#define E_USER_ACTUATORS   200


#define LED_PRIO           170
#define TIME_PRIO          160
#define SENSOR_PRIO        120
#define I2C_POLL_PRIO      110
#define CS_PRIO            100

#define CS_PERIOD 2000L

#define NB_LOGS 4

/* generic to all boards */
struct genboard {
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* dc motors */
	struct pwm_mc pwm_mc_mod1_ch2;
	struct pwm_mc pwm_mc_mod2_ch1;

	/* brushless motors */
	struct dac_mc dac_mc_left;
	
	/* servos */
	struct pwm_servo pwm_servo_oc1;
	struct pwm_servo pwm_servo_oc2;
	struct pwm_servo pwm_servo_oc3;
	struct pwm_servo pwm_servo_oc4;

	/* ax12 interface */
	AX12 ax12;

	/* log */
	uint8_t logs[NB_LOGS+1];
	uint8_t log_level;
	uint8_t debug;
};

struct cs_block {
	uint8_t on;
	uint8_t calibrated;
	uint8_t blocking;
  	struct cs cs;
  	struct pid_filter pid;
	struct quadramp_filter qr;
	struct blocking_detection bd;
};

/* mech specific */
struct slavedspic {

	/* misc flags */
	uint8_t flags;
#define DO_ENCODERS  1
#define DO_CS        2
#define DO_BD        4
#define DO_POWER     8

	/* control systems */
  	struct cs_block stands_exchanger;

	/* actuators */
	stands_blade_t stands_blade_l, stands_blade_r;
	stands_clamp_t stands_clamp_l, stands_clamp_r;
	stands_elevator_t stands_elevator_l, stands_elevator_r;
	stands_tower_clamps_t stands_tower_clamps;

	cup_clamp_popcorn_door_t cup_clamp_popcorn_door_l, cup_clamp_popcorn_door_r;
	popcorn_tray_t popcorn_tray;
	popcorn_ramps_t popcorn_ramps;
	cup_clamp_front_t cup_clamp_front;
	cup_holder_front_t cup_holder_front;

	/* systems */
	popcorn_system_t ps;
	stands_system_t ss[2];

	/* infos */
	uint8_t status;

	/* infos */
	uint8_t our_color;
};

extern struct genboard gen;
extern struct slavedspic slavedspic;


#define WAIT_COND_OR_TIMEOUT(cond, timeout)                   \
({                                                            \
        microseconds __us = time_get_us2();                   \
        uint8_t __ret = 1;                                    \
        while(! (cond)) {                                     \
                if (time_get_us2() - __us > (timeout)*1000L) {\
                        __ret = 0;                            \
                        break;                                \
                }                                             \
        }                                                     \
        __ret;                                                \
})

#endif
