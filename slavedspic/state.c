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
 *  Revision : $Id: state.c,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*   *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011) *  Javier Baliñas Santos <javier@arc-robots.org> * *  Code ported to family of microcontrollers dsPIC from *  state.c,v 1.4 2009/05/27 20:04:07 zer0 Exp. */


#include <math.h>
#include <string.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <rdline.h>
#include <vt100.h>

#include "../common/i2c_commands.h"

#include "ax12_user.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "state.h"
#include "main.h"

#define STMCH_DEBUG(args...) DEBUG(E_USER_ST_MACH, args)
#define STMCH_NOTICE(args...) NOTICE(E_USER_ST_MACH, args)
#define STMCH_ERROR(args...) ERROR(E_USER_ST_MACH, args)

/* shorter aliases for this file */
#define INIT				I2C_SLAVEDSPIC_MODE_INIT
#define TOKEN_TAKE		I2C_SLAVEDSPIC_MODE_TOKEN_TAKE
#define TOKEN_EJECT		I2C_SLAVEDSPIC_MODE_TOKEN_EJECT
#define TOKEN_STOP		I2C_SLAVEDSPIC_MODE_TOKEN_STOP
#define TOKEN_SHOW		I2C_SLAVEDSPIC_MODE_TOKEN_SHOW
#define TOKEN_PUSH_L		I2C_SLAVEDSPIC_MODE_TOKEN_PUSH_L
#define TOKEN_PUSH_R		I2C_SLAVEDSPIC_MODE_TOKEN_PUSH_R
#define TOKEN_OUT			I2C_SLAVEDSPIC_MODE_TOKEN_OUT
#define MIRROR_POS		I2C_SLAVEDSPIC_MODE_MIRROR_POS

static struct i2c_cmd_slavedspic_set_mode mainboard_command;
static volatile uint8_t prev_state;
static volatile uint8_t mode_changed = 0;
uint8_t state_debug = 0;

/* local values of mirror cmd */
static uint8_t mirror_side;
static uint16_t mirror_pos;


/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_slavedspic_set_mode *cmd)
{
	prev_state = mainboard_command.mode;
	memcpy(&mainboard_command, cmd, sizeof(mainboard_command));
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, mainboard_command.mode);

	/* state machines based modes */
	if (mainboard_command.mode == TOKEN_TAKE) {
		slavedspic.ts[mainboard_command.ts.side].state_rqst = TS_STATE_TAKE;
		slavedspic.ts[mainboard_command.ts.side].speed_rqst =
			 (uint16_t)(((uint16_t)mainboard_command.ts.speed_div4<<2)|0x0003);
		slavedspic.ts[mainboard_command.ts.side].state_changed = 1;
	}
	else if (mainboard_command.mode == TOKEN_EJECT) {
		slavedspic.ts[mainboard_command.ts.side].state_rqst = TS_STATE_EJECT;
		slavedspic.ts[mainboard_command.ts.side].speed_rqst =
			 (uint16_t)(((uint16_t)mainboard_command.ts.speed_div4<<2)|0x0003);
		slavedspic.ts[mainboard_command.ts.side].state_changed = 1;	
	}
	else if (mainboard_command.mode == TOKEN_STOP) {
		slavedspic.ts[mainboard_command.ts.side].state_rqst = TS_STATE_STOP;
		slavedspic.ts[mainboard_command.ts.side].speed_rqst = 0;
		slavedspic.ts[mainboard_command.ts.side].state_changed = 1;	
	}
	else if (mainboard_command.mode == TOKEN_PUSH_L) {
		slavedspic.ts[mainboard_command.ts.side].state_rqst = TS_STATE_PUSH_L;
		slavedspic.ts[mainboard_command.ts.side].speed_rqst =
			 (uint16_t)(((uint16_t)mainboard_command.ts.speed_div4<<2)|0x0003);
		slavedspic.ts[mainboard_command.ts.side].state_changed = 1;
	}
	else if (mainboard_command.mode == TOKEN_PUSH_R) {
		slavedspic.ts[mainboard_command.ts.side].state_rqst = TS_STATE_PUSH_R;
		slavedspic.ts[mainboard_command.ts.side].speed_rqst =
			 (uint16_t)(((uint16_t)mainboard_command.ts.speed_div4<<2)|0x0003);
		slavedspic.ts[mainboard_command.ts.side].state_changed = 1;
	}
	else if (mainboard_command.mode == TOKEN_OUT) {
		slavedspic.ts[mainboard_command.ts.side].state_rqst = TS_STATE_OUT;
		slavedspic.ts[mainboard_command.ts.side].speed_rqst =
			 (uint16_t)(((uint16_t)mainboard_command.ts.speed_div4<<2)|0x0003);
		slavedspic.ts[mainboard_command.ts.side].state_changed = 1;
	}
	else if (mainboard_command.mode == MIRROR_POS) {

		/* mode_changed */
		mode_changed = 1;

		/* save side */
		mirror_side = mainboard_command.mirror.side;

		/* save position */
		mirror_pos = ((uint16_t)mainboard_command.mirror.pos_h << 8);
		mirror_pos |= (0x00FF & (uint16_t)mainboard_command.mirror.pos_l);
	}
	/* one shoot mode */
	else
		mode_changed = 1;

	return 0;
}

/* get last mode */
uint8_t state_get_mode(void)
{
	return mainboard_command.mode;
}


/* check that state is the one in parameter and that state changed */
uint8_t state_check_update(uint8_t mode)
{
	if ((mode == mainboard_command.mode) && (mode_changed == 1)){
		mode_changed = 0;
		return 1;
	}
	return 0;
}

/* debug state machines step to step */
void state_debug_wait_key_pressed(void)
{
	if (!state_debug)
		return;
	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* init mode */
static void state_do_init(void)
{
	if (!state_check_update(INIT))
		return;

	state_init();
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
}

/* mirror pos mode */
static void state_do_mirror_pos(void)
{
	if (!state_check_update(MIRROR_POS))
		return;

	/* set position of mirror */
	mirror_pos_set(mirror_side, mirror_pos);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
}

/* token system init */
void token_system_init(token_system_t *ts, uint8_t belts_side,
								uint8_t sensor_stop, uint8_t sensor_catched)
{
	ts->state = TS_STATE_IDLE;
	ts->speed = 0;
	ts->state_changed = 0;
	ts->state_rqst = 0;
	ts->speed_rqst = 0;
	ts->token_catched = 0;
	ts->belts_blocked = 0;
	ts->belts_side = belts_side;
	ts->sensor_stop = sensor_stop;
	ts->sensor_catched = sensor_catched;

	/* apply to belts */
	belts_mode_set(ts->belts_side, BELTS_MODE_OUT, ts->speed);
}

/* manage a token system */
void token_system_manage(token_system_t *ts)
{
#define BELTS_LOAD_TH 						1020
#define BELTS_BLOCKED_SAMPLE_TIME_US	10000
#define BELTS_BLOCKING_TIMES_TH			50

	static uint8_t flag_catched = 0;
//	static microseconds time_us = 0;
//	static uint8_t blocking_times = 0;

#ifdef SENSOR_STOP_TIME
	static microseconds us = 0;
#endif

	/* update state */
	if(ts->state_changed){
		ts->state_changed = 0;
		ts->state = ts->state_rqst;
		ts->speed = ts->speed_rqst;
		STMCH_DEBUG("%s mode=%d", __FUNCTION__, ts->state);
	}

	/* XXX update info */
	ts->token_catched = sensor_get(ts->sensor_catched);

	/* ejecute state */
	switch(ts->state){
		case TS_STATE_IDLE:
			break;

		case TS_STATE_TAKE:
			//if(!sensor_get(ts->sensor_stop)){
				belts_mode_set(ts->belts_side, BELTS_MODE_IN, ts->speed);

				STMCH_DEBUG("waiting %s stop...",
								ts->belts_side==I2C_SIDE_REAR?"REAR":"FRONT");

				ts->state = TS_STATE_WAITING_STOP;
			//}
			//else
			//	ts->state = TS_STATE_IDLE;
			//break;

		case TS_STATE_WAITING_STOP:

//			/* XXX update info */
//			ts->token_catched = sensor_get(ts->sensor_catched);

			if(ts->token_catched && flag_catched==0){
				flag_catched = 1;
#ifdef SENSOR_STOP_TIME
				us = time_get_us2();
#endif
				STMCH_DEBUG("token %s catched!",
								ts->belts_side==I2C_SIDE_REAR?"REAR":"FRONT");
			}

			/* stop belts when sensor */
#ifdef SENSOR_STOP_TIME
			if((time_get_us2() - us > 500000L) && (flag_catched == 1)) {
#else
			if(sensor_get(ts->sensor_stop)){
#endif
				belts_mode_set(ts->belts_side, BELTS_MODE_OUT, 0);

				STMCH_DEBUG("token %s stoped!",
								ts->belts_side==I2C_SIDE_REAR?"REAR":"FRONT");
				
				flag_catched = 0;
				ts->state = TS_STATE_IDLE;
			}

#ifdef BELT_BLOCKING
			/* stop belts when blocked */
			if(ABS(time_get_us() - time_us) > BELTS_BLOCKED_SAMPLE_TIME_US)	
			{
				time_us = time_get_us();	


				//STMCH_ERROR("left load %d", belts_load_get(ts->belts_side));
				if(belts_load_get(ts->belts_side)>=BELTS_LOAD_TH) {
					
					blocking_times ++;
						
					STMCH_ERROR("%s belts BLOCKED!!",
						 				 ts->belts_side==I2C_SIDE_REAR?"REAR":"FRONT");
				
					/* belts blocked after N times */
					if(blocking_times > BELTS_BLOCKING_TIMES_TH) {
						blocking_times = 0;	
						
		
						belts_mode_set(ts->belts_side, BELTS_MODE_OUT, 0);
						ts->belts_blocked = 1;
						ts->state = TS_STATE_IDLE;
					}
				}
				else
					blocking_times = 0;
			}
			else
				ts->belts_blocked = 0;
#endif
			break;

		case TS_STATE_EJECT:
			belts_mode_set(ts->belts_side, BELTS_MODE_OUT, ts->speed);

			STMCH_DEBUG("waiting %s free...",
							ts->belts_side==I2C_SIDE_REAR?"REAR":"FRONT");

			ts->state = TS_STATE_WAITING_FREE;
			break;
	
		case TS_STATE_WAITING_FREE:
//			/* XXX update info */
//			ts->token_catched = sensor_get(ts->sensor_catched);

			/* stop belts when sensor */
			if(!ts->token_catched){
				belts_mode_set(ts->belts_side, BELTS_MODE_OUT, 0);

				STMCH_DEBUG("token %s free!",
								ts->belts_side==I2C_SIDE_REAR?"REAR":"FRONT");		
				ts->state = TS_STATE_IDLE;
			}
			break;

		case TS_STATE_STOP:
			belts_mode_set(ts->belts_side, BELTS_MODE_OUT, 0);
			ts->state = TS_STATE_IDLE;
			break;
			
		case TS_STATE_PUSH_L:
		case TS_STATE_SHOW:
			belts_mode_set(ts->belts_side, BELTS_MODE_RIGHT, ts->speed);
			ts->state = TS_STATE_IDLE;
			break;

		case TS_STATE_PUSH_R:
			belts_mode_set(ts->belts_side, BELTS_MODE_LEFT, ts->speed);
			ts->state = TS_STATE_IDLE;
			break;

		case TS_STATE_OUT:
			belts_mode_set(ts->belts_side, BELTS_MODE_OUT, ts->speed);
			ts->state = TS_STATE_IDLE;
			break;

		default:
			belts_mode_set(ts->belts_side, BELTS_MODE_OUT, 0);
			ts->state = TS_STATE_IDLE;
			break;
	}
}

static void state_do_token_systems(void)
{
	/* manage systems */
	token_system_manage(&slavedspic.ts[I2C_SIDE_FRONT]);
	token_system_manage(&slavedspic.ts[I2C_SIDE_REAR]);
}

/* state machines */
void state_machines(void)
{
	state_do_init();
	state_do_token_systems();
	state_do_mirror_pos();
}

void state_init(void)
{
	mainboard_command.mode = INIT;

	token_system_init(&slavedspic.ts[I2C_SIDE_FRONT], BELTS_SIDE_FRONT, 
							S_FRONT_TOKEN_STOP, S_FRONT_TOKEN_CATCHED);

	token_system_init(&slavedspic.ts[I2C_SIDE_REAR], BELTS_SIDE_REAR, 
							S_REAR_TOKEN_STOP, S_REAR_TOKEN_CATCHED);
}

