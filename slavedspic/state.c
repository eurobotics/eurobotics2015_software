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
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2012)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  state.c,v 1.4 2009/05/27 20:04:07 zer0 Exp.
 */


#include <math.h>
#include <string.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>

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
#define INIT					I2C_SLAVEDSPIC_MODE_INIT
#define POWER_OFF				I2C_SLAVEDSPIC_MODE_POWER_OFF
#define STANDS_BLADE			I2C_SLAVEDSPIC_MODE_STANDS_BLADE
#define STANDS_CLAMP			I2C_SLAVEDSPIC_MODE_STANDS_CLAMP
#define STANDS_ELEVATOR			I2C_SLAVEDSPIC_MODE_STANDS_ELEVATOR
#define STANDS_TOWER_CLAMPS		I2C_SLAVEDSPIC_MODE_STANDS_TOWER_CLAMPS
#define CUP_CLAMP_POPCORN_DOOR	I2C_SLAVEDSPIC_MODE_CUP_CLAMP_POPCORN_DOOR
#define POPCORN_TRAY			I2C_SLAVEDSPIC_MODE_POPCORN_TRAY
#define POPCORN_RAMPS			I2C_SLAVEDSPIC_MODE_POPCORN_RAMPS
#define CUP_CLAMP_FRONT			I2C_SLAVEDSPIC_MODE_CUP_CLAMP_FRONT
#define CUP_HOLDER_FRONT		I2C_SLAVEDSPIC_MODE_CUP_HOLDER_FRONT

#define POPCORN_SYSTEM 			I2C_SLAVEDSPIC_MODE_POPCORN_SYSTEM
#define STANDS_SYSTEM			I2C_SLAVEDSPIC_MODE_STANDS_SYSTEM



static struct i2c_cmd_slavedspic_set_mode mainboard_command;
static volatile uint8_t prev_state;
static volatile uint8_t mode_changed = 0;
uint8_t state_debug = 0;

/* set status value */
void state_set_status(uint8_t val)
{
	uint8_t flags;

	IRQ_LOCK(flags);
	slavedspic.status = val;
	IRQ_UNLOCK(flags);

	STMCH_DEBUG("state = %s", slavedspic.status==I2C_SLAVEDSPIC_STATUS_BUSY? "BUSY":"READY");
}

/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_slavedspic_set_mode *cmd)
{
	prev_state = mainboard_command.mode;
	memcpy(&mainboard_command, cmd, sizeof(mainboard_command));
	//STMCH_DEBUG("%s mode=%d", __FUNCTION__, mainboard_command.mode); FIXME: bloking???

	/* XXX power off mode */
	if (mainboard_command.mode == POWER_OFF) {

		/* stands_exchanger */
//		slavedspic.stands_exchanger.on = 0;
		pwm_mc_set(&gen.pwm_mc_mod2_ch1, 0);
		BRAKE_ON();

		/* ax12 */
		ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x00);

		wait_ms(100);
		pwm_servo_disable();

		state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
	}
	else {
		mode_changed = 1;
	}
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

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	state_init();
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

  	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}


/**
 * *************** simple actuators modes ***********
 */

/* set stands_blade mode */
void state_do_stands_blade_mode(void)
{
	uint8_t err = 0;

	/* return if no update */
	if (!state_check_update(STANDS_BLADE))
		return;

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/*** LEFT STANDS_BLADE */
	if(mainboard_command.stands_blade.type == I2C_STANDS_BLADE_TYPE_LEFT) {
		if(stands_blade_set_mode(&slavedspic.stands_blade_l, mainboard_command.stands_blade.mode, mainboard_command.stands_blade.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		err = stands_blade_wait_end(&slavedspic.stands_blade_l);
	}	

	/*** RIGHT STANDS_BLADE */
	else if(mainboard_command.stands_blade.type == I2C_STANDS_BLADE_TYPE_RIGHT) {
		if(stands_blade_set_mode(&slavedspic.stands_blade_r, mainboard_command.stands_blade.mode, mainboard_command.stands_blade.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		err = stands_blade_wait_end(&slavedspic.stands_blade_r);
	}	

	if(err & END_BLOCKING)
		STMCH_DEBUG("STANDS_BLADE mode ends with BLOCKING");

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set stands_clamp mode */
void state_do_stands_clamp_mode(void)
{
	/* return if no update */
	if (!state_check_update(STANDS_CLAMP))
		return;

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/*** LEFT STANDS_CLAMP */
	if(mainboard_command.stands_clamp.type == I2C_STANDS_CLAMP_TYPE_LEFT) {
		if(stands_clamp_set_mode(&slavedspic.stands_clamp_l, mainboard_command.stands_clamp.mode, mainboard_command.stands_clamp.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());
	}	

	/*** RIGHT STANDS_CLAMP */
	else if(mainboard_command.stands_clamp.type == I2C_STANDS_CLAMP_TYPE_RIGHT) {
		if(stands_clamp_set_mode(&slavedspic.stands_clamp_r, mainboard_command.stands_clamp.mode, mainboard_command.stands_clamp.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());
	}	

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set stands_elevator mode */
void state_do_stands_elevator_mode(void)
{
	uint8_t err = 0;

	/* return if no update */
	if (!state_check_update(STANDS_ELEVATOR))
		return;

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/*** LEFT STANDS_ELEVATOR */
	if(mainboard_command.stands_elevator.type == I2C_STANDS_ELEVATOR_TYPE_LEFT) {
		if(stands_elevator_set_mode(&slavedspic.stands_elevator_l, mainboard_command.stands_elevator.mode, mainboard_command.stands_elevator.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		err = stands_elevator_wait_end(&slavedspic.stands_elevator_l);
	}	

	/*** RIGHT STANDS_ELEVATOR */
	else if(mainboard_command.stands_elevator.type == I2C_STANDS_ELEVATOR_TYPE_RIGHT) {
		if(stands_elevator_set_mode(&slavedspic.stands_elevator_r, mainboard_command.stands_elevator.mode, mainboard_command.stands_elevator.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		err = stands_elevator_wait_end(&slavedspic.stands_elevator_r);
	}	

	if(err & END_BLOCKING)
		STMCH_DEBUG("STANDS_ELEVATOR mode ends with BLOCKING");

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set stands_tower_clamps mode */
void state_do_stands_tower_clamps_mode(void)
{
	/* return if no update */
	if (!state_check_update(STANDS_TOWER_CLAMPS))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set stands_tower_clamps mode */
	if(stands_tower_clamps_set_mode(&slavedspic.stands_tower_clamps, mainboard_command.stands_tower_clamps.mode, mainboard_command.stands_tower_clamps.offset))
		STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set cup_clamp_popcorn_door mode */
void state_do_cup_clamp_popcorn_door_mode(void)
{
	uint8_t err = 0;

	/* return if no update */
	if (!state_check_update(CUP_CLAMP_POPCORN_DOOR))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set cup_clamp_popcorn_door mode */
	if(mainboard_command.cup_clamp_popcorn_door.type == I2C_CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT) {
		if(cup_clamp_popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_l, mainboard_command.cup_clamp_popcorn_door.mode, mainboard_command.cup_clamp_popcorn_door.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		err = stands_elevator_wait_end(&slavedspic.stands_elevator_l);
	}
	else if(mainboard_command.cup_clamp_popcorn_door.type == I2C_CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT) {
		if(cup_clamp_popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_r, mainboard_command.cup_clamp_popcorn_door.mode, mainboard_command.cup_clamp_popcorn_door.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		err = stands_elevator_wait_end(&slavedspic.stands_elevator_r);
	}

	if(err & END_BLOCKING)
		STMCH_DEBUG("CUP_CLAMP_POPCORN_DOOR mode ends with BLOCKING");

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set popcorn_tray mode */
void state_do_popcorn_tray_mode(void)
{
	/* return if no update */
	if (!state_check_update(POPCORN_TRAY))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set popcorn_tray mode */
	if(popcorn_tray_set_mode(&slavedspic.popcorn_tray, mainboard_command.popcorn_tray.mode, mainboard_command.popcorn_tray.offset))
		STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set popcorn_ramps mode */
void state_do_popcorn_ramps_mode(void)
{
	/* return if no update */
	if (!state_check_update(POPCORN_RAMPS))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set popcorn_ramps mode */
	if(popcorn_ramps_set_mode(&slavedspic.popcorn_ramps, mainboard_command.popcorn_ramps.mode, mainboard_command.popcorn_ramps.offset))
		STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

	popcorn_ramps_wait_end(&slavedspic.popcorn_ramps);

	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set cup_clamp_front mode */
void state_do_cup_clamp_front_mode(void)
{
	/* return if no update */
	if (!state_check_update(CUP_CLAMP_FRONT))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set cup_clamp_front mode */
	if(cup_clamp_front_set_mode(&slavedspic.cup_clamp_front, mainboard_command.cup_clamp_front.mode, mainboard_command.cup_clamp_front.offset))
		STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

	cup_clamp_front_wait_end(&slavedspic.cup_clamp_front);

	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}


/* set cup_holder_front mode */
void state_do_cup_holder_front_mode(void)
{
	/* return if no update */
	if (!state_check_update(CUP_HOLDER_FRONT))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set cup_holder_front mode */
	if(cup_holder_front_set_mode(&slavedspic.cup_holder_front, mainboard_command.cup_holder_front.mode, mainboard_command.cup_holder_front.offset))
		STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

	cup_holder_front_wait_end(&slavedspic.cup_holder_front);

	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}


/**
 * *************** multiple actuators modes ***********
 */

/* do popcorn_system */
void popcorn_system_init(popcorn_system_t *ps)
{
	ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
	ps->mode_changed = 0;
	ps->mode_rqst = 0;
	ps->cup_front_catched = 0;
	ps->cup_rear_catched = 0;
	ps->machine_popcorns_catched = 0;
}

void do_cup_front_catch_and_drop(popcorn_system_t *ps)
{
	static microseconds us = 0;
	static int substate = 1;
	static int tries = 0;

	switch(substate)
	{
		case 1:
			cup_clamp_front_set_mode(&slavedspic.cup_clamp_front, CUP_CLAMP_FRONT_MODE_CUP_LOCKED, 0);
			substate = 2;
			tries++;

			break;

		case 2:
			if(cup_clamp_front_test_traj_end(&slavedspic.cup_clamp_front) & (END_NEAR|END_TRAJ)) {
				substate = 3;
				tries = 0;
			}
			else if(cup_clamp_front_test_traj_end(&slavedspic.cup_clamp_front) & (END_TIME|END_BLOCKING)) {
				if(tries < 3)
					substate = 1;
				else if(tries >= 3) {
					STMCH_ERROR("%s cup_clamp_front BLOCKED!!");
					cup_clamp_front_set_mode(&slavedspic.cup_clamp_front, CUP_CLAMP_FRONT_MODE_OPEN, 0);
					goto exit_do_cup_front_catch_and_drop;
				}
			}

			break;

		case 3:
			cup_holder_front_set_mode(&slavedspic.cup_holder_front, CUP_HOLDER_FRONT_MODE_CUP_HOLD, 0);
			substate = 4;
			tries++;

			break;

		case 4:
			if(cup_holder_front_test_traj_end(&slavedspic.cup_holder_front) & (END_NEAR|END_TRAJ)) {
				us = time_get_us2();
				substate = 5;
				tries = 0;
			}
			else if(cup_holder_front_test_traj_end(&slavedspic.cup_holder_front) & (END_TIME|END_BLOCKING)) {
				if(tries < 3)
					substate = 3;
				else if(tries >= 3) {
					STMCH_ERROR("%s cup_holder_front BLOCKED!!");
					cup_holder_front_set_mode(&slavedspic.cup_holder_front, CUP_HOLDER_FRONT_MODE_HIDE, 0);
					goto exit_do_cup_front_catch_and_drop;
				}
			}

			break;
		case 5:
			if(time_get_us2() - us > 500000L)
				ps->cup_front_catched = 1;
				goto exit_do_cup_front_catch_and_drop;
			
			break;

		default:
			goto exit_do_cup_front_catch_and_drop;
	}

	return;

exit_do_cup_front_catch_and_drop:
		ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
		substate = 1;
		tries = 0;
}

void do_cup_front_release(popcorn_system_t *ps)
{
	static int substate = 1;
	static int tries = 0;

	switch(substate)
	{
		case 1:
			cup_holder_front_set_mode(&slavedspic.cup_holder_front, CUP_HOLDER_FRONT_MODE_HIDE, 0);
			substate = 2;
			tries++;

			break;

		case 2:
			if(cup_holder_front_test_traj_end(&slavedspic.cup_holder_front) & (END_NEAR|END_TRAJ)) {
				substate = 3;
				tries = 0;
			}
			else if(cup_holder_front_test_traj_end(&slavedspic.cup_holder_front) & (END_TIME|END_BLOCKING)) {
				if(tries < 3)
					substate = 1;
				else if(tries >= 3) {
					STMCH_ERROR("%s cup_holder_front BLOCKED!!");
					cup_holder_front_set_mode(&slavedspic.cup_holder_front, CUP_HOLDER_FRONT_MODE_CUP_HOLD, 0);
					goto exit_do_cup_front_release;
				}
			}

			break;

		case 3:
			cup_clamp_front_set_mode(&slavedspic.cup_clamp_front, CUP_CLAMP_FRONT_MODE_OPEN, 0);
			substate = 4;
			tries++;

			break;

		case 4:
			if(cup_clamp_front_test_traj_end(&slavedspic.cup_clamp_front) & (END_NEAR|END_TRAJ)) {
				ps->cup_front_catched = 0;
				goto exit_do_cup_front_release;
			}
			else if(cup_clamp_front_test_traj_end(&slavedspic.cup_clamp_front) & (END_TIME|END_BLOCKING)) {
				if(tries < 3)
					substate = 3;
				else if(tries >= 3) {
					STMCH_ERROR("%s cup_clamp_front BLOCKED!!");
					cup_clamp_front_set_mode(&slavedspic.cup_clamp_front, CUP_CLAMP_FRONT_MODE_CUP_LOCKED, 0);
					goto exit_do_cup_front_release;
				}
			}

			break;

		default:
			goto exit_do_cup_front_release;
	}

	return;

exit_do_cup_front_release:
		ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
		substate = 1;
		tries = 0;
}

void do_cup_rear_open(popcorn_system_t *ps)
{
	static int substate = 0;
	static int tries = 0;
	static int cup_clamp_l_last_mode = 0;
	static int cup_clamp_r_last_mode = 0;
				
	switch(substate)
	{
		case 0:
			cup_clamp_l_last_mode = slavedspic.cup_clamp_popcorn_door_l.mode;
			cup_clamp_r_last_mode = slavedspic.cup_clamp_popcorn_door_r.mode;
			substate = 1;

			break;

		case 1:
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_r, CUP_CLAMP_MODE_OPEN, 0);
			substate = 2;
			tries++;

			break;

		case 2:
			if(cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_r) & (END_NEAR|END_TRAJ)) {
				substate = 3;
				tries = 0;
			}
			else if(cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_r) & (END_TIME|END_BLOCKING)) {
				if(tries < 3)
					substate = 1;
				else if(tries >= 3) {
					STMCH_ERROR("%s cup_clamp_popcorn_door_r BLOCKED!!");
					cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_r, cup_clamp_r_last_mode, 0);
					goto exit_do_cup_rear_open;
				}
			}

			break;

		case 3:
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_l, CUP_CLAMP_MODE_OPEN, 0);
			substate = 4;
			tries++;

			break;

		case 4:
			if(cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l) & (END_NEAR|END_TRAJ)) {
				goto exit_do_cup_rear_open;
				ps->cup_rear_catched = 0;
			}
			else if(cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l) & (END_TIME|END_BLOCKING)) {
				if(tries < 3)
					substate = 3;
				else if(tries >= 3) {
					STMCH_ERROR("%s cup_clamp_popcorn_door_l BLOCKED!!");
					cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_l, cup_clamp_l_last_mode, 0);
					goto exit_do_cup_rear_open;
				}
			}

			break;

		default:
			goto exit_do_cup_rear_open;
	}

	return;

exit_do_cup_rear_open:
		ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
		substate = 0;
		tries = 0;
}

void do_cup_rear_catch(popcorn_system_t *ps)
{
	static int substate = 1;
	static int tries = 0;
	static int cup_clamp_l_last_mode = 0;
	static int cup_clamp_r_last_mode = 0;

	switch(substate)
	{
		case 0:
			if(&slavedspic.cup_clamp_popcorn_door_l.mode == CUP_CLAMP_MODE_HIDE ||
						&slavedspic.cup_clamp_popcorn_door_r.mode == CUP_CLAMP_MODE_HIDE) {
				STMCH_ERROR("OPEN BEFORE YOU TRY TO CATCH!!");
				goto exit_do_cup_rear_catch;
			}

			cup_clamp_l_last_mode = slavedspic.cup_clamp_popcorn_door_l.mode;
			cup_clamp_r_last_mode = slavedspic.cup_clamp_popcorn_door_r.mode;
			substate = 1;

			break;

		case 1:
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_l, CUP_CLAMP_MODE_LOCKED, 0);
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_r, CUP_CLAMP_MODE_LOCKED, 0);
			substate = 2;
			tries++;

			break;

		case 2:
			if((cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l) & (END_NEAR|END_TRAJ)) &&
					(cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_r) & (END_NEAR|END_TRAJ))) {
				ps->cup_rear_catched = 1;
				goto exit_do_cup_rear_catch;
			}
			else if((cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l) & (END_TIME|END_BLOCKING)) ||
					(cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_r) & (END_TIME|END_BLOCKING))) {
				if(tries < 3)
					substate = 1;
				else if(tries >= 3) {
					if(cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l) & (END_TIME|END_BLOCKING))
						STMCH_ERROR("%s cup_clamp_popcorn_door_l BLOCKED!!");
					else
						STMCH_ERROR("%s cup_clamp_popcorn_door_r BLOCKED!!");

					cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_l, cup_clamp_l_last_mode, 0);
					cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_r, cup_clamp_r_last_mode, 0);
					goto exit_do_cup_rear_catch;
				}
			}

			break;

		default:
			goto exit_do_cup_rear_catch;
	}

	return;

exit_do_cup_rear_catch:
		ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
		substate = 0;
		tries = 0;
}

void popcorn_system_manage(popcorn_system_t *ps)
{
	/* update mode */
	if(ps->mode_changed){
		ps->mode_changed = 0;
		ps->mode = ps->mode_rqst;
		STMCH_DEBUG("%s mode=%d", __FUNCTION__, ps->mode);
	}

	switch(ps->mode)
	{
		case I2C_SLAVEDSPIC_MODE_PS_IDLE:
			break;

		case I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_CATCH_AND_DROP:
			do_cup_front_catch_and_drop(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_RELEASE:
			do_cup_front_release(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_OPEN:
			do_cup_rear_open(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_CATCH:
			do_cup_rear_catch(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_RELEASE:
//			if(!flag_clamp_door_l_open && !flag_clamp_door_r_open) {
//				popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_r, CUP_CLAMP_MODE_OPEN, 0);
//				flag_clamp_door_r_open = 2;
//				us = time_get_us2();
//			}
//			else if(!flag_clamp_door_l_open && flag_clamp_door_r_open) {
//				if(time_get_us2() - us > 300000L) {
//					popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_l, CUP_CLAMP_MODE_OPEN, 0);
//					flag_clamp_door_l_open = 2;
//				}
//			}
//			else {
//				popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_l, CUP_CLAMP_MODE_OPEN, 0);
//				popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_r, CUP_CLAMP_MODE_OPEN, 0);
//				flag_clamp_door_l_open = 2;
//				flag_clamp_door_r_open = 2;
//			}
//
//			if((cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l) & (END_NEAR|END_TRAJ)) &&
//					(cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_r) & (END_NEAR|END_TRAJ))) {
//				ps->cup_rear_catched = 0;
//				ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
//			}

			break;

		case I2C_SLAVEDSPIC_MODE_PS_MACHINES_READY:
//			if(!flag_tray_open) {
//				popcorn_tray_set_mode(&slavedspic.popcorn_tray, POPCORN_TRAY_MODE_OPEN, 0);
//				us = time_get_us2();
//				flag_ramps_open = 0;
//				flag_tray_open = 1;
//			}
//
//			if((time_get_us2() - us > 300000L) && flag_tray_open && !flag_ramps_open) {
//				popcorn_ramps_set_mode(&slavedspic.popcorn_ramps, POPCORN_RAMPS_MODE_OPEN, 0);
//				flag_ramps_open = 1;
//			}
//
//			if((popcorn_ramps_test_traj_end(&slavedspic.popcorn_ramps) & (END_NEAR|END_TRAJ)) && flag_ramps_open)
//				ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;

			break;

		case I2C_SLAVEDSPIC_MODE_PS_MACHINES_HARVEST:
//			if(!flag_tray_open || !flag_ramps_open) {
//				ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
//				return;
//			}
//			else
//			{
//				popcorn_ramps_set_mode(&slavedspic.popcorn_ramps, POPCORN_RAMPS_MODE_HARVEST, 0);
//
//				if((popcorn_ramps_test_traj_end(&slavedspic.popcorn_ramps) & (END_NEAR|END_TRAJ))) {
//					us = time_get_us2();
//					ps->machine_popcorns_catched = 1;
//				}
//
//				if((time_get_us2() - us > 500000L) && ps->machine_popcorns_catched)
//					ps->mode = I2C_SLAVEDSPIC_MODE_PS_MACHINES_END;
//			}

			break;

		case I2C_SLAVEDSPIC_MODE_PS_MACHINES_END:
//			popcorn_ramps_set_mode(&slavedspic.popcorn_ramps, POPCORN_RAMPS_MODE_HIDE, 0);
//
//			if(popcorn_ramps_test_traj_end(&slavedspic.popcorn_ramps) & (END_NEAR|END_TRAJ)) {
//				flag_ramps_open = 0;
//				popcorn_tray_set_mode(&slavedspic.popcorn_tray, POPCORN_TRAY_MODE_CLOSE, 0);
//				flag_tray_open = 0;
//				ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
//			}

			break;

		case I2C_SLAVEDSPIC_MODE_PS_STOCK_DROP:
//			if(!flag_clamp_door_l_open && !flag_clamp_door_r_open) {
//				popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_r, POPCORN_DOOR_MODE_OPEN, 0);
//				flag_clamp_door_r_open = 2;
//				us = time_get_us2();
//			}
//			else if(!flag_clamp_door_l_open && flag_clamp_door_r_open) {
//				if(time_get_us2() - us > 300000L) {
//					popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_l, POPCORN_DOOR_MODE_OPEN, 0);
//					flag_clamp_door_l_open = 2;
//				}
//			}
//			else {
//				popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_l, POPCORN_DOOR_MODE_OPEN, 0);
//				popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_r, POPCORN_DOOR_MODE_OPEN, 0);
//				flag_clamp_door_l_open = 2;
//				flag_clamp_door_r_open = 2;
//			}
//
//			if((cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l) & (END_NEAR|END_TRAJ)) &&
//					(cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_r) & (END_NEAR|END_TRAJ))) {
//				ps->cup_rear_catched = 0;
//				ps->machine_popcorns_catched = 0;
//				us = time_get_us2();
//			}
//
//			if((time_get_us2() - us > 1000000L) && !ps->machine_popcorns_catched)
//				ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;

			break;

		case I2C_SLAVEDSPIC_MODE_PS_STOCK_END:
//			if(ps->cup_rear_catched) {
//				ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
//				return;
//			}
//				
//			popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_l, POPCORN_DOOR_MODE_OPEN, 0);
//
//			if((cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l) & (END_NEAR|END_TRAJ))) {
//				flag_clamp_door_l_open = 0;				
//				popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_r, POPCORN_DOOR_MODE_OPEN, 0);
//			}
//
//			if((cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l) & (END_NEAR|END_TRAJ)) && !flag_clamp_door_l_open) {
//				flag_clamp_door_r_open = 0;
//				ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
//			}

			break;

		default:
			ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
			break;
	}
}

void state_do_popcorn_system(void)
{
	popcorn_system_manage(&slavedspic.ps);
}

/* do stands_system */
void stands_system_init(stands_system_t *ss, uint8_t stand_sensor, stands_blade_t *blade, stands_clamp_t *clamp, stands_elevator_t *elevator)
{
	ss->mode = I2C_SLAVEDSPIC_MODE_SS_IDLE;
	ss->mode_changed = 0;
	ss->mode_rqst = 0;
	ss->stored_stands = 0;
	ss->stand_sensor = stand_sensor;
	ss->blade = blade;
	ss->clamp = clamp;
	ss->elevator = elevator;
}

void stands_system_manage(stands_system_t *ss, stands_system_t *ss_slave)
{
	/* update mode */
	if(ss->mode_changed){
		ss->mode_changed = 0;
		ss->mode = ss->mode_rqst;
		STMCH_DEBUG("%s mode=%d", __FUNCTION__, ss->mode);
	}

	switch(ss->mode)
	{
		case I2C_SLAVEDSPIC_MODE_SS_IDLE:
			break;

		case I2C_SLAVEDSPIC_MODE_SS_HIDE:
			//si el retenedor está abierto:
				//cerrar retenedor

			//cerrar pinza
			//Cuando la pinza y el retenedor estén cerrados:
				//Si el número de stands es menor que 4:
					//subir elevador
					//Cuando el elevador esté subido:
						//Esconder brazo
						//cuando el brazo esté escondido
						
				//si no:
					//dejar elevador abajo con un offset para que los stands no arrastren
					//cuando el elevador esté posicionado:
						//Mover el brazo a la posición de empujar stand

			//cuando la pinza, el retenedor, el elevador y el brazo estén "escondidos"
				//cambiar modo a idle

			break;

		case I2C_SLAVEDSPIC_MODE_SS_HARVESTING:
			//si hay 4 stands en la columna:
				//cambiar estado a idle
				//return

			//colocar brazo
			//colocar brazo esclavo
			//cuando los brazos estén colocados después de la inicialización:
				//si el sensor lateral detecta objeto:
					//mover el brazo para empujar stand

			//cuando el brazo esté colocado después de la inicialización
				//abrir pinza
				//cuando la pinza esté abierta:
					//bajar elevador
					//cuando el elevador esté bajado:
						//sumar 1 stand almacenado
						//cambiar a modo hide

			break;

		case I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT:
			//si hay stands en el esclavo, ponerlo en modo build_spotlight

			//si el sistema coincide con el lado donde se va a construir:
				//si no hay más stands en la otra torre ni disponibles para coger:
					//cambiar a release spotlight
				//si no:
					//si el retenedor está cerrado
						//subir torre
					//cuando la torre esté subida:
						//si no hay brazos tras la columna secundaria:
							//mover los dos brazos tras la columna secundaria
					//cuando la torre esté subida y esté activado el flag de stand preparado para almacenar:
						//presionar stand con un brazo
					//cuando la torre esté subida y el brazo presionando:
						//abrir pinza
						//cuando la pinza esté abierta:
							//bajar elevador
							//cuando el elevador esté bajado:
								//sumar 1 stand almacenado
								//si quedan stands en la otra torre:
									//cerrar pinza

			//si es la otra columna:
				//si hay más de 0 stands y no hay ningún brazo detrás de la columna:
					//subir elevador

				//si no:
					//si el número de stands en la columna es mayor que 0:
						//posicionar carro
						//cuando el elevador, el carro y los brazos estén posicionados:
							//bajar torre
						//cuando la torre esté bajada:
							//soltar pinza
						//cuando la pinza esté abierta:
							//subir elevador a media altura
						//cuando el elevador esté a media altura:
							//cerrar pinza
						//cuando la pinza esté cerrada y el elevador a media altura:
							//dar offset al elevador
						//cuando el elevador esté posicionado a media altura con offset y los brazos estén escondidos:
							//desplazar stand con el carro
						//cuando el carro esté en medio:
							//activar flag de stand listo para almacenar
							//restar 1 stand almacenado en torre secundaria
					//si no:
						//cambiar a modo hide

			break;

		case I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT:
			//si el spotlight no está almacenado en el sistema:
				//cambiar a modo idle
				//return

			//si el brazo izquierdo está presionando el stand:
				//mover el brazo izquierdo a la posición central
			//si el brazo derecho está presionando el stand:
				//mover el brazo derecho a la posición central

			//abrir pinza
			//abrir retenedor
			//cuando la pinza, el retenedor y los brazos se hayan posicionado:
				//cambiar a estado idle

			break;

		default:
			ss->mode = I2C_SLAVEDSPIC_MODE_SS_IDLE;
			break;
	}
}

void state_do_stands_systems(void)
{
	stands_system_manage(&slavedspic.ss[I2C_SIDE_LEFT], &slavedspic.ss[I2C_SIDE_RIGHT]);
	stands_system_manage(&slavedspic.ss[I2C_SIDE_RIGHT], &slavedspic.ss[I2C_SIDE_LEFT]);
}


#if 0

/* set infos */
void state_do_set_infos(void)
{
	if (!state_check_update(SET_INFOS))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	if(mainboard_command.set_infos.nb_goldbars_in_boot >= 0)
		slavedspic.nb_goldbars_in_boot = mainboard_command.set_infos.nb_goldbars_in_boot;
	if(mainboard_command.set_infos.nb_goldbars_in_mouth >= 0)
		slavedspic.nb_goldbars_in_mouth = mainboard_command.set_infos.nb_goldbars_in_mouth;
	if(mainboard_command.set_infos.nb_coins_in_boot >= 0)
		slavedspic.nb_coins_in_boot = mainboard_command.set_infos.nb_coins_in_boot;
	if(mainboard_command.set_infos.nb_coins_in_mouth >= 0)
		slavedspic.nb_coins_in_mouth = mainboard_command.set_infos.nb_coins_in_mouth;

	STMCH_DEBUG("nb_goldbars_in_boot = %d", slavedspic.nb_goldbars_in_boot);
	STMCH_DEBUG("nb_goldbars_in_mouth = %d", slavedspic.nb_goldbars_in_mouth);
	STMCH_DEBUG("nb_coins_in_boot = %d", slavedspic.nb_coins_in_boot);
	STMCH_DEBUG("nb_coins_in_mouth = %d", slavedspic.nb_coins_in_mouth);

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

#endif

/* state machines */
void state_machines(void)
{
	state_do_init();

	/* simple actuator modes */
	state_do_stands_blade_mode();
	state_do_stands_clamp_mode();
	state_do_stands_elevator_mode();
	state_do_stands_tower_clamps_mode();
	state_do_cup_clamp_popcorn_door_mode();
	state_do_popcorn_tray_mode();
	state_do_popcorn_ramps_mode();
	state_do_cup_clamp_front_mode();
	state_do_cup_holder_front_mode();

	/* multiple actuators modes */
	state_do_popcorn_system();
	state_do_stands_systems();

#if 0
	state_do_set_infos();
#endif
}

void state_init(void)
{
	mainboard_command.mode = INIT;

	/* enable pwm servos */
	pwm_servo_enable();

	/* enable ax12 torque */
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0xFF);

	/* start positions */

	/* close gadgets */
	stands_blade_set_mode(&slavedspic.stands_blade_l, STANDS_BLADE_MODE_HIDE_LEFT, 0);
	stands_blade_set_mode(&slavedspic.stands_blade_r, STANDS_BLADE_MODE_HIDE_RIGHT, 0);

	stands_clamp_set_mode(&slavedspic.stands_clamp_l, STANDS_CLAMP_MODE_OPEN, 0);
	stands_clamp_set_mode(&slavedspic.stands_clamp_r, STANDS_CLAMP_MODE_OPEN, 0);

	stands_elevator_set_mode(&slavedspic.stands_elevator_l, STANDS_ELEVATOR_MODE_UP, 0);
	stands_elevator_set_mode(&slavedspic.stands_elevator_r, STANDS_ELEVATOR_MODE_UP, 0);

	stands_tower_clamps_set_mode(&slavedspic.stands_tower_clamps, STANDS_TOWER_CLAMPS_MODE_LOCK, 0);

	cup_clamp_popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_l, CUP_CLAMP_MODE_HIDE, 0);
	cup_clamp_popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_r, CUP_CLAMP_MODE_HIDE, 0);

	popcorn_tray_set_mode(&slavedspic.popcorn_tray, POPCORN_TRAY_MODE_CLOSE, 0);
	popcorn_ramps_set_mode(&slavedspic.popcorn_ramps, POPCORN_RAMPS_MODE_HIDE, 0);

	cup_clamp_front_set_mode(&slavedspic.cup_clamp_front, CUP_CLAMP_FRONT_MODE_OPEN, 0);
	cup_holder_front_set_mode(&slavedspic.cup_holder_front, CUP_HOLDER_FRONT_MODE_HIDE, 0);

	BRAKE_OFF();
	//slavedspic.stands_exchanger.on = 1;
	//stands_exchanger_calibrate();
	//Desplazar carro junto a la torre secundaria

	/* systems init */
	popcorn_system_init(&slavedspic.ps);
	stands_system_init(&slavedspic.ss[I2C_SIDE_LEFT], S_STAND_INSIDE_L, &slavedspic.stands_blade_l, &slavedspic.stands_clamp_l, &slavedspic.stands_elevator_l);
	stands_system_init(&slavedspic.ss[I2C_SIDE_RIGHT], S_STAND_INSIDE_R, &slavedspic.stands_blade_r, &slavedspic.stands_clamp_r, &slavedspic.stands_elevator_r);
}

