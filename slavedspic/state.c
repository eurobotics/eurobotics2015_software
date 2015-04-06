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
 *  Javier Balias Santos <javier@arc-robots.org>
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
#define SET_INFOS				I2C_SLAVEDSPIC_MODE_SET_INFOS



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
		slavedspic.stands_exchanger.on = 0;
		pwm_mc_set(&gen.pwm_mc_mod2_ch1, 0);
		BRAKE_ON();

		/* ax12 */
		ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x00);

		wait_ms(100);
		pwm_servo_disable();

		state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
	}
	else if (mainboard_command.mode == I2C_SLAVEDSPIC_MODE_POPCORN_SYSTEM) {
		slavedspic.ps.mode_rqst = mainboard_command.popcorn_system.mode;
		slavedspic.ps.mode_changed = 1;
	}
	else if (mainboard_command.mode == I2C_SLAVEDSPIC_MODE_STANDS_SYSTEM) {
		slavedspic.ss[mainboard_command.stands_system.side].mode_rqst = mainboard_command.stands_system.mode;
		slavedspic.ss[mainboard_command.stands_system.side].blade_angle = mainboard_command.stands_system.blade_angle;
		slavedspic.ss[mainboard_command.stands_system.side].mode_changed = 1;	
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

#ifdef SIMPLE_ACTUATORS
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
	uint8_t err = 0;

	/* return if no update */
	if (!state_check_update(STANDS_TOWER_CLAMPS))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set stands_tower_clamps mode */
	if(stands_tower_clamps_set_mode(&slavedspic.stands_tower_clamps, mainboard_command.stands_tower_clamps.mode, mainboard_command.stands_tower_clamps.offset))
		STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

	err = stands_tower_clamps_wait_end(&slavedspic.stands_tower_clamps);

	if(err & END_BLOCKING)
		STMCH_DEBUG("TOWER CLAMPS mode ends with BLOCKING");

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
	uint8_t err = 0;

	/* return if no update */
	if (!state_check_update(POPCORN_RAMPS))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set popcorn_ramps mode */
	if(popcorn_ramps_set_mode(&slavedspic.popcorn_ramps, mainboard_command.popcorn_ramps.mode, mainboard_command.popcorn_ramps.offset))
		STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

	err = popcorn_ramps_wait_end(&slavedspic.popcorn_ramps);

	if(err & END_BLOCKING)
		STMCH_DEBUG("RAMPS mode ends with BLOCKING");

	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set cup_clamp_front mode */
void state_do_cup_clamp_front_mode(void)
{
	uint8_t err;

	/* return if no update */
	if (!state_check_update(CUP_CLAMP_FRONT))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set cup_clamp_front mode */
	if(cup_clamp_front_set_mode(&slavedspic.cup_clamp_front, mainboard_command.cup_clamp_front.mode, mainboard_command.cup_clamp_front.offset))
		STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

	err = cup_clamp_front_wait_end(&slavedspic.cup_clamp_front);

	if(err & END_BLOCKING)
		STMCH_DEBUG("CUP_CLAMP_FRONT mode ends with BLOCKING");

	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}


/* set cup_holder_front mode */
void state_do_cup_holder_front_mode(void)
{
	uint8_t err = 0;

	/* return if no update */
	if (!state_check_update(CUP_HOLDER_FRONT))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set cup_holder_front mode */
	if(cup_holder_front_set_mode(&slavedspic.cup_holder_front, mainboard_command.cup_holder_front.mode, mainboard_command.cup_holder_front.offset))
		STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

	err = cup_holder_front_wait_end(&slavedspic.cup_holder_front);

	if(err & END_BLOCKING)
		STMCH_DEBUG("CUP_HOLDER_FRONT mode ends with BLOCKING");

	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}
#endif

/**
 * *************** multiple actuators modes ***********
 */
#ifdef ACTUATOR_SYSTEMS

/* do popcorn_system */
void popcorn_system_init(popcorn_system_t *ps)
{
	ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
	ps->mode_changed = 0;
	ps->mode_rqst = 0;
	ps->status = STATUS_READY;
}

uint8_t do_cup_front_catch_and_drop(popcorn_system_t *ps)
{
	uint8_t ret = 0;

	switch(ps->substate)
	{
		case SAVE:
			if(slavedspic.cup_clamp_front.mode != CUP_CLAMP_FRONT_MODE_OPEN) {
				STMCH_ERROR("FIRST, OPEN CUP_CLAMP_FRONT!!");
				return STATUS_BLOCKED;
			}

		case CATCH_CUP_FRONT:
			cup_clamp_front_set_mode(&slavedspic.cup_clamp_front, CUP_CLAMP_FRONT_MODE_CUP_LOCKED, 0);
			ps->substate = WAITING_CUP_FRONT_CAUGHT;
			break;

		case WAITING_CUP_FRONT_CAUGHT:
			ret = cup_clamp_front_test_traj_end(&slavedspic.cup_clamp_front);

			if(ret & END_TRAJ)
				ps->substate = RAISE_CUP_FRONT;
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("cup_clamp_front BLOCKED!!");
				return STATUS_BLOCKED;
			}
			break;

		case RAISE_CUP_FRONT:
			cup_holder_front_set_mode(&slavedspic.cup_holder_front, CUP_HOLDER_FRONT_MODE_CUP_HOLD, 0);
			ps->substate = WAITING_CUP_FRONT_RAISED;
			break;

		case WAITING_CUP_FRONT_RAISED:
			ret = cup_holder_front_test_traj_end(&slavedspic.cup_holder_front);

			if(ret & END_TRAJ) {
				ps->cup_front_catched = 1;
				ps->cup_front_popcorns_harvested = 1;
				return STATUS_DONE;
			}
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("cup_holder_front BLOCKED!!");
				return STATUS_BLOCKED;
			}
			break;
		
		default:
			ps->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_cup_front_release(popcorn_system_t *ps)
{
	uint8_t ret = 0;

	switch(ps->substate)
	{
		case SAVE:
			if(slavedspic.stands_tower_clamps.mode != STANDS_TOWER_CLAMPS_MODE_LOCK) {
				STMCH_ERROR("FIRST, LOCK STANDS_TOWER_CLAMPS!!");
				return STATUS_BLOCKED;
			}

		case DOWN_CUP_FRONT:
			cup_holder_front_set_mode(&slavedspic.cup_holder_front, CUP_HOLDER_FRONT_MODE_READY, 0);
			ps->substate = WAITING_CUP_FRONT_DOWNED;

			break;

		case WAITING_CUP_FRONT_DOWNED:
			ret = cup_holder_front_test_traj_end(&slavedspic.cup_holder_front);

			if(ret & END_TRAJ)
				ps->substate = RELEASE_CUP_FRONT;
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("cup_holder_front BLOCKED!!");
				return STATUS_BLOCKED;
			}

			break;

		case RELEASE_CUP_FRONT:
			cup_clamp_front_set_mode(&slavedspic.cup_clamp_front, CUP_CLAMP_FRONT_MODE_OPEN, 0);
			ps->substate = WAITING_CUP_FRONT_RELEASED;
			break;

		case WAITING_CUP_FRONT_RELEASED:
			ret = cup_clamp_front_test_traj_end(&slavedspic.cup_clamp_front);

			if(ret & END_TRAJ) {
				ps->cup_front_catched = 0;
				return STATUS_DONE;
			}
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("cup_clamp_front BLOCKED!!");
				return STATUS_BLOCKED;
			}
			break;

		default:
			ps->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_cup_front_hide(popcorn_system_t *ps)
{
	uint8_t ret_c = 0;
	uint8_t ret_h = 0;

	switch(ps->substate)
	{
		case SAVE:
			if(ps->cup_front_catched == 1) {
				STMCH_ERROR("FIRST, RELEASE CUP_FRONT!!");
				return STATUS_BLOCKED;
			}

		case HIDE_CUP_FRONT:
			cup_clamp_front_set_mode(&slavedspic.cup_clamp_front, CUP_CLAMP_FRONT_MODE_CUP_LOCKED, 0);
			cup_holder_front_set_mode(&slavedspic.cup_holder_front, CUP_HOLDER_FRONT_MODE_HIDE, 0);
			ps->substate = WAITING_CUP_FRONT_HIDDEN;

			break;

		case WAITING_CUP_FRONT_HIDDEN:
			ret_c = cup_clamp_front_test_traj_end(&slavedspic.cup_clamp_front);
			ret_h = cup_holder_front_test_traj_end(&slavedspic.cup_holder_front);

			if((ret_c & END_TRAJ) && (ret_h & END_TRAJ))
				return STATUS_DONE;
			else if((ret_c & END_BLOCKING) || (ret_h & END_BLOCKING)) {
				if(!(ret_c & END_TRAJ))
					STMCH_ERROR("cup_clamp_front BLOCKED!!");
				else if(!(ret_h & END_TRAJ))
					STMCH_ERROR("cup_holder_front BLOCKED!!");

				return STATUS_BLOCKED;
			}
			break;

		default:
			ps->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_cup_rear_open(popcorn_system_t *ps)
{
	uint8_t ret = 0;
				
	switch(ps->substate)
	{
		case SAVE:

		case OPEN_RIGHT_CUP_REAR:
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_r, CUP_CLAMP_MODE_OPEN, 0);
			ps->substate = WAITING_CUP_REAR_OPENED_RIGHT;
			break;

		case WAITING_CUP_REAR_OPENED_RIGHT:
			ret = cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_r);

			if(ret & (END_NEAR|END_TRAJ)) {
				ps->substate = OPEN_LEFT_CUP_REAR;
			}
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("cup_clamp_popcorn_door_r BLOCKED!!");
				return STATUS_BLOCKED;
			}
			break;

		case OPEN_LEFT_CUP_REAR:
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_l, CUP_CLAMP_MODE_OPEN, 0);
			ps->substate = WAITING_CUP_REAR_OPENED_LEFT;
			break;

		case WAITING_CUP_REAR_OPENED_LEFT:
			ret = cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l);

			if(ret & END_TRAJ) {
				ps->cup_rear_catched = 0;
				return STATUS_DONE;
			}
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("cup_clamp_popcorn_door_l BLOCKED!!");
				return STATUS_BLOCKED;
			}
			break;

		default:
			ps->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_cup_rear_catch(popcorn_system_t *ps)
{
	uint8_t ret_l = 0;
	uint8_t ret_r = 0;

	switch(ps->substate)
	{
		case SAVE:
			if(slavedspic.cup_clamp_popcorn_door_l.mode == CUP_CLAMP_MODE_HIDE ||
					&slavedspic.cup_clamp_popcorn_door_r.mode == CUP_CLAMP_MODE_HIDE) {
				STMCH_ERROR("OPEN BEFORE YOU TRY TO CATCH!!");
				return STATUS_BLOCKED;
			}

		case CATCH_CUP_REAR:
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_l, CUP_CLAMP_MODE_LOCKED, 0);
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_r, CUP_CLAMP_MODE_LOCKED, 0);
			ps->substate = WAITING_CUP_REAR_CAUGHT;
			break;

		case WAITING_CUP_REAR_CAUGHT:
			ret_l = cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l);
			ret_r = cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_r);

			if((ret_l & END_TRAJ) && (ret_r & END_TRAJ)) {
				ps->cup_rear_catched = 1;
				return STATUS_DONE;
			}
			else if((ret_l & END_BLOCKING) || (ret_r & END_BLOCKING)) {
				if(!(ret_l & END_TRAJ))
					STMCH_ERROR("cup_clamp_popcorn_door_l BLOCKED!!");
				else if(!(ret_r & END_TRAJ))
					STMCH_ERROR("cup_clamp_popcorn_door_r BLOCKED!!");

				return STATUS_BLOCKED;
			}
			break;

		default:
			ps->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_cup_rear_release(popcorn_system_t *ps)
{
	uint8_t ret_l = 0;
	uint8_t ret_r = 0;

	switch(ps->substate)
	{
		case SAVE:
			if(ps->cup_rear_catched == 0) {
				STMCH_ERROR("THERE ISN'T ANY CUP_REAR CATCHED!!");
				return STATUS_BLOCKED;
			}

		case RELEASE_CUP_REAR:
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_l, CUP_CLAMP_MODE_OPEN, 0);
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_r, CUP_CLAMP_MODE_OPEN, 0);
			ps->substate = WAITING_CUP_REAR_RELEASED;
			break;

		case WAITING_CUP_REAR_RELEASED:
			ret_l = cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l);
			ret_r = cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_r);

			if((ret_l & END_TRAJ) && (ret_r & END_TRAJ)) {
				ps->cup_rear_catched = 0;
				return STATUS_DONE;
			}
			else if((ret_l & END_BLOCKING) || (ret_r & END_BLOCKING)) {
				if(!(ret_l & END_TRAJ))
					STMCH_ERROR("cup_clamp_popcorn_door_l BLOCKED!!");
				else if(!(ret_r & END_TRAJ))
					STMCH_ERROR("cup_clamp_popcorn_door_r BLOCKED!!");

				return STATUS_BLOCKED;
			}
			break;

		default:
			ps->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_machines_ready(popcorn_system_t *ps)
{
	static microseconds us = 0;
	uint8_t ret = 0;

	switch(ps->substate)
	{
		case SAVE:

		case OPEN_TRAY:
			popcorn_tray_set_mode(&slavedspic.popcorn_tray, POPCORN_TRAY_MODE_OPEN, 0);
			us = time_get_us2();
			ps->substate = WAITING_TRAY_OPENED;
			break;

		case WAITING_TRAY_OPENED:
			if(time_get_us2() - us > 500000L)
				ps->substate = OPEN_RAMPS;
			break;

		case OPEN_RAMPS:
			popcorn_ramps_set_mode(&slavedspic.popcorn_ramps, POPCORN_RAMPS_MODE_OPEN, 0);
			ps->substate = WAITING_RAMPS_OPENED;
			break;

		case WAITING_RAMPS_OPENED:
			ret = popcorn_ramps_test_traj_end(&slavedspic.popcorn_ramps);

			if(ret & END_TRAJ)
				return STATUS_DONE;
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("popcorn_ramps BLOCKED!!");
				return STATUS_BLOCKED;
			}
			break;

		default:
			ps->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_machines_harvest(popcorn_system_t *ps)
{
	static microseconds us = 0;
	uint8_t ret = 0;

	switch(ps->substate)
	{
		case SAVE:
			if(slavedspic.popcorn_ramps.mode != POPCORN_RAMPS_MODE_OPEN) {
				STMCH_ERROR("OPEN BEFORE YOU TRY TO HARVEST!!");
				return STATUS_BLOCKED;
			}
			
		case MOVE_RAMPS:
			popcorn_ramps_set_mode(&slavedspic.popcorn_ramps, POPCORN_RAMPS_MODE_HARVEST, 0);
			ps->substate = WAITING_RAMPS_MOVED;
			break;

		case WAITING_RAMPS_MOVED:
			ret = popcorn_ramps_test_traj_end(&slavedspic.popcorn_ramps);

			if(ret & END_TRAJ) {
				us = time_get_us2();
				ps->substate = WAITING_POPCORNS_HARVESTED;
			}
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("popcorn_ramps BLOCKED!!");
				return STATUS_BLOCKED;
			}
			break;

		case WAITING_POPCORNS_HARVESTED:
			if(time_get_us2() - us > 1500000L) {
				ps->machine_popcorns_catched = 1;
				return STATUS_DONE;
			}
			break;

		default:
			ps->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_machines_end(popcorn_system_t *ps)
{
	uint8_t ret = 0;

	switch(ps->substate)
	{
		case SAVE:
			if(slavedspic.popcorn_tray.mode == POPCORN_TRAY_MODE_CLOSE) {
				STMCH_ERROR("THE SYSTEM IS CLOSED!!");
				return STATUS_BLOCKED;
			}
			
		case CLOSE_RAMPS:
			popcorn_ramps_set_mode(&slavedspic.popcorn_ramps, POPCORN_RAMPS_MODE_HIDE, 0);
			ps->substate = WAITING_RAMPS_CLOSED;
			break;

		case WAITING_RAMPS_CLOSED:
			ret = popcorn_ramps_test_traj_end(&slavedspic.popcorn_ramps);

			if(ret & END_TRAJ)
				ps->substate = CLOSE_TRAY;
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("popcorn_ramps BLOCKED!!");
				return STATUS_BLOCKED;
			}
			break;

		case CLOSE_TRAY:
			popcorn_tray_set_mode(&slavedspic.popcorn_tray, POPCORN_TRAY_MODE_CLOSE, 0);
			return STATUS_DONE;
			break;

		default:
			ps->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_stock_drop(popcorn_system_t *ps)
{
	static microseconds us = 0;
	uint8_t ret_l = 0;
	uint8_t ret_r = 0;

	switch(ps->substate)
	{
		case SAVE:
			if(slavedspic.cup_clamp_popcorn_door_r.mode == CUP_CLAMP_MODE_HIDE) {
				STMCH_ERROR("FIRST, TRY TO OPEN CUP_CLAMP_REAR!!");
				return STATUS_BLOCKED;
			}
			else if(ps->cup_rear_catched == 0 && ps->machine_popcorns_catched == 0 && ps->cup_front_popcorns_harvested == 0) {
				STMCH_ERROR("NO STORED POPCORNS!!");
				return STATUS_BLOCKED;
			}
			
		case OPEN_DOORS:
			popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_l, POPCORN_DOOR_MODE_OPEN, 0);
			popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_r, POPCORN_DOOR_MODE_OPEN, 0);
			ps->substate = WAITING_DOORS_OPENED;
			break;

		case WAITING_DOORS_OPENED:
			ret_l = cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l);
			ret_r = cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_r);

			if((ret_l & END_TRAJ) && (ret_r & END_TRAJ)) {
				ps->cup_rear_catched = 0;
				ps->cup_front_popcorns_harvested = 0;
				ps->machine_popcorns_catched = 0;
				us = time_get_us2();
				ps->substate = WAITING_STOCK_DROPPED;
			}
			else if((ret_l & END_BLOCKING) || (ret_r & END_BLOCKING)) {
				if(!(ret_l & END_TRAJ))
					STMCH_ERROR("cup_clamp_popcorn_door_l BLOCKED!!");
				else if(!(ret_r & END_TRAJ))
					STMCH_ERROR("cup_clamp_popcorn_door_r BLOCKED!!");

				return STATUS_BLOCKED;
			}
			break;

		case WAITING_STOCK_DROPPED:
			if(time_get_us2() - us > 1000000L) {
				return STATUS_DONE;
			}
			break;

		default:
			ps->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_stock_end(popcorn_system_t *ps)
{
	uint8_t ret = 0;

	switch(ps->substate)
	{
		case SAVE:
			if(ps->cup_rear_catched == 1) {
				STMCH_ERROR("FIRST, RELEASE CUP_REAR!!");
				return STATUS_BLOCKED;
			}
			
		case CLOSE_LEFT_CLAMP:
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_l, CUP_CLAMP_MODE_HIDE, 0);
			ps->substate = WAITING_LEFT_CLAMP_CLOSED;

			break;

		case WAITING_LEFT_CLAMP_CLOSED:
			ret = cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_l);

			if(ret & (END_NEAR|END_TRAJ))
				ps->substate = CLOSE_RIGHT_CLAMP;
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("cup_clamp_popcorn_door_l BLOCKED!!");
				return STATUS_BLOCKED;
			}
			break;

		case CLOSE_RIGHT_CLAMP:
			cup_clamp_set_mode(&slavedspic.cup_clamp_popcorn_door_r, CUP_CLAMP_MODE_HIDE, 0);
			ps->substate = WAITING_RIGHT_CLAMP_CLOSED;
			break;

		case WAITING_RIGHT_CLAMP_CLOSED:
			ret = cup_clamp_popcorn_door_test_traj_end(&slavedspic.cup_clamp_popcorn_door_r);

			if(ret & END_TRAJ)
				return STATUS_DONE;
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("cup_clamp_popcorn_door_r BLOCKED!!");
				return STATUS_BLOCKED;
			}
			break;

		default:
			ps->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

void popcorn_system_manage(popcorn_system_t *ps)
{
	static microseconds us = 0;

	/* update mode */
	if(ps->mode_changed){
		ps->mode_changed = 0;
		ps->mode = ps->mode_rqst;
		STMCH_DEBUG("%s mode=%d", __FUNCTION__, ps->mode);
	}
	else if(time_get_us2() - us < 5000L) {
		return;
	}
	us = time_get_us2();

	switch(ps->mode)
	{
		case I2C_SLAVEDSPIC_MODE_PS_IDLE:
			break;

		case I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_CATCH_AND_DROP:
			ps->status = do_cup_front_catch_and_drop(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_READY:
		case I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_RELEASE:
			ps->status = do_cup_front_release(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_HIDE:
			ps->status = do_cup_front_hide(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_OPEN:
			ps->status = do_cup_rear_open(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_CATCH:
			ps->status = do_cup_rear_catch(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_RELEASE:
			ps->status = do_cup_rear_release(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_MACHINES_READY:
			ps->status = do_machines_ready(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_MACHINES_HARVEST:
			ps->status = do_machines_harvest(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_MACHINES_END:
			ps->status = do_machines_end(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_STOCK_DROP:
			ps->status = do_stock_drop(ps);
			break;

		case I2C_SLAVEDSPIC_MODE_PS_STOCK_END:
			ps->status = do_stock_end(ps);
			break;

		default:
			ps->substate = SAVE;
			ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
			ps->status = STATUS_ERROR;
			break;
	}

	/* status parser */
	switch(ps->status)
	{
		case STATUS_READY:
		case STATUS_BUSY:
			break;

		case STATUS_DONE:

			/**
	         * In some cases,  goto automaticaly to other modes after 
			 * finish the current one. It's because we have the DONE state
			 * previous to READY.
			 */

			ps->substate = SAVE;

			if(ps->mode == I2C_SLAVEDSPIC_MODE_PS_MACHINES_HARVEST)
				ps->mode = I2C_SLAVEDSPIC_MODE_PS_MACHINES_END;
			else {
				ps->mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
				ps->status = STATUS_READY;
			}
			break;

		case STATUS_BLOCKED:
		case STATUS_ERROR:
		default:
			ps->substate = SAVE;
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
	ss->status = STATUS_READY;
	ss->us = 0;
}

uint8_t do_hide_tower(stands_system_t *ss)
{
	uint8_t ret = 0;
	uint8_t ret2 = 0;
	uint8_t ret3 = 0;

	STMCH_DEBUG("%s substate %d \tmodo %d \tspotlight_substate %d!!", __FUNCTION__, ss->spotlight_mode, ss->substate, ss->spotlight_substate);

	switch(ss->substate)
	{
		case SAVE:

		case CLOSE_CLAMPS:
			stands_tower_clamps_set_mode(&slavedspic.stands_tower_clamps, STANDS_TOWER_CLAMPS_MODE_LOCK, 0);
			stands_clamp_set_mode(ss->clamp, STANDS_CLAMP_MODE_CLOSE, 0);
			ss->us = time_get_us2();
			ss->substate = WAITING_CLAMPS_CLOSED;

			break;

		case WAITING_CLAMPS_CLOSED:
			ret = stands_tower_clamps_test_traj_end(&slavedspic.stands_tower_clamps);

			if((ret & END_TRAJ) && (time_get_us2() - ss->us > 200000L))
				ss->substate = LIFT_ELEVATOR;

			break;

		case LIFT_ELEVATOR:
			if(ss->stored_stands < 4 || ss->mode == I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT)
				stands_elevator_set_mode(ss->elevator, STANDS_ELEVATOR_MODE_UP, 0);
			else {
				if(ss->elevator->type == STANDS_ELEVATOR_TYPE_LEFT)
					stands_elevator_set_mode(ss->elevator, STANDS_ELEVATOR_MODE_DOWN, -50);
				else
					stands_elevator_set_mode(ss->elevator, STANDS_ELEVATOR_MODE_DOWN, 50);
			}

			ss->substate = WAITING_ELEVATOR_LIFTED;

			break;

		case WAITING_ELEVATOR_LIFTED:
			ret = stands_elevator_test_traj_end(ss->elevator);

			if(ret & END_TRAJ) {
				if(ss->mode == I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT && ss->spotlight_mode == SM_SECONDARY)
					return STATUS_DONE;
				ss->substate = HIDE_BLADE;
			}
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("stands_elevator_%s BLOCKED!!", ss->elevator->type? "r":"l");
				return STATUS_BLOCKED;
			}

			break;

		case HIDE_BLADE:
			if(ss->mode == I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT) {
				if(ss->blade->mode == STANDS_BLADE_MODE_PUSH_STAND_LEFT && (slavedspic.stands_blade_l.mode == STANDS_BLADE_MODE_HIDE_RIGHT || slavedspic.stands_blade_r.mode == STANDS_BLADE_MODE_HIDE_RIGHT))
					stands_blade_set_mode(ss->blade, STANDS_BLADE_MODE_HIDE_LEFT, 0);
				else if(ss->blade->mode == STANDS_BLADE_MODE_PUSH_STAND_LEFT && slavedspic.stands_blade_l.mode != STANDS_BLADE_MODE_HIDE_RIGHT && slavedspic.stands_blade_r.mode != STANDS_BLADE_MODE_HIDE_RIGHT) {
					if(slavedspic.stands_elevator_r.mode == STANDS_ELEVATOR_MODE_UP) {
						stands_blade_set_mode(&slavedspic.stands_blade_l, STANDS_BLADE_MODE_HIDE_RIGHT, 0);
						stands_blade_set_mode(&slavedspic.stands_blade_r, STANDS_BLADE_MODE_HIDE_RIGHT, 0);
						stands_exchanger_set_position(STANDS_EXCHANGER_POSITION_MAX_mm);
					}
					else
						break;
				}
				else if(ss->blade->mode == STANDS_BLADE_MODE_PUSH_STAND_RIGHT && (slavedspic.stands_blade_l.mode == STANDS_BLADE_MODE_HIDE_LEFT || slavedspic.stands_blade_r.mode == STANDS_BLADE_MODE_HIDE_LEFT))
					stands_blade_set_mode(ss->blade, STANDS_BLADE_MODE_HIDE_RIGHT, 0);
				else if(ss->blade->mode == STANDS_BLADE_MODE_PUSH_STAND_RIGHT && slavedspic.stands_blade_l.mode != STANDS_BLADE_MODE_HIDE_LEFT && slavedspic.stands_blade_r.mode != STANDS_BLADE_MODE_HIDE_LEFT) {
					if(slavedspic.stands_elevator_l.mode == STANDS_ELEVATOR_MODE_UP) {
						stands_blade_set_mode(&slavedspic.stands_blade_l, STANDS_BLADE_MODE_HIDE_LEFT, 0);
						stands_blade_set_mode(&slavedspic.stands_blade_r, STANDS_BLADE_MODE_HIDE_LEFT, 0);
						stands_exchanger_set_position(STANDS_EXCHANGER_POSITION_MIN_mm);
					}
					else
						break;
				}
			}
			else {
				if(ss->stored_stands < 4) {
					if(ss->elevator->type == STANDS_ELEVATOR_TYPE_LEFT)
						stands_blade_set_mode(ss->blade, STANDS_BLADE_MODE_HIDE_LEFT, 0);
					else
						stands_blade_set_mode(ss->blade, STANDS_BLADE_MODE_HIDE_RIGHT, 0);
				}
				else {
					if(ss->elevator->type == STANDS_ELEVATOR_TYPE_LEFT)
						stands_blade_set_mode(ss->blade, STANDS_BLADE_MODE_PUSH_STAND_LEFT, 0);
					else
						stands_blade_set_mode(ss->blade, STANDS_BLADE_MODE_PUSH_STAND_RIGHT, 0);
				}
			}

			ss->substate = WAITING_BLADE_HIDDEN;

			break;

		case WAITING_BLADE_HIDDEN:
			if(ss->mode == I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT) {
				ret = stands_blade_test_traj_end(&slavedspic.stands_blade_l);
				ret2 = stands_blade_test_traj_end(&slavedspic.stands_blade_r);
				ret3 = stands_exchanger_test_traj_end();

				if((ret & (END_NEAR|END_TRAJ)) && (ret2 & (END_NEAR|END_TRAJ)) && (ret3 & END_TRAJ))
					return STATUS_DONE;
				else if((ret & END_BLOCKING) || (ret2 & END_BLOCKING) || (ret3 & END_BLOCKING)) {
					if(!(ret & (END_NEAR|END_TRAJ)))
						STMCH_ERROR("stands_blade_l BLOCKED!!");
					else if(!(ret2 & (END_NEAR|END_TRAJ)))
						STMCH_ERROR("stands_blade_r BLOCKED!!");
					else if(!(ret3 & END_TRAJ))
						STMCH_ERROR("stands_exchanger BLOCKED!!");
					return STATUS_BLOCKED;
				}
			}
			else {
				ret = stands_blade_test_traj_end(ss->blade);

				if(ret & (END_NEAR|END_TRAJ))
					return STATUS_DONE;
				else if(ret & END_BLOCKING) {
					if(ss->blade->type == STANDS_BLADE_TYPE_LEFT)
						STMCH_ERROR("stands_blade_l BLOCKED!!");
					else
						STMCH_ERROR("stands_blade_r BLOCKED!!");

					return STATUS_BLOCKED;
				}
			}

			break;

		default:
			ss->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_harvest_stand(stands_system_t *ss)
{
	uint8_t ret = 0;

	STMCH_DEBUG("%s substate %d \tmodo %d \tspotlight_substate %d!!", __FUNCTION__, ss->spotlight_mode, ss->substate, ss->spotlight_substate);

	switch(ss->substate)
	{
		case SAVE:
			if(ss->mode == I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT) {
				if(ss->elevator->type == STANDS_ELEVATOR_TYPE_LEFT) {
					if(slavedspic.stands_blade_l.mode != STANDS_BLADE_MODE_HIDE_LEFT)
						ss->blade = &slavedspic.stands_blade_l;
					else
						ss->blade = &slavedspic.stands_blade_r;
				}
				else {
					if(slavedspic.stands_blade_l.mode != STANDS_BLADE_MODE_HIDE_RIGHT)
						ss->blade = &slavedspic.stands_blade_r;
					else
						ss->blade = &slavedspic.stands_blade_l;
				}

				ss->substate = PUSH_STAND;
				break;
			}
			else {
				if(ss->blade->type == STANDS_BLADE_TYPE_LEFT)
					ss->blade = &slavedspic.stands_blade_l;
				else
					ss->blade = &slavedspic.stands_blade_r;
			}

		case READY_BLADE:
			if(ss->mode != I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT) {
				stands_blade_set_mode(ss->blade, STANDS_BLADE_MODE_SET_ANGLE, ss->blade_angle);
				ss->substate = WAITING_BLADE_READY;
			}
			else
				ss->substate = PUSH_STAND;

			break;

		case WAITING_BLADE_READY:
			ret = stands_blade_test_traj_end(ss->blade);

			if(ret & END_TRAJ)
				ss->substate = PUSH_STAND;
			else if(ret & END_BLOCKING) {
				if(ss->blade->type == STANDS_BLADE_TYPE_LEFT)
					STMCH_ERROR("stands_blade_l BLOCKED!!");
				else
					STMCH_ERROR("stands_blade_r BLOCKED!!");

				return STATUS_BLOCKED;
			}

			break;

		case PUSH_STAND:
			if(sensor_get(ss->stand_sensor)) {
				if(ss->elevator->type == STANDS_ELEVATOR_TYPE_LEFT)
					stands_blade_set_mode(ss->blade, STANDS_BLADE_MODE_PUSH_STAND_LEFT, 0);
				else
					stands_blade_set_mode(ss->blade, STANDS_BLADE_MODE_PUSH_STAND_RIGHT, 0);

				ss->substate = WAITING_STAND_PUSHED;
			}

			break;

		case WAITING_STAND_PUSHED:
			ret = stands_blade_test_traj_end(ss->blade);

			if(ret & END_TRAJ) {
				if(ss->stored_stands > 3)
					ss->substate = DESCEND_TOWER;
				else
					ss->substate = OPEN_CLAMP;
			}
			else if(ret & END_BLOCKING) {
				if(ss->blade->mode == STANDS_BLADE_TYPE_LEFT)
					STMCH_ERROR("stands_blade_l BLOCKED!!");
				else
					STMCH_ERROR("stands_blade_r BLOCKED!!");

				return STATUS_BLOCKED;
			}

			break;

		case DESCEND_TOWER:
			if(ss->elevator->type == STANDS_ELEVATOR_TYPE_LEFT)
				stands_elevator_set_mode(ss->elevator, STANDS_ELEVATOR_MODE_UP, -150);
			else
				stands_elevator_set_mode(ss->elevator, STANDS_ELEVATOR_MODE_UP, 150);

			ss->substate = WAITING_TOWER_DESCENDED;

			break;

		case WAITING_TOWER_DESCENDED:
			ret = stands_elevator_test_traj_end(ss->elevator);

			if(ret & END_TRAJ)
				ss->substate = OPEN_CLAMP;
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("stands_elevator_%c BLOCKED!!", ss->elevator->type? "r":"l");
				return STATUS_BLOCKED;
			}

			break;

		case OPEN_CLAMP:
			stands_clamp_set_mode(ss->clamp, STANDS_CLAMP_MODE_OPEN, 0);
			ss->us = time_get_us2();
			ss->substate = WAITING_CLAMP_OPENED;

			break;

		case WAITING_CLAMP_OPENED:
			if(time_get_us2() - ss->us > 200000L)
				ss->substate = DESCEND_ELEVATOR;

			break;

		case DESCEND_ELEVATOR:
			stands_elevator_set_mode(ss->elevator, STANDS_ELEVATOR_MODE_DOWN, 0);
			ss->substate = WAITING_ELEVATOR_DESCENDED;

			break;

		case WAITING_ELEVATOR_DESCENDED:
			ret = stands_elevator_test_traj_end(ss->elevator);

			if(ret & END_TRAJ) {
				ss->stored_stands++;
				return STATUS_DONE;
			}
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("stands_elevator_%c BLOCKED!!", ss->elevator->type? "r":"l");
				return STATUS_BLOCKED;
			}

			break;

		default:
			ss->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_release_stand(stands_system_t *ss)
{
	uint8_t ret = 0;

	STMCH_DEBUG("%s substate %d \tmodo %d \tspotlight_substate %d!!", __FUNCTION__, ss->spotlight_mode, ss->substate, ss->spotlight_substate);

	switch(ss->substate)
	{
		case SAVE:

		case DESCEND_ELEVATOR:
			stands_elevator_set_mode(ss->elevator, STANDS_ELEVATOR_MODE_DOWN, 0);
			ss->substate = WAITING_ELEVATOR_DESCENDED;

			break;

		case WAITING_ELEVATOR_DESCENDED:
			ret = stands_elevator_test_traj_end(ss->elevator);

			if(ret & END_TRAJ)
				ss->substate = OPEN_CLAMP;
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("stands_elevator_%c BLOCKED!!", ss->elevator->type? "r":"l");
				return STATUS_BLOCKED;
			}

			break;

		case OPEN_CLAMP:
			stands_clamp_set_mode(ss->clamp, STANDS_CLAMP_MODE_OPEN, 0);
			ss->us = time_get_us2();
			ss->substate = WAITING_CLAMP_OPENED;

			break;

		case WAITING_CLAMP_OPENED:
			if(time_get_us2() - ss->us > 500000L)
				ss->substate = LIFT_ELEVATOR;

			break;

		case LIFT_ELEVATOR:
			stands_elevator_set_mode(ss->elevator, STANDS_ELEVATOR_MODE_UP, 100);
			ss->substate = WAITING_ELEVATOR_LIFTED;

			break;

		case WAITING_ELEVATOR_LIFTED:
			ret = stands_elevator_test_traj_end(ss->elevator);

			if(ret & END_TRAJ) {
				ss->stored_stands--;
				return STATUS_DONE;
			}
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("stands_elevator_%c BLOCKED!!", ss->elevator->type? "r":"l");
				return STATUS_BLOCKED;
			}

			break;

		default:
			ss->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_center_stand(stands_system_t *ss)
{
	uint8_t ret = 0;

	STMCH_DEBUG("%s substate %d \tmodo %d \tspotlight_substate %d!!", __FUNCTION__, ss->spotlight_mode, ss->substate, ss->spotlight_substate);

	switch(ss->substate)
	{
		case SAVE:

		case GO_CENTER:
			if((slavedspic.stands_blade_l.mode == STANDS_BLADE_MODE_HIDE_LEFT || slavedspic.stands_blade_l.mode == STANDS_BLADE_MODE_HIDE_RIGHT) &&
					(slavedspic.stands_blade_r.mode == STANDS_BLADE_MODE_HIDE_LEFT || slavedspic.stands_blade_r.mode == STANDS_BLADE_MODE_HIDE_RIGHT) &&
					(stands_blade_test_traj_end(&slavedspic.stands_blade_l) & END_TRAJ) && (stands_blade_test_traj_end(&slavedspic.stands_blade_r) & END_TRAJ)) {
				stands_exchanger_set_position(STANDS_EXCHANGER_POSITION_CENTER_mm);
				ss->substate = WAITING_CENTERED;
			}

			break;

		case WAITING_CENTERED:
			ret = stands_exchanger_test_traj_end();

			if(ret & END_TRAJ)
				ss->substate = RETURN_HOME;
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("stands_exchanger BLOCKED!!");
				return STATUS_BLOCKED;
			}

			break;

		case RETURN_HOME:
			if(ss->elevator->type == STANDS_ELEVATOR_TYPE_LEFT)
				stands_exchanger_set_position(STANDS_EXCHANGER_POSITION_MIN_mm);
			else
				stands_exchanger_set_position(STANDS_EXCHANGER_POSITION_MAX_mm);

			ss->substate = WAITING_RETURNED;
			
			break;

		case WAITING_RETURNED:
			ret = stands_exchanger_test_traj_end();

			if(ret & END_TRAJ)
				return STATUS_DONE;
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("stands_exchanger BLOCKED!!");
				return STATUS_BLOCKED;
			}

			break;

		default:
			ss->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

uint8_t do_build_spotlight(stands_system_t *ss, stands_system_t *ss_slave)
{
	static uint8_t init_flag = 0;

	if(ss->spotlight_mode != SM_PRINCIPAL && ss->spotlight_mode != SM_SECONDARY) {
		if(ss_slave->stored_stands == 0) {
			STMCH_ERROR("THERE AREN'T STANDS IN SECONDARY TOWER!!");
			return STATUS_DONE;
		}

		ss->spotlight_mode = SM_PRINCIPAL;
//		ss->mode_rqst = I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT;
//		ss->mode_changed = 1;
		ss_slave->spotlight_mode = SM_SECONDARY;
		ss_slave->mode_rqst = I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT;
		ss_slave->mode_changed = 1;
	}

	if(ss->spotlight_mode == SM_PRINCIPAL) {
		switch(ss->spotlight_substate) {
			case IDLE:
//				break;

			case HIDE_TOWER:
				ss->spotlight_status = do_hide_tower(ss);
				break;

			case HARVEST_STAND:
				ss->spotlight_status = do_harvest_stand(ss);
				break;
		}
	}
	else if(ss->spotlight_mode == SM_SECONDARY) {
		switch(ss->spotlight_substate) {
			case IDLE:
//				break;

			case HIDE_TOWER:
				ss->spotlight_status = do_hide_tower(ss);
				break;

			case CENTER_STAND:
				ss->spotlight_status = do_center_stand(ss);
				break;

			case RELEASE_STAND:
				ss->spotlight_status = do_release_stand(ss);
				break;
		}
	}

	switch(ss->spotlight_status)
	{
		case STATUS_DONE:

			if(ss->spotlight_mode == SM_PRINCIPAL) {
				if(ss->spotlight_substate == HIDE_TOWER) {
					ss->spotlight_substate = HARVEST_STAND;
					ss->spotlight_status = STATUS_READY;
				}
				else if(ss->spotlight_substate == HARVEST_STAND) {
					if(ss_slave->stored_stands == 0) {
						ss->spotlight_substate = IDLE;
						goto exit_done;
					}
					else {
						ss->spotlight_substate = HIDE_TOWER;
						ss->spotlight_status = STATUS_READY;
					}
				}
			}
			else if(ss->spotlight_mode == SM_SECONDARY) {
				if(ss->spotlight_substate == HIDE_TOWER) {
					if(!init_flag) {
						ss->spotlight_substate = RELEASE_STAND;
						ss->spotlight_status = STATUS_READY;
						init_flag = 1;
					}
					else {
						ss->spotlight_substate = CENTER_STAND;
						ss->spotlight_status = STATUS_READY;
					}
				}
				else if(ss->spotlight_substate == CENTER_STAND) {
					if(ss->stored_stands == 0) {
						ss->spotlight_substate = IDLE;
						goto exit_done;
					}
					else {
						ss->spotlight_substate = RELEASE_STAND;
						ss->spotlight_status = STATUS_READY;
					}
				}
				else if(ss->spotlight_substate == RELEASE_STAND) {
					ss->spotlight_substate = HIDE_TOWER;
					ss->spotlight_status = STATUS_READY;
				}
			}
			break;

		case STATUS_BLOCKED:
			goto exit_blocked;

			break;

		default:
			break;
	}

	return STATUS_BUSY;

exit_done:
	ss->spotlight_substate = IDLE;
	ss->spotlight_mode = 0;
	init_flag = 0;
	return STATUS_DONE;

exit_blocked:
	ss->spotlight_substate = IDLE;
	ss->spotlight_mode = 0;
	init_flag = 0;
	return STATUS_BLOCKED;
}

uint8_t do_release_spotlight(stands_system_t *ss)
{
	uint8_t ret = 0;
	uint8_t ret_sbl = 0;
	uint8_t ret_sbr = 0;

	STMCH_DEBUG("%s substate %d \tmodo %d \tspotlight_substate %d!!", __FUNCTION__, ss->spotlight_mode, ss->substate, ss->spotlight_substate);

	switch(ss->substate) {
		case SAVE:
			if(slavedspic.cup_holder_front.mode != CUP_HOLDER_FRONT_MODE_HIDE) {
				STMCH_ERROR("FIRST, HIDE CUP_HOLDER_FRONT!!");
				return STATUS_BLOCKED;
			}

		case DESCEND_ELEVATOR:
			stands_elevator_set_mode(ss->elevator, STANDS_ELEVATOR_MODE_DOWN, 0);
			ss->substate = WAITING_ELEVATOR_DESCENDED;

			break;

		case WAITING_ELEVATOR_DESCENDED:
			ret = stands_elevator_test_traj_end(ss->elevator);

			if(ret & END_TRAJ)
				ss->substate = OPEN_ALL;
			else if(ret & END_BLOCKING) {
				STMCH_ERROR("stands_elevator_%c BLOCKED!!", ss->elevator->type? "r":"l");
				return STATUS_BLOCKED;
			}

			break;

		case OPEN_ALL:
			if(slavedspic.stands_blade_l.mode == STANDS_BLADE_MODE_PUSH_STAND_LEFT ||
						slavedspic.stands_blade_l.mode == STANDS_BLADE_MODE_PUSH_STAND_RIGHT)
				stands_blade_set_mode(&slavedspic.stands_blade_l, STANDS_BLADE_MODE_CENTER, 0);
			
			if(slavedspic.stands_blade_r.mode == STANDS_BLADE_MODE_PUSH_STAND_LEFT ||
						slavedspic.stands_blade_r.mode == STANDS_BLADE_MODE_PUSH_STAND_RIGHT)
				stands_blade_set_mode(&slavedspic.stands_blade_r, STANDS_BLADE_MODE_CENTER, 0);

			if(ss->clamp->type == STANDS_CLAMP_TYPE_LEFT)
				stands_tower_clamps_set_mode(&slavedspic.stands_tower_clamps, STANDS_TOWER_CLAMPS_MODE_UNLOCK_LEFT, 0);
			else
				stands_tower_clamps_set_mode(&slavedspic.stands_tower_clamps, STANDS_TOWER_CLAMPS_MODE_UNLOCK_RIGHT, 0);

			if(ss->clamp->type == STANDS_CLAMP_TYPE_LEFT)
				stands_clamp_set_mode(ss->clamp, STANDS_CLAMP_MODE_FULL_OPEN, 0);
			else
				stands_clamp_set_mode(ss->clamp, STANDS_CLAMP_MODE_FULL_OPEN, 0);

			ss->us = time_get_us2();
			ss->substate = WAITING_ALL_OPENED;

			break;

		case WAITING_ALL_OPENED:
			ret = stands_tower_clamps_test_traj_end(&slavedspic.stands_tower_clamps);

			if(slavedspic.stands_blade_l.mode == STANDS_BLADE_MODE_PUSH_STAND_LEFT ||
			   slavedspic.stands_blade_l.mode == STANDS_BLADE_MODE_PUSH_STAND_RIGHT) {
				ret_sbl = stands_blade_test_traj_end(&slavedspic.stands_blade_l);
			}
			else
				ret_sbl = END_TRAJ;

			if(slavedspic.stands_blade_r.mode == STANDS_BLADE_MODE_PUSH_STAND_LEFT ||
			   slavedspic.stands_blade_r.mode == STANDS_BLADE_MODE_PUSH_STAND_RIGHT) {
				ret_sbr = stands_blade_test_traj_end(&slavedspic.stands_blade_r);
			}
			else
				ret_sbr = END_TRAJ;
	

			if((ret & END_TRAJ) && (ret_sbl & END_TRAJ) && (ret_sbr & END_TRAJ) && (time_get_us2() - ss->us > 300000L)) {
				ss->stored_stands = 0;
				return STATUS_DONE;
			}
			else if((ret & END_BLOCKING) || (ret_sbl & END_BLOCKING) || (ret_sbr & END_BLOCKING)) {
				if(!(ret & END_TRAJ))
					STMCH_ERROR("stand_tower_clamps BLOCKED!!");
				else if(!(ret_sbl & END_TRAJ))
					STMCH_ERROR("stands_blade_l BLOCKED!!");
				else if(!(ret_sbr & END_TRAJ))
					STMCH_ERROR("stands_blade_r BLOCKED!!");

				return STATUS_BLOCKED;
			}

			break;

		default:
			ss->substate = SAVE;
			return STATUS_ERROR;
			break;
	}

	return STATUS_BUSY;
}

void stands_system_manage(stands_system_t *ss, stands_system_t *ss_slave)
{
	/* update mode */
	if(ss->mode_changed){
		ss->mode_changed = 0;
		ss->mode = ss->mode_rqst;
		STMCH_DEBUG("%s mode=%d", __FUNCTION__, ss->mode);
	}
	else if(time_get_us2() - ss->us_system < 5000L)
		return;
	ss->us_system = time_get_us2();

	switch(ss->mode)
	{
		case I2C_SLAVEDSPIC_MODE_SS_IDLE:
			break;

		case I2C_SLAVEDSPIC_MODE_SS_HIDE_TOWER:
			ss->status = do_hide_tower(ss);
			break;

		case I2C_SLAVEDSPIC_MODE_SS_HARVEST_STAND:
			if(ss->stored_stands < 4)
				ss->status = do_harvest_stand(ss);
			else {
				ss->status = STATUS_BLOCKED;
				STMCH_ERROR("THERE IS NO ROOM!!");
			}
			break;

		case I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT:
			ss->status = do_build_spotlight(ss, ss_slave);
			break;

		case I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT:
			ss->status = do_release_spotlight(ss);
			break;

		default:
			ss->mode = I2C_SLAVEDSPIC_MODE_SS_IDLE;
			ss->status = STATUS_ERROR;
			break;
	}

	/* status parser */
	switch(ss->status)
	{
		case STATUS_READY:
		case STATUS_BUSY:
			break;

		case STATUS_DONE:

			/**
	         * In some cases,  goto automaticaly to other modes after 
			 * finish the current one. It's because we have the DONE state
			 * previous to READY.
			 */

			ss->substate = SAVE;

			if(ss->mode == I2C_SLAVEDSPIC_MODE_SS_HARVEST_STAND)
				ss->mode = I2C_SLAVEDSPIC_MODE_SS_HIDE_TOWER;
			else if(ss->mode == I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT && ss->spotlight_mode == SM_PRINCIPAL) {
				ss->mode = I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT;

				if(ss->elevator->type == STANDS_ELEVATOR_TYPE_LEFT) {
					ss->blade = &slavedspic.stands_blade_l;
					ss_slave->blade = &slavedspic.stands_blade_r;
				}
				else {
					ss->blade = &slavedspic.stands_blade_r;
					ss_slave->blade = &slavedspic.stands_blade_l;
				}
			}
			else {
				ss->mode = I2C_SLAVEDSPIC_MODE_SS_IDLE;
				ss->status = STATUS_READY;
			}
			break;

		case STATUS_BLOCKED:
		case STATUS_ERROR:
		default:
			ss->substate = SAVE;
			ss->mode = I2C_SLAVEDSPIC_MODE_SS_IDLE;
			break;
	}
}

void state_do_stands_systems(void)
{
	stands_system_manage(&slavedspic.ss[I2C_SIDE_LEFT], &slavedspic.ss[I2C_SIDE_RIGHT]);
	stands_system_manage(&slavedspic.ss[I2C_SIDE_RIGHT], &slavedspic.ss[I2C_SIDE_LEFT]);
}
#endif


/* set infos */
void state_do_set_infos(void)
{
	if (!state_check_update(SET_INFOS))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	if(mainboard_command.set_infos.cup_front_catched >= 0)
		slavedspic.ps.cup_front_catched = mainboard_command.set_infos.cup_front_catched;

	if(mainboard_command.set_infos.cup_rear_catched >= 0)
		slavedspic.ps.cup_rear_catched  = mainboard_command.set_infos.cup_rear_catched;

	if(mainboard_command.set_infos.machine_popcorns_catched >= 0)
		slavedspic.ps.machine_popcorns_catched = mainboard_command.set_infos.machine_popcorns_catched;

	if(mainboard_command.set_infos.stored_stands_l >= 0)
		slavedspic.ss[I2C_SIDE_LEFT].stored_stands = mainboard_command.set_infos.stored_stands_l;

	if(mainboard_command.set_infos.stored_stands_r >= 0)
		slavedspic.ss[I2C_SIDE_RIGHT].stored_stands = mainboard_command.set_infos.stored_stands_r;

	STMCH_DEBUG("cup_front_catched = %d", slavedspic.ps.cup_front_catched);
	STMCH_DEBUG("cup_rear_catched = %d", slavedspic.ps.cup_rear_catched);
	STMCH_DEBUG("machine_popcorns_catched = %d", slavedspic.ps.machine_popcorns_catched);
	STMCH_DEBUG("stored_stands_l = %d", slavedspic.ss[I2C_SIDE_LEFT].stored_stands);
	STMCH_DEBUG("stored_stands_r = %d", slavedspic.ss[I2C_SIDE_RIGHT].stored_stands);

   	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}


/* state machines */
void state_machines(void)
{
	state_do_init();

#ifdef SIMPLE_ACTUATORS
	/* simple actuator modes */
	/* XXX blocking implementation */
	state_do_stands_blade_mode();
	state_do_stands_clamp_mode();
	state_do_stands_elevator_mode();
	state_do_stands_tower_clamps_mode();
	state_do_cup_clamp_popcorn_door_mode();
	state_do_popcorn_tray_mode();
	state_do_popcorn_ramps_mode();
	state_do_cup_clamp_front_mode();
	state_do_cup_holder_front_mode();
#endif

#ifdef ACTUATOR_SYSTEMS
	/* multiple actuators modes */
	state_do_popcorn_system();
	state_do_stands_systems();
#endif

	state_do_set_infos();
}

void state_init(void)
{
	microseconds us = 0;

	if(sensor_get(S_STAND_INSIDE_L) || sensor_get(S_STAND_INSIDE_R)) {
		STMCH_ERROR("%s THERE ARE STANDS BLOCKING!!", __FUNCTION__);
		return;
	}

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

	cup_clamp_front_set_mode(&slavedspic.cup_clamp_front, CUP_CLAMP_FRONT_MODE_CUP_LOCKED, 0);
	cup_holder_front_set_mode(&slavedspic.cup_holder_front, CUP_HOLDER_FRONT_MODE_HIDE, 0);

	popcorn_tray_set_mode(&slavedspic.popcorn_tray, POPCORN_TRAY_MODE_OPEN, 0);
	us = time_get_us2();

	STMCH_DEBUG ("POPCORN door R");
	cup_clamp_popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_r, CUP_CLAMP_MODE_OPEN, 0);
	cup_clamp_popcorn_door_wait_end(&slavedspic.cup_clamp_popcorn_door_r);

	STMCH_DEBUG ("POPCORN door L");
	cup_clamp_popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_l, CUP_CLAMP_MODE_HIDE, 0);
	cup_clamp_popcorn_door_wait_end(&slavedspic.cup_clamp_popcorn_door_l);

	cup_clamp_popcorn_door_set_mode(&slavedspic.cup_clamp_popcorn_door_r, CUP_CLAMP_MODE_HIDE, 0);
	while(time_get_us2() - us < 1000000L);

	STMCH_DEBUG ("RAMPS");
	popcorn_ramps_set_mode(&slavedspic.popcorn_ramps, POPCORN_RAMPS_MODE_HIDE, 0);
	popcorn_ramps_wait_end(&slavedspic.popcorn_ramps);

	popcorn_tray_set_mode(&slavedspic.popcorn_tray, POPCORN_TRAY_MODE_CLOSE, 0);

	BRAKE_OFF();
	slavedspic.stands_exchanger.on = 1;
	stands_exchanger_calibrate();

//	if(build_spotlight_side == I2C_SIDE_LEFT)
//		stands_exchanger_set_position(STANDS_EXCHANGER_POSITION_MIN_mm);
//	else
//		stands_exchanger_set_position(STANDS_EXCHANGER_POSITION_MAX_mm);
//	stands_exchanger_wait_end();

#ifdef ACTUATOR_SYSTEMS
	/* systems init */
	popcorn_system_init(&slavedspic.ps);
	stands_system_init(&slavedspic.ss[I2C_SIDE_LEFT], S_STAND_INSIDE_L, &slavedspic.stands_blade_l, &slavedspic.stands_clamp_l, &slavedspic.stands_elevator_l);
	stands_system_init(&slavedspic.ss[I2C_SIDE_RIGHT], S_STAND_INSIDE_R, &slavedspic.stands_blade_r, &slavedspic.stands_clamp_r, &slavedspic.stands_elevator_r);
#endif
}
