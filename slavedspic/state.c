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

/*   *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2012) *  Javier Baliñas Santos <javier@arc-robots.org> * *  Code ported to family of microcontrollers dsPIC from *  state.c,v 1.4 2009/05/27 20:04:07 zer0 Exp. */


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
#define POWER_OFF			I2C_SLAVEDSPIC_MODE_POWER_OFF
#define FINGERS			I2C_SLAVEDSPIC_MODE_FINGERS
#define ARMS				I2C_SLAVEDSPIC_MODE_ARMS
#define HOOK				I2C_SLAVEDSPIC_MODE_HOOK
#define BOOT				I2C_SLAVEDSPIC_MODE_BOOT
#define TRAY				I2C_SLAVEDSPIC_MODE_TRAY
#define TURBINE_ANGLE	I2C_SLAVEDSPIC_MODE_TURBINE_ANGLE
#define TURBINE_BLOW		I2C_SLAVEDSPIC_MODE_TURBINE_BLOW
#define LIFT_HEIGHT		I2C_SLAVEDSPIC_MODE_LIFT_HEIGHT

#define HARVEST			I2C_SLAVEDSPIC_MODE_HARVEST
#define STORE				I2C_SLAVEDSPIC_MODE_STORE
#define DUMP				I2C_SLAVEDSPIC_MODE_DUMP

static struct i2c_cmd_slavedspic_set_mode mainboard_command;
static volatile uint8_t prev_state;
static volatile uint8_t mode_changed = 0;
uint8_t state_debug = 0;

/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_slavedspic_set_mode *cmd)
{
	prev_state = mainboard_command.mode;
	memcpy(&mainboard_command, cmd, sizeof(mainboard_command));
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, mainboard_command.mode);

	/* XXX power off mode */
	if (mainboard_command.mode == POWER_OFF) {
	}
	
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

/* set fingers mode */
void state_do_fingers_mode(void)
{		
	static int8_t done_floor = 1;
	static int8_t done_totem = 1;

	/* pull fingers blocking if they hasn't done yet */
	if(!done_floor) {
		done_floor = fingers_check_mode_done(&slavedspic.fingers_floor);
		slavedspic.fingers_floor_blocked = (done_floor == -1? 1 : 0);
	}
	if(!done_totem) {
		done_totem = fingers_check_mode_done(&slavedspic.fingers_totem);
		slavedspic.fingers_totem_blocked = (done_totem == -1? 1 : 0);
	}

	/* return if no update */
	if (!state_check_update(FINGERS))
		return;

	/* set fingers mode */
	if(mainboard_command.fingers.type == I2C_FINGERS_TYPE_FLOOR) {
		if(fingers_set_mode(&slavedspic.fingers_floor, mainboard_command.fingers.mode, mainboard_command.fingers.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		done_floor = 0;
	}	
	else if(mainboard_command.fingers.type == I2C_FINGERS_TYPE_TOTEM) {
		if(fingers_set_mode(&slavedspic.fingers_totem, mainboard_command.fingers.mode, mainboard_command.fingers.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		done_totem = 0;
	}

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
}

/* set arms mode */
void state_do_arms_mode(void)
{
	static int8_t done_left = 1;
	static int8_t done_right = 1;

	/* pull fingers blocking if they hasn't done yet */
	if(!done_right) {
		done_right = arm_check_mode_done(&slavedspic.arm_right);
		slavedspic.arm_right_blocked = (done_right == -1? 1 : 0);
	}
	if(!done_left) {
		done_left = arm_check_mode_done(&slavedspic.arm_left);
		slavedspic.arm_left_blocked = (done_left == -1? 1 : 0);
	}

	/* return if no update */
	if (!state_check_update(ARMS))
		return;

	/* set fingers mode */
	if(mainboard_command.arm.type == I2C_ARM_TYPE_RIGHT) {
		if(arm_set_mode(&slavedspic.arm_right, mainboard_command.arm.mode, mainboard_command.arm.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		done_right = 0;
	}	
	else if(mainboard_command.arm.type == I2C_ARM_TYPE_LEFT) {
		if(arm_set_mode(&slavedspic.arm_left, mainboard_command.arm.mode, mainboard_command.arm.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		done_left = 0;
	}
}

/* set lift heigh */
void state_do_lift_height(void)
{
	if (!state_check_update(LIFT_HEIGHT))
		return;

	lift_set_height(mainboard_command.lift.height);
	while (!lift_check_height_reached());

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
}

/* set turbine angle */
void state_do_turbine_angle(void)
{
	if (!state_check_update(TURBINE_ANGLE))
		return;

	turbine_set_angle(&slavedspic.turbine, 
							mainboard_command.turbine.angle_deg, mainboard_command.turbine.angle_speed);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
}

/* set turbine blow speed */
void state_do_turbine_blow(void)
{
	if (!state_check_update(TURBINE_BLOW))
		return;

	turbine_set_blow_speed(&slavedspic.turbine, mainboard_command.turbine.blow_speed);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
}

/* set hook mode */
void state_do_hook_mode(void)
{
	if (!state_check_update(HOOK))
		return;

	hook_set_mode(&slavedspic.hook, mainboard_command.hook.mode);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
}

/* set boot mode */
void state_do_boot_mode(void)
{
	if (!state_check_update(BOOT))
		return;

	boot_set_mode(&slavedspic.boot, mainboard_command.boot.mode);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
}

/* set tray mode */
void state_do_tray_mode(void)
{
	if (!state_check_update(TRAY))
		return;

	if(mainboard_command.tray.type == I2C_TRAY_TYPE_RECEPTION)
		tray_set_mode(&slavedspic.tray_reception, mainboard_command.tray.mode);
	else if(mainboard_command.tray.type == I2C_TRAY_TYPE_STORE)
		tray_set_mode(&slavedspic.tray_store, mainboard_command.tray.mode);
	else if(mainboard_command.tray.type == I2C_TRAY_TYPE_BOOT)
		tray_set_mode(&slavedspic.tray_boot, mainboard_command.tray.mode);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
}


/* set harvest mode */
void state_do_harvest_mode(void)
{
#define LIFT_HEIGHT_INFRONT_GOLDBAR_TOTEM 	10000
#define LIFT_HEIGHT_OVER_TOTEM					10000
#define LIFT_HEIGHT_OVER_GOLDBAR_FLOOR			10000
#define LIFT_HEIGHT_OVER_COINS_FLOOR			10000
#define LIFT_HEIGHT_NEAR_GOLDBAR_FLOOR			10000
#define LIFT_HEIGHT_NEAR_COINS_FLOOR			10000

#define TRIES_HARVEST_GOLDBAR_SEA_MAX	3

	uint8_t err = 0;
	
	if (!state_check_update(HARVEST))
		return;

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	/* notice status and update mode*/
	slavedspic.status = I2C_SLAVEDSPIC_STATUS_BUSY;
	slavedspic.harvest_mode = mainboard_command.harvest.mode;

	switch(slavedspic.harvest_mode)
	{
		case I2C_HARVEST_MODE_PREPARE_TOTEM:
			
			/* open all fingers */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HUG, 0);
			
			err = wait_fingers_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = wait_fingers_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			/* turbine in front of goldbar totem */
			turbine_set_angle(&slavedspic.turbine, 90, ANGLE_SPEED_FAST);
			lift_set_height(LIFT_HEIGHT_INFRONT_GOLDBAR_TOTEM);
			err = wait_fingers_end();
			if(err & END_BLOCKING)
				break;

			break;

		case I2C_HARVEST_MODE_PREPARE_GOLDBAR_TOTEM:

			/* open totem fingers & close floor fingers */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_CLOSE, 0);

			err = wait_fingers_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = wait_fingers_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			/* turbine in front of goldbar totem */
			turbine_set_angle(&slavedspic.turbine, 90, ANGLE_SPEED_FAST);
			lift_set_height(LIFT_HEIGHT_INFRONT_GOLDBAR_TOTEM);
			err = wait_lift_end();
			if(err & END_BLOCKING)
				break;
			
			break;

		case I2C_HARVEST_MODE_PREPARE_GOLDBAR_SEA:

			/* open all fingers */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_OPEN, 0);
			
			err = wait_fingers_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = wait_fingers_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			/* turbine looking to the floor */
			turbine_set_angle(&slavedspic.turbine, 0, ANGLE_SPEED_FAST);

			/* lift over a goldbar height */
			lift_set_height(LIFT_HEIGHT_OVER_GOLDBAR_SEA);
			err = wait_lift_end();
			if(err & END_BLOCKING)
				break;
			
			break;
			
		case I2C_HARVEST_MODE_PREPARE_COINS_TOTEM:

			/* hold fingers floor & open fingers totem */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HOLD, 0);

			err = wait_fingers_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = wait_fingers_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			/* lift over totem */
			turbine_set_angle(&slavedspic.turbine, 90, ANGLE_SPEED_FAST);
			lift_set_height(LIFT_HEIGHT_OVER_TOTEM_TOTEM);
			err = wait_lift_end();
			if(err & END_BLOCKING)
				break;
			
			break;

		case I2C_HARVEST_MODE_PREPARE_COINS_FLOOR:

			/* open all fingers */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_OPEN, 0);

			err = wait_fingers_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = wait_fingers_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			/* turbine looking down and over a coin heigh */
			turbine_set_angle(&slavedspic.turbine, 90, ANGLE_SPEED_FAST);
			lift_set_height(LIFT_HEIGHT_OVER_COINS_SEA);
			err = wait_lift_end();
			if(err & END_BLOCKING)
				break;

			break;

		case I2C_HARVEST_MODE_COINS_ISLE:

			/* close fingers floor */
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_CLOSE, 0);

			err = wait_fingers_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			break;

		case I2C_HARVEST_MODE_COINS_FLOOR:

			/* all fingers on hold mode */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_HOLD, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HOLD, 0);

			err = wait_fingers_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = wait_fingers_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			break;

		case I2C_HARVEST_MODE_COINS_TOTEM:

			/* fingers floor holding coins */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_CLOSE, 0);

			err = wait_fingers_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			break;

		case I2C_HARVEST_MODE_GOLDBAR_TOTEM:

			/* turbine sucking up */
			turbine_set_blow_speed(&slavedspic.turbine, TURBINE_SUCKUP_SPEED);

			break;

		case I2C_HARVEST_MODE_GOLDBAR_FLOOR:

			/* turbine sucking up */
			turbine_set_blow_speed(&slavedspic.turbine, TURBINE_SUCKUP_SPEED);
			time_wait_ms(400);

			while(!sensor_object_is_catched() && tries < TRIES_HARVEST_GOLDBAR_SEA_MAX) 
			{		
				/* go down until sensor detects the goldbar */
				lift_set_height(LIFT_HEIGHT_NEAR_GOLDBAR_FLOOR);

				while(!err && sensor_object_is_catched())
					err = lift_check_height_reached();
	
				/* XXX hard stop ? */
				lift_hard_stop();
	
				/* go a bit up */
				lift_set_height(LIFT_HEIGHT_OVER_GOLDBAR_FLOOR);
			
				err = wait_lift_end();
				if(err & END_BLOCKING)
					break;

				/* next trie ? */
				tries ++;
			}
			break;

		default:
			break;
	}

	if(err & END_BLOCKING)
		STMCH_DEBUG("HARVEST mode ends with BLOCKING");

	/* notice status and update mode*/
	slavedspic.status = I2C_SLAVEDSPIC_STATUS_READY;

}

/* set store mode */
void state_do_store_mode(void)
{
	uint8_t err = 0;
	
	if (!state_check_update(STORE))
		return;

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	/* notice status and update mode*/
	slavedspic.status = I2C_SLAVEDSPIC_STATUS_BUSY;
	slavedspic.store_mode = mainboard_command.store.mode;

	switch(slavedspic.harvest_mode)
	{
		case I2C_STORE_MODE_TOTEM_IN_MOUTH:

			/* totem fingers mode open */

			/* lift over (coins + goldbar) height */

			/* turbine looking down */

			/* all fingers mode hold */

			/* turbine off */

			/* wait no object catched */

			break;

		case I2C_STORE_MODE_TOTEM_IN_BOOT:

			/* totem fingers mode open */

			/* lift in front of store input */

			/* turn turbine for put totem into store system */

			/* all fingers mode hold */

			/* turbine off */

			/* wait no object cached */

			/* turbine looking down */

			/* goto safe height */

			break;

		case I2C_STORE_MODE_COINS_IN_MOUTH:

			/* turbine looking down */

			/* all fingers on hold mode */

			break;

		case I2C_STORE_MODE_COINS_IN_BOOT:

			/* while found coins on mouth or N times */

			/* turbine looking down */

			 

			break;

		default:
			break;
	}

	if(err & END_BLOCKING)
		STMCH_DEBUG("HARVEST mode ends with BLOCKING");

	/* notice status and update mode*/
	slavedspic.status = I2C_SLAVEDSPIC_STATUS_READY;
}

/* set dump mode */
void state_do_dump_mode(void)
{
	if (!state_check_update(DUMP))
		return;

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
}


/* state machines */
void state_machines(void)
{
	state_do_init();

	/* actuator level */
	state_do_fingers_mode();
	state_do_arms_mode();
	state_do_lift_height();
	state_do_turbine_angle();
	state_do_turbine_blow();
	state_do_hook_mode();
	state_do_boot_mode();
	state_do_tray_mode();

	/* actuators abstraction level */
	state_do_harvest_mode();
	state_do_store_mode();
	state_do_dump_mode();
}

void state_init(void)
{
	mainboard_command.mode = INIT;

	//token_system_init(&slavedspic.ts[I2C_SIDE_FRONT], BELTS_SIDE_FRONT, 
	//						S_FRONT_TOKEN_STOP, S_FRONT_TOKEN_CATCHED);

	//token_system_init(&slavedspic.ts[I2C_SIDE_REAR], BELTS_SIDE_REAR, 
	//						S_REAR_TOKEN_STOP, S_REAR_TOKEN_CATCHED);
}

