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

#if 0

/* shorter aliases for this file */
#define INIT				I2C_SLAVEDSPIC_MODE_INIT
#define POWER_OFF			I2C_SLAVEDSPIC_MODE_POWER_OFF
#define FINGERS			I2C_SLAVEDSPIC_MODE_FINGERS
#define ARM				   I2C_SLAVEDSPIC_MODE_ARM
#define HOOK				I2C_SLAVEDSPIC_MODE_HOOK
#define BOOT				I2C_SLAVEDSPIC_MODE_BOOT
#define TRAY				I2C_SLAVEDSPIC_MODE_TRAY
#define TURBINE_ANGLE	I2C_SLAVEDSPIC_MODE_TURBINE_ANGLE
#define TURBINE_BLOW		I2C_SLAVEDSPIC_MODE_TURBINE_BLOW
#define LIFT_HEIGHT		I2C_SLAVEDSPIC_MODE_LIFT_HEIGHT

#define HARVEST			I2C_SLAVEDSPIC_MODE_HARVEST
#define STORE				I2C_SLAVEDSPIC_MODE_STORE
#define DUMP				I2C_SLAVEDSPIC_MODE_DUMP
#define SET_INFOS			I2C_SLAVEDSPIC_MODE_SET_INFOS

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
}


/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_slavedspic_set_mode *cmd)
{
	prev_state = mainboard_command.mode;
	memcpy(&mainboard_command, cmd, sizeof(mainboard_command));
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, mainboard_command.mode);

	/* XXX power off mode */
	if (mainboard_command.mode == POWER_OFF) {

		/* turbine */
		turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);
		turbine_power_off(&slavedspic.turbine);

		/* servos */
		tray_set_mode(&slavedspic.tray_reception, TRAY_MODE_DOWN);
		tray_set_mode(&slavedspic.tray_store, TRAY_MODE_DOWN);
		tray_set_mode(&slavedspic.tray_boot, TRAY_MODE_DOWN);

		/* lift */
		slavedspic.lift.on = 0;
		dac_mc_set(&gen.dac_mc_left, 0);
		BRAKE_ON();

		/* ax12 */
		ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x00);

		wait_ms(100);
		pwm_servo_disable();
      
      state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
	}
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

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	state_init();
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

  	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set fingers mode */
void state_do_fingers_mode(void)
{		
	/* return if no update */
	if (!state_check_update(FINGERS))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set fingers mode */
	if(mainboard_command.fingers.type == I2C_FINGERS_TYPE_FLOOR || mainboard_command.fingers.type == I2C_FINGERS_TYPE_FLOOR_LEFT || mainboard_command.fingers.type == I2C_FINGERS_TYPE_FLOOR_RIGHT) {
		slavedspic.fingers_floor.type = mainboard_command.fingers.type;
      if(fingers_set_mode(&slavedspic.fingers_floor, mainboard_command.fingers.mode, mainboard_command.fingers.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		fingers_wait_end(&slavedspic.fingers_floor);
	}	
	else if(mainboard_command.fingers.type == I2C_FINGERS_TYPE_TOTEM || mainboard_command.fingers.type == I2C_FINGERS_TYPE_TOTEM_LEFT || mainboard_command.fingers.type == I2C_FINGERS_TYPE_TOTEM_RIGHT) {
		slavedspic.fingers_totem.type = mainboard_command.fingers.type;
		if(fingers_set_mode(&slavedspic.fingers_totem, mainboard_command.fingers.mode, mainboard_command.fingers.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		fingers_wait_end(&slavedspic.fingers_totem);
	}

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set arms mode */
void state_do_arms_mode(void)
{
	/* return if no update */
	if (!state_check_update(ARM))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set fingers mode */
	if(mainboard_command.arm.type == I2C_ARM_TYPE_RIGHT) {
		if(arm_set_mode(&slavedspic.arm_right, mainboard_command.arm.mode, mainboard_command.arm.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		arm_wait_end(&slavedspic.arm_right);
	}	
	else if(mainboard_command.arm.type == I2C_ARM_TYPE_LEFT) {
		if(arm_set_mode(&slavedspic.arm_left, mainboard_command.arm.mode, mainboard_command.arm.offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		arm_wait_end(&slavedspic.arm_left);
	}
   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set lift heigh */
void state_do_lift_height(void)
{
	if (!state_check_update(LIFT_HEIGHT))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	lift_set_height(mainboard_command.lift.height * 1000);
	lift_wait_end();

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());
   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set turbine angle */
void state_do_turbine_angle(void)
{
	if (!state_check_update(TURBINE_ANGLE))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	turbine_set_angle(&slavedspic.turbine, 
							mainboard_command.turbine.angle_deg, mainboard_command.turbine.angle_speed);

   time_wait_ms(400);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set turbine blow speed */
void state_do_turbine_blow(void)
{
	if (!state_check_update(TURBINE_BLOW))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	turbine_set_blow_speed(&slavedspic.turbine, mainboard_command.turbine.blow_speed << 2);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set hook mode */
void state_do_hook_mode(void)
{
	if (!state_check_update(HOOK))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	hook_set_mode(&slavedspic.hook, mainboard_command.hook.mode);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

   hook_wait_end(&slavedspic.hook);

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set boot mode */
void state_do_boot_mode(void)
{
	if (!state_check_update(BOOT))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	boot_set_mode(&slavedspic.boot, mainboard_command.boot.mode);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

   boot_wait_end(&slavedspic.boot);

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set tray mode */
void state_do_tray_mode(void)
{
	if (!state_check_update(TRAY))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	if(mainboard_command.tray.type == I2C_TRAY_TYPE_RECEPTION)
		tray_set_mode(&slavedspic.tray_reception, mainboard_command.tray.mode);
	else if(mainboard_command.tray.type == I2C_TRAY_TYPE_STORE)
		tray_set_mode(&slavedspic.tray_store, mainboard_command.tray.mode);
	else if(mainboard_command.tray.type == I2C_TRAY_TYPE_BOOT)
		tray_set_mode(&slavedspic.tray_boot, mainboard_command.tray.mode);


	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}


/* set harvest mode */
void state_do_harvest_mode(void)
{
#define LIFT_HEIGHT_INFRONT_GOLDBAR_TOTEM 	44000
#define LIFT_HEIGHT_OVER_TOTEM_OLD				12000
#define LIFT_HEIGHT_OVER_TOTEM					5000
#define LIFT_HEIGHT_OVER_GOLDBAR					32000
#define LIFT_HEIGHT_OVER_COINS					35000
#define LIFT_HEIGHT_NEAR_GOLDBAR					40000

#define TRIES_HARVEST_GOLDBAR_FLOOR_MAX	3

	uint8_t err = 0;
	uint8_t tries = 0;
	
	if (!state_check_update(HARVEST))
		return;

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);
	slavedspic.harvest_mode = mainboard_command.harvest.mode;

	switch(slavedspic.harvest_mode)
	{
		case I2C_HARVEST_MODE_PREPARE_TOTEM:
			
			/* open all fingers */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HUG, 0);
			
			/* XXX */
			time_wait_ms(100);

			/* turbine in front of goldbar totem */
			lift_set_height(LIFT_HEIGHT_INFRONT_GOLDBAR_TOTEM);
			time_wait_ms(100);
			turbine_set_angle(&slavedspic.turbine, 90, TURBINE_ANGLE_SPEED_FAST);
			err = lift_wait_end();
			if(err & END_BLOCKING)
				break;

			err = fingers_wait_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = fingers_wait_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			break;

		case I2C_HARVEST_MODE_PREPARE_GOLDBAR_TOTEM:

			/* open totem fingers & close floor fingers */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_CLOSE, 0);

			/* XXX */
			time_wait_ms(100);
			
			/* XXX */
			//err = fingers_wait_end(&slavedspic.fingers_floor);
			//if(err & END_BLOCKING)
			//	break;

			/* turbine in front of goldbar totem */
			turbine_set_angle(&slavedspic.turbine, 90, TURBINE_ANGLE_SPEED_FAST);
			lift_set_height(LIFT_HEIGHT_INFRONT_GOLDBAR_TOTEM);
			err = lift_wait_end();
			if(err & END_BLOCKING)
				break;

			err = fingers_wait_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;
			
			break;

		case I2C_HARVEST_MODE_PREPARE_GOLDBAR_FLOOR:

			/* open all fingers */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_OPEN, 0);
		
			/* XXX */
			time_wait_ms(100);	

			/* turbine looking to the floor */
			turbine_set_angle(&slavedspic.turbine, TURBINE_ANGLE_ZERO, TURBINE_ANGLE_SPEED_FAST);

			/* lift over a goldbar height */
			lift_set_height(LIFT_HEIGHT_OVER_GOLDBAR);
			err = lift_wait_end();
			if(err & END_BLOCKING)
				break;
		
			err = fingers_wait_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = fingers_wait_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;
	
			break;
			
		case I2C_HARVEST_MODE_PREPARE_COINS_TOTEM:

			/* hold fingers floor & open fingers totem */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HOLD, 0);

			err = fingers_wait_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			/* lift over totem */
#ifdef old_version
			turbine_set_angle(&slavedspic.turbine, 90, TURBINE_ANGLE_SPEED_FAST);
			lift_set_height(LIFT_HEIGHT_OVER_TOTEM);
			err = lift_wait_end();
			if(err & END_BLOCKING)
				break;
#else
			lift_set_height(LIFT_HEIGHT_OVER_TOTEM);
			err = lift_wait_end();
			if(err & END_BLOCKING)
				break;
#endif
			err = fingers_wait_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;
			
			break;

		case I2C_HARVEST_MODE_PREPARE_COINS_FLOOR:

			/* open all fingers */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_OPEN, 0);

			/* XXX */
			time_wait_ms(100);

			/* turbine looking down and over a coin heigh */
			turbine_set_angle(&slavedspic.turbine, TURBINE_ANGLE_ZERO, TURBINE_ANGLE_SPEED_FAST);
			lift_set_height(LIFT_HEIGHT_OVER_TOTEM);
			err = lift_wait_end();
			if(err & END_BLOCKING)
				break;

			err = fingers_wait_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = fingers_wait_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			break;

		case I2C_HARVEST_MODE_COINS_ISLE:

			/* close fingers floor */
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_CLOSE, 0);

			err = fingers_wait_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			slavedspic.nb_coins_in_mouth += 3;

			break;

		case I2C_HARVEST_MODE_COINS_FLOOR:

			/* all fingers on hold mode */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_HOLD, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HOLD, 0);

			err = fingers_wait_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = fingers_wait_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			slavedspic.nb_coins_in_mouth += 4;

			break;

		case I2C_HARVEST_MODE_COINS_TOTEM:

			/* fingers floor holding coins */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_CLOSE, 0);

			err = fingers_wait_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			slavedspic.nb_coins_in_mouth += 4;

			break;

		case I2C_HARVEST_MODE_GOLDBAR_TOTEM:
      {
         uint8_t sensors_line_b1;
         uint8_t sensors_line_b2;

         sensors_line_b1=sensor_get(S_TURBINE_LINE_B1);
         sensors_line_b2=sensor_get(S_TURBINE_LINE_B2);

			/* turbine sucking up */
			turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_FAST);

         //XXX
         if(! (sensors_line_b1&&sensors_line_b2)) 
         {
   			/* check sensors */
   			if(WAIT_COND_OR_TIMEOUT(sensor_get(S_TURBINE_LINE_B1) && sensor_get(S_TURBINE_LINE_B2), 2000))
   				STMCH_DEBUG("Goldbar catched!");
   			else {
   				STMCH_DEBUG("Goldbar not there");
   				turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);								
   			}
         }        
         else time_wait_ms(600);

			/* uncomment for debug */
			//time_wait_ms(800);
			//turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);
      }   
		break;

		case I2C_HARVEST_MODE_GOLDBAR_FLOOR:

			/* turbine sucking up */
			turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_FAST);

			/* floor fingers closed */
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_CLOSE, 0);

			err = fingers_wait_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

			while(tries < TRIES_HARVEST_GOLDBAR_FLOOR_MAX) 
			{		
				/* go down until sensor detects the goldbar */
				lift_set_height(LIFT_HEIGHT_NEAR_GOLDBAR);

				STMCH_DEBUG("go down until sensor detects the goldbar");

				while(!err && (sensor_get_all() == 0))
					err = lift_check_height_reached();
	
				/* XXX hard stop ? */
				if(!err) {
					lift_hard_stop();
					STMCH_DEBUG("sensor detects the goldbar");
				}

				/* wait to catch goldbar */
				time_wait_ms(300);

				/* go a bit up */
				lift_set_height(LIFT_HEIGHT_OVER_GOLDBAR);
			
				STMCH_DEBUG("go up");

				err = lift_wait_end();
				if(err & END_BLOCKING)
					break;

				/* check sensors */
				if(WAIT_COND_OR_TIMEOUT(sensor_get_all(), 300)) {
					STMCH_DEBUG("Goldbar catched!");
					break;
				}

				/* next trie ? */
				tries ++;
			}

			/* check sensors */
			if(sensor_get_all() == 0) {
				STMCH_DEBUG("Goldbar not there");
				turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);								
			}

			/* uncomment for debug */
			//time_wait_ms(1000);
			//turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);								

			/* all fingers mode hold */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_HOLD, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HOLD, 0);

			break;

		default:
			break;
	}

	if(err & END_BLOCKING)
		STMCH_DEBUG("HARVEST mode ends with BLOCKING");

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* store constants */
#define LIFT_HEIGHT_OVER_GOLDBAR_AND_COINS	20000
#define LIFT_HEIGHT_STORE_GOLDBAR_1				2000
#define LIFT_HEIGHT_STORE_GOLDBAR_2				4000
#define LIFT_HEIGHT_STORE_COINS					5000
#define LIFT_HEIGHT_SAFE							20000

#define STORE_TIMES_MAX		10


/* store a goldbar in mouth */
uint8_t store_goldbar_in_mouth(void)
{
	uint8_t err = 0;

	/* XXX */
	if(turbine_get_angle(&slavedspic.turbine) > 20 )
	{
		/* totem fingers mode open */
		fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);

		err = fingers_wait_end(&slavedspic.fingers_totem);
		if(err & END_BLOCKING)
			goto end;
	}
	else {
		/* turbine off */
		turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);

		/* all fingers mode hold */
		fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_HOLD, 0);
		fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HOLD, 0);
	}

	/* lift over (coins + goldbar) height */
	lift_set_height(LIFT_HEIGHT_OVER_GOLDBAR_AND_COINS);
	err = lift_wait_end();
	if(err & END_BLOCKING)
		goto end;
				
	/* turbine looking down */
	turbine_set_angle(&slavedspic.turbine, TURBINE_ANGLE_ZERO, TURBINE_ANGLE_SPEED_FAST);
	

	/* XXX */
	if(turbine_get_angle(&slavedspic.turbine) > 20 )
	{
		/* XXX */
		time_wait_ms(100);

		/* all fingers mode hold */
		fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_HOLD, 0);
		fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HOLD, 0);

		/* check if totem is still catched */
		if(sensor_get_all() == 0) {
			turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);
			STMCH_DEBUG("Catched goldbar lost! :(");
			goto end;
		}

		/* turbine off */
		turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);
	}

	/* wait no object catched */
	if(WAIT_COND_OR_TIMEOUT((sensor_get_all() == 0), 1000)) {
		slavedspic.nb_goldbars_in_mouth ++;
		STMCH_DEBUG("Goldbar stored in mouth :)");
	}

end:
	return err;
}

/* store a goldbar in boot, return END_TRAJ or END_BLOCKING */
uint8_t store_goldbar_in_boot(void)
{
	uint8_t err = 0;
	uint16_t sensors_saved = 0;
	int16_t turbine_angle;

	/* check goldbar */
	if(sensor_get_all() == 0) {
		STMCH_DEBUG("There is no Goldbar :(");
		//turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);
		//break;
	}

	/* store tray down */
	tray_set_mode(&slavedspic.tray_store, TRAY_MODE_DOWN);

	/* XXX */
	turbine_angle = turbine_get_angle(&slavedspic.turbine);
	if(turbine_angle > 20 ) {
		/* totem fingers mode open */
		fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
		err = fingers_wait_end(&slavedspic.fingers_totem);
		if(err & END_BLOCKING)
			goto end;	
	}

	/* lift in front of store input */
	lift_set_height(LIFT_HEIGHT_STORE_GOLDBAR_1);

	/* XXX */
	time_wait_ms(100);				

	/* turn turbine for put totem into store system */
	turbine_set_angle(&slavedspic.turbine, TURBINE_ANGLE_ZERO, TURBINE_ANGLE_SPEED_FAST);
	err = lift_wait_end();
	if(err & END_BLOCKING)
		goto end;

	/* all fingers mode hold */
	//fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_HOLD, 0);
	//fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HOLD, 0);

	/* turn turbine for put goldbar into store system */
	turbine_set_angle(&slavedspic.turbine, -45, TURBINE_ANGLE_SPEED_FAST);

	fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_HOLD, 0);

	/* XXX */
	time_wait_ms(100);	


	/* check sensors */
	sensors_saved = sensor_get_all();
	if(sensors_saved == 0) {
		STMCH_DEBUG("Goldbar lost :(");
		//turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);	
	}

	/* lift in front of store input */
	lift_set_height(LIFT_HEIGHT_STORE_GOLDBAR_2);
	err = lift_wait_end();
	if(err & END_BLOCKING)
		goto end;

	/* turn turbine for put totem into store system */
	turbine_set_angle(&slavedspic.turbine, -85, TURBINE_ANGLE_SPEED_FAST);

	/* XXX */
	time_wait_ms(100);

	/* turbine off */
	turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);

	/* wait no object cached */
	if(WAIT_COND_OR_TIMEOUT((sensor_get_all() != sensors_saved), 1000)) {
		slavedspic.nb_goldbars_in_boot ++;
		STMCH_DEBUG("Goldbar stored in boot :) (%d)", slavedspic.nb_goldbars_in_boot);
	}
	else {
		/* XXX comment if no debug */
		//slavedspic.nb_goldbars_in_boot ++; 
		STMCH_DEBUG("Seems goldbar NOT stored in boot :S (%d)", slavedspic.nb_goldbars_in_boot);
	}

	/* XXX */
	time_wait_ms(400);

	/* turn turbine for put totem into store system */
	turbine_set_angle(&slavedspic.turbine, -45, TURBINE_ANGLE_SPEED_FAST);

	/* XXX */
	time_wait_ms(100); //100

	/* lift in front of store input */
	lift_set_height(LIFT_HEIGHT_STORE_GOLDBAR_1);
	err = lift_wait_end();
	if(err & END_BLOCKING)
		goto end;

	/* turn turbine for put totem into store system */
	turbine_set_angle(&slavedspic.turbine, TURBINE_ANGLE_ZERO, TURBINE_ANGLE_SPEED_FAST);

	/* XXX */
	time_wait_ms(100);

	/* store tray up */
	if(slavedspic.nb_goldbars_in_boot < 4)
		tray_set_mode(&slavedspic.tray_store, TRAY_MODE_UP);

	/* goto safe height */
	lift_set_height(LIFT_HEIGHT_OVER_TOTEM);
	err = lift_wait_end();
	if(err & END_BLOCKING)
		goto end;

/*
	err = fingers_wait_end(&slavedspic.fingers_totem);
	if(err & END_BLOCKING)
		goto end;

	err = fingers_wait_end(&slavedspic.fingers_floor);
	if(err & END_BLOCKING)
		goto end;
*/

end:

	return err;
}

/* store a goldbar in boot, return END_TRAJ or END_BLOCKING */
uint8_t store_coins_in_boot(void)
{
#define LIFT_HEIGHT_TOTEM_FINGERS 	13100
#define FINGERS_TOTEM_PUSH_OFFSET	-20

	uint8_t err = 0;
	uint16_t sensors_saved = 0;
	int16_t turbine_angle;

	/* check goldbar */
	if(sensor_get_all() == 0) {
		STMCH_DEBUG("There is no Coins :(");
		//turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);
		//break;
	}

	/* store tray down */
	tray_set_mode(&slavedspic.tray_store, TRAY_MODE_DOWN);

	/* XXX */
	turbine_angle = turbine_get_angle(&slavedspic.turbine);
	if(turbine_angle > 20 ) {

		/* totem fingers mode open */
		fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
		err = fingers_wait_end(&slavedspic.fingers_totem);
		if(err & END_BLOCKING)
			goto end;	
	}

	/* turn turbine for put totem into store system */
	turbine_set_angle(&slavedspic.turbine, 10, TURBINE_ANGLE_SPEED_FAST);

	/* XXX */
	time_wait_ms(100);

	/* lift in front totem fingers */
	lift_set_height(LIFT_HEIGHT_TOTEM_FINGERS);
	err = lift_wait_end();
	if(err & END_BLOCKING)
		goto end;

	/* push coins with fingers */
	fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_CLOSE, FINGERS_TOTEM_PUSH_OFFSET);
	err = fingers_wait_end(&slavedspic.fingers_totem);
	//if(err & END_BLOCKING)
	//	goto end;	

	/* XXX */
	time_wait_ms(100);

	/* lift in front of store input */
	lift_set_height(LIFT_HEIGHT_STORE_COINS);

	err = lift_wait_end();
	if(err & END_BLOCKING)
		goto end;

	/* turbine off */
	turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);

	/* check sensors */
	sensors_saved = sensor_get_all();
	if(sensors_saved == 0) {
		STMCH_DEBUG("Coins lost :(");
		//turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);	
	}

	/* all fingers mode hold */
	fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_HOLD, -50);
	fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HOLD, 0);

	/* turn turbine for put totem into store system */
	turbine_set_angle(&slavedspic.turbine, -85, TURBINE_ANGLE_SPEED_FAST);

	/* XXX */
	time_wait_ms(100);

	/* reception tray up */
	tray_set_mode(&slavedspic.tray_reception, TRAY_MODE_UP);

	/* wait no object cached */
	if(WAIT_COND_OR_TIMEOUT(0, 500)) {
		//slavedspic.nb_coins_in_boot += 2;
		STMCH_DEBUG("Coins stored in boot :) (%d)", slavedspic.nb_coins_in_boot);
	}
	else {
		/* XXX comment if no debug */
		slavedspic.nb_coins_in_boot += 2; 
		STMCH_DEBUG("Seems Coins NOT stored in boot :S (%d)", slavedspic.nb_coins_in_boot);
	}

	/* reception tray down */
	tray_set_mode(&slavedspic.tray_reception, TRAY_MODE_DOWN);

	/* XXX */
	time_wait_ms(100);

	/* turn turbine for put totem into store system */
	turbine_set_angle(&slavedspic.turbine, 10, TURBINE_ANGLE_SPEED_FAST);

	/* XXX */
	time_wait_ms(200);

	/* store tray up */
	tray_set_mode(&slavedspic.tray_store, TRAY_MODE_UP);

	/* goto safe height */
	lift_set_height(LIFT_HEIGHT_SAFE);
	err = lift_wait_end();
	if(err & END_BLOCKING)
		goto end;

	err = fingers_wait_end(&slavedspic.fingers_totem);
	if(err & END_BLOCKING)
		goto end;

	err = fingers_wait_end(&slavedspic.fingers_floor);
	if(err & END_BLOCKING)
		goto end;

end:
	return err;
}

/* set store mode */
void state_do_store_mode(void)
{
#define LIFT_HEIGHT_NEAR_COINS	44000

	uint8_t err = 0,nb_tries=0;
	uint8_t store_times, cnt_times = 0;
	
	if (!state_check_update(STORE))
		return;

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);
	slavedspic.store_mode = mainboard_command.store.mode;
	store_times =  mainboard_command.store.times;

	switch(slavedspic.store_mode)
	{

		case I2C_STORE_MODE_GOLDBAR_IN_MOUTH:
			err = store_goldbar_in_mouth();
			break;

		case I2C_STORE_MODE_GOLDBAR_IN_BOOT:
			/* XXX only 3 goldbars */
			if(slavedspic.nb_goldbars_in_boot >= 3) 
				err = store_goldbar_in_mouth();
			else{
			   do{
			   	err = store_goldbar_in_boot();
               nb_tries++;
            } while((err & END_BLOCKING) && (nb_tries<2));  
         }                
			break;

		case I2C_STORE_MODE_MOUTH_IN_BOOT:

			/* while found coins on mouth or N times */
			while(1) 
			{
				/* exit if a number of times */
				if(store_times != 0 && cnt_times > store_times)
					break;

				/* exit after maximun times */
				if(cnt_times > STORE_TIMES_MAX)
					break;

				/* turbine looking down */
				turbine_set_angle(&slavedspic.turbine, TURBINE_ANGLE_ZERO, TURBINE_ANGLE_SPEED_FAST);

				/* tubine suck up on */			
				turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_FAST);

				/* go near trasure */
				lift_set_height(LIFT_HEIGHT_NEAR_COINS);

				/* wait end traj or object catched */
				err = 0;
				while(!err && sensor_get_all() == 0)
					err = lift_check_height_reached();

				/* XXX hard stop ? */
				lift_hard_stop();

				/* XXX wait for catch the coins */
				if(slavedspic.nb_goldbars_in_mouth)
					time_wait_ms(500);
				else
					time_wait_ms(200);					

				/* exit if no object catched */	
				if(sensor_get_all() == 0) {
					turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);
					lift_set_height(LIFT_HEIGHT_SAFE);
					err = lift_wait_end();
					break;
				}

				/* store coins/goldbar */
				if(slavedspic.nb_goldbars_in_mouth) {
					err = store_goldbar_in_boot();
					if(slavedspic.nb_goldbars_in_mouth > 0)
						slavedspic.nb_goldbars_in_mouth --;
				}
				else {
					err = store_coins_in_boot();
					slavedspic.nb_coins_in_mouth -= 2;
					if((int8_t)slavedspic.nb_coins_in_mouth < 0)
						slavedspic.nb_coins_in_mouth = 0;	
				}

				cnt_times++;
			}

			break;

		default:
			break;
	}

	if(err & END_BLOCKING)
		STMCH_DEBUG("HARVEST mode ends with BLOCKING");

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set dump mode */
void state_do_dump_mode(void)
{
	uint8_t err = 0;
	if (!state_check_update(DUMP))
		return;

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);
	slavedspic.dump_mode = mainboard_command.dump.mode;

	switch(slavedspic.dump_mode)
	{
		case I2C_DUMP_MODE_PREPARE_HOLD:
         hook_set_mode(&slavedspic.hook, HOOK_MODE_SHOW);
         tray_set_mode(&slavedspic.tray_store,TRAY_MODE_DOWN);
			
			/* XXX */
			time_wait_ms(100);

			err = 0;
			break;

      /*XXX this should be called I2C_DUMP_MODE_MOUTH*/
		case I2C_DUMP_MODE_PREPARE_MOUTH:			
         /* open all fingers */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_HUG, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_HUG, 0);
			
			/* XXX */
			time_wait_ms(100);

			lift_set_height(LIFT_HEIGHT_OVER_TOTEM);
			time_wait_ms(100);
			err = lift_wait_end();
			if(err & END_BLOCKING)
				break;

			err = fingers_wait_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = fingers_wait_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;

         break;
   
		case I2C_DUMP_MODE_PREPARE_BOOT:	
         boot_set_mode(&slavedspic.boot, BOOT_MODE_OPEN_FULL);
         tray_set_mode(&slavedspic.tray_store,TRAY_MODE_DOWN);
			
			/* XXX */
			time_wait_ms(100);

			err = 0;
         break;

		case I2C_DUMP_MODE_BOOT:
#ifdef old_version
         hook_set_mode(&slavedspic.hook, HOOK_MODE_OPEN_HOLD);
			/* XXX */
			time_wait_ms(200);

         boot_set_mode(&slavedspic.boot, BOOT_MODE_OPEN_HOLD);
         tray_set_mode(&slavedspic.tray_store,TRAY_MODE_DOWN);
         tray_set_mode(&slavedspic.tray_boot,TRAY_MODE_VIBRATE);
			time_wait_ms(1000);

         tray_set_mode(&slavedspic.tray_store,TRAY_MODE_VIBRATE);
			time_wait_ms(2000);

         tray_set_mode(&slavedspic.tray_boot,TRAY_MODE_DOWN);
         tray_set_mode(&slavedspic.tray_store,TRAY_MODE_DOWN);

         boot_set_mode(&slavedspic.boot, BOOT_MODE_CLOSE);
			time_wait_ms(200);
         hook_set_mode(&slavedspic.hook, HOOK_MODE_SHOW);
			err = 0;	
#else
         boot_set_mode(&slavedspic.boot, BOOT_MODE_OPEN_FULL);
         tray_set_mode(&slavedspic.tray_boot,TRAY_MODE_VIBRATE);
         tray_set_mode(&slavedspic.tray_store,TRAY_MODE_VIBRATE);
			time_wait_ms(1000);

         tray_set_mode(&slavedspic.tray_boot,TRAY_MODE_DOWN);
         tray_set_mode(&slavedspic.tray_store,TRAY_MODE_DOWN);

			err = 0;	
#endif
         break;

		case I2C_DUMP_MODE_BOOT_BLOWING:
      	STMCH_DEBUG("TO DO");
			err = 0;	
         break;

		case I2C_DUMP_MODE_MOUTH_BLOWING:
      	STMCH_DEBUG("TO DO");
			err = 0;	
         break;

		case I2C_DUMP_MODE_END_BOOT:
         hook_set_mode(&slavedspic.hook, HOOK_MODE_HIDE);
			
			/* XXX */
			time_wait_ms(100);
			err = 0;	
         break;

		case I2C_DUMP_MODE_END_MOUTH:		
         /* open all fingers */
			fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_CLOSE, 0);
			fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_CLOSE, 0);
			
			/* XXX */
			time_wait_ms(100);

			err = fingers_wait_end(&slavedspic.fingers_totem);
			if(err & END_BLOCKING)
				break;

			err = fingers_wait_end(&slavedspic.fingers_floor);
			if(err & END_BLOCKING)
				break;
         break;
    }
  
	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

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

	/* abstract modes */
	state_do_harvest_mode();
	state_do_store_mode();
	state_do_dump_mode();
	state_do_set_infos();
}

void state_init(void)
{
	mainboard_command.mode = INIT;

	pwm_servo_enable();
	BRAKE_OFF();
	slavedspic.lift.on = 1;
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0xFF);

	time_wait_ms(100);

	/* start positions */
	fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_OPEN, 0);
	fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_OPEN, 0);
	fingers_wait_end(&slavedspic.fingers_totem);

	turbine_set_angle(&slavedspic.turbine, 0, TURBINE_ANGLE_SPEED_FAST);
	turbine_set_blow_speed(&slavedspic.turbine, TURBINE_BLOW_SPEED_OFF);
	turbine_power_on(&slavedspic.turbine);

	time_wait_ms(500);

	arm_set_mode(&slavedspic.arm_left, ARM_MODE_HIDE, 0);
	arm_set_mode(&slavedspic.arm_right, ARM_MODE_HIDE, 0);

	hook_set_mode(&slavedspic.hook, HOOK_MODE_HIDE);
	boot_set_mode(&slavedspic.boot, BOOT_MODE_CLOSE);

	tray_set_mode(&slavedspic.tray_reception, TRAY_MODE_DOWN);
	tray_set_mode(&slavedspic.tray_store, TRAY_MODE_DOWN);
	tray_set_mode(&slavedspic.tray_boot, TRAY_MODE_DOWN);

	/* lift calibration */
	lift_calibrate();
	lift_set_height(LIFT_HEIGHT_MIN_mm);

	fingers_set_mode(&slavedspic.fingers_floor, FINGERS_MODE_CLOSE, 0);
	fingers_set_mode(&slavedspic.fingers_totem, FINGERS_MODE_CLOSE, 0);
}

#endif
