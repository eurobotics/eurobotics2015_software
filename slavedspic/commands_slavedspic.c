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
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

/*   *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011) *  Javier Baliñas Santos <javier@arc-robots.org> * *  Code ported to family of microcontrollers dsPIC from *  commands_mechboard.c,v 1.5 2009/05/27 20:04:07 zer0 Exp */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <time.h>
#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"

#include "sensor.h"
#include "cmdline.h"
#include "state.h"
#include "i2c_protocol.h"
#include "actuator.h"
#include "cs.h"
#include "main.h"

extern uint16_t state_debug;

struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};


/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	u08 bit=0;

	struct cmd_event_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("all"))) {
		bit = 0xFF;
		if (!strcmp_P(res->arg2, PSTR("on")))
			slavedspic.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			slavedspic.flags &= (~bit);
		else { /* show */
			printf_P(PSTR("encoders is %s\r\n"), 
				 (DO_ENCODERS & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("cs is %s\r\n"), 
				 (DO_CS & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("bd is %s\r\n"), 
				 (DO_BD & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("power is %s\r\n"), 
				 (DO_POWER & slavedspic.flags) ? "on":"off");
		}
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("encoders")))
		bit = DO_ENCODERS;
	if (!strcmp_P(res->arg1, PSTR("cs")))
		bit = DO_CS;
	if (!strcmp_P(res->arg1, PSTR("bd")))
		bit = DO_BD;
	if (!strcmp_P(res->arg1, PSTR("power")))
		bit = DO_POWER;

	if (!strcmp_P(res->arg2, PSTR("on")))
		slavedspic.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		slavedspic.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\r\n"), res->arg1, 
		      (bit & slavedspic.flags) ? "on":"off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
prog_char str_event_arg1[] = "all#encoders#cs#bd#power";
parse_pgm_token_string_t cmd_event_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg1, str_event_arg1);
prog_char str_event_arg2[] = "on#off#show";
parse_pgm_token_string_t cmd_event_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg2, str_event_arg2);

prog_char help_event[] = "Enable/disable events";
parse_pgm_inst_t cmd_event = {
	.f = cmd_event_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_event,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_event_arg0, 
		(prog_void *)&cmd_event_arg1, 
		(prog_void *)&cmd_event_arg2, 
		NULL,
	},
};

/**********************************************************/
/* Color */

/* this structure is filled when cmd_color is parsed successfully */
struct cmd_color_result {
	fixed_string_t arg0;
	fixed_string_t color;
};

/* function called when cmd_color is parsed successfully */
static void cmd_color_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_color_result *res = (struct cmd_color_result *) parsed_result;
	if (!strcmp_P(res->color, PSTR("purple"))) {
		slavedspic.our_color = I2C_COLOR_PURPLE;
	}
	else if (!strcmp_P(res->color, PSTR("red"))) {
		slavedspic.our_color = I2C_COLOR_RED;
	}
	printf_P(PSTR("Done\r\n"));
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "purple#red";
parse_pgm_token_string_t cmd_color_color = TOKEN_STRING_INITIALIZER(struct cmd_color_result, color, str_color_color);

prog_char help_color[] = "Set our color";
parse_pgm_inst_t cmd_color = {
	.f = cmd_color_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_color,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_color_arg0, 
		(prog_void *)&cmd_color_color, 
		NULL,
	},
};


/**********************************************************/
/* lift */

/* this structure is filled when cmd_lift is parsed successfully */
struct cmd_lift_result {
	fixed_string_t arg0;
	int32_t arg1;
};

/* function called when cmd_lift is parsed successfully */
static void cmd_lift_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_lift_result *res = (struct cmd_lift_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;
	//microseconds t1, t2;

	if (!strcmp_P(res->arg0, PSTR("lift"))) {
#if 0
		lift_set_height(res->arg1);
	
		t1 = time_get_us2();
		while (!lift_check_height_reached()) {
			t2 = time_get_us2();
			if (t2 - t1 > 20000) {
				dump_cs_debug("lift", &slavedspic.lift.cs);
				t1 = t2;
			}
		}
#else
	command.mode = I2C_SLAVEDSPIC_MODE_LIFT_HEIGHT;
	command.lift.height = res->arg1;
	state_set_mode(&command);
#endif

	}
	else if (!strcmp_P(res->arg0, PSTR("lift_calibrate")))
		lift_calibrate();

}

prog_char str_lift_arg0[] = "lift#lift_calibrate";
parse_pgm_token_string_t cmd_lift_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_lift_result, arg0, str_lift_arg0);
parse_pgm_token_num_t cmd_lift_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_lift_result, arg1, INT32);

prog_char help_lift[] = "set lift height";
parse_pgm_inst_t cmd_lift = {
	.f = cmd_lift_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_lift,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_lift_arg0, 
		(prog_void *)&cmd_lift_arg1,
		NULL,
	},
};

/**********************************************************/
/* turbine */

/* this structure is filled when cmd_turbine is parsed successfully */
struct cmd_turbine_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg2;
};

/* function called when cmd_turbine is parsed successfully */
static void cmd_turbine_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_turbine_result *res = (struct cmd_turbine_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;	

	if(data) {
		if (!strcmp_P(res->arg1, PSTR("on")))
			turbine_power_on(&slavedspic.turbine);
		else if (!strcmp_P(res->arg1, PSTR("off")))
			turbine_power_off(&slavedspic.turbine);		
	}
	else {
#if 0
		if (!strcmp_P(res->arg1, PSTR("angle"))) {
			turbine_set_angle(&slavedspic.turbine, res->arg2, slavedspic.turbine.angle_speed);
			while(!turbine_check_angle_reached(&slavedspic.turbine));
		}
		else if (!strcmp_P(res->arg1, PSTR("speed_angle")))
			slavedspic.turbine.angle_speed = res->arg2;
		else if (!strcmp_P(res->arg1, PSTR("blow")))
			turbine_set_blow_speed(&slavedspic.turbine, res->arg2);
#else
		if (!strcmp_P(res->arg1, PSTR("angle"))) {
			command.mode = I2C_SLAVEDSPIC_MODE_TURBINE_ANGLE;
			command.turbine.angle_deg = res->arg2;
			command.turbine.angle_speed = slavedspic.turbine.angle_speed;
			state_set_mode(&command);			
		}
		else if (!strcmp_P(res->arg1, PSTR("speed_angle"))) {
			slavedspic.turbine.angle_speed = res->arg2;			
		}
		else if (!strcmp_P(res->arg1, PSTR("blow"))) {
			command.mode = I2C_SLAVEDSPIC_MODE_TURBINE_BLOW;
			command.turbine.blow_speed = res->arg2;
			state_set_mode(&command);
		}
#endif
	}

	/* show */
	printf("angle = %d (%d, %d)\n\r", turbine_get_angle(&slavedspic.turbine), slavedspic.turbine.angle_pos, slavedspic.turbine.angle_speed);
	printf("blow = %d\n\r", turbine_get_blow_speed(&slavedspic.turbine));	

}

prog_char str_turbine_arg0[] = "turbine";
parse_pgm_token_string_t cmd_turbine_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_turbine_result, arg0, str_turbine_arg0);
prog_char str_turbine_arg1[] = "angle#speed_angle#blow";
parse_pgm_token_string_t cmd_turbine_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_turbine_result, arg1, str_turbine_arg1);
parse_pgm_token_num_t cmd_turbine_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_turbine_result, arg2, INT16);

prog_char help_turbine[] = "set turbine angle and blow";
parse_pgm_inst_t cmd_turbine = {
	.f = cmd_turbine_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_turbine,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_turbine_arg0, 
		(prog_void *)&cmd_turbine_arg1,
		(prog_void *)&cmd_turbine_arg2,
		NULL,
	},
};

/**********************************************************/
/* turbine show/on/off */

prog_char str_turbine_show_arg1[] = "on#off#show";
parse_pgm_token_string_t cmd_turbine_show_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_turbine_result, arg1, str_turbine_show_arg1);

prog_char help_turbine_show[] = "set turbine angle and blow";
parse_pgm_inst_t cmd_turbine_show = {
	.f = cmd_turbine_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_turbine_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_turbine_arg0, 
		(prog_void *)&cmd_turbine_show_arg1,
		NULL,
	},
};

/**********************************************************/
/* fingers */

/* this structure is filled when cmd_fingers is parsed successfully */
struct cmd_fingers_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
	int16_t arg3;
};

/* function called when cmd_fingers is parsed successfully */
static void cmd_fingers_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_fingers_result *res = (struct cmd_fingers_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;	
	//uint8_t mode;
	//fingers_t *fingers = 0;
	
#if 0
	if (!strcmp_P(res->arg1, PSTR("totem")))
		fingers = &slavedspic.fingers_totem;
	else if (!strcmp_P(res->arg1, PSTR("floor")))
		fingers = &slavedspic.fingers_floor;

	mode = fingers->mode;

	if (!strcmp_P(res->arg2, PSTR("hug")))
		mode = FINGERS_MODE_HUG;
	else if (!strcmp_P(res->arg2, PSTR("open")))
		mode = FINGERS_MODE_OPEN;
	else if (!strcmp_P(res->arg2, PSTR("close")))
		mode = FINGERS_MODE_CLOSE;
	else if (!strcmp_P(res->arg2, PSTR("hold")))
		mode = FINGERS_MODE_HOLD;
	else if (!strcmp_P(res->arg2, PSTR("pushin")))
		mode = FINGERS_MODE_PUSHIN;
		

	fingers_set_mode(fingers, mode, 0);
	fingers_wait_end(fingers);
	printf("done\n\r");

#else
	if (!strcmp_P(res->arg1, PSTR("totem")))
		command.fingers.type = I2C_FINGERS_TYPE_TOTEM;
	else
		command.fingers.type = I2C_FINGERS_TYPE_FLOOR;

	if (!strcmp_P(res->arg2, PSTR("hug")))
		command.fingers.mode = I2C_FINGERS_MODE_HUG;
	else if (!strcmp_P(res->arg2, PSTR("open")))
		command.fingers.mode = I2C_FINGERS_MODE_OPEN;
	else if (!strcmp_P(res->arg2, PSTR("close")))
		command.fingers.mode = I2C_FINGERS_MODE_CLOSE;
	else if (!strcmp_P(res->arg2, PSTR("hold")))
		command.fingers.mode = I2C_FINGERS_MODE_HOLD;
	else
		command.fingers.mode = I2C_FINGERS_MODE_PUSHIN;

	command.mode = I2C_SLAVEDSPIC_MODE_FINGERS;
	command.fingers.offset = res->arg3;
	state_set_mode(&command);
#endif
}

prog_char str_fingers_arg0[] = "fingers";
parse_pgm_token_string_t cmd_fingers_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_fingers_result, arg0, str_fingers_arg0);
prog_char str_fingers_arg1[] = "totem#floor";
parse_pgm_token_string_t cmd_fingers_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_fingers_result, arg1, str_fingers_arg1);
prog_char str_fingers_arg2[] = "hug#open#hold#close#pushin";
parse_pgm_token_string_t cmd_fingers_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_fingers_result, arg2, str_fingers_arg2);
parse_pgm_token_num_t cmd_fingers_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_fingers_result, arg3, INT16);


prog_char help_fingers[] = "set fingers mode";
parse_pgm_inst_t cmd_fingers = {
	.f = cmd_fingers_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_fingers,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_fingers_arg0, 
		(prog_void *)&cmd_fingers_arg1,
		(prog_void *)&cmd_fingers_arg2,
		(prog_void *)&cmd_fingers_arg3,
		NULL,
	},
};

/**********************************************************/
/* arm */

/* this structure is filled when cmd_arm is parsed successfully */
struct cmd_arm_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
	int16_t arg3;
};

/* function called when cmd_arm is parsed successfully */
static void cmd_arm_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_arm_result *res = (struct cmd_arm_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;
	//uint8_t mode;
	//arm_t *arm = 0;

#if 0
	if (!strcmp_P(res->arg1, PSTR("right")))
		arm = &slavedspic.arm_right;
	else if (!strcmp_P(res->arg1, PSTR("left")))
		arm = &slavedspic.arm_left;	

	mode = arm->mode;

	if (!strcmp_P(res->arg2, PSTR("hide")))
		mode = ARM_MODE_HIDE;
	else if (!strcmp_P(res->arg2, PSTR("show")))
		mode = ARM_MODE_SHOW;
	else if (!strcmp_P(res->arg2, PSTR("goldbar")))
		mode = ARM_MODE_PUSH_GOLDBAR;
	else if (!strcmp_P(res->arg2, PSTR("floor")))
		mode = ARM_MODE_PUSH_FLOOR;
		
	arm_set_mode(arm, mode, 0);
	arm_wait_end(arm);
	printf("done\n\r");

#else
	if (!strcmp_P(res->arg1, PSTR("right")))
		command.arm.type = I2C_ARM_TYPE_RIGHT;
	else 
		command.arm.type = I2C_ARM_TYPE_LEFT;

	if (!strcmp_P(res->arg2, PSTR("hide")))
		command.arm.mode = ARM_MODE_HIDE;
	else if (!strcmp_P(res->arg2, PSTR("show")))
		command.arm.mode = ARM_MODE_SHOW;
	else if (!strcmp_P(res->arg2, PSTR("goldbar")))
		command.arm.mode = ARM_MODE_PUSH_GOLDBAR;
	else
		command.arm.mode = ARM_MODE_PUSH_FLOOR;

	command.mode = I2C_SLAVEDSPIC_MODE_ARM;
	command.arm.offset = res->arg3;
	state_set_mode(&command);
#endif
}

prog_char str_arm_arg0[] = "arm";
parse_pgm_token_string_t cmd_arm_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_result, arg0, str_arm_arg0);
prog_char str_arm_arg1[] = "left#right";
parse_pgm_token_string_t cmd_arm_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_arm_result, arg1, str_arm_arg1);
prog_char str_arm_arg2[] = "hide#show#goldbar#floor";
parse_pgm_token_string_t cmd_arm_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_arm_result, arg2, str_arm_arg2);
parse_pgm_token_num_t cmd_arm_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_arm_result, arg3, INT16);

prog_char help_arm[] = "set arm mode";
parse_pgm_inst_t cmd_arm = {
	.f = cmd_arm_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_arm,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_arm_arg0, 
		(prog_void *)&cmd_arm_arg1,
		(prog_void *)&cmd_arm_arg2,
		(prog_void *)&cmd_arm_arg3,
		NULL,
	},
};

/**********************************************************/
/* tray */

/* this structure is filled when cmd_tray is parsed successfully */
struct cmd_tray_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};

/* function called when cmd_tray is parsed successfully */
static void cmd_tray_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_tray_result *res = (struct cmd_tray_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;
	//uint8_t mode;
	//tray_t *tray = 0;

#if 0
	if (!strcmp_P(res->arg1, PSTR("reception")))
		tray = &slavedspic.tray_reception;
	else if (!strcmp_P(res->arg1, PSTR("store")))
		tray = &slavedspic.tray_store;	
	else if (!strcmp_P(res->arg1, PSTR("boot")))
		tray = &slavedspic.tray_boot;	

	mode = tray->mode;

	if (!strcmp_P(res->arg2, PSTR("up")))
		mode = TRAY_MODE_UP;
	else if (!strcmp_P(res->arg2, PSTR("down")))
		mode = TRAY_MODE_DOWN;
	else if (!strcmp_P(res->arg2, PSTR("vibrate")))
		mode = TRAY_MODE_VIBRATE;
		
	tray_set_mode(tray, mode);
	printf("done\n\r");

#else

	if (!strcmp_P(res->arg1, PSTR("reception")))
		command.tray.type = I2C_TRAY_TYPE_RECEPTION;
	else if (!strcmp_P(res->arg1, PSTR("store")))
		command.tray.type = I2C_TRAY_TYPE_STORE;	
	else
		command.tray.type = I2C_TRAY_TYPE_BOOT;

	if (!strcmp_P(res->arg2, PSTR("up")))
		command.tray.mode = I2C_TRAY_MODE_UP;
	else if (!strcmp_P(res->arg2, PSTR("down")))
		command.tray.mode = I2C_TRAY_MODE_DOWN;
	else
		command.tray.mode = I2C_TRAY_MODE_VIBRATE;

	command.mode = I2C_SLAVEDSPIC_MODE_TRAY;
	state_set_mode(&command);

#endif
}

prog_char str_tray_arg0[] = "tray";
parse_pgm_token_string_t cmd_tray_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_tray_result, arg0, str_tray_arg0);
prog_char str_tray_arg1[] = "reception#store#boot";
parse_pgm_token_string_t cmd_tray_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_tray_result, arg1, str_tray_arg1);
prog_char str_tray_arg2[] = "up#down#vibrate";
parse_pgm_token_string_t cmd_tray_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_tray_result, arg2, str_tray_arg2);

prog_char help_tray[] = "set tray mode";
parse_pgm_inst_t cmd_tray = {
	.f = cmd_tray_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_tray,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_tray_arg0, 
		(prog_void *)&cmd_tray_arg1,
		(prog_void *)&cmd_tray_arg2,
		NULL,
	},
};

/**********************************************************/
/* boot */

/* this structure is filled when cmd_boot is parsed successfully */
struct cmd_boot_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_boot is parsed successfully */
static void cmd_boot_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_boot_result *res = (struct cmd_boot_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;
	//uint8_t mode;

#if 0
	mode = slavedspic.boot.mode;

	if (!strcmp_P(res->arg1, PSTR("full_open")))
		mode = BOOT_MODE_OPEN_FULL;
	else if (!strcmp_P(res->arg1, PSTR("hold_open")))
		mode = BOOT_MODE_OPEN_HOLD;	
	else if (!strcmp_P(res->arg1, PSTR("close")))
		mode = BOOT_MODE_CLOSE;
		
	boot_set_mode(&slavedspic.boot, mode);
	//while(!boot_check_mode_done(&slavedspic.boot));
	printf("done\n\r");

#else
	if (!strcmp_P(res->arg1, PSTR("full_open")))
		command.boot.mode = I2C_BOOT_MODE_OPEN_FULL;
	else if (!strcmp_P(res->arg1, PSTR("hold_open")))
		command.boot.mode = I2C_BOOT_MODE_OPEN_HOLD;	
	else
		command.boot.mode = I2C_BOOT_MODE_CLOSE;

	command.mode = I2C_SLAVEDSPIC_MODE_BOOT;
	state_set_mode(&command);

#endif
}

prog_char str_boot_arg0[] = "boot";
parse_pgm_token_string_t cmd_boot_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_boot_result, arg0, str_boot_arg0);
prog_char str_boot_arg1[] = "full_open#hold_open#close";
parse_pgm_token_string_t cmd_boot_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_boot_result, arg1, str_boot_arg1);

prog_char help_boot[] = "set boot mode";
parse_pgm_inst_t cmd_boot = {
	.f = cmd_boot_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_boot,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_boot_arg0, 
		(prog_void *)&cmd_boot_arg1,
		NULL,
	},
};


/**********************************************************/
/* hook */

/* this structure is filled when cmd_hook is parsed successfully */
struct cmd_hook_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_hook is parsed successfully */
static void cmd_hook_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_hook_result *res = (struct cmd_hook_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;
	//uint8_t mode;

#if 0
	mode = slavedspic.hook.mode;

	if (!strcmp_P(res->arg1, PSTR("hide")))
		mode = HOOK_MODE_HIDE;
	else if (!strcmp_P(res->arg1, PSTR("show")))
		mode = HOOK_MODE_SHOW;	
	else if (!strcmp_P(res->arg1, PSTR("fuckyou")))
		mode = HOOK_MODE_FUCKYOU;
	else if (!strcmp_P(res->arg1, PSTR("open_hold")))
		mode = HOOK_MODE_OPEN_HOLD;
		
	hook_set_mode(&slavedspic.hook, mode);
	//while(!hook_check_mode_done(&slavedspic.hook));
	printf("done\n\r");

#else
	if (!strcmp_P(res->arg1, PSTR("hide")))
		command.hook.mode = I2C_HOOK_MODE_HIDE;
	else if (!strcmp_P(res->arg1, PSTR("show")))
		command.hook.mode = I2C_HOOK_MODE_SHOW;	
	else if (!strcmp_P(res->arg1, PSTR("fuckyou")))
		command.hook.mode = I2C_HOOK_MODE_FUCKYOU;
	else
		command.hook.mode = I2C_HOOK_MODE_OPEN_HOLD;

	command.mode = I2C_SLAVEDSPIC_MODE_HOOK;
	state_set_mode(&command);
#endif
}

prog_char str_hook_arg0[] = "hook";
parse_pgm_token_string_t cmd_hook_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_hook_result, arg0, str_hook_arg0);
prog_char str_hook_arg1[] = "hide#show#fuckyou#open_hold";
parse_pgm_token_string_t cmd_hook_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_hook_result, arg1, str_hook_arg1);

prog_char help_hook[] = "set hook mode";
parse_pgm_inst_t cmd_hook = {
	.f = cmd_hook_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_hook,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_hook_arg0, 
		(prog_void *)&cmd_hook_arg1,
		NULL,
	},
};

/**********************************************************/
/* harvest */

/* this structure is filled when cmd_harvest is parsed successfully */
struct cmd_harvest_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_harvest is parsed successfully */
static void cmd_harvest_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_harvest_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp(res->arg1, "prep_totem"))
		command.harvest.mode = I2C_HARVEST_MODE_PREPARE_TOTEM;
	else if (!strcmp(res->arg1, "prep_goldbar_totem"))
		command.harvest.mode = I2C_HARVEST_MODE_PREPARE_GOLDBAR_TOTEM;
	else if (!strcmp(res->arg1, "prep_goldbar_floor"))
		command.harvest.mode = I2C_HARVEST_MODE_PREPARE_GOLDBAR_FLOOR;
	else if (!strcmp(res->arg1, "prep_coins_totem"))
		command.harvest.mode = I2C_HARVEST_MODE_PREPARE_COINS_TOTEM;
	else if (!strcmp(res->arg1, "prep_coins_floor"))
		command.harvest.mode = I2C_HARVEST_MODE_PREPARE_COINS_FLOOR;
	else if (!strcmp(res->arg1, "coins_isle"))
		command.harvest.mode = I2C_HARVEST_MODE_COINS_ISLE;
	else if (!strcmp(res->arg1, "coins_floor"))
		command.harvest.mode = I2C_HARVEST_MODE_COINS_FLOOR;
	else if (!strcmp(res->arg1, "coins_totem"))
		command.harvest.mode = I2C_HARVEST_MODE_COINS_TOTEM;
	else if (!strcmp(res->arg1, "goldbar_totem"))
		command.harvest.mode = I2C_HARVEST_MODE_GOLDBAR_TOTEM;
	else if (!strcmp(res->arg1, "goldbar_floor"))
		command.harvest.mode = I2C_HARVEST_MODE_GOLDBAR_FLOOR;

	command.mode = I2C_SLAVEDSPIC_MODE_HARVEST;
	state_set_mode(&command);
}

prog_char str_harvest_arg0[] = "harvest";
parse_pgm_token_string_t cmd_harvest_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_harvest_result, arg0, str_harvest_arg0);
prog_char str_harvest_arg1[] = "prep_totem#prep_goldbar_totem#prep_goldbar_floor#prep_coins_totem#prep_coins_floor#"
										"coins_isle#coins_floor#coins_totem#goldbar_totem#goldbar_floor";
parse_pgm_token_string_t cmd_harvest_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_harvest_result, arg1, str_harvest_arg1);


prog_char help_harvest[] = "prepare for/or harvest something";
parse_pgm_inst_t cmd_harvest = {
	.f = cmd_harvest_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_harvest,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_harvest_arg0, 
		(prog_void *)&cmd_harvest_arg1, 
		NULL,
	},
};

/**********************************************************/
/* store */

/* this structure is filled when cmd_store is parsed successfully */
struct cmd_store_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t arg2;
};

/* function called when cmd_store is parsed successfully */
static void cmd_store_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_store_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp(res->arg1, "goldbar_mouth"))
		command.store.mode = I2C_STORE_MODE_GOLDBAR_IN_MOUTH;
	else if (!strcmp(res->arg1, "goldbar_boot"))
		command.store.mode = I2C_STORE_MODE_GOLDBAR_IN_BOOT;
	else if (!strcmp(res->arg1, "mouth_in_boot"))
		command.store.mode = I2C_STORE_MODE_MOUTH_IN_BOOT;

	command.store.times = res->arg2;
	command.mode = I2C_SLAVEDSPIC_MODE_STORE;
	state_set_mode(&command);
}

prog_char str_store_arg0[] = "store";
parse_pgm_token_string_t cmd_store_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_store_result, arg0, str_store_arg0);
prog_char str_store_arg1[] = "goldbar_mouth#goldbar_boot#mouth_in_boot";
parse_pgm_token_string_t cmd_store_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_store_result, arg1, str_store_arg1);
parse_pgm_token_num_t cmd_store_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_store_result, arg2, UINT8);


prog_char help_store[] = "store something in somewhere (n times)";
parse_pgm_inst_t cmd_store = {
	.f = cmd_store_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_store,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_store_arg0, 
		(prog_void *)&cmd_store_arg1, 
		(prog_void *)&cmd_store_arg2, 
		NULL,
	},
};


/**********************************************************/
/* dump */

/* this structure is filled when cmd_dump is parsed successfully */
struct cmd_dump_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_dump is parsed successfully */
static void cmd_dump_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_dump_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp(res->arg1, "prep_hold"))
		command.dump.mode = I2C_DUMP_MODE_PREPARE_HOLD;
	else if (!strcmp(res->arg1, "prep_mouth"))
		command.dump.mode = I2C_DUMP_MODE_PREPARE_MOUTH;
	else if (!strcmp(res->arg1, "prep_boot"))
		command.dump.mode = I2C_DUMP_MODE_PREPARE_BOOT;
	else if (!strcmp(res->arg1, "boot"))
		command.dump.mode = I2C_DUMP_MODE_BOOT;
	else if (!strcmp(res->arg1, "boot_blowing"))
		command.dump.mode = I2C_DUMP_MODE_BOOT_BLOWING;
	else if (!strcmp(res->arg1, "mouth_blowing"))
		command.dump.mode = I2C_DUMP_MODE_MOUTH_BLOWING;
	else if (!strcmp(res->arg1, "end_boot"))
		command.dump.mode = I2C_DUMP_MODE_END_BOOT;
	else if (!strcmp(res->arg1, "end_mouth"))
		command.dump.mode = I2C_DUMP_MODE_END_MOUTH;

	command.mode = I2C_SLAVEDSPIC_MODE_DUMP;
	state_set_mode(&command);
}

prog_char str_dump_arg0[] = "dump";
parse_pgm_token_string_t cmd_dump_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_dump_result, arg0, str_dump_arg0);
prog_char str_dump_arg1[] = "prep_hold#prep_mouth#prep_boot#boot#boot_blowing#mouth_blowing#end_boot#end_mouth";
parse_pgm_token_string_t cmd_dump_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_dump_result, arg1, str_dump_arg1);


prog_char help_dump[] = "dump something to somewhere";
parse_pgm_inst_t cmd_dump = {
	.f = cmd_dump_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_dump,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_dump_arg0, 
		(prog_void *)&cmd_dump_arg1, 
		NULL,
	},
};



/**********************************************************/
/* State2 */

/* this structure is filled when cmd_state2 is parsed successfully */
struct cmd_state2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
	uint8_t arg3;
	
};

/* function called when cmd_state2 is parsed successfully */
static void cmd_state2_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_state2_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp(res->arg1, "init")) {
		command.mode = I2C_SLAVEDSPIC_MODE_INIT;
		state_set_mode(&command);
	}	
	else if (!strcmp(res->arg1, "power_off")) {
		command.mode = I2C_SLAVEDSPIC_MODE_POWER_OFF;
		state_set_mode(&command);
	}
	else if (!strcmp(res->arg1, "status")) {

		/* actuators blocking flags */
		printf("fingers_floor_blocked = %d\r\n", slavedspic.fingers_floor.blocking);
		printf("fingers_totem_blocked = %d\r\n", slavedspic.fingers_totem.blocking);
		printf("arm_right_blocked = %d\r\n", slavedspic.arm_right.blocking);
		printf("arm_left_blocked = %d\r\n", slavedspic.arm_left.blocking);
		printf("lift_blocked = %d\r\n", slavedspic.lift.blocking);
	
		/* sensors */
		printf("turbine_sensors = %d\r\n", sensor_get_all());
	
		/* infos */
		printf("status = %s\r\n", slavedspic.status == I2C_SLAVEDSPIC_STATUS_BUSY? "BUSY":"READY");
	
		printf("harvest_mode = %d\r\n", slavedspic.harvest_mode);
		printf("store_mode = %d\r\n", slavedspic.store_mode);
		printf("dump_mode = %d\r\n", slavedspic.dump_mode);
	
		printf("nb_goldbars_in_boot = %d\r\n", slavedspic.nb_goldbars_in_boot);
		printf("nb_goldbars_in_mouth = %d\r\n", slavedspic.nb_goldbars_in_mouth);
		printf("nb_coins_in_boot = %d\r\n", slavedspic.nb_coins_in_boot);
		printf("nb_coins_in_mouth = %d\r\n", slavedspic.nb_coins_in_mouth);
	}
}

prog_char str_state2_arg0[] = "state";
parse_pgm_token_string_t cmd_state2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg0, str_state2_arg0);
prog_char str_state2_arg1[] = "init#power_off#status";
parse_pgm_token_string_t cmd_state2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg1, str_state2_arg1);

prog_char help_state2[] = "set slavedspic mode";
parse_pgm_inst_t cmd_state2 = {
	.f = cmd_state2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state2_arg0, 
		(prog_void *)&cmd_state2_arg1,
		NULL,
	},
};


/**********************************************************/
/* State3 */

/* this structure is filled when cmd_state3 is parsed successfully */
struct cmd_state3_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
	
};

/* function called when cmd_state3 is parsed successfully */
static void cmd_state3_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_state3_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	command.set_infos.nb_goldbars_in_boot = -1;
	command.set_infos.nb_goldbars_in_mouth = -1;
	command.set_infos.nb_coins_in_boot = -1;
	command.set_infos.nb_coins_in_mouth = -1;

	if (!strcmp(res->arg1, "nb_goldbars_in_boot")) {
		command.set_infos.nb_goldbars_in_boot = res->arg2;
	}
	else if (!strcmp(res->arg1, "nb_goldbars_in_mouth")) {
		command.set_infos.nb_goldbars_in_mouth = res->arg2;
	}
	else if (!strcmp(res->arg1, "nb_coins_in_boot")) {
		command.set_infos.nb_coins_in_boot = res->arg2;
	}
	else if (!strcmp(res->arg1, "nb_coins_in_mouth")) {
		command.set_infos.nb_coins_in_mouth = res->arg2;
	}

	command.mode = I2C_SLAVEDSPIC_MODE_SET_INFOS;
	state_set_mode(&command);
}

prog_char str_state3_arg0[] = "set_infos";
parse_pgm_token_string_t cmd_state3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg0, str_state3_arg0);
prog_char str_state3_arg1[] = "nb_goldbars_in_boot#nb_goldbars_in_mouth#nb_coins_in_boot#nb_coins_in_mouth";
parse_pgm_token_string_t cmd_state3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg1, str_state3_arg1);
parse_pgm_token_num_t cmd_state3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_state3_result, arg2, INT8);

prog_char help_state3[] = "set slavedspic mode";
parse_pgm_inst_t cmd_state3 = {
	.f = cmd_state3_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state3,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state3_arg0, 
		(prog_void *)&cmd_state3_arg1, 
		(prog_void *)&cmd_state3_arg2,
		NULL,
	},
};

#ifdef notyet
/**********************************************************/
/* State_Machine */

/* this structure is filled when cmd_state_machine is parsed successfully */
struct cmd_state_machine_result {
	fixed_string_t arg0;
};

/* function called when cmd_state_machine is parsed successfully */
static void cmd_state_machine_parsed(__attribute__((unused)) void *parsed_result,
				     __attribute__((unused)) void *data)
{
	state_machines();
}

prog_char str_state_machine_arg0[] = "state_machine";
parse_pgm_token_string_t cmd_state_machine_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_machine_result, arg0, str_state_machine_arg0);

prog_char help_state_machine[] = "launch state machine";
parse_pgm_inst_t cmd_state_machine = {
	.f = cmd_state_machine_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state_machine,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state_machine_arg0, 
		NULL,
	},
};

/**********************************************************/
/* State_Debug */

/* this structure is filled when cmd_state_debug is parsed successfully */
struct cmd_state_debug_result {
	fixed_string_t arg0;
	uint8_t on;
};

/* function called when cmd_state_debug is parsed successfully */
static void cmd_state_debug_parsed(void *parsed_result,
				   __attribute__((unused)) void *data)
{
	//struct cmd_state_debug_result *res = parsed_result;
	//state_debug = res->on;
}

prog_char str_state_debug_arg0[] = "state_debug";
parse_pgm_token_string_t cmd_state_debug_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_debug_result, arg0, str_state_debug_arg0);
parse_pgm_token_num_t cmd_state_debug_on = TOKEN_NUM_INITIALIZER(struct cmd_state_debug_result, on, UINT8);

prog_char help_state_debug[] = "Set debug timer for state machine";
parse_pgm_inst_t cmd_state_debug = {
	.f = cmd_state_debug_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state_debug,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state_debug_arg0, 
		(prog_void *)&cmd_state_debug_on, 
		NULL,
	},
};

/**********************************************************/
/* Test */

/* this structure is filled when cmd_test is parsed successfully */
struct cmd_test_result {
	fixed_string_t arg0;
};

/* function called when cmd_test is parsed successfully */
static void cmd_test_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
}

prog_char str_test_arg0[] = "test";
parse_pgm_token_string_t cmd_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_test_result, arg0, str_test_arg0);

prog_char help_test[] = "Test function";
parse_pgm_inst_t cmd_test = {
	.f = cmd_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_test_arg0, 
		NULL,
	},
};

#endif
