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
	microseconds t1, t2;

	if (!strcmp_P(res->arg0, PSTR("lift"))) {
		lift_set_height(res->arg1);
	
		t1 = time_get_us2();
		while (!lift_check_height_reached()) {
			t2 = time_get_us2();
			if (t2 - t1 > 20000) {
				dump_cs_debug("lift", &slavedspic.lift.cs);
				t1 = t2;
			}
		}
	}
	if (!strcmp_P(res->arg0, PSTR("lift_calibrate")))
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
	
	if(data) {
		if (!strcmp_P(res->arg1, PSTR("on")))
			turbine_power_on(&slavedspic.turbine);
		else if (!strcmp_P(res->arg1, PSTR("off")))
			turbine_power_off(&slavedspic.turbine);		
	}
	else {
		if (!strcmp_P(res->arg1, PSTR("angle"))) {
			turbine_set_angle(&slavedspic.turbine, res->arg2, slavedspic.turbine.angle_speed);
			while(!turbine_check_angle_reached(&slavedspic.turbine));
		}
		else if (!strcmp_P(res->arg1, PSTR("speed_angle")))
			slavedspic.turbine.angle_speed = res->arg2;
		else if (!strcmp_P(res->arg1, PSTR("blow")))
			turbine_set_blow_speed(&slavedspic.turbine, res->arg2);
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
};

/* function called when cmd_fingers is parsed successfully */
static void cmd_fingers_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_fingers_result *res = (struct cmd_fingers_result *) parsed_result;
	uint8_t mode;
	fingers_t *fingers = 0;
	
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
	while(!fingers_check_mode_done(fingers));
	printf("done\n\r");

}

prog_char str_fingers_arg0[] = "fingers";
parse_pgm_token_string_t cmd_fingers_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_fingers_result, arg0, str_fingers_arg0);
prog_char str_fingers_arg1[] = "totem#floor";
parse_pgm_token_string_t cmd_fingers_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_fingers_result, arg1, str_fingers_arg1);
prog_char str_fingers_arg2[] = "hug#open#hold#close#pushin";
parse_pgm_token_string_t cmd_fingers_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_fingers_result, arg2, str_fingers_arg2);

prog_char help_fingers[] = "set fingers mode";
parse_pgm_inst_t cmd_fingers = {
	.f = cmd_fingers_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_fingers,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_fingers_arg0, 
		(prog_void *)&cmd_fingers_arg1,
		(prog_void *)&cmd_fingers_arg2,
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
};

/* function called when cmd_arm is parsed successfully */
static void cmd_arm_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_arm_result *res = (struct cmd_arm_result *) parsed_result;
	uint8_t mode;
	arm_t *arm = 0;

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
	while(!arm_check_mode_done(arm));
	printf("done\n\r");

}

prog_char str_arm_arg0[] = "arm";
parse_pgm_token_string_t cmd_arm_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_arm_result, arg0, str_arm_arg0);
prog_char str_arm_arg1[] = "left#right";
parse_pgm_token_string_t cmd_arm_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_arm_result, arg1, str_arm_arg1);
prog_char str_arm_arg2[] = "hide#show#goldbar#floor";
parse_pgm_token_string_t cmd_arm_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_arm_result, arg2, str_arm_arg2);

prog_char help_arm[] = "set arm mode";
parse_pgm_inst_t cmd_arm = {
	.f = cmd_arm_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_arm,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_arm_arg0, 
		(prog_void *)&cmd_arm_arg1,
		(prog_void *)&cmd_arm_arg2,
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
	uint8_t mode;
	tray_t *tray = 0;

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
	uint8_t mode;

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
	uint8_t mode;

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
/* State1 */

/* this structure is filled when cmd_state1 is parsed successfully */
struct cmd_state1_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_state1 is parsed successfully */
static void cmd_state1_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
#ifdef notyet
	struct cmd_state1_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;


	if (!strcmp(res->arg1, "status")){
	
		printf("\n\r");

		printf("rear: %d %d %d \n\r", 
				slavedspic.ts[I2C_SIDE_REAR].state,
				slavedspic.ts[I2C_SIDE_REAR].belts_blocked,
				slavedspic.ts[I2C_SIDE_REAR].token_catched);
	
		printf("front: %d %d %d \n\r", 
				slavedspic.ts[I2C_SIDE_FRONT].state,
				slavedspic.ts[I2C_SIDE_FRONT].belts_blocked,
				slavedspic.ts[I2C_SIDE_FRONT].token_catched);

		printf("\n\r");

		return;
	}
	else{
		if (!strcmp(res->arg1, "init"))
			command.mode = I2C_SLAVEDSPIC_MODE_INIT;
		else if (!strcmp(res->arg1, "rear_stop")){
			command.mode = I2C_SLAVEDSPIC_MODE_TOKEN_STOP;
			command.ts.side = I2C_SIDE_REAR;
		}
		else if (!strcmp(res->arg1, "front_stop")){
			command.mode = I2C_SLAVEDSPIC_MODE_TOKEN_STOP;
			command.ts.side = I2C_SIDE_FRONT;
		}

		state_set_mode(&command);
	}
#endif
}

prog_char str_state1_arg0[] = "state";
parse_pgm_token_string_t cmd_state1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state1_result, arg0, str_state1_arg0);
prog_char str_state1_arg1[] = "status#init#rear_stop#front_stop";
parse_pgm_token_string_t cmd_state1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state1_result, arg1, str_state1_arg1);

prog_char help_state1[] = "set slavedspic mode";
parse_pgm_inst_t cmd_state1 = {
	.f = cmd_state1_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state1,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state1_arg0, 
		(prog_void *)&cmd_state1_arg1, 
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
#ifdef notyet
	struct cmd_state2_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;


	if (!strcmp(res->arg1, "rear")) {
		command.ts.side = I2C_SIDE_REAR;
	}
	else if (!strcmp(res->arg1, "front"))
		command.ts.side = I2C_SIDE_FRONT;

	if (!strcmp(res->arg2, "take"))
		command.mode = I2C_SLAVEDSPIC_MODE_TOKEN_TAKE;
	else if (!strcmp(res->arg2, "eject"))
		command.mode = I2C_SLAVEDSPIC_MODE_TOKEN_EJECT;
	else if (!strcmp(res->arg2, "show"))
		command.mode = I2C_SLAVEDSPIC_MODE_TOKEN_SHOW;
	else if (!strcmp(res->arg2, "push_r"))
		command.mode = I2C_SLAVEDSPIC_MODE_TOKEN_PUSH_R;
	else if (!strcmp(res->arg2, "push_l"))
		command.mode = I2C_SLAVEDSPIC_MODE_TOKEN_PUSH_L;

	command.ts.speed_div4 = res->arg3;
	
	state_set_mode(&command);
#endif
}

prog_char str_state2_arg0[] = "state";
parse_pgm_token_string_t cmd_state2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg0, str_state2_arg0);
prog_char str_state2_arg1[] = "rear#front";
parse_pgm_token_string_t cmd_state2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg1, str_state2_arg1);
prog_char str_state2_arg2[] = "take#eject#show#push_l#push_r";
parse_pgm_token_string_t cmd_state2_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg2, str_state2_arg2);
parse_pgm_token_num_t cmd_state2_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_state2_result, arg3, UINT8);

prog_char help_state2[] = "set slavedspic mode";
parse_pgm_inst_t cmd_state2 = {
	.f = cmd_state2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state2_arg0, 
		(prog_void *)&cmd_state2_arg1, 
		(prog_void *)&cmd_state2_arg2,
		(prog_void *)&cmd_state2_arg3,
		NULL,
	},
};


/**********************************************************/
/* State3 */

/* this structure is filled when cmd_state3 is parsed successfully */
struct cmd_state3_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint16_t arg2;
	
};

/* function called when cmd_state3 is parsed successfully */
static void cmd_state3_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
#ifdef notyet
	struct cmd_state3_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;


	if (!strcmp(res->arg1, "mright")) {
		command.mirror.side = I2C_MIRROR_SIDE_RIGHT;
	}
	else if (!strcmp(res->arg1, "mleft"))
		command.mirror.side = I2C_MIRROR_SIDE_LEFT;

	command.mirror.pos_h = (uint8_t)(res->arg2 >> 8);
	command.mirror.pos_l = (uint8_t)(0x00FF & res->arg2);

	command.mode = I2C_SLAVEDSPIC_MODE_MIRROR_POS;

	state_set_mode(&command);

	printf("done\n\r");
#endif
}

prog_char str_state3_arg0[] = "state";
parse_pgm_token_string_t cmd_state3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg0, str_state3_arg0);
prog_char str_state3_arg1[] = "mright#mleft";
parse_pgm_token_string_t cmd_state3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg1, str_state3_arg1);
parse_pgm_token_num_t cmd_state3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_state3_result, arg2, UINT16);

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


