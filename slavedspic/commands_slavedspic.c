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

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  commands_mechboard.c,v 1.5 2009/05/27 20:04:07 zer0 Exp
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <clock_time.h>
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
	if (!strcmp_P(res->color, PSTR("yellow"))) {
		slavedspic.our_color = I2C_COLOR_YELLOW;
	}
	else if (!strcmp_P(res->color, PSTR("green"))) {
		slavedspic.our_color = I2C_COLOR_GREEN;
	}
	printf_P(PSTR("Done\r\n"));
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "yellow#green";
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
/* stands_exchanger */

/* this structure is filled when cmd_stands_exchanger is parsed successfully */
struct cmd_stands_exchanger_result {
	fixed_string_t arg0;
	int32_t arg1;
};

/* function called when cmd_stands_exchanger is parsed successfully */
static void cmd_stands_exchanger_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{

	struct cmd_stands_exchanger_result *res = (struct cmd_stands_exchanger_result *) parsed_result;

	microseconds t1, t2;

	if (!strcmp_P(res->arg0, PSTR("exchanger"))) {
		stands_exchanger_set_position(res->arg1);
	
		t1 = time_get_us2();

		while (!stands_exchanger_test_traj_end()) {
			t2 = time_get_us2();
			if (t2 - t1 > 5000) {
				dump_cs_debug("exchanger", &slavedspic.stands_exchanger.cs);
				t1 = t2;
			}
		}
#if 0
		while (!cmdline_keypressed()) {
			t2 = time_get_us2();
			if (t2 - t1 > 5000) {
				dump_cs_debug("exchanger", &slavedspic.stands_exchanger.cs);
				t1 = t2;
			}
		}
#endif
	}
	else if (!strcmp_P(res->arg0, PSTR("exchanger_calibrate")))
		stands_exchanger_calibrate();
}

prog_char str_stands_exchanger_arg0[] = "exchanger#exchanger_calibrate";
parse_pgm_token_string_t cmd_stands_exchanger_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_stands_exchanger_result, arg0, str_stands_exchanger_arg0);
parse_pgm_token_num_t cmd_stands_exchanger_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_stands_exchanger_result, arg1, INT32);

prog_char help_stands_exchanger[] = "set stands_exchanger position";
parse_pgm_inst_t cmd_stands_exchanger = {
	.f = cmd_stands_exchanger_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_stands_exchanger,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_stands_exchanger_arg0, 
		(prog_void *)&cmd_stands_exchanger_arg1,
		NULL,
	},
};


/**********************************************************/
/* stands_blade */

/* this structure is filled when cmd_stands_blade is parsed successfully */
struct cmd_stands_blade_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_stands_blade is parsed successfully */
static void cmd_stands_blade_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_stands_blade_result *res = (struct cmd_stands_blade_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg0, PSTR("blade_left")))
		command.stands_blade.type = I2C_STANDS_BLADE_TYPE_LEFT;
	else if (!strcmp_P(res->arg0, PSTR("blade_right")))
		command.stands_blade.type = I2C_STANDS_BLADE_TYPE_RIGHT;	

	if (!strcmp_P(res->arg1, PSTR("hide_left")))
		command.stands_blade.mode = I2C_STANDS_BLADE_MODE_HIDE_LEFT;
	else if (!strcmp_P(res->arg1, PSTR("push_left")))
		command.stands_blade.mode = I2C_STANDS_BLADE_MODE_PUSH_STAND_LEFT;	
	else if (!strcmp_P(res->arg1, PSTR("center")))
		command.stands_blade.mode = I2C_STANDS_BLADE_MODE_CENTER;	
	else if (!strcmp_P(res->arg1, PSTR("push_right")))
		command.stands_blade.mode = I2C_STANDS_BLADE_MODE_PUSH_STAND_RIGHT;	
	else if (!strcmp_P(res->arg1, PSTR("hide_right")))
		command.stands_blade.mode = I2C_STANDS_BLADE_MODE_HIDE_RIGHT;	
	else if (!strcmp_P(res->arg1, PSTR("angle")))
		command.stands_blade.mode = I2C_STANDS_BLADE_MODE_SET_ANGLE;

	command.stands_blade.offset = res->arg2;

	command.mode = I2C_SLAVEDSPIC_MODE_STANDS_BLADE;
	state_set_mode(&command);
}

prog_char str_stands_blade_arg0[] = "blade_left#blade_right";
parse_pgm_token_string_t cmd_stands_blade_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_stands_blade_result, arg0, str_stands_blade_arg0);
prog_char str_stands_blade_arg1[] = "hide_left#push_left#center#push_right#hide_right#angle";
parse_pgm_token_string_t cmd_stands_blade_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_stands_blade_result, arg1, str_stands_blade_arg1);
parse_pgm_token_num_t cmd_stands_blade_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_stands_blade_result, arg2, INT8);

prog_char help_stands_blade[] = "set stands_blade mode, offset";
parse_pgm_inst_t cmd_stands_blade = {
	.f = cmd_stands_blade_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_stands_blade,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_stands_blade_arg0, 
		(prog_void *)&cmd_stands_blade_arg1,
		(prog_void *)&cmd_stands_blade_arg2,
		NULL,
	},
};


/**********************************************************/
/* stands_clamp */

/* this structure is filled when cmd_stands_clamp is parsed successfully */
struct cmd_stands_clamp_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_stands_clamp is parsed successfully */
static void cmd_stands_clamp_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_stands_clamp_result *res = (struct cmd_stands_clamp_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg0, PSTR("clamp_left")))
		command.stands_clamp.type = I2C_STANDS_CLAMP_TYPE_LEFT;
	else if (!strcmp_P(res->arg0, PSTR("clamp_right")))
		command.stands_clamp.type = I2C_STANDS_CLAMP_TYPE_RIGHT;	

	if (!strcmp_P(res->arg1, PSTR("full_open")))
		command.stands_clamp.mode = I2C_STANDS_CLAMP_MODE_FULL_OPEN;
	else if (!strcmp_P(res->arg1, PSTR("open")))
		command.stands_clamp.mode = I2C_STANDS_CLAMP_MODE_OPEN;
	else if (!strcmp_P(res->arg1, PSTR("close")))
		command.stands_clamp.mode = I2C_STANDS_CLAMP_MODE_CLOSE;	

	command.stands_clamp.offset = res->arg2;

	command.mode = I2C_SLAVEDSPIC_MODE_STANDS_CLAMP;
	state_set_mode(&command);
}

prog_char str_stands_clamp_arg0[] = "clamp_left#clamp_right";
parse_pgm_token_string_t cmd_stands_clamp_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_stands_clamp_result, arg0, str_stands_clamp_arg0);
prog_char str_stands_clamp_arg1[] = "full_open#open#close";
parse_pgm_token_string_t cmd_stands_clamp_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_stands_clamp_result, arg1, str_stands_clamp_arg1);
parse_pgm_token_num_t cmd_stands_clamp_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_stands_clamp_result, arg2, INT8);

prog_char help_stands_clamp[] = "set stands_clamp mode, offset";
parse_pgm_inst_t cmd_stands_clamp = {
	.f = cmd_stands_clamp_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_stands_clamp,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_stands_clamp_arg0, 
		(prog_void *)&cmd_stands_clamp_arg1,
		(prog_void *)&cmd_stands_clamp_arg2,
		NULL,
	},
};


/**********************************************************/
/* stands_elevator */

/* this structure is filled when cmd_stands_elevator is parsed successfully */
struct cmd_stands_elevator_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_stands_elevator is parsed successfully */
static void cmd_stands_elevator_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_stands_elevator_result *res = (struct cmd_stands_elevator_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg0, PSTR("elevator_left")))
		command.stands_elevator.type = I2C_STANDS_ELEVATOR_TYPE_LEFT;
	else if (!strcmp_P(res->arg0, PSTR("elevator_right")))
		command.stands_elevator.type = I2C_STANDS_ELEVATOR_TYPE_RIGHT;	

	if (!strcmp_P(res->arg1, PSTR("up")))
		command.stands_elevator.mode = I2C_STANDS_ELEVATOR_MODE_UP;
	else if (!strcmp_P(res->arg1, PSTR("down")))
		command.stands_elevator.mode = I2C_STANDS_ELEVATOR_MODE_DOWN;	

	command.stands_elevator.offset = res->arg2;

	command.mode = I2C_SLAVEDSPIC_MODE_STANDS_ELEVATOR;
	state_set_mode(&command);
}

prog_char str_stands_elevator_arg0[] = "elevator_left#elevator_right";
parse_pgm_token_string_t cmd_stands_elevator_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_stands_elevator_result, arg0, str_stands_elevator_arg0);
prog_char str_stands_elevator_arg1[] = "up#down";
parse_pgm_token_string_t cmd_stands_elevator_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_stands_elevator_result, arg1, str_stands_elevator_arg1);
parse_pgm_token_num_t cmd_stands_elevator_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_stands_elevator_result, arg2, INT8);

prog_char help_stands_elevator[] = "set stands_elevator mode, offset";
parse_pgm_inst_t cmd_stands_elevator = {
	.f = cmd_stands_elevator_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_stands_elevator,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_stands_elevator_arg0, 
		(prog_void *)&cmd_stands_elevator_arg1,
		(prog_void *)&cmd_stands_elevator_arg2,
		NULL,
	},
};


/**********************************************************/
/* stands_tower_clamps */

/* this structure is filled when cmd_stands_tower_clamps is parsed successfully */
struct cmd_stands_tower_clamps_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_stands_tower_clamps is parsed successfully */
static void cmd_stands_tower_clamps_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_stands_tower_clamps_result *res = (struct cmd_stands_tower_clamps_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("unlock_left")))
		command.stands_tower_clamps.mode = I2C_STANDS_TOWER_CLAMPS_MODE_UNLOCK_LEFT;
	else if (!strcmp_P(res->arg1, PSTR("lock")))
		command.stands_tower_clamps.mode = I2C_STANDS_TOWER_CLAMPS_MODE_LOCK;
	else if (!strcmp_P(res->arg1, PSTR("unlock_right")))
		command.stands_tower_clamps.mode = I2C_STANDS_TOWER_CLAMPS_MODE_UNLOCK_RIGHT;

	command.stands_tower_clamps.offset = res->arg2;	

	command.mode = I2C_SLAVEDSPIC_MODE_STANDS_TOWER_CLAMPS;
	state_set_mode(&command);
}

prog_char str_stands_tower_clamps_arg0[] = "tower_clamps";
parse_pgm_token_string_t cmd_stands_tower_clamps_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_stands_tower_clamps_result, arg0, str_stands_tower_clamps_arg0);
prog_char str_stands_tower_clamps_arg1[] = "unlock_left#lock#unlock_right";
parse_pgm_token_string_t cmd_stands_tower_clamps_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_stands_tower_clamps_result, arg1, str_stands_tower_clamps_arg1);
parse_pgm_token_num_t cmd_stands_tower_clamps_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_stands_tower_clamps_result, arg2, INT8);

prog_char help_stands_tower_clamps[] = "set stands_tower_clamps mode, offset";
parse_pgm_inst_t cmd_stands_tower_clamps = {
	.f = cmd_stands_tower_clamps_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_stands_tower_clamps,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_stands_tower_clamps_arg0, 
		(prog_void *)&cmd_stands_tower_clamps_arg1,
		(prog_void *)&cmd_stands_tower_clamps_arg2,
		NULL,
	},
};


/**********************************************************/
/* cup_clamp_popcorn_door */

/* this structure is filled when cmd_cup_clamp_popcorn_door is parsed successfully */
struct cmd_cup_clamp_popcorn_door_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_cup_clamp_popcorn_door is parsed successfully */
static void cmd_cup_clamp_popcorn_door_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_cup_clamp_popcorn_door_result *res = (struct cmd_cup_clamp_popcorn_door_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg0, PSTR("clamp_door_left")))
		command.cup_clamp_popcorn_door.type = I2C_CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT;
	else if (!strcmp_P(res->arg0, PSTR("clamp_door_right")))
		command.cup_clamp_popcorn_door.type = I2C_CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT;	

	if (!strcmp_P(res->arg1, PSTR("hide")))
		command.cup_clamp_popcorn_door.mode = I2C_CUP_CLAMP_MODE_HIDE;
	else if (!strcmp_P(res->arg1, PSTR("cup_locked")))
		command.cup_clamp_popcorn_door.mode = I2C_CUP_CLAMP_MODE_LOCKED;	
	else if (!strcmp_P(res->arg1, PSTR("cup_unlocked")))
		command.cup_clamp_popcorn_door.mode = I2C_CUP_CLAMP_MODE_OPEN;	
	else if (!strcmp_P(res->arg1, PSTR("door_close")))
		command.cup_clamp_popcorn_door.mode = I2C_POPCORN_DOOR_MODE_CLOSE;	
	else if (!strcmp_P(res->arg1, PSTR("door_open")))
		command.cup_clamp_popcorn_door.mode = I2C_POPCORN_DOOR_MODE_OPEN;	

	command.cup_clamp_popcorn_door.offset = res->arg2;

	command.mode = I2C_SLAVEDSPIC_MODE_CUP_CLAMP_POPCORN_DOOR;
	state_set_mode(&command);
}

prog_char str_cup_clamp_popcorn_door_arg0[] = "clamp_door_left#clamp_door_right";
parse_pgm_token_string_t cmd_cup_clamp_popcorn_door_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_cup_clamp_popcorn_door_result, arg0, str_cup_clamp_popcorn_door_arg0);
prog_char str_cup_clamp_popcorn_door_arg1[] = "hide#cup_locked#cup_unlocked#door_close#door_open";
parse_pgm_token_string_t cmd_cup_clamp_popcorn_door_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_cup_clamp_popcorn_door_result, arg1, str_cup_clamp_popcorn_door_arg1);
parse_pgm_token_num_t cmd_cup_clamp_popcorn_door_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_cup_clamp_popcorn_door_result, arg2, INT8);

prog_char help_cup_clamp_popcorn_door[] = "set cup_clamp_popcorn_door mode, offset";
parse_pgm_inst_t cmd_cup_clamp_popcorn_door = {
	.f = cmd_cup_clamp_popcorn_door_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_cup_clamp_popcorn_door,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_cup_clamp_popcorn_door_arg0, 
		(prog_void *)&cmd_cup_clamp_popcorn_door_arg1,
		(prog_void *)&cmd_cup_clamp_popcorn_door_arg2,
		NULL,
	},
};


/**********************************************************/
/* popcorn_tray */

/* this structure is filled when cmd_popcorn_tray is parsed successfully */
struct cmd_popcorn_tray_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_popcorn_tray is parsed successfully */
static void cmd_popcorn_tray_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_popcorn_tray_result *res = (struct cmd_popcorn_tray_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("open")))
		command.popcorn_tray.mode = I2C_POPCORN_TRAY_MODE_OPEN;
	else if (!strcmp_P(res->arg1, PSTR("close")))
		command.popcorn_tray.mode = I2C_POPCORN_TRAY_MODE_CLOSE;	

	command.popcorn_tray.offset = res->arg2;	

	command.mode = I2C_SLAVEDSPIC_MODE_POPCORN_TRAY;
	state_set_mode(&command);
}

prog_char str_popcorn_tray_arg0[] = "tray";
parse_pgm_token_string_t cmd_popcorn_tray_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_popcorn_tray_result, arg0, str_popcorn_tray_arg0);
prog_char str_popcorn_tray_arg1[] = "open#close";
parse_pgm_token_string_t cmd_popcorn_tray_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_popcorn_tray_result, arg1, str_popcorn_tray_arg1);
parse_pgm_token_num_t cmd_popcorn_tray_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_popcorn_tray_result, arg2, INT8);

prog_char help_popcorn_tray[] = "set popcorn_tray mode, offset";
parse_pgm_inst_t cmd_popcorn_tray = {
	.f = cmd_popcorn_tray_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_popcorn_tray,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_popcorn_tray_arg0, 
		(prog_void *)&cmd_popcorn_tray_arg1,
		(prog_void *)&cmd_popcorn_tray_arg2,
		NULL,
	},
};


/**********************************************************/
/* popcorn_ramps */

/* this structure is filled when cmd_popcorn_ramps is parsed successfully */
struct cmd_popcorn_ramps_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_popcorn_ramps is parsed successfully */
static void cmd_popcorn_ramps_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_popcorn_ramps_result *res = (struct cmd_popcorn_ramps_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("hide")))
		command.popcorn_ramps.mode = I2C_POPCORN_RAMPS_MODE_HIDE;
	else if (!strcmp_P(res->arg1, PSTR("harvest")))
		command.popcorn_ramps.mode = I2C_POPCORN_RAMPS_MODE_HARVEST;	
	else if (!strcmp_P(res->arg1, PSTR("open")))
		command.popcorn_ramps.mode = I2C_POPCORN_RAMPS_MODE_OPEN;	

	command.popcorn_ramps.offset = res->arg2;

	command.mode = I2C_SLAVEDSPIC_MODE_POPCORN_RAMPS;
	state_set_mode(&command);
}

prog_char str_popcorn_ramps_arg0[] = "ramps";
parse_pgm_token_string_t cmd_popcorn_ramps_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_popcorn_ramps_result, arg0, str_popcorn_ramps_arg0);
prog_char str_popcorn_ramps_arg1[] = "hide#harvest#open";
parse_pgm_token_string_t cmd_popcorn_ramps_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_popcorn_ramps_result, arg1, str_popcorn_ramps_arg1);
parse_pgm_token_num_t cmd_popcorn_ramps_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_popcorn_ramps_result, arg2, INT8);

prog_char help_popcorn_ramps[] = "set popcorn_ramps mode, offset";
parse_pgm_inst_t cmd_popcorn_ramps = {
	.f = cmd_popcorn_ramps_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_popcorn_ramps,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_popcorn_ramps_arg0, 
		(prog_void *)&cmd_popcorn_ramps_arg1,
		(prog_void *)&cmd_popcorn_ramps_arg2,
		NULL,
	},
};


/**********************************************************/
/* cup_clamp_front */

/* this structure is filled when cmd_cup_clamp_front is parsed successfully */
struct cmd_cup_clamp_front_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_cup_clamp_front is parsed successfully */
static void cmd_cup_clamp_front_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_cup_clamp_front_result *res = (struct cmd_cup_clamp_front_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("open")))
		command.cup_clamp_front.mode = I2C_CUP_CLAMP_FRONT_MODE_OPEN;
	else if (!strcmp_P(res->arg1, PSTR("cup_locked")))
		command.cup_clamp_front.mode = I2C_CUP_CLAMP_FRONT_MODE_CUP_LOCKED;

	command.cup_clamp_front.offset = res->arg2;

	command.mode = I2C_SLAVEDSPIC_MODE_CUP_CLAMP_FRONT;
	state_set_mode(&command);
}

prog_char str_cup_clamp_front_arg0[] = "cup_clamp_front";
parse_pgm_token_string_t cmd_cup_clamp_front_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_cup_clamp_front_result, arg0, str_cup_clamp_front_arg0);
prog_char str_cup_clamp_front_arg1[] = "open#cup_locked";
parse_pgm_token_string_t cmd_cup_clamp_front_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_cup_clamp_front_result, arg1, str_cup_clamp_front_arg1);
parse_pgm_token_num_t cmd_cup_clamp_front_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_cup_clamp_front_result, arg2, INT8);

prog_char help_cup_clamp_front[] = "set cup_clamp_front mode, offset";
parse_pgm_inst_t cmd_cup_clamp_front = {
	.f = cmd_cup_clamp_front_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_cup_clamp_front,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_cup_clamp_front_arg0, 
		(prog_void *)&cmd_cup_clamp_front_arg1,
		(prog_void *)&cmd_cup_clamp_front_arg2,
		NULL,
	},
};


/**********************************************************/
/* cup_holder_front */

/* this structure is filled when cmd_cup_holder_front is parsed successfully */
struct cmd_cup_holder_front_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_cup_holder_front is parsed successfully */
static void cmd_cup_holder_front_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_cup_holder_front_result *res = (struct cmd_cup_holder_front_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("cup_hold")))
		command.cup_holder_front.mode = I2C_CUP_HOLDER_FRONT_MODE_CUP_HOLD;
	else if (!strcmp_P(res->arg1, PSTR("ready")))
		command.cup_holder_front.mode = I2C_CUP_HOLDER_FRONT_MODE_READY;
	else if (!strcmp_P(res->arg1, PSTR("hide")))
		command.cup_holder_front.mode = I2C_CUP_HOLDER_FRONT_MODE_HIDE;

	command.cup_holder_front.offset = res->arg2;

	command.mode = I2C_SLAVEDSPIC_MODE_CUP_HOLDER_FRONT;
	state_set_mode(&command);
}

prog_char str_cup_holder_front_arg0[] = "cup_holder_front";
parse_pgm_token_string_t cmd_cup_holder_front_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_cup_holder_front_result, arg0, str_cup_holder_front_arg0);
prog_char str_cup_holder_front_arg1[] = "cup_hold#ready#hide";
parse_pgm_token_string_t cmd_cup_holder_front_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_cup_holder_front_result, arg1, str_cup_holder_front_arg1);
parse_pgm_token_num_t cmd_cup_holder_front_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_cup_holder_front_result, arg2, INT8);

prog_char help_cup_holder_front[] = "set cup_holder_front mode, offset";
parse_pgm_inst_t cmd_cup_holder_front = {
	.f = cmd_cup_holder_front_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_cup_holder_front,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_cup_holder_front_arg0, 
		(prog_void *)&cmd_cup_holder_front_arg1,
		(prog_void *)&cmd_cup_holder_front_arg2,
		NULL,
	},
};


/**********************************************************/
/* popcorn_system */

/* this structure is filled when cmd_popcorn_system is parsed successfully */
struct cmd_popcorn_system_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_popcorn_system is parsed successfully */
static void cmd_popcorn_system_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_popcorn_system_result *res = (struct cmd_popcorn_system_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("idle")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
	else if (!strcmp_P(res->arg1, PSTR("cup_f_ready")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_READY;
	else if (!strcmp_P(res->arg1, PSTR("cup_f_drop")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_CATCH_AND_DROP;
	else if (!strcmp_P(res->arg1, PSTR("cup_f_release")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_RELEASE;
	else if (!strcmp_P(res->arg1, PSTR("cup_f_hide")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_HIDE;
	else if (!strcmp_P(res->arg1, PSTR("cup_r_open")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_OPEN;
	else if (!strcmp_P(res->arg1, PSTR("cup_r_catch")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_CATCH;
	else if (!strcmp_P(res->arg1, PSTR("cup_r_release")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_RELEASE;
	else if (!strcmp_P(res->arg1, PSTR("machines_ready")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_MACHINES_READY;
	else if (!strcmp_P(res->arg1, PSTR("machines_harvest")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_MACHINES_HARVEST;
	else if (!strcmp_P(res->arg1, PSTR("machines_end")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_MACHINES_END;
	else if (!strcmp_P(res->arg1, PSTR("stock_drop")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_STOCK_DROP;
	else if (!strcmp_P(res->arg1, PSTR("stock_end")))
		command.popcorn_system.mode = I2C_SLAVEDSPIC_MODE_PS_STOCK_END;

	command.mode = I2C_SLAVEDSPIC_MODE_POPCORN_SYSTEM;
	state_set_mode(&command);
}

prog_char str_popcorn_system_arg0[] = "ps";
parse_pgm_token_string_t cmd_popcorn_system_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_popcorn_system_result, arg0, str_popcorn_system_arg0);
prog_char str_popcorn_system_arg1[] = "idle#cup_f_ready#cup_f_drop#cup_f_release#cup_f_hide#cup_r_open#cup_r_catch#cup_r_release#machines_ready#machines_harvest#machines_end#stock_drop#stock_end";
parse_pgm_token_string_t cmd_popcorn_system_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_popcorn_system_result, arg1, str_popcorn_system_arg1);

prog_char help_popcorn_system[] = "set popcorn_system mode";
parse_pgm_inst_t cmd_popcorn_system = {
	.f = cmd_popcorn_system_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_popcorn_system,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_popcorn_system_arg0, 
		(prog_void *)&cmd_popcorn_system_arg1,
		NULL,
	},
};


/**********************************************************/
/* stands_system */

/* this structure is filled when cmd_stands_system is parsed successfully */
struct cmd_stands_system_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
	uint8_t arg3;
};

/* function called when cmd_stands_system is parsed successfully */
static void cmd_stands_system_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_stands_system_result *res = (struct cmd_stands_system_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("left")))
		command.stands_system.side = I2C_SIDE_LEFT;
	else if (!strcmp_P(res->arg1, PSTR("right")))
		command.stands_system.side = I2C_SIDE_RIGHT;

	if (!strcmp_P(res->arg2, PSTR("idle")))
		command.stands_system.mode = I2C_SLAVEDSPIC_MODE_SS_IDLE;
	else if (!strcmp_P(res->arg2, PSTR("hide_tower")))
		command.stands_system.mode = I2C_SLAVEDSPIC_MODE_SS_HIDE_TOWER;
	else if (!strcmp_P(res->arg2, PSTR("harvest_stand_do")))
		command.stands_system.mode = I2C_SLAVEDSPIC_MODE_SS_HARVEST_STAND_DO;
	else if (!strcmp_P(res->arg2, PSTR("harvest_stand_ready")))
		command.stands_system.mode = I2C_SLAVEDSPIC_MODE_SS_HARVEST_STAND_READY;
	else if (!strcmp_P(res->arg2, PSTR("build_spotlight")))
		command.stands_system.mode = I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT;
	else if (!strcmp_P(res->arg2, PSTR("release_spotlight")))
		command.stands_system.mode = I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT;

	command.stands_system.blade_angle = res->arg3;

	command.mode = I2C_SLAVEDSPIC_MODE_STANDS_SYSTEM;
	state_set_mode(&command);
}

prog_char str_stands_system_arg0[] = "ss";
parse_pgm_token_string_t cmd_stands_system_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_stands_system_result, arg0, str_stands_system_arg0);
prog_char str_stands_system_arg1[] = "left#right";
parse_pgm_token_string_t cmd_stands_system_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_stands_system_result, arg1, str_stands_system_arg1);
prog_char str_stands_system_arg2[] = "idle#hide_tower#harvest_stand_do#harvest_stand_ready#build_spotlight#release_spotlight";
parse_pgm_token_string_t cmd_stands_system_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_stands_system_result, arg2, str_stands_system_arg2);
parse_pgm_token_num_t cmd_stands_system_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_stands_system_result, arg3, UINT8);

prog_char help_stands_system[] = "set stands_system side, mode, blade angle";
parse_pgm_inst_t cmd_stands_system = {
	.f = cmd_stands_system_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_stands_system,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_stands_system_arg0, 
		(prog_void *)&cmd_stands_system_arg1,
		(prog_void *)&cmd_stands_system_arg2,
		(prog_void *)&cmd_stands_system_arg3,
		NULL,
	},
};


/**********************************************************/
/* State2 */

/* this structure is filled when cmd_state2 is parsed successfully */
struct cmd_state2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
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

		printf("not implemented");
#if 0
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
#endif
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

	command.set_infos.cup_front_catched = -1;
	command.set_infos.cup_rear_catched = -1;
	command.set_infos.machine_popcorns_catched = -1;
	command.set_infos.stored_stands_l = -1;
	command.set_infos.stored_stands_r = -1;

	if (!strcmp(res->arg1, "cup_front_catched")) {
		command.set_infos.cup_front_catched = res->arg2;
	}
	else if (!strcmp(res->arg1, "cup_rear_catched")) {
		command.set_infos.cup_rear_catched = res->arg2;
	}
	else if (!strcmp(res->arg1, "machines_catched")) {
		command.set_infos.machine_popcorns_catched = res->arg2;
	}
	else if (!strcmp(res->arg1, "stands_left")) {
		command.set_infos.stored_stands_l = res->arg2;
	}
	else if (!strcmp(res->arg1, "stands_right")) {
		command.set_infos.stored_stands_r = res->arg2;
	}

	command.mode = I2C_SLAVEDSPIC_MODE_SET_INFOS;
	state_set_mode(&command);
}

prog_char str_state3_arg0[] = "set_infos";
parse_pgm_token_string_t cmd_state3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg0, str_state3_arg0);
prog_char str_state3_arg1[] = "cup_front_catched#cup_rear_catched#machines_catched#stands_left#stands_right";
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



//#if 0
//#ifdef notyet
///**********************************************************/
///* State_Machine */
//
///* this structure is filled when cmd_state_machine is parsed successfully */
//struct cmd_state_machine_result {
//	fixed_string_t arg0;
//};
//
///* function called when cmd_state_machine is parsed successfully */
//static void cmd_state_machine_parsed(__attribute__((unused)) void *parsed_result,
//				     __attribute__((unused)) void *data)
//{
//	state_machines();
//}
//
//prog_char str_state_machine_arg0[] = "state_machine";
//parse_pgm_token_string_t cmd_state_machine_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_machine_result, arg0, str_state_machine_arg0);
//
//prog_char help_state_machine[] = "launch state machine";
//parse_pgm_inst_t cmd_state_machine = {
//	.f = cmd_state_machine_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state_machine,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state_machine_arg0, 
//		NULL,
//	},
//};
//
///**********************************************************/
///* State_Debug */
//
///* this structure is filled when cmd_state_debug is parsed successfully */
//struct cmd_state_debug_result {
//	fixed_string_t arg0;
//	uint8_t on;
//};
//
///* function called when cmd_state_debug is parsed successfully */
//static void cmd_state_debug_parsed(void *parsed_result,
//				   __attribute__((unused)) void *data)
//{
//	//struct cmd_state_debug_result *res = parsed_result;
//	//state_debug = res->on;
//}
//
//prog_char str_state_debug_arg0[] = "state_debug";
//parse_pgm_token_string_t cmd_state_debug_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_debug_result, arg0, str_state_debug_arg0);
//parse_pgm_token_num_t cmd_state_debug_on = TOKEN_NUM_INITIALIZER(struct cmd_state_debug_result, on, UINT8);
//
//prog_char help_state_debug[] = "Set debug timer for state machine";
//parse_pgm_inst_t cmd_state_debug = {
//	.f = cmd_state_debug_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_state_debug,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_state_debug_arg0, 
//		(prog_void *)&cmd_state_debug_on, 
//		NULL,
//	},
//};
//
//#endif
