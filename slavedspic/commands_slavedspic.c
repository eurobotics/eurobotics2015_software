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
		bit = 0;
		if (!strcmp_P(res->arg2, PSTR("on")))
			slavedspic.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			slavedspic.flags &= bit;
		else { /* show */
			//printf_P(PSTR("encoders is %s\r\n"), 
			//	 (DO_ENCODERS & slavedspic.flags) ? "on":"off");
		}
		return;
	}

	//if (!strcmp_P(res->arg1, PSTR("encoders")))
	//	bit = DO_ENCODERS;

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
prog_char str_event_arg1[] = "all";
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
	if (!strcmp_P(res->color, PSTR("red"))) {
		slavedspic.our_color = I2C_COLOR_RED;
	}
	else if (!strcmp_P(res->color, PSTR("blue"))) {
		slavedspic.our_color = I2C_COLOR_BLUE;
	}
	printf_P(PSTR("Done\r\n"));
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "red#blue";
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
/* belts */

/* this structure is filled when cmd_belts is parsed successfully */
struct cmd_belts_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
	uint16_t arg3;
};

/* function called when cmd_belts is parsed successfully */
static void cmd_belts_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_belts_result *res = (struct cmd_belts_result *) parsed_result;

	uint8_t side, mode, sensor;
	microseconds time_us;

	/* side */
	if (!strcmp(res->arg1, "rear")){
		side = BELTS_SIDE_REAR;
		sensor = S_REAR_TOKEN_STOP;
	}
	else{
		side = BELTS_SIDE_FRONT;
		sensor = S_FRONT_TOKEN_STOP;
	}

	/* mode */
	if (!strcmp(res->arg2, "in"))
		mode = BELTS_MODE_IN;
	else if (!strcmp(res->arg2, "out"))
		mode = BELTS_MODE_OUT;
	else if (!strcmp(res->arg2, "left"))
		mode = BELTS_MODE_LEFT;
	else
		mode = BELTS_MODE_RIGHT;
	
	/* execute */
	time_us = time_get_us2();
	belts_mode_set(side, mode, res->arg3);

	/* test performance */
#if 0
	printf("press a key for end ...\n\r");
	do{
		printf("load = %d\n\r", (uint16_t)belts_load_get(side));
		
		/* stop if final carrier is reached */
		if(sensor_get(sensor) && (mode==BELTS_MODE_IN))
			belts_set_mode(side, BELTS_MODE_OUT, 0);
		
		wait_ms(50);
	}while(!cmdline_keypressed());
#else
	printf("press a key for end ...\n\r");
	if(mode==BELTS_MODE_IN){
		while(!cmdline_keypressed() && !sensor_get(sensor));
		time_us = time_get_us2() - time_us;
		printf("input time = %d ms\n\r", (int16_t)(time_us/1000));
	}
	else if(mode==BELTS_MODE_OUT)
		WAIT_COND_OR_TIMEOUT(cmdline_keypressed(), 700);
	else
		while(!cmdline_keypressed());			
#endif

	/* stop belts */
	belts_mode_set(side, BELTS_MODE_OUT, 0);
	printf("done\r\n");
}

prog_char str_belts_arg0[] = "belts";
parse_pgm_token_string_t cmd_belts_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_belts_result, arg0, str_belts_arg0);
prog_char str_belts_arg1[] = "rear#front";
parse_pgm_token_string_t cmd_belts_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_belts_result, arg1, str_belts_arg1);
prog_char str_belts_arg2[] = "in#out#left#right";
parse_pgm_token_string_t cmd_belts_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_belts_result, arg2, str_belts_arg2);
parse_pgm_token_num_t cmd_belts_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_belts_result, arg3, UINT16);


prog_char help_belts[] = "manage belts";
parse_pgm_inst_t cmd_belts = {
	.f = cmd_belts_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_belts,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_belts_arg0, 
		(prog_void *)&cmd_belts_arg1, 
		(prog_void *)&cmd_belts_arg2, 
		(prog_void *)&cmd_belts_arg3, 
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
	struct cmd_state_debug_result *res = parsed_result;
	state_debug = res->on;
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


