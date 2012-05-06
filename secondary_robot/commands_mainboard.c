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
 *  Revision : $Id: commands_mainboard.c,v 1.8 2009/05/27 20:04:07 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  commands_mainboard.c,v 1.8 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <time.h>
#include <i2c_mem.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"

#include "main.h"
#include "sensor.h"
#include "cmdline.h"
#include "strat.h"
#include "strat_utils.h"
#include "strat_base.h"
#include "i2c_protocol.h"
#include "actuator.h"
#include "beacon.h"

extern int8_t beacon_connected;

struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};


/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, void *data)
{
	u08 bit=0;

	struct cmd_event_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("all"))) {
		bit = DO_ENCODERS | DO_CS | DO_RS | DO_POS |
			DO_BD | DO_TIMER | DO_POWER | DO_OPP;
		if (!strcmp_P(res->arg2, PSTR("on")))
			mainboard.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			mainboard.flags &= bit;
		else { /* show */
			printf_P(PSTR("encoders is %s\r\n"), 
				 (DO_ENCODERS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("cs is %s\r\n"), 
				 (DO_CS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("rs is %s\r\n"), 
				 (DO_RS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("pos is %s\r\n"), 
				 (DO_POS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("bd is %s\r\n"), 
				 (DO_BD & mainboard.flags) ? "on":"off");
			printf_P(PSTR("timer is %s\r\n"), 
				 (DO_TIMER & mainboard.flags) ? "on":"off");
			printf_P(PSTR("power is %s\r\n"), 
				 (DO_POWER & mainboard.flags) ? "on":"off");
			printf_P(PSTR("opp is %s\r\n"), 
				 (DO_OPP & mainboard.flags) ? "on":"off");
		}
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("encoders")))
		bit = DO_ENCODERS;
	else if (!strcmp_P(res->arg1, PSTR("cs"))) {
		strat_hardstop();
		bit = DO_CS;
	}
	else if (!strcmp_P(res->arg1, PSTR("rs")))
		bit = DO_RS;
	else if (!strcmp_P(res->arg1, PSTR("pos")))
		bit = DO_POS;
	else if (!strcmp_P(res->arg1, PSTR("bd")))
		bit = DO_BD;
	else if (!strcmp_P(res->arg1, PSTR("timer"))) {
		time_reset();
		bit = DO_TIMER;
	}
	else if (!strcmp_P(res->arg1, PSTR("power")))
		bit = DO_POWER;
	else if (!strcmp_P(res->arg1, PSTR("opp")))
		bit = DO_OPP;

	if (!strcmp_P(res->arg2, PSTR("on")))
		mainboard.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		if (!strcmp_P(res->arg1, PSTR("cs"))) {
		 	pwm_mc_set(LEFT_MOTOR, 0);
			pwm_mc_set(RIGHT_MOTOR, 0);
		}
		mainboard.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\r\n"), res->arg1, 
		      (bit & mainboard.flags) ? "on":"off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
prog_char str_event_arg1[] = "all#encoders#cs#rs#pos#bd#timer#power#opp";
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
/* Opponent tests */

/* this structure is filled when cmd_opponent is parsed successfully */
struct cmd_opponent_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
};

/* function called when cmd_opponent is parsed successfully */
static void cmd_opponent_parsed(void *parsed_result, void *data)
{
	int16_t x,y,d,a;
	int16_t x2, y2, d2, a2;
	int8_t opp1, opp2;
//	microseconds us;

	do {

#ifdef TWO_OPPONENTS
	opp1 = get_opponent_xyda(&x, &y, &d, &a);
	opp2 = get_opponent2_xyda(&x2, &y2, &d2, &a2);

	if (opp1 == -1 && opp2 == -1)
		printf_P(PSTR("No opponent\r\n"));
	else if (opp1 == -1)
		printf_P(PSTR("opp1 not there / opp2 x=%d y=%d, d=%d a=%d\r\n"), x2, y2, d2, a2);
	else if (opp2 == -1)
		printf_P(PSTR("opp1 x=%d y=%d, d=%d a=%d / opp2 not there\r\n"), x, y, d, a);
	else
		printf_P(PSTR("opp1 x=%d y=%d, d=%d a=%d / opp2 x=%d y=%d, d=%d a=%d\r\n"),
							 x, y, d, a, x2, y2, d2, a2);
#else
	if (get_opponent_xyda(&x, &y, &d, &a) == -1)
		printf_P(PSTR("No opponent\r\n"));
	else
		printf_P(PSTR("x=%d y=%d, d=%d a=%d\r\n"), x, y, d, a);
#endif

//	us = time_get_us2();
//	while(time_get_us2()-us < 200000L) {
//		beacon_opponent_pulling();
//	}

	wait_ms(200);

	} while (!cmdline_keypressed());
}

prog_char str_opponent_arg0[] = "opponent";
parse_pgm_token_string_t cmd_opponent_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg0, str_opponent_arg0);
prog_char str_opponent_arg1[] = "show";
parse_pgm_token_string_t cmd_opponent_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg1, str_opponent_arg1);

prog_char help_opponent[] = "Show (x,y) opponent";
parse_pgm_inst_t cmd_opponent = {
	.f = cmd_opponent_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_opponent,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_opponent_arg0, 
		(prog_void *)&cmd_opponent_arg1, 
		NULL,
	},
};


prog_char str_opponent_arg1_set[] = "set";
parse_pgm_token_string_t cmd_opponent_arg1_set = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg1, str_opponent_arg1_set);
parse_pgm_token_num_t cmd_opponent_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_opponent_result, arg2, INT32);
parse_pgm_token_num_t cmd_opponent_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_opponent_result, arg3, INT32);

prog_char help_opponent_set[] = "Set (x,y) opponent";
parse_pgm_inst_t cmd_opponent_set = {
	.f = cmd_opponent_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_opponent_set,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_opponent_arg0, 
		(prog_void *)&cmd_opponent_arg1_set,
		(prog_void *)&cmd_opponent_arg2, 
		(prog_void *)&cmd_opponent_arg3, 
		NULL,
	},
};



/**********************************************************/
/* Start */

/* this structure is filled when cmd_start is parsed successfully */
struct cmd_start_result {
	fixed_string_t arg0;
};

/* function called when cmd_start is parsed successfully */
static void cmd_start_parsed(void *parsed_result, void *data)
{	
	//strat_game();
}

prog_char str_start_arg0[] = "start";
parse_pgm_token_string_t cmd_start_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_start_result, arg0, str_start_arg0);

prog_char help_start[] = "Start the robot";
parse_pgm_inst_t cmd_start = {
	.f = cmd_start_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_start,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_start_arg0, 
		NULL,
	},
};


#if 0
/**********************************************************/
/* Start */

/* this structure is filled when cmd_start is parsed successfully */
struct cmd_start_result {
	fixed_string_t arg0;
	fixed_string_t color;
	fixed_string_t debug;
};

/* function called when cmd_start is parsed successfully */
static void cmd_start_parsed(void *parsed_result, void *data)
{
	struct cmd_start_result *res = parsed_result;
	uint8_t old_level = gen.log_level;
	int8_t c;
	
	gen.logs[NB_LOGS] = E_USER_STRAT;
	if (!strcmp_P(res->debug, PSTR("debug"))) {
		strat_infos.dump_enabled = 1;
		gen.log_level = 5;
	}
	else {
		strat_infos.dump_enabled = 0;
		gen.log_level = 0;
	}

 retry:
	printf_P(PSTR("Press a key when beacon ready, 'q' for skip \r\n"));
	c = -1;
	while(c == -1){
		c = cmdline_getchar();
	}
	
	if(c == 'q'){
		printf("Play without beacon\r\n");
	}
	else{
		beacon_cmd_wt11_call();
		WAIT_COND_OR_TIMEOUT((beacon_connected==1),5000);
		if(!beacon_connected){
			printf("Beacon connection FAIL, reseting local wt11\r\n");
			beacon_cmd_wt11_local_reset();
			goto retry;
		}
		else{
			printf("Beacon connection SUCESS!\r\n");		
 retry_on:
			beacon_cmd_beacon_on();
			
			printf("is beacon running? (s/n)\n\r");
			c = -1;
			while(c == -1){
				c = cmdline_getchar();
			}
			if(c == 'n'){
				wait_ms(100);
				goto retry_on;	
			}
		}
	}
	
	if (!strcmp_P(res->color, PSTR("blue"))) {
		mainboard.our_color = I2C_COLOR_PURPLE;
		//beacon_cmd_color();
	}
	else if (!strcmp_P(res->color, PSTR("red"))) {
		mainboard.our_color = I2C_COLOR_RED;
		//beacon_cmd_color();
	}

	strat_start();

	gen.logs[NB_LOGS] = 0;
	gen.log_level = old_level;
}

prog_char str_start_arg0[] = "start";
parse_pgm_token_string_t cmd_start_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_start_result, arg0, str_start_arg0);
prog_char str_start_color[] = "blue#red";
parse_pgm_token_string_t cmd_start_color = TOKEN_STRING_INITIALIZER(struct cmd_start_result, color, str_start_color);
prog_char str_start_debug[] = "debug#match";
parse_pgm_token_string_t cmd_start_debug = TOKEN_STRING_INITIALIZER(struct cmd_start_result, debug, str_start_debug);

prog_char help_start[] = "Start the robot";
parse_pgm_inst_t cmd_start = {
	.f = cmd_start_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_start,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_start_arg0, 
		(prog_void *)&cmd_start_color, 
		(prog_void *)&cmd_start_debug, 
		NULL,
	},
};

#endif
/**********************************************************/
/* Color */

/* this structure is filled when cmd_color is parsed successfully */
struct cmd_color_result {
	fixed_string_t arg0;
	fixed_string_t color;
};

/* function called when cmd_color is parsed successfully */
static void cmd_color_parsed(void *parsed_result, void *data)
{
	struct cmd_color_result *res = (struct cmd_color_result *) parsed_result;
	if (!strcmp_P(res->color, PSTR("red"))) {
		mainboard.our_color = I2C_COLOR_RED;
	}
	else if (!strcmp_P(res->color, PSTR("purple"))) {
		mainboard.our_color = I2C_COLOR_PURPLE;
	}
	printf_P(PSTR("Done\r\n"));
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "red#purple";
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
/* treasure */

/* this structure is filled when cmd_dump is parsed successfully */
struct cmd_treasure_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int16_t arg2;
	int16_t arg3;
};

/* function called when cmd_dump is parsed successfully */
static void cmd_treasure_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_treasure_result *res = parsed_result;

	if (!strcmp(res->arg1, "pickupmap"))
		strat_pickup_map();
}

prog_char str_treasure_arg0[] = "treasure";
parse_pgm_token_string_t cmd_treasure_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_treasure_result, arg0, str_treasure_arg0);
prog_char str_treasure_arg1[] = "pickupmap";
parse_pgm_token_string_t cmd_treasure_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_treasure_result, arg1, str_treasure_arg1);
parse_pgm_token_num_t cmd_treasure_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_treasure_result, arg2, INT32);
parse_pgm_token_num_t cmd_treasure_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_treasure_result, arg3, INT32);


prog_char help_treasure[] = "treasure functions";
parse_pgm_inst_t cmd_treasure = {
	.f = cmd_treasure_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_treasure,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_treasure_arg0, 
		(prog_void *)&cmd_treasure_arg1, 
		(prog_void *)&cmd_treasure_arg2, 
		(prog_void *)&cmd_treasure_arg3, 
		NULL,
	},
};

/**********************************************************/
/* actuator */

/* this structure is filled when cmd_dump is parsed successfully */
struct cmd_actuator_result {
	fixed_string_t arg0;
	uint16_t arg1;
};

/* function called when cmd_dump is parsed successfully */
static void cmd_actuator_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_actuator_result *res = parsed_result;
	uint8_t err = 0;

	if (!strcmp(res->arg0, "arm")) {
		if(arm_set_pos(res->arg1))
			printf("arm_set_pos error\n\r");

		//err = arm_wait_end();
		
		if(err == END_BLOCKING)
			printf("END_BLOCKING\n\r");
	}
	else if (!strcmp(res->arg0, "torque_arm")) {

		if(res->arg1 == 0)
			err = arm_disable_torque();
		else
			err = arm_enable_torque();
		
		if(err)
			printf("set torque error\n\r");
	}
	else if (!strcmp(res->arg0, "teeth")) {
		if(teeth_set_pos(res->arg1))
			printf("teeth_set_pos error\n\r");

		//err = teeth_wait_end();
		
		if(err == END_BLOCKING)
			printf("END_BLOCKING\n\r");
	}
}

prog_char str_actuator_arg0[] = "arm#torque_arm#teeth";
parse_pgm_token_string_t cmd_actuator_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_actuator_result, arg0, str_actuator_arg0);
parse_pgm_token_num_t cmd_actuator_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_actuator_result, arg1, UINT16);

prog_char help_actuator[] = "set actuator possition";
parse_pgm_inst_t cmd_actuator = {
	.f = cmd_actuator_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_actuator,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_actuator_arg0, 
		(prog_void *)&cmd_actuator_arg1,  
		NULL,
	},
};

/**********************************************************/
/* beacon */
#ifdef notyet
/* this structure is filled when cmd_interact is parsed successfully */
struct cmd_beacon_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

static void cmd_beacon_parsed(void * parsed_result, void * data)
{
	int16_t c;
	int8_t cmd = 0;
	struct vt100 vt100;

	struct cmd_beacon_result *res = parsed_result;

	vt100_init(&vt100);
	
	if(!strcmp_P(res->arg1, "raw"))
	{
		/* init vt100 character set */
		vt100_init(&vt100);
		
		/* interact */
		while(cmd != KEY_CTRL_C) 
		{
			/* received from slave */			if((c = uart_recv_nowait(MUX_UART))!= -1)
				/* echo */				uart_send_nowait(CMDLINE_UART,c);
			
			/* send to slavedspic */
			c = cmdline_getchar();
			if (c == -1) {
				continue;
			}

			/* check exit cmd */
			cmd = vt100_parser(&vt100, c);
			/* send to slave */
			uart_send_nowait(MUX_UART,c);	
		}
	}	
	else if(!strcmp_P(res->arg1, "wt11_reset")){
		beacon_cmd_wt11_local_reset();
	}
	else if(!strcmp_P(res->arg1, "call")){
		beacon_cmd_wt11_call();
	}
	else if(!strcmp_P(res->arg1, "wt11_close")){
		beacon_cmd_wt11_close();
	}
	else if(!strcmp_P(res->arg1, "on")){
		beacon_cmd_beacon_on();
	}
	else if(!strcmp_P(res->arg1, "watchdog_on")){
		beacon_cmd_beacon_on();
	}
	else if(!strcmp_P(res->arg1, "off")){
		beacon_cmd_beacon_off();
	}
	else if(!strcmp_P(res->arg1, "color")){
		beacon_cmd_color();
	}
	else if(!strcmp_P(res->arg1, "opponent")){
		//beacon_cmd_opponent();
		beacon_pull_opponent();
	}

	printf("Done\n\r");
}

prog_char str_beacon_arg0[] = "beacon";
parse_pgm_token_string_t cmd_beacon_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_result, arg0, str_beacon_arg0);
prog_char str_beacon_arg1[] = "raw#wt11_reset#call#wt11_close#on#watchdog_on#off#color#opponent";
parse_pgm_token_string_t cmd_beacon_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_result, arg1, str_beacon_arg1);

prog_char help_beacon[] = "beacon commads";
parse_pgm_inst_t cmd_beacon = {
	.f = cmd_beacon_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_beacon,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_beacon_arg0, 
		(prog_void *)&cmd_beacon_arg1, 
		NULL,
	},
};
#endif


/**********************************************************/
/* Interact */

///* this structure is filled when cmd_interact is parsed successfully */
//struct cmd_interact_result {
//	fixed_string_t arg0;
//};
//
//static void print_cs(void)
//{
//	printf_P(PSTR("cons_d=% .8ld cons_a=% .8ld fil_d=% .8ld fil_a=% .8ld "
//		      "err_d=% .8ld err_a=% .8ld out_d=% .8ld out_a=% .8ld\r\n"), 
//		 cs_get_consign(&mainboard.distance.cs),
//		 cs_get_consign(&mainboard.angle.cs),
//		 cs_get_filtered_consign(&mainboard.distance.cs),
//		 cs_get_filtered_consign(&mainboard.angle.cs),
//		 cs_get_error(&mainboard.distance.cs),
//		 cs_get_error(&mainboard.angle.cs),
//		 cs_get_out(&mainboard.distance.cs),
//		 cs_get_out(&mainboard.angle.cs));
//}
//
//static void print_pos(void)
//{
//	printf_P(PSTR("x=% .8d y=% .8d a=% .8d\r\n"), 
//		 position_get_x_s16(&mainboard.pos),
//		 position_get_y_s16(&mainboard.pos),
//		 position_get_a_deg_s16(&mainboard.pos));
//}
//
//static void print_time(void)
//{
//	printf_P(PSTR("time %d\r\n"),(int)time_get_s());
//}
//
//
//static void print_sensors(void)
//{
//#ifdef notyet
//	if (sensor_robot_start_switch())
//		printf_P(PSTR("Start switch | "));
//	else
//		printf_P(PSTR("             | "));
//
//	if (IR_DISP_SENSOR())
//		printf_P(PSTR("IR disp | "));
//	else
//		printf_P(PSTR("        | "));
//
//	printf_P(PSTR("\r\n"));
//#endif
//}
//
//static void print_pid(void)
//{
//	printf_P(PSTR("P=% .8ld I=% .8ld D=% .8ld out=% .8ld | "
//		      "P=% .8ld I=% .8ld D=% .8ld out=% .8ld\r\n"),
//		 pid_get_value_in(&mainboard.distance.pid) * pid_get_gain_P(&mainboard.distance.pid),
//		 pid_get_value_I(&mainboard.distance.pid) * pid_get_gain_I(&mainboard.distance.pid),
//		 pid_get_value_D(&mainboard.distance.pid) * pid_get_gain_D(&mainboard.distance.pid),
//		 pid_get_value_out(&mainboard.distance.pid),
//		 pid_get_value_in(&mainboard.angle.pid) * pid_get_gain_P(&mainboard.angle.pid),
//		 pid_get_value_I(&mainboard.angle.pid) * pid_get_gain_I(&mainboard.angle.pid),
//		 pid_get_value_D(&mainboard.angle.pid) * pid_get_gain_D(&mainboard.angle.pid),
//		 pid_get_value_out(&mainboard.angle.pid));
//}
//
//#define PRINT_POS       (1<<0)
//#define PRINT_PID       (1<<1)
//#define PRINT_CS        (1<<2)
//#define PRINT_SENSORS   (1<<3)
//#define PRINT_TIME      (1<<4)
//#define PRINT_BLOCKING  (1<<5)
//
//static void cmd_interact_parsed(void * parsed_result, void * data)
//{
//	int c;
//	int8_t cmd;
//	uint8_t print = 0;
//	struct vt100 vt100;
//
//	vt100_init(&vt100);
//
//	printf_P(PSTR("Display debugs:\r\n"
//		      "  1:pos\r\n"
//		      "  2:pid\r\n"
//		      "  3:cs\r\n"
//		      "  4:sensors\r\n"
//		      "  5:time\r\n"
//		      /* "  6:blocking\r\n" */
//		      "Commands:\r\n"
//		      "  arrows:move\r\n"
//		      "  space:stop\r\n"
//		      "  q:quit\r\n"));
//
//	/* stop motors */
//	mainboard.flags &= (~DO_CS);
//	dac_set_and_save(LEFT_MOTOR, 0);
//	dac_set_and_save(RIGHT_MOTOR, 0);
//
//	while(1) {
//		if (print & PRINT_POS) {
//			print_pos();
//		}
//
//		if (print & PRINT_PID) {
//			print_pid();
//		}
//
//		if (print & PRINT_CS) {
//			print_cs();
//		}
//
//		if (print & PRINT_SENSORS) {
//			print_sensors();
//		}
//
//		if (print & PRINT_TIME) {
//			print_time();
//		}
///* 		if (print & PRINT_BLOCKING) { */
///* 			printf_P(PSTR("%s %s blocking=%d\r\n"),  */
///* 				 mainboard.blocking ? "BLOCK1":"      ", */
///* 				 rs_is_blocked(&mainboard.rs) ? "BLOCK2":"      ", */
///* 				 rs_get_blocking(&mainboard.rs)); */
///* 		} */
//
//		c = cmdline_getchar();
//		if (c == -1) {
//			wait_ms(10);
//			continue;
//		}
//		cmd = vt100_parser(&vt100, c);
//		if (cmd == -2) {
//			wait_ms(10);
//			continue;
//		}
//		
//		if (cmd == -1) {
//			switch(c) {
//			case '1': print ^= PRINT_POS; break;
//			case '2': print ^= PRINT_PID; break;
//			case '3': print ^= PRINT_CS; break;
//			case '4': print ^= PRINT_SENSORS; break;
//			case '5': print ^= PRINT_TIME; break;
//			case '6': print ^= PRINT_BLOCKING; break;
//
//			case 'q': 
//				if (mainboard.flags & DO_CS)
//					strat_hardstop();
//				dac_set_and_save(LEFT_MOTOR, 0);
//				dac_set_and_save(RIGHT_MOTOR, 0);
//				return;
//			case ' ':
//				dac_set_and_save(LEFT_MOTOR, 0);
//				dac_set_and_save(RIGHT_MOTOR, 0);
//				break;
//			default: 
//				break;
//			}
//		}
//		else {
//			switch(cmd) {
//			case KEY_UP_ARR: 
//				dac_set_and_save(LEFT_MOTOR, 6000);
//				dac_set_and_save(RIGHT_MOTOR, 6000);
//				break;
//			case KEY_LEFT_ARR: 
//				dac_set_and_save(LEFT_MOTOR, -6000);
//				dac_set_and_save(RIGHT_MOTOR, 6000);
//				break;
//			case KEY_DOWN_ARR: 
//				dac_set_and_save(LEFT_MOTOR, -6000);
//				dac_set_and_save(RIGHT_MOTOR, -6000);
//				break;
//			case KEY_RIGHT_ARR:
//				dac_set_and_save(LEFT_MOTOR, 6000);
//				dac_set_and_save(RIGHT_MOTOR, -6000);
//				break;
//			}
//		}
//		wait_ms(10);
//	}
//}
//
//prog_char str_interact_arg0[] = "interact";
//parse_pgm_token_string_t cmd_interact_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_interact_result, arg0, str_interact_arg0);
//
//prog_char help_interact[] = "Interactive mode";
//parse_pgm_inst_t cmd_interact = {
//	.f = cmd_interact_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_interact,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_interact_arg0, 
//		NULL,
//	},
//};


/**********************************************************/
/* Rs tests */

///* this structure is filled when cmd_rs is parsed successfully */
//struct cmd_rs_result {
//	fixed_string_t arg0;
//	fixed_string_t arg1;
//};
//
///* function called when cmd_rs is parsed successfully */
//static void cmd_rs_parsed(void *parsed_result, void *data)
//{
//	//	struct cmd_rs_result *res = parsed_result;
//	do {
//		printf_P(PSTR("angle cons=% .6ld in=% .6ld out=% .6ld / "), 
//			 cs_get_consign(&mainboard.angle.cs),
//			 cs_get_filtered_feedback(&mainboard.angle.cs),
//			 cs_get_out(&mainboard.angle.cs));
//		printf_P(PSTR("distance cons=% .6ld in=% .6ld out=% .6ld / "), 
//			 cs_get_consign(&mainboard.distance.cs),
//			 cs_get_filtered_feedback(&mainboard.distance.cs),
//			 cs_get_out(&mainboard.distance.cs));
//		printf_P(PSTR("l=% .4ld r=% .4ld\r\n"), mainboard.dac_l,
//			 mainboard.dac_r);
//		wait_ms(100);
//	} while(!cmdline_keypressed());
//}
//
//prog_char str_rs_arg0[] = "rs";
//parse_pgm_token_string_t cmd_rs_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg0, str_rs_arg0);
//prog_char str_rs_arg1[] = "show";
//parse_pgm_token_string_t cmd_rs_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg1, str_rs_arg1);
//
//prog_char help_rs[] = "Show rs (robot system) values";
//parse_pgm_inst_t cmd_rs = {
//	.f = cmd_rs_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_rs,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_rs_arg0, 
//		(prog_void *)&cmd_rs_arg1, 
//		NULL,
//	},
//};

/**********************************************************/
/* I2cdebug */

///* this structure is filled when cmd_i2cdebug is parsed successfully */
//struct cmd_i2cdebug_result {
//	fixed_string_t arg0;
//};
//
///* function called when cmd_i2cdebug is parsed successfully */
//static void cmd_i2cdebug_parsed(void * parsed_result, void * data)
//{
//	i2c_debug();
//	i2c_protocol_debug();
//}
//
//prog_char str_i2cdebug_arg0[] = "i2cdebug";
//parse_pgm_token_string_t cmd_i2cdebug_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_i2cdebug_result, arg0, str_i2cdebug_arg0);
//
//prog_char help_i2cdebug[] = "I2c debug infos";
//parse_pgm_inst_t cmd_i2cdebug = {
//	.f = cmd_i2cdebug_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_i2cdebug,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_i2cdebug_arg0, 
//		NULL,
//	},
//};

