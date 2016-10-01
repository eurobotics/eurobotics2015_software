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
 *  Javier Balias Santos <javier@arc-robots.org>
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
#include <dac_mc.h>
#include <pwm_servo.h>
#include <clock_time.h>
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

#ifdef HOST_VERSION
#include <hostsim.h>
#endif

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
//#include "beacon.h"
#include "bt_protocol.h"
#include "robotsim.h"
#include "strat_base.h"
#include "strat_avoid.h"
#include "strat_utils.h"
#include "wt11.h"


#ifdef HOST_VERSION
#define COMPILE_COMMANDS_MAINBOARD
#define COMPILE_COMMANDS_MAINBOARD_OPTIONALS
#endif

static uint8_t beacon_addr [] = {0x00, 0x07 ,0x80, 0x85, 0x04, 0x70};
static uint8_t robot_2nd_addr [] = {0x00, 0x07 ,0x80, 0x9A, 0xB8, 0x73};


struct cmd_event_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    fixed_string_t arg2;
};

/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, void *data)
{
    uint16_t bit = 0;

    struct cmd_event_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("all")))
    {
        bit = DO_ENCODERS | DO_CS | DO_RS | DO_POS |
                DO_BD | DO_TIMER | DO_POWER | DO_BEACON | DO_ROBOT_2ND;
        if (!strcmp_P(res->arg2, PSTR("on")))
            mainboard.flags |= bit;
        else if (!strcmp_P(res->arg2, PSTR("off")))
            mainboard.flags &= bit;
        else
        { /* show */
            printf_P(PSTR("encoders is %s\r\n"),
                     (DO_ENCODERS & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("cs is %s\r\n"),
                     (DO_CS & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("rs is %s\r\n"),
                     (DO_RS & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("pos is %s\r\n"),
                     (DO_POS & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("bd is %s\r\n"),
                     (DO_BD & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("timer is %s\r\n"),
                     (DO_TIMER & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("power is %s\r\n"),
                     (DO_POWER & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("beacon is %s\r\n"),
                     (DO_BEACON & mainboard.flags) ? "on" : "off");
           printf_P(PSTR("robot_2nd is %s\r\n"),
                     (DO_ROBOT_2ND & mainboard.flags) ? "on" : "off");
        }
        return;
    }

    if (!strcmp_P(res->arg1, PSTR("encoders")))
        bit = DO_ENCODERS;
    else if (!strcmp_P(res->arg1, PSTR("cs")))
    {
        strat_hardstop();
        bit = DO_CS;
    }
    else if (!strcmp_P(res->arg1, PSTR("rs")))
        bit = DO_RS;
    else if (!strcmp_P(res->arg1, PSTR("pos")))
        bit = DO_POS;
    else if (!strcmp_P(res->arg1, PSTR("bd")))
        bit = DO_BD;
    else if (!strcmp_P(res->arg1, PSTR("timer")))
    {
        time_reset();
        bit = DO_TIMER;
    }
    else if (!strcmp_P(res->arg1, PSTR("power")))
        bit = DO_POWER;
    else if (!strcmp_P(res->arg1, PSTR("beacon")))
        bit = DO_BEACON;
   else if (!strcmp_P(res->arg1, PSTR("robot_2nd")))
        bit = DO_ROBOT_2ND;


    if (!strcmp_P(res->arg2, PSTR("on")))
        mainboard.flags |= bit;
    else if (!strcmp_P(res->arg2, PSTR("off")))
    {
        if (!strcmp_P(res->arg1, PSTR("cs")))
        {
#ifdef HOST_VERSION
            robotsim_pwm(LEFT_MOTOR, 0);
            robotsim_pwm(RIGHT_MOTOR, 0);
#else
            dac_mc_set(LEFT_MOTOR, 0);
            dac_mc_set(RIGHT_MOTOR, 0);
#endif
        }
        mainboard.flags &= (~bit);
    }
    printf_P(PSTR("%s is %s\r\n"), res->arg1,
             (bit & mainboard.flags) ? "on" : "off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
prog_char str_event_arg1[] = "all#encoders#cs#rs#pos#bd#timer#power#beacon#robot_2nd";
parse_pgm_token_string_t cmd_event_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg1, str_event_arg1);
prog_char str_event_arg2[] = "on#off#show";
parse_pgm_token_string_t cmd_event_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg2, str_event_arg2);

//prog_char help_event[] = "Enable/disable events";
parse_pgm_inst_t cmd_event = {
    .f = cmd_event_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_event,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_event_arg0,
        (prog_void *) & cmd_event_arg1,
        (prog_void *) & cmd_event_arg2,
        NULL,
    },
};


/**********************************************************/
/* Opponent tests */

/* this structure is filled when cmd_opponent is parsed successfully */
struct cmd_opponent_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    int32_t arg2;
    int32_t arg3;
};

/* function called when cmd_opponent is parsed successfully */
static void cmd_opponent_parsed(void *parsed_result, void *data)
{
    struct cmd_opponent_result * res = parsed_result;

    int16_t x, y, d, a;
    int16_t x2, y2, d2, a2;
    int16_t x_r2nd, y_r2nd, d_r2nd, a_r2nd, a_abs_r2nd;
    int8_t opp1, opp2, r2nd;
    //	microseconds us;

    if (data)
    {
        beaconboard.opponent1_d = res->arg2;
        beaconboard.opponent1_a = res->arg3;
        beaconboard.opponent1_x = 0;
    }
    else
        do
        {

#ifdef TWO_OPPONENTS
            opp1 = get_opponent1_xyda(&x, &y, &d, &a);
            opp2 = get_opponent2_xyda(&x2, &y2, &d2, &a2);
            r2nd = get_robot_2nd_xyda(&x_r2nd, &y_r2nd, &d_r2nd, &a_r2nd);
            get_robot_2nd_a_abs(&a_abs_r2nd);


            if (opp1 == -1)
                printf_P(PSTR("opp1 not there"));
            else
                printf_P(PSTR("opp1 x=%.4d y=%.4d, d=%.4d a=%+.3d"), x, y, d, a);

            if (opp2 == -1)
                printf_P(PSTR(" / opp2 not there"));
            else
                printf_P(PSTR(" / opp2 x=%.4d y=%.4d, d=%.4d a=%+.3d"), x2, y2, d2, a2);

#ifdef ROBOT_2ND
            if (r2nd == -1)
                printf_P(PSTR(" / r2nd not there"));
            else
                printf_P(PSTR(" / r2nd x=%.4d y=%.4d, d=%.4d a=%+.3d (%+.3d)"), x_r2nd, y_r2nd, d_r2nd, a_r2nd, a_abs_r2nd);
#endif

            printf("\n\r");

#else
            if (get_opponent1_xyda(&x, &y, &d, &a) == -1)
                printf_P(PSTR("No opponent\r\n"));
            else
                printf_P(PSTR("x=%d y=%d, d=%d a=%d\r\n"), x, y, d, a);
#endif

            //	us = time_get_us2();
            //	while(time_get_us2()-us < 200000L) {
            //		beacon_opponent_pulling();
            //	}

            wait_ms(100);

        }
        while (!cmdline_keypressed());
}

prog_char str_opponent_arg0[] = "opponent";
parse_pgm_token_string_t cmd_opponent_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg0, str_opponent_arg0);
prog_char str_opponent_arg1[] = "show";
parse_pgm_token_string_t cmd_opponent_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg1, str_opponent_arg1);

//prog_char help_opponent[] = "Show (x,y) opponent";
parse_pgm_inst_t cmd_opponent = {
    .f = cmd_opponent_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_opponent,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_opponent_arg0,
        (prog_void *) & cmd_opponent_arg1,
        NULL,
    },
};

#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/
prog_char str_opponent_arg1_set[] = "set";
parse_pgm_token_string_t cmd_opponent_arg1_set = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg1, str_opponent_arg1_set);
parse_pgm_token_num_t cmd_opponent_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_opponent_result, arg2, INT32);
parse_pgm_token_num_t cmd_opponent_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_opponent_result, arg3, INT32);

//prog_char help_opponent_set[] = "Set (x,y) opponent";
parse_pgm_inst_t cmd_opponent_set = {
    .f = cmd_opponent_parsed, /* function to call */
    .data = (void*) 1, /* 2nd arg of func */
    //.help_str = help_opponent_set,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_opponent_arg0,
        (prog_void *) & cmd_opponent_arg1_set,
        (prog_void *) & cmd_opponent_arg2,
        (prog_void *) & cmd_opponent_arg3,
        NULL,
    },
};
#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/

/**********************************************************/
/* Init match */

/* from commands_traj.c */
void auto_position(void);

/* this structure is filled when cmd_init is parsed successfully */
struct cmd_init_result
{
    fixed_string_t arg0;
    fixed_string_t color;
};

uint8_t beacon_link_ok = 0, robot_link_ok = 0;

uint8_t robot_link_is_ok (void)
{
	return (robot_link_ok == 1);
}

/* function called when cmd_init is parsed successfully */
static void cmd_init_parsed(void *parsed_result, void *data)
{
    struct cmd_init_result *res = parsed_result;
	int8_t c;


	/* set main robot color */
    if (!strcmp_P(res->color, PSTR("green"))){
        mainboard.our_color = I2C_COLOR_GREEN;
	}
    else if (!strcmp_P(res->color, PSTR("yellow"))){
        mainboard.our_color = I2C_COLOR_YELLOW;
	}

	/* autopos main robot */
	auto_position();

	/* init main robot mechanics */
	i2c_slavedspic_mode_init();

#ifndef HOST_VERSION
	/* XXX reset local wt11 */
	//wt11_reset_mux();
	//time_wait_ms (1000);

	/* beacon */
beacon_retry:
   	printf_P(PSTR("TURN ON and OFF the beacon\r\n"));
   	printf_P(PSTR("Press a key when beacon ready, 'q' for skip \r\n"));
	c = cmdline_getchar_wait();

    if (c != 'q') {
	   	printf_P(PSTR("Trying to open beacon link ...\n"));
		wt11_open_link_mux(beacon_addr, &beaconboard.link_id);

ask_beacon_ok:
		printf_P(PSTR("Is beacon link OK? (y/n) \r\n"));
		c = cmdline_getchar_wait();
		if (c == 'n')
			goto beacon_retry;
		else if (c == 'y')
			beacon_link_ok = 1;
		else
			goto ask_beacon_ok;
    }
	else {
    	printf("Play without beacon\r\n");
	}

	/* secondary robot */
robot_retry:
   	printf_P(PSTR("TURN ON and OFF the secondary robot\r\n"));
   	printf_P(PSTR("Press a key when robot ready, 'q' for skip \r\n"));
	c = cmdline_getchar_wait();

    if (c != 'q') {
	   	printf_P(PSTR("Trying to open robot link ...\n"));
		wt11_open_link_mux(robot_2nd_addr, &robot_2nd.link_id);

ask_robot_ok:
		printf_P(PSTR("Is robot link OK? (y/n) \r\n"));
		c = cmdline_getchar_wait();
		if (c == 'n')
			goto robot_retry;
		else if (c == 'y')
			robot_link_ok = 1;
		else
			goto ask_robot_ok;
    }
	else {
    	printf("Play without secondary robot\r\n");
	}
#else
		/* on HOST */
		robot_2nd.link_id = 0;
		beaconboard.link_id = 1;
		beacon_link_ok = 1;
		robot_link_ok = 1;
#endif

	/* enable bt protocol events */
	if (beacon_link_ok && robot_link_ok)
		mainboard.flags |=  DO_BT_PROTO;
	else if (beacon_link_ok)
		mainboard.flags |=  DO_BEACON;
	else if (robot_link_ok)
		mainboard.flags |=  DO_ROBOT_2ND;

	time_wait_ms (200);

	/* secondary robot init */
	if (robot_link_ok) {
		/* set secondary robot color */
		bt_robot_2nd_set_color ();
		bt_robot_2nd_wait_ack();

		/* autopos secondary robot */
		bt_robot_2nd_autopos();
		bt_robot_2nd_wait_ack();
		bt_robot_2nd_wait_end();
	}

	//printf ("Done\n\r");
}

prog_char str_init_arg0[] = "init";
parse_pgm_token_string_t cmd_init_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_init_result, arg0, str_init_arg0);
prog_char str_init_color[] = "green#yellow";
parse_pgm_token_string_t cmd_init_color = TOKEN_STRING_INITIALIZER(struct cmd_init_result, color, str_init_color);

//prog_char help_init[] = "Init the robots";
parse_pgm_inst_t cmd_init = {
    .f = cmd_init_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_init,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_init_arg0,
        (prog_void *) & cmd_init_color,
        NULL,
    },
};



/**********************************************************/
/* Start */

/* this structure is filled when cmd_start is parsed successfully */
struct cmd_start_result
{
    fixed_string_t arg0;
    fixed_string_t strategy;
    fixed_string_t debug;
};

/* function called when cmd_start is parsed successfully */
static void cmd_start_parsed(void *parsed_result, void *data)
{
    struct cmd_start_result *res = parsed_result;
    uint8_t old_level = gen.log_level;
    int8_t c;

    gen.logs[NB_LOGS] = E_USER_STRAT;
    if (!strcmp_P(res->debug, PSTR("debug")))
    {
        strat_infos.dump_enabled = 1;
        gen.log_level = 5;
    }
    else if (!strcmp_P(res->debug, PSTR("step_debug")))
    {
        strat_infos.dump_enabled = 1;
        strat_infos.debug_step = 1;
        gen.log_level = 5;
    }
    else
    {
        strat_infos.dump_enabled = 0;
        gen.log_level = 0;
    }

	if (!strcmp_P(res->strategy, PSTR("qualification")))
    {
		strat_infos.match_strategy=STR_QUALIFICATION;
		strat_infos.conf.flags = 0;	
		strat_infos.conf.flags |= CONF_FLAG_DO_TOWER;	
		//strat_infos.conf.flags |= CONF_FLAG_DO_ESCAPE_UPPER_ZONE;	
		
    }
    else if (!strcmp_P(res->strategy, PSTR("finals")))
    {
		strat_infos.match_strategy=STR_QUALIFICATION;
		strat_infos.conf.flags = 0;
		strat_infos.conf.flags |= CONF_FLAG_DO_TOWER;	
		//strat_infos.conf.flags |= CONF_FLAG_DO_ESCAPE_UPPER_ZONE;	
		strat_infos.conf.flags |= CONF_FLAG_DO_STAND_FAST_GROUP_1;
    }
    else if (!strcmp_P(res->strategy, PSTR("base")))
    {
	   strat_infos.match_strategy=STR_BASE;
    }

    else if (!strcmp_P(res->strategy, PSTR("homologation")))
    {
	   strat_infos.match_strategy=STR_HOMOLOGATION;
    }
    else
    {
    	printf_P(PSTR("ERROR: repeat and select a valid strategy! Returning. \r\n"));
    	return;
    }

	/* flags */
	strat_smart[MAIN_ROBOT].current_strategy=0;
	strat_smart[SEC_ROBOT].current_strategy=0;
	strat_set_next_sec_strategy();
	strat_set_next_main_strategy();
    strat_infos.dump_enabled = 1;
    strat_dump_conf();

#ifndef HOST_VERSION
#ifdef old_version

    printf_P(PSTR("Press a key when beacon ready, 'q' for skip \r\n"));
    c = -1;
    while (c == -1) {
        c = cmdline_getchar();
    }

    if (c == 'q') {
        printf("Play without beacon\r\n");
    }
    else
    {
retry_on:
        /* start beacon */
		bt_beacon_set_on();

        printf("is beacon running? (y/n)\n\r");
        c = -1;
        while (c == -1)
        {
            c = cmdline_getchar();
        }
        if (c == 'n')
        {
            wait_ms(100);
            goto retry_on;
        }
    }
#else
	/* turn on beacon and confirm it's working */
	if (beacon_link_ok)
	{
retry_on:
        /* start beacon */
		bt_beacon_set_on();

        printf("Is beacon running? (y/n)\n\r");
        c = -1;
        while (c == -1)
        {
            c = cmdline_getchar();
        }
        if (c == 'n')
        {
            wait_ms(100);
            goto retry_on;
        }
	}
#endif /* old_version */
#endif /* HOST_VERSION */

    strat_start();

    gen.logs[NB_LOGS] = 0;
    gen.log_level = old_level;
}

prog_char str_start_arg0[] = "start";
parse_pgm_token_string_t cmd_start_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_start_result, arg0, str_start_arg0);
prog_char str_start_strategy[] = "base#homologation#qualification#finals";
parse_pgm_token_string_t cmd_start_strategy = TOKEN_STRING_INITIALIZER(struct cmd_start_result, strategy, str_start_strategy);
prog_char str_start_debug[] = "debug#step_debug#match";
parse_pgm_token_string_t cmd_start_debug = TOKEN_STRING_INITIALIZER(struct cmd_start_result, debug, str_start_debug);

//prog_char help_start[] = "Start the robot";
parse_pgm_inst_t cmd_start = {
    .f = cmd_start_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_start,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_start_arg0,
        (prog_void *) & cmd_start_strategy,
        (prog_void *) & cmd_start_debug,
        NULL,
    },
};


/**********************************************************/
/* Color */

/* this structure is filled when cmd_color is parsed successfully */
struct cmd_color_result
{
    fixed_string_t arg0;
    fixed_string_t color;
};

/* function called when cmd_color is parsed successfully */
static void cmd_color_parsed(void *parsed_result, void *data)
{
    struct cmd_color_result *res = (struct cmd_color_result *) parsed_result;
    if (!strcmp_P(res->color, PSTR("yellow"))) {
        mainboard.our_color = I2C_COLOR_YELLOW;
    }
    else if (!strcmp_P(res->color, PSTR("green"))) {
        mainboard.our_color = I2C_COLOR_GREEN;
    }
    //printf_P(PSTR("Done\r\n"));
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "yellow#green";
parse_pgm_token_string_t cmd_color_color = TOKEN_STRING_INITIALIZER(struct cmd_color_result, color, str_color_color);

//prog_char help_color[] = "Set our color";
parse_pgm_inst_t cmd_color = {
    .f = cmd_color_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_color,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_color_arg0,
        (prog_void *) & cmd_color_color,
        NULL,
    },
};

/**********************************************************/
/* beacon */

/* this structure is filled when cmd_interact is parsed successfully */
struct cmd_beacon_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

static void cmd_beacon_parsed(void * parsed_result, void * data)
{

   struct vt100 vt100;

#ifndef HOST_VERSION
   int16_t c;
   int8_t cmd = 0;
	uint16_t mainboard_flags;
	uint8_t flags;
#endif

   struct cmd_beacon_result *res = parsed_result;

   vt100_init(&vt100);


	if(!strcmp_P(res->arg1, "raw")) {
#ifdef HOST_VERSION
		printf("not implemented\n");
#else
		/* save flags and disable BT_PROTO */
		IRQ_LOCK (flags);
		mainboard_flags = mainboard.flags;
		mainboard.flags &= (~DO_BT_PROTO);
		IRQ_UNLOCK (flags);

		/* init vt100 character set */
		vt100_init(&vt100);

		wt11_flush ();

		/* interact */
		while(cmd != KEY_CTRL_C)
		{
			/* link --> cmd line */
			wt11_bypass_to_stdo (beaconboard.link_id);

			/* cmd line --> link */
			c = cmdline_getchar();
			if (c == -1) {
				continue;
			}

			/* check exit cmd */
			cmd = vt100_parser(&vt100, c);

			/* send to link */
			wt11_send_mux(beaconboard.link_id, (uint8_t *)&c, 1);
		}

		/* restore flags */
		IRQ_LOCK (flags);
		mainboard.flags = mainboard_flags;
		IRQ_UNLOCK (flags);
#endif
	}
	else if (!strcmp_P(res->arg1, PSTR("open")))
#ifdef HOST_VERSION
		beaconboard.link_id = 1;
	else
		printf("not implemented\n");
#else
		wt11_open_link_mux(beacon_addr, &beaconboard.link_id);

	else if (!strcmp_P(res->arg1, PSTR("close")))
		wt11_close_link_mux(beaconboard.link_id);

    else if (!strcmp_P(res->arg1, "on"))
		bt_beacon_set_on ();

    else if (!strcmp_P(res->arg1, "watchdog_on"))
		bt_beacon_set_on_watchdog ();

    else if (!strcmp_P(res->arg1, "off"))
		bt_beacon_set_off ();

    else if (!strcmp_P(res->arg1, "color"))
		bt_beacon_set_color ();

    else if (!strcmp_P(res->arg1, "status"))
		bt_beacon_req_status ();
#endif
}

prog_char str_beacon_arg0[] = "beacon";
parse_pgm_token_string_t cmd_beacon_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_result, arg0, str_beacon_arg0);
prog_char str_beacon_arg1[] = "raw#open#close#on#watchdog_on#off#color#status";
parse_pgm_token_string_t cmd_beacon_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_result, arg1, str_beacon_arg1);

//prog_char help_beacon[] = "beacon commads";
parse_pgm_inst_t cmd_beacon = {
    .f = cmd_beacon_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_beacon,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_beacon_arg0,
        (prog_void *) & cmd_beacon_arg1,
        NULL,
    },
};

#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/

/**********************************************************/
/* robot 2nd */

/* this structure is filled when cmd_interact is parsed successfully */
struct cmd_robot_2nd_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

static void cmd_robot_2nd_parsed(void * parsed_result, void * data)
{
   	int16_t c;
   	int8_t cmd = 0;
   	struct vt100 vt100;
	uint16_t mainboard_flags;
	uint8_t flags;
	uint8_t err = 0;

   	struct cmd_robot_2nd_result *res = parsed_result;

   	vt100_init(&vt100);

	if(!strcmp_P(res->arg1, "raw"))
	{

		/* save flags and disable BT_PROTO */
		IRQ_LOCK (flags);
		mainboard_flags = mainboard.flags;
		mainboard.flags &= (~DO_BT_PROTO);
		IRQ_UNLOCK (flags);

		/* init vt100 character set */
		vt100_init(&vt100);

#ifndef HOST_VERSION
		wt11_flush ();
#endif

		/* interact */
		while(cmd != KEY_CTRL_C)
		{
			/* link --> cmd line */
			wt11_bypass_to_stdo (robot_2nd.link_id);

			/* cmd line --> link */
			c = cmdline_getchar();
			if (c == -1) {
				continue;
			}

			/* check exit cmd */
			cmd = vt100_parser(&vt100, c);

			/* send to link */
			wt11_send_mux(robot_2nd.link_id, (uint8_t *)&c, 1);
		}

		/* restore flags */
		IRQ_LOCK (flags);
		mainboard.flags = mainboard_flags;
		IRQ_UNLOCK (flags);
	}
	else if (!strcmp_P(res->arg1, PSTR("open")))
#ifdef HOST_VERSION
		robot_2nd.link_id = 0;
#else
		wt11_open_link_mux(robot_2nd_addr, &robot_2nd.link_id);
#endif
	else if (!strcmp_P(res->arg1, PSTR("close")))
#ifdef HOST_VERSION
		robot_2nd.link_id = 0xFF;
#else
		wt11_close_link_mux(robot_2nd.link_id);
#endif
    else if (!strcmp_P(res->arg1, "color")) {
		bt_robot_2nd_set_color ();

		/* wait until command has been received */
		if (bt_robot_2nd_wait_ack()!=0)
			printf_P(PSTR("bt cmd ERROR\n\r"));
	}

    else if (!strcmp_P(res->arg1, "autopos")) {

		bt_robot_2nd_autopos ();

		/* wait until command has been received */
		if (bt_robot_2nd_wait_ack()!=0)
			printf_P(PSTR("bt cmd ERROR\n\r"));

		/* wait end traj */
		err = bt_robot_2nd_wait_end();
	    printf_P(PSTR("traj returned %s\r\n"), get_err(err));
	}

    else if (!strcmp_P(res->arg1, "status"))
		bt_robot_2nd_req_status ();

    else if (!strcmp_P(res->arg1, "show")) {

		 do
		 {
				printf ("cmd %d ret: %d %d %d %d\n\r", robot_2nd.cmd_id, robot_2nd.cmd_ret,
													robot_2nd.cmd_args_checksum_send,
													robot_2nd.cmd_args_checksum_recv,
													robot_2nd.valid_status);
				printf ("color %s\n\r", robot_2nd.color == I2C_COLOR_YELLOW? "yellow":"green");
				printf ("done_flags 0x%.4X\n\r", robot_2nd.done_flags);
				printf ("pos abs(%d %d %d) rel(%d %d)\n\r",
												robot_2nd.x, robot_2nd.y, robot_2nd.a_abs,
												robot_2nd.a, robot_2nd.d);

				printf ("opp1 (%d %d %d %d)\n\r", robot_2nd.opponent1_x, robot_2nd.opponent1_y,
														robot_2nd.opponent1_a, robot_2nd.opponent1_d);
				printf ("opp2 (%d %d %d %d)\n\r", robot_2nd.opponent2_x, robot_2nd.opponent2_y,
														robot_2nd.opponent2_a, robot_2nd.opponent2_d);

            wait_ms(200);

        }
        while (!cmdline_keypressed());
	}

	else if (!strcmp_P(res->arg1, PSTR("init_position"))){

//				strat_infos.strat_smart_sec = INIT_ROBOT_2ND;
	}

}

prog_char str_robot_2nd_arg0[] = "robot_2nd";
parse_pgm_token_string_t cmd_robot_2nd_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_robot_2nd_result, arg0, str_robot_2nd_arg0);
prog_char str_robot_2nd_arg1[] = "raw#open#close#color#autopos#status#show#init_position";
parse_pgm_token_string_t cmd_robot_2nd_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_robot_2nd_result, arg1, str_robot_2nd_arg1);

//prog_char help_robot_2nd[] = "robot_2nd commads";
parse_pgm_inst_t cmd_robot_2nd = {
    .f = cmd_robot_2nd_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_robot_2nd,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_robot_2nd_arg0,
        (prog_void *) & cmd_robot_2nd_arg1,
        NULL,
    },
};


/**********************************************************/
/* Robot 2nd goto function */

/* this structure is filled when cmd_goto is parsed successfully */
struct cmd_robot_2nd_goto_result
{
    fixed_string_t arga;
    fixed_string_t arg0;
    fixed_string_t arg1;
    int32_t arg2;
    int32_t arg3;
    int32_t arg4;
};

/* function called when cmd_goto is parsed successfully */
static void cmd_robot_2nd_goto_parsed(void * parsed_result, void * data)
{
    struct cmd_robot_2nd_goto_result * res = parsed_result;
    uint8_t err = END_ERROR;

    /* TODO commented functions */

    if (!strcmp_P(res->arg1, PSTR("a_rel")))
    {
        //bt_robot_2nd_goto_a_rel(res->arg2);
    }
    else if (!strcmp_P(res->arg1, PSTR("d_rel")))
    {
        //bt_robot_2nd_goto_d_rel(res->arg2);
    }
    else if (!strcmp_P(res->arg1, PSTR("a_abs")))
    {
        //bt_robot_2nd_goto_a_abs(res->arg2);
    }
    else if (!strcmp_P(res->arg1, PSTR("a_to_xy")))
    {
        //bt_robot_2nd_goto_turnto_xy(res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("a_behind_xy")))
    {
        //bt_robot_2nd_goto_turnto_xy_behind(res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("xy_rel")))
    {
        //bt_robot_2nd_goto_xy_rel(res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("da_rel")))
    {
        //bt_robot_2nd_goto_d_a_rel(res->arg2, res->arg3);
    }


    else if (!strcmp_P(res->arg1, PSTR("xy_abs")))
    {
        bt_robot_2nd_goto_xy_abs(res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("avoid")))
    {
        bt_robot_2nd_goto_and_avoid(res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("avoid_fw")))
    {
        bt_robot_2nd_goto_and_avoid_forward(res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("avoid_bw")))
    {
        bt_robot_2nd_goto_and_avoid_backward(res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("xy_abs_fow")))
    {
        bt_robot_2nd_goto_forward_xy_abs(res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("xy_abs_back")))
    {
        bt_robot_2nd_goto_backward_xy_abs(res->arg2, res->arg3);
    }

	/* wait until command has been received */
	if (bt_robot_2nd_wait_ack()!=0)
		printf_P(PSTR("bt cmd ERROR\n\r"));

	/* wait end traj */
	err = bt_robot_2nd_wait_end();
    printf_P(PSTR("traj returned %s\r\n"), get_err(err));

}

prog_char str_robot_2nd_goto_arga[] = "robot_2nd";
parse_pgm_token_string_t cmd_robot_2nd_goto_arga = TOKEN_STRING_INITIALIZER(struct cmd_robot_2nd_goto_result, arga, str_robot_2nd_goto_arga);

prog_char str_robot_2nd_goto_arg0[] = "goto";
parse_pgm_token_string_t cmd_robot_2nd_goto_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_robot_2nd_goto_result, arg0, str_robot_2nd_goto_arg0);
prog_char str_robot_2nd_goto_arg1_a[] = "d_rel#a_rel#a_abs";
parse_pgm_token_string_t cmd_robot_2nd_goto_arg1_a = TOKEN_STRING_INITIALIZER(struct cmd_robot_2nd_goto_result, arg1, str_robot_2nd_goto_arg1_a);
parse_pgm_token_num_t cmd_robot_2nd_goto_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_robot_2nd_goto_result, arg2, INT32);

/* 1 params */
//prog_char help_robot_2nd_goto1[] = "Change orientation of the mainboard";
parse_pgm_inst_t cmd_robot_2nd_goto1 = {
    .f = cmd_robot_2nd_goto_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_robot_2nd_goto1,
    .tokens =
    { /* token list, NULL terminated */
		(prog_void *) & cmd_robot_2nd_goto_arga,
        (prog_void *) & cmd_robot_2nd_goto_arg0,
        (prog_void *) & cmd_robot_2nd_goto_arg1_a,
        (prog_void *) & cmd_robot_2nd_goto_arg2,
        NULL,
    },
};

prog_char str_robot_2nd_goto_arg1_b[] = "xy_rel#xy_abs#xy_abs_fow#xy_abs_back#da_rel#a_to_xy#avoid#avoid_fw#avoid_bw#a_behind_xy";
parse_pgm_token_string_t cmd_robot_2nd_goto_arg1_b = TOKEN_STRING_INITIALIZER(struct cmd_robot_2nd_goto_result, arg1, str_robot_2nd_goto_arg1_b);
parse_pgm_token_num_t cmd_robot_2nd_goto_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_robot_2nd_goto_result, arg3, INT32);

/* 2 params */
//prog_char help_robot_2nd_goto2[] = "Go to a (x,y) or (d,a) position";
parse_pgm_inst_t cmd_robot_2nd_goto2 = {
    .f = cmd_robot_2nd_goto_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_robot_2nd_goto2,
    .tokens =
    { /* token list, NULL terminated */
		(prog_void *) & cmd_robot_2nd_goto_arga,
        (prog_void *) & cmd_robot_2nd_goto_arg0,
        (prog_void *) & cmd_robot_2nd_goto_arg1_b,
        (prog_void *) & cmd_robot_2nd_goto_arg2,
        (prog_void *) & cmd_robot_2nd_goto_arg3,
        NULL,
    },
};


/**********************************************************/
/* Robot 2nd task function */

/* this structure is filled when cmd_goto is parsed successfully */
struct cmd_robot_2nd_bt_task_result
{
    fixed_string_t arga;
    fixed_string_t arg0;
    fixed_string_t arg1;
    int32_t arg2;
    int32_t arg3;
};

/* function called when cmd_bt_task is parsed successfully */
static void cmd_robot_2nd_bt_task_parsed(void * parsed_result, void * data)
{
    struct cmd_robot_2nd_bt_task_result * res = parsed_result;
    uint8_t err = END_ERROR;

    if (!strcmp_P(res->arg1, PSTR("pick_cup_front")))
    {
       bt_robot_2nd_bt_task_pick_cup(res->arg2, res->arg3, BT_SIDE_FRONT);
    }
    else if (!strcmp_P(res->arg1, PSTR("pick_cup_rear")))
    {
       bt_robot_2nd_bt_task_pick_cup(res->arg2, res->arg3, BT_SIDE_REAR);
    }

    else if (!strcmp_P(res->arg1, PSTR("carpet")))
    {
        bt_robot_2nd_bt_task_carpet();
    }

    else if (!strcmp_P(res->arg1, PSTR("stairs")))
    {
        bt_robot_2nd_bt_task_stairs();
    }

    else if (!strcmp_P(res->arg1, PSTR("bring_cup_front")))
    {
        bt_robot_2nd_bt_task_bring_cup_cinema(res->arg2, res->arg3, BT_SIDE_FRONT);
	}
    else if (!strcmp_P(res->arg1, PSTR("bring_cup_rear")))
    {
        bt_robot_2nd_bt_task_bring_cup_cinema(res->arg2, res->arg3, BT_SIDE_REAR);
	}

    else if (!strcmp_P(res->arg1, PSTR("clap")))
    {
        bt_robot_2nd_bt_task_clapperboard(res->arg2, res->arg3);
	}


	/* wait until command has been received */
	bt_robot_2nd_wait_ack();

	/* wait end traj */
	err = bt_robot_2nd_wait_end();
    printf_P(PSTR("traj returned %s\r\n"), get_err(err));
}

prog_char str_robot_2nd_bt_task_arga[] = "robot_2nd";
parse_pgm_token_string_t cmd_robot_2nd_bt_task_arga = TOKEN_STRING_INITIALIZER(struct cmd_robot_2nd_bt_task_result, arga, str_robot_2nd_bt_task_arga);

prog_char str_robot_2nd_bt_task_arg0[] = "bt_task";
parse_pgm_token_string_t cmd_robot_2nd_bt_task_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_robot_2nd_bt_task_result, arg0, str_robot_2nd_bt_task_arg0);
prog_char str_robot_2nd_bt_task_arg1[] = "carpet#stairs";
parse_pgm_token_string_t cmd_robot_2nd_bt_task_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_robot_2nd_bt_task_result, arg1, str_robot_2nd_bt_task_arg1);

/* 1 params */
//prog_char help_robot_2nd_bt_task[] = "robot_2nd tasks";
parse_pgm_inst_t cmd_robot_2nd_bt_task = {
    .f = cmd_robot_2nd_bt_task_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_robot_2nd_bt_task,
    .tokens =
    { /* token list, NULL terminated */
		(prog_void *) & cmd_robot_2nd_bt_task_arga,
        (prog_void *) & cmd_robot_2nd_bt_task_arg0,
        (prog_void *) & cmd_robot_2nd_bt_task_arg1,
        NULL,
    },
};


prog_char str_robot_2nd_bt_task2_arg1[] = "pick_cup_front#pick_cup_rear#bring_cup_front#bring_cup_rear#clap";
parse_pgm_token_string_t cmd_robot_2nd_bt_task2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_robot_2nd_bt_task_result, arg1, str_robot_2nd_bt_task2_arg1);
parse_pgm_token_num_t cmd_robot_2nd_bt_task2_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_robot_2nd_bt_task_result, arg2, INT32);
parse_pgm_token_num_t cmd_robot_2nd_bt_task2_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_robot_2nd_bt_task_result, arg3, INT32);

/* 3 params */
//prog_char help_robot_2nd_bt_task2[] = "robot_2nd tasks: x y";
parse_pgm_inst_t cmd_robot_2nd_bt_task2 = {
    .f = cmd_robot_2nd_bt_task_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_robot_2nd_bt_task2,
    .tokens =
    { /* token list, NULL terminated */
		(prog_void *) & cmd_robot_2nd_bt_task_arga,
        (prog_void *) & cmd_robot_2nd_bt_task_arg0,
        (prog_void *) & cmd_robot_2nd_bt_task2_arg1,
        (prog_void *) & cmd_robot_2nd_bt_task2_arg2,
        (prog_void *) & cmd_robot_2nd_bt_task2_arg3,
        NULL,
    },
};

#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/


/**********************************************************/
/* slavedspic */

/* this structure is filled when cmd_slavedspic is parsed successfully */
struct cmd_slavedspic_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

/* function called when cmd_slavedspic is parsed successfully */
static void cmd_slavedspic_parsed(void *parsed_result, void *data)
{
    struct cmd_slavedspic_result *res = parsed_result;
    static uint8_t led_flag = 0;
    int16_t c;
    int8_t cmd = 0;
    struct vt100 vt100;
    uint8_t mainboard_flags;
    uint8_t flags;

    if (!strcmp_P(res->arg1, "raw"))
    {
#ifdef HOST_VERSION
        printf("not implemented\n");
#else
        /* disable opponent event */
        IRQ_LOCK(flags);
        mainboard_flags = mainboard.flags;
        mainboard.flags &= ~(DO_BT_PROTO);
        IRQ_UNLOCK(flags);

        /* flush rx buffer */
        while (uart_recv_nowait(MUX_UART) != -1);

        /* remap UART */
        set_uart_mux(SLAVEDSPIC_CHANNEL);

        /* init vt100 character set */
        vt100_init(&vt100);

        /* interact */
        while (cmd != KEY_CTRL_C)
        {
            /* received from slave */
            if ((c = uart_recv_nowait(MUX_UART)) != -1)
                /* echo */
                uart_send_nowait(CMDLINE_UART, c);

            /* send to slavedspic */
            c = cmdline_getchar();
            if (c == -1)
            {
                continue;
            }

            /* check exit cmd */
            cmd = vt100_parser(&vt100, c);

            /* send to slave */
            uart_send_nowait(MUX_UART, c);
        }

        /* remap UART */
        set_uart_mux(BEACON_CHANNEL);

        /* restore opponent event */
        IRQ_LOCK(flags);
        mainboard.flags = mainboard_flags;
        IRQ_UNLOCK(flags);
#endif
    }
    else if (!strcmp(res->arg1, "init"))
    {
        i2c_slavedspic_mode_init();
    }
    else if (!strcmp(res->arg1, "info"))
    {
        printf("TODO");
    }
    else if (!strcmp(res->arg1, "led"))
    {
        i2c_led_control(I2C_SLAVEDSPIC_ADDR, 1, led_flag);
        led_flag ^= 1;
    }
    else if (!strcmp(res->arg1, "poweroff"))
    {
        i2c_slavedspic_mode_power_off();
    }


    //printf("done \r\n");
    return;
}

prog_char str_slavedspic_arg0[] = "slavedspic";
parse_pgm_token_string_t cmd_slavedspic_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_slavedspic_result, arg0, str_slavedspic_arg0);
prog_char str_slavedspic_arg1[] = "raw#init#led#poweroff";
parse_pgm_token_string_t cmd_slavedspic_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_slavedspic_result, arg1, str_slavedspic_arg1);

//prog_char help_slavedspic[] = "control slavedspic";
parse_pgm_inst_t cmd_slavedspic = {
    .f = cmd_slavedspic_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_slavedspic,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_slavedspic_arg0,
        (prog_void *) & cmd_slavedspic_arg1,
        NULL,
    },
};


/**********************************************************/
/* Robot sensors test */

/* this structure is filled when cmd_sensor is parsed successfully */
struct cmd_sensor_robot_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};


#define _BV64(i) ((uint64_t)1 << i)

/* function called when cmd_sensor is parsed successfully */
static void cmd_sensor_robot_parsed(void *parsed_result, void *data)
{
    struct cmd_sensor_robot_result *res = parsed_result;
    volatile int64_t sensor_prev = 0;
    uint8_t sensor_event = 0;
    uint8_t i;
    char sensor_string[15];

    if (!strcmp(res->arg1, "robot"))
    {
        do
        {
            for (i = 0; i < SENSOR_MAX; i++)
            {

                if (sensor_get(i) && ((sensor_prev & _BV64(i)) == 0))
                {
                    sensor_event = 1;
                    sensor_prev |= _BV64(i);
                }
                else if (!sensor_get(i) && ((sensor_prev & _BV64(i)) != 0))
                {
                    sensor_event = 1;
                    sensor_prev &= (~_BV64(i));
                }

                if (sensor_event)
                {

                    sensor_event = 0;

                    switch (i)
                    {
                    case S_START_SWITCH: strcpy(sensor_string, "START");
                        break;
                    case S_OPPONENT_FRONT_R: strcpy(sensor_string, "OP_FRONT_R");
                        break;
                    case S_OPPONENT_FRONT_L: strcpy(sensor_string, "OP_FRONT_L");
                        break;
                    case S_OPPONENT_REAR_R: strcpy(sensor_string, "OP_REAR_R");
                        break;
                    case S_OPPONENT_REAR_L: strcpy(sensor_string, "OP_REAR_L");
                        break;
                    case S_CUP_REAR: strcpy(sensor_string, "CUP_REAR");
                        break;

                    default: strcpy(sensor_string, "S_UNKNOW");
                        continue;
                    }

                    printf("%s %s!\n\r", sensor_string, (sensor_prev & _BV64(i)) ? "on" : "off");

                }
            }

            wait_ms(100);
        }
        while (!cmdline_keypressed());
    }
}

prog_char str_sensor_robot_arg0[] = "sensor";
parse_pgm_token_string_t cmd_sensor_robot_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_sensor_robot_result, arg0, str_sensor_robot_arg0);
prog_char str_sensor_robot_arg1[] = "robot";
parse_pgm_token_string_t cmd_sensor_robot_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_sensor_robot_result, arg1, str_sensor_robot_arg1);

//prog_char help_sensor_robot[] = "Show robot sensors";
parse_pgm_inst_t cmd_sensor_robot = {
    .f = cmd_sensor_robot_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_sensor_robot,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_sensor_robot_arg0,
        (prog_void *) & cmd_sensor_robot_arg1,
        NULL,
    },
};


#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/

/**********************************************************/
/* Interact */

/* this structure is filled when cmd_interact is parsed successfully */
struct cmd_interact_result
{
    fixed_string_t arg0;
};

static void print_cs(void)
{
    printf_P(PSTR("cons_d=% .8"PRIi32" cons_a=% .8"PRIi32" fil_d=% .8"PRIi32" fil_a=% .8"PRIi32" "
                  "err_d=% .8"PRIi32" err_a=% .8"PRIi32" out_d=% .8"PRIi32" out_a=% .8"PRIi32"\r\n"),
             cs_get_consign(&mainboard.distance.cs),
             cs_get_consign(&mainboard.angle.cs),
             cs_get_filtered_consign(&mainboard.distance.cs),
             cs_get_filtered_consign(&mainboard.angle.cs),
             cs_get_error(&mainboard.distance.cs),
             cs_get_error(&mainboard.angle.cs),
             cs_get_out(&mainboard.distance.cs),
             cs_get_out(&mainboard.angle.cs));
}

static void print_pos(void)
{
    printf_P(PSTR("x=% .8d y=% .8d a=% .8d\r\n"),
             position_get_x_s16(&mainboard.pos),
             position_get_y_s16(&mainboard.pos),
             position_get_a_deg_s16(&mainboard.pos));
}

static void print_time(void)
{
    printf_P(PSTR("time %d\r\n"), (int16_t) time_get_s());
}

static void print_sensors(void)
{
#ifdef notyet
    if (sensor_start_switch())
        printf_P(PSTR("Start switch | "));
    else
        printf_P(PSTR("             | "));

    if (IR_DISP_SENSOR())
        printf_P(PSTR("IR disp | "));
    else
        printf_P(PSTR("        | "));

    printf_P(PSTR("\r\n"));
#endif
}

static void print_pid(void)
{
    printf_P(PSTR("P=% .8"PRIi32" I=% .8"PRIi32" D=% .8"PRIi32" out=% .8"PRIi32" | "
                  "P=% .8"PRIi32" I=% .8"PRIi32" D=% .8"PRIi32" out=% .8"PRIi32"\r\n"),
             pid_get_value_in(&mainboard.distance.pid) * pid_get_gain_P(&mainboard.distance.pid),
             pid_get_value_I(&mainboard.distance.pid) * pid_get_gain_I(&mainboard.distance.pid),
             pid_get_value_D(&mainboard.distance.pid) * pid_get_gain_D(&mainboard.distance.pid),
             pid_get_value_out(&mainboard.distance.pid),
             pid_get_value_in(&mainboard.angle.pid) * pid_get_gain_P(&mainboard.angle.pid),
             pid_get_value_I(&mainboard.angle.pid) * pid_get_gain_I(&mainboard.angle.pid),
             pid_get_value_D(&mainboard.angle.pid) * pid_get_gain_D(&mainboard.angle.pid),
             pid_get_value_out(&mainboard.angle.pid));
}

#define PRINT_POS       (1<<0)
#define PRINT_PID       (1<<1)
#define PRINT_CS        (1<<2)
#define PRINT_SENSORS   (1<<3)
#define PRINT_TIME      (1<<4)
#define PRINT_BLOCKING  (1<<5)

static void cmd_interact_parsed(void * parsed_result, void * data)
{
    int c;
    int8_t cmd;
    uint8_t print = 0;
    struct vt100 vt100;

    vt100_init(&vt100);

    printf_P(PSTR("Display debugs:\r\n"
                  "  1:pos\r\n"
                  "  2:pid\r\n"
                  "  3:cs\r\n"
                  "  4:sensors\r\n"
                  "  5:time\r\n"
                  /* "  6:blocking\r\n" */
                  "Commands:\r\n"
                  "  arrows:move\r\n"
                  "  space:stop\r\n"
                  "  q:quit\r\n"));

    /* stop motors */
    mainboard.flags &= (~DO_CS);
    dac_set_and_save(LEFT_MOTOR, 0);
    dac_set_and_save(RIGHT_MOTOR, 0);

    while (1)
    {
        if (print & PRINT_POS)
        {
            print_pos();
        }

        if (print & PRINT_PID)
        {
            print_pid();
        }

        if (print & PRINT_CS)
        {
            print_cs();
        }

        if (print & PRINT_SENSORS)
        {
            print_sensors();
        }

        if (print & PRINT_TIME)
        {
            print_time();
        }
        /* 		if (print & PRINT_BLOCKING) { */
        /* 			printf_P(PSTR("%s %s blocking=%d\r\n"),  */
        /* 				 mainboard.blocking ? "BLOCK1":"      ", */
        /* 				 rs_is_blocked(&mainboard.rs) ? "BLOCK2":"      ", */
        /* 				 rs_get_blocking(&mainboard.rs)); */
        /* 		} */

        c = cmdline_getchar();
        if (c == -1)
        {
            wait_ms(10);
            continue;
        }
        cmd = vt100_parser(&vt100, c);
        if (cmd == -2)
        {
            wait_ms(10);
            continue;
        }

        if (cmd == -1)
        {
            switch (c)
            {
            case '1': print ^= PRINT_POS;
                break;
            case '2': print ^= PRINT_PID;
                break;
            case '3': print ^= PRINT_CS;
                break;
            case '4': print ^= PRINT_SENSORS;
                break;
            case '5': print ^= PRINT_TIME;
                break;
            case '6': print ^= PRINT_BLOCKING;
                break;

            case 'q':
                if (mainboard.flags & DO_CS)
                    strat_hardstop();
                dac_set_and_save(LEFT_MOTOR, 0);
                dac_set_and_save(RIGHT_MOTOR, 0);
                return;
            case ' ':
                dac_set_and_save(LEFT_MOTOR, 0);
                dac_set_and_save(RIGHT_MOTOR, 0);
                break;
            default:
                break;
            }
        }
        else
        {
#ifdef HOST_VERSION
#define PWM_INTERACT 1200
#else
#define PWM_INTERACT 1200
#endif
            switch (cmd)
            {
            case KEY_UP_ARR:
                dac_set_and_save(LEFT_MOTOR, PWM_INTERACT);
                dac_set_and_save(RIGHT_MOTOR, PWM_INTERACT);
                break;
            case KEY_LEFT_ARR:
                dac_set_and_save(LEFT_MOTOR, -PWM_INTERACT);
                dac_set_and_save(RIGHT_MOTOR, PWM_INTERACT);
                break;
            case KEY_DOWN_ARR:
                dac_set_and_save(LEFT_MOTOR, -PWM_INTERACT);
                dac_set_and_save(RIGHT_MOTOR, -PWM_INTERACT);
                break;
            case KEY_RIGHT_ARR:
                dac_set_and_save(LEFT_MOTOR, PWM_INTERACT);
                dac_set_and_save(RIGHT_MOTOR, -PWM_INTERACT);
                break;
            }
        }
        wait_ms(10);
    }
}

prog_char str_interact_arg0[] = "interact";
parse_pgm_token_string_t cmd_interact_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_interact_result, arg0, str_interact_arg0);

//prog_char help_interact[] = "Interactive mode";
parse_pgm_inst_t cmd_interact = {
    .f = cmd_interact_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_interact,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_interact_arg0,
        NULL,
    },
};



/**********************************************************/
/* Rs tests */

/* this structure is filled when cmd_rs is parsed successfully */
struct cmd_rs_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

/* function called when cmd_rs is parsed successfully */
static void cmd_rs_parsed(void *parsed_result, void *data)
{
    //	struct cmd_rs_result *res = parsed_result;
    do
    {
        printf_P(PSTR("angle cons=% .6"PRIi32" in=% .6"PRIi32" out=% .6"PRIi32" / "),
                 cs_get_consign(&mainboard.angle.cs),
                 cs_get_filtered_feedback(&mainboard.angle.cs),
                 cs_get_out(&mainboard.angle.cs));
        printf_P(PSTR("distance cons=% .6"PRIi32" in=% .6"PRIi32" out=% .6"PRIi32" / "),
                 cs_get_consign(&mainboard.distance.cs),
                 cs_get_filtered_feedback(&mainboard.distance.cs),
                 cs_get_out(&mainboard.distance.cs));
        printf_P(PSTR("l=% .4"PRIi32" r=% .4"PRIi32"\r\n"), mainboard.dac_l,
                 mainboard.dac_r);
        wait_ms(100);
    }
    while (!cmdline_keypressed());
}

prog_char str_rs_arg0[] = "rs";
parse_pgm_token_string_t cmd_rs_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg0, str_rs_arg0);
prog_char str_rs_arg1[] = "show";
parse_pgm_token_string_t cmd_rs_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg1, str_rs_arg1);

//prog_char help_rs[] = "Show rs (robot system) values";
parse_pgm_inst_t cmd_rs = {
    .f = cmd_rs_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_rs,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_rs_arg0,
        (prog_void *) & cmd_rs_arg1,
        NULL,
    },
};

#ifdef TRAJECTORY_MANAGER_V3
/**********************************************************/
/* Clitoid */

/* this structure is filled when cmd_clitoid is parsed successfully */
struct cmd_clitoid_result
{
    fixed_string_t arg0;
    float alpha_deg;
    float beta_deg;
    float R_mm;
    float Vd;
    float Amax;
    float d_inter_mm;
};

/* function called when cmd_test is parsed successfully */
static void cmd_clitoid_parsed(void *parsed_result, void *data)
{
    struct cmd_clitoid_result *res = parsed_result;
    /* 	clitoid(res->alpha_deg, res->beta_deg, res->R_mm, */
    /* 		res->Vd, res->Amax, res->d_inter_mm); */
    double x = position_get_x_double(&mainboard.pos);
    double y = position_get_y_double(&mainboard.pos);
    double a = position_get_a_rad_double(&mainboard.pos);

    strat_set_speed(res->Vd, SPEED_ANGLE_FAST);
    trajectory_clitoid(&mainboard.traj, x, y, a, 150.,
                       res->alpha_deg, res->beta_deg, res->R_mm,
                       res->d_inter_mm);
}

prog_char str_clitoid_arg0[] = "clitoid";
parse_pgm_token_string_t cmd_clitoid_arg0 =
        TOKEN_STRING_INITIALIZER(struct cmd_clitoid_result,
                                 arg0, str_clitoid_arg0);
parse_pgm_token_num_t cmd_clitoid_alpha_deg =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              alpha_deg, FLOAT);
parse_pgm_token_num_t cmd_clitoid_beta_deg =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              beta_deg, FLOAT);
parse_pgm_token_num_t cmd_clitoid_R_mm =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              R_mm, FLOAT);
parse_pgm_token_num_t cmd_clitoid_Vd =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              Vd, FLOAT);
parse_pgm_token_num_t cmd_clitoid_Amax =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              Amax, FLOAT);
parse_pgm_token_num_t cmd_clitoid_d_inter_mm =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              d_inter_mm, FLOAT);

//prog_char help_clitoid[] = "do a clitoid (alpha, beta, R, Vd, Amax, d_inter)";
parse_pgm_inst_t cmd_clitoid = {
    .f = cmd_clitoid_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_clitoid,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_clitoid_arg0,
        (prog_void *) & cmd_clitoid_alpha_deg,
        (prog_void *) & cmd_clitoid_beta_deg,
        (prog_void *) & cmd_clitoid_R_mm,
        (prog_void *) & cmd_clitoid_Vd,
        (prog_void *) & cmd_clitoid_Amax,
        (prog_void *) & cmd_clitoid_d_inter_mm,
        NULL,
    },
};
#endif /* TRAJECTORY_MANAGER_V3 */
/**********************************************************/
/* Time_Monitor */

/* this structure is filled when cmd_time_monitor is parsed successfully */
struct cmd_time_monitor_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

/* function called when cmd_time_monitor is parsed successfully */
static void cmd_time_monitor_parsed(void *parsed_result, void *data)
{
#ifndef HOST_VERSION
    /*
            struct cmd_time_monitor_result *res = parsed_result;
            uint16_t seconds;

            if (!strcmp_P(res->arg1, PSTR("reset"))) {
                    eeprom_write_word(EEPROM_TIME_ADDRESS, 0);
            }
            seconds = eeprom_read_word(EEPROM_TIME_ADDRESS);
            printf_P(PSTR("Running since %d mn %d\r\n"), seconds/60, seconds%60);
     */
#endif
}

prog_char str_time_monitor_arg0[] = "time_monitor";
parse_pgm_token_string_t cmd_time_monitor_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_time_monitor_result, arg0, str_time_monitor_arg0);
prog_char str_time_monitor_arg1[] = "show#reset";
parse_pgm_token_string_t cmd_time_monitor_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_time_monitor_result, arg1, str_time_monitor_arg1);

//prog_char help_time_monitor[] = "Show since how long we are running";
parse_pgm_inst_t cmd_time_monitor = {
    .f = cmd_time_monitor_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_time_monitor,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_time_monitor_arg0,
        (prog_void *) & cmd_time_monitor_arg1,
        NULL,
    },
};

/**********************************************************/
/* Sleep */

/* this structure is filled when cmd_sleep is parsed successfully */
struct cmd_sleep_result
{
    fixed_string_t arg0;
    uint32_t ms;
};

/* function called when cmd_sleep is parsed successfully */
static void cmd_sleep_parsed(void *parsed_result, void *data)
{
    struct cmd_sleep_result *res = parsed_result;
    time_wait_ms(res->ms);
}

prog_char str_sleep_arg0[] = "sleep";
parse_pgm_token_string_t cmd_sleep_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_sleep_result, arg0, str_sleep_arg0);
parse_pgm_token_num_t cmd_sleep_ms = TOKEN_NUM_INITIALIZER(struct cmd_sleep_result, ms, UINT32);

//prog_char help_sleep[] = "Sleep during some miliseconds";
parse_pgm_inst_t cmd_sleep = {
    .f = cmd_sleep_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_sleep,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_sleep_arg0,
        (prog_void *) & cmd_sleep_ms,
        NULL,
    },
};


#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/

//#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/

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
	uint8_t mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;

	if (!strcmp_P(res->arg1, PSTR("idle")))
		mode = I2C_SLAVEDSPIC_MODE_PS_IDLE;
	else if (!strcmp_P(res->arg1, PSTR("front_ready")))
		mode = I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_READY;
	else if (!strcmp_P(res->arg1, PSTR("front_catch")))
		mode = I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_CATCH_AND_DROP;
	else if (!strcmp_P(res->arg1, PSTR("front_release")))
		mode = I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_RELEASE;
	else if (!strcmp_P(res->arg1, PSTR("front_hide")))
		mode = I2C_SLAVEDSPIC_MODE_PS_CUP_FRONT_HIDE;
	else if (!strcmp_P(res->arg1, PSTR("rear_open")))
		mode = I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_OPEN;
	else if (!strcmp_P(res->arg1, PSTR("rear_catch")))
		mode = I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_CATCH;
	else if (!strcmp_P(res->arg1, PSTR("rear_release")))
		mode = I2C_SLAVEDSPIC_MODE_PS_CUP_REAR_RELEASE;
	else if (!strcmp_P(res->arg1, PSTR("machines_ready")))
		mode = I2C_SLAVEDSPIC_MODE_PS_MACHINES_READY;
	else if (!strcmp_P(res->arg1, PSTR("machines_harvest")))
		mode = I2C_SLAVEDSPIC_MODE_PS_MACHINES_HARVEST;
	else if (!strcmp_P(res->arg1, PSTR("machines_end")))
		mode = I2C_SLAVEDSPIC_MODE_PS_MACHINES_END;
	else if (!strcmp_P(res->arg1, PSTR("dump")))
		mode = I2C_SLAVEDSPIC_MODE_PS_STOCK_DROP;
	else if (!strcmp_P(res->arg1, PSTR("dump_end")))
		mode = I2C_SLAVEDSPIC_MODE_PS_STOCK_END;

	i2c_slavedspic_mode_ps(mode);
}

prog_char str_popcorn_system_arg0[] = "ps";
parse_pgm_token_string_t cmd_popcorn_system_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_popcorn_system_result, arg0, str_popcorn_system_arg0);
prog_char str_popcorn_system_arg1[] = "idle#front_ready#front_catch#front_release#front_hide#rear_open#rear_catch#rear_release#machines_ready#machines_harvest#machines_end#dump#dump_end";
parse_pgm_token_string_t cmd_popcorn_system_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_popcorn_system_result, arg1, str_popcorn_system_arg1);

//prog_char help_popcorn_system[] = "set popcorn_system mode: ps mode";
parse_pgm_inst_t cmd_popcorn_system = {
	.f = cmd_popcorn_system_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	//.help_str = help_popcorn_system,
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
	uint8_t mode = I2C_SLAVEDSPIC_MODE_SS_IDLE;
	uint8_t side = I2C_SIDE_LEFT;

	/* XXX */
	res->arg3 = 0;

	if (!strcmp_P(res->arg1, PSTR("left")))
		side = I2C_SIDE_LEFT;
	else if (!strcmp_P(res->arg1, PSTR("right")))
		side = I2C_SIDE_RIGHT;
	else if (!strcmp_P(res->arg1, PSTR("all")))
		side = I2C_SIDE_ALL;

	if (!strcmp_P(res->arg2, PSTR("idle")))
		mode = I2C_SLAVEDSPIC_MODE_SS_IDLE;
	else if (!strcmp_P(res->arg2, PSTR("hide")))
		mode = I2C_SLAVEDSPIC_MODE_SS_HIDE_TOWER;
    else if (!strcmp_P(res->arg2, PSTR("ready")))
		mode = I2C_SLAVEDSPIC_MODE_SS_HARVEST_STAND_READY;
	else if (!strcmp_P(res->arg2, PSTR("harvest")))
		mode = I2C_SLAVEDSPIC_MODE_SS_HARVEST_STAND_DO;
	else if (!strcmp_P(res->arg2, PSTR("build")))
		mode = I2C_SLAVEDSPIC_MODE_SS_BUILD_SPOTLIGHT;
	else if (!strcmp_P(res->arg2, PSTR("release")))
		mode = I2C_SLAVEDSPIC_MODE_SS_RELEASE_SPOTLIGHT;

	if (side != I2C_SIDE_ALL) {
		if (!strcmp_P(res->arg2, PSTR("ready")))
			i2c_slavedspic_mode_ss_harvest_ready(side, res->arg3);
        else if (!strcmp_P(res->arg2, PSTR("harvest")))
			i2c_slavedspic_mode_ss_harvest_do(side, res->arg3);
		else
			i2c_slavedspic_mode_ss(mode, side);
	}
	else {
		if (!strcmp_P(res->arg2, PSTR("ready"))) {
			i2c_slavedspic_mode_ss_harvest_ready(I2C_SIDE_LEFT, res->arg3);
			i2c_slavedspic_mode_ss_harvest_ready(I2C_SIDE_RIGHT, res->arg3);
		}
        else if (!strcmp_P(res->arg2, PSTR("harvest"))) {
			i2c_slavedspic_mode_ss_harvest_do(I2C_SIDE_LEFT, res->arg3);
			i2c_slavedspic_mode_ss_harvest_do(I2C_SIDE_RIGHT, res->arg3);
		}
		else {
			i2c_slavedspic_mode_ss(mode, I2C_SIDE_LEFT);
			i2c_slavedspic_mode_ss(mode, I2C_SIDE_RIGHT);
		}
	}
}

prog_char str_stands_system_arg0[] = "ss";
parse_pgm_token_string_t cmd_stands_system_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_stands_system_result, arg0, str_stands_system_arg0);
prog_char str_stands_system_arg1[] = "left#right#all";
parse_pgm_token_string_t cmd_stands_system_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_stands_system_result, arg1, str_stands_system_arg1);
prog_char str_stands_system_arg2[] = "idle#hide#ready#harvest#build#release";
parse_pgm_token_string_t cmd_stands_system_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_stands_system_result, arg2, str_stands_system_arg2);
parse_pgm_token_num_t cmd_stands_system_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_stands_system_result, arg3, UINT8);

//prog_char help_stands_system[] = "set stands_system mode: ss mode blade angle side";
parse_pgm_inst_t cmd_stands_system = {
	.f = cmd_stands_system_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	//.help_str = help_stands_system,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_stands_system_arg0,
		(prog_void *)&cmd_stands_system_arg2,
		//(prog_void *)&cmd_stands_system_arg3,
		(prog_void *)&cmd_stands_system_arg1,
		NULL,
	},
};
//#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/

