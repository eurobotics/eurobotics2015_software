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
 *  Javier Bali\F1as Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  commands_traj.c,v 1.7 2009/05/27 20:04:07 zer0 Exp.
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
#include <encoders_dspic.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <f64.h>
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

#include "main.h"
#include "cs.h"
#include "cmdline.h"
#include "strat_utils.h"
#include "strat_base.h"
#include "strat_avoid.h"
#include "strat.h"
#include "../common/i2c_commands.h"
#include "bt_protocol.h"

#ifdef HOST_VERSION
#define COMPILE_COMMANDS_TRAJ
#define COMPILE_COMMANDS_TRAJ_OPTIONALS
#endif

#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)

/**********************************************************/
/* Traj_Speeds for trajectory_manager */

/* this structure is filled when cmd_traj_speed is parsed successfully */
struct cmd_traj_speed_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    int16_t s;
};

/* function called when cmd_traj_speed is parsed successfully */
static void cmd_traj_speed_parsed(void *parsed_result, void *data)
{
    struct cmd_traj_speed_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("angle")))
    {
        trajectory_set_speed(&mainboard.traj, mainboard.traj.d_speed, res->s);
    }
    else if (!strcmp_P(res->arg1, PSTR("distance")))
    {
        trajectory_set_speed(&mainboard.traj, res->s, mainboard.traj.a_speed);
    }
    /* else it is a "show" */

    printf_P(PSTR("angle %d, distance %d\r\n"),
             mainboard.traj.a_speed,
             mainboard.traj.d_speed);
}

prog_char str_traj_speed_arg0[] = "traj_speed";
parse_pgm_token_string_t cmd_traj_speed_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg0, str_traj_speed_arg0);
prog_char str_traj_speed_arg1[] = "angle#distance";
parse_pgm_token_string_t cmd_traj_speed_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg1, str_traj_speed_arg1);
parse_pgm_token_num_t cmd_traj_speed_s = TOKEN_NUM_INITIALIZER(struct cmd_traj_speed_result, s, INT16);

//prog_char help_traj_speed[] = "Set traj_speed values for trajectory manager";
parse_pgm_inst_t cmd_traj_speed = {
    .f = cmd_traj_speed_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_traj_speed,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_traj_speed_arg0,
        (prog_void *) & cmd_traj_speed_arg1,
        (prog_void *) & cmd_traj_speed_s,
        NULL,
    },
};

/* show */

prog_char str_traj_speed_show_arg[] = "show";
parse_pgm_token_string_t cmd_traj_speed_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg1, str_traj_speed_show_arg);

//prog_char help_traj_speed_show[] = "Show traj_speed values for trajectory manager";
parse_pgm_inst_t cmd_traj_speed_show = {
    .f = cmd_traj_speed_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_traj_speed_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_traj_speed_arg0,
        (prog_void *) & cmd_traj_speed_show_arg,
        NULL,
    },
};

#ifdef COMPILE_COMMANDS_TRAJ_OPTIONALS /*-------------------------------------*/

#ifdef TRAJECTORY_MANAGER_V3

/**********************************************************/
/* Traj_Accs for trajectory_manager */

/* this structure is filled when cmd_traj_acc is parsed successfully */
struct cmd_traj_acc_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float s;
};

/* function called when cmd_traj_acc is parsed successfully */
static void cmd_traj_acc_parsed(void *parsed_result, void *data)
{
    struct cmd_traj_acc_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("angle")))
    {
        trajectory_set_acc(&mainboard.traj, mainboard.traj.d_acc, res->s);
    }
    else if (!strcmp_P(res->arg1, PSTR("distance")))
    {
        trajectory_set_acc(&mainboard.traj, res->s, mainboard.traj.a_acc);
    }
    /* else it is a "show" */

    printf_P(PSTR("angle %d, distance %d\r\n"),
             mainboard.traj.a_acc,
             mainboard.traj.d_acc);
}

prog_char str_traj_acc_arg0[] = "traj_acc";
parse_pgm_token_string_t cmd_traj_acc_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_traj_acc_result, arg0, str_traj_acc_arg0);
prog_char str_traj_acc_arg1[] = "angle#distance";
parse_pgm_token_string_t cmd_traj_acc_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_traj_acc_result, arg1, str_traj_acc_arg1);
parse_pgm_token_num_t cmd_traj_acc_s = TOKEN_NUM_INITIALIZER(struct cmd_traj_acc_result, s, INT16);

//prog_char help_traj_acc[] = "Set traj_acc values for trajectory manager";
parse_pgm_inst_t cmd_traj_acc = {
    .f = cmd_traj_acc_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_traj_acc,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_traj_acc_arg0,
        (prog_void *) & cmd_traj_acc_arg1,
        (prog_void *) & cmd_traj_acc_s,
        NULL,
    },
};

/* show */

prog_char str_traj_acc_show_arg[] = "show";
parse_pgm_token_string_t cmd_traj_acc_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_traj_acc_result, arg1, str_traj_acc_show_arg);

//prog_char help_traj_acc_show[] = "Show traj_acc values for trajectory manager";
parse_pgm_inst_t cmd_traj_acc_show = {
    .f = cmd_traj_acc_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_traj_acc_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_traj_acc_arg0,
        (prog_void *) & cmd_traj_acc_show_arg,
        NULL,
    },
};


/**********************************************************/
/* circle coef configuration */

/* this structure is filled when cmd_circle_coef is parsed successfully */
struct cmd_circle_coef_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float circle_coef;
};

/* function called when cmd_circle_coef is parsed successfully */
static void cmd_circle_coef_parsed(void *parsed_result, void *data)
{
    struct cmd_circle_coef_result *res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("set")))
    {
        trajectory_set_circle_coef(&mainboard.traj, res->circle_coef);
    }

    printf_P(PSTR("circle_coef set %2.2f\r\n"), mainboard.traj.circle_coef);
}

prog_char str_circle_coef_arg0[] = "circle_coef";
parse_pgm_token_string_t cmd_circle_coef_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_circle_coef_result, arg0, str_circle_coef_arg0);
prog_char str_circle_coef_arg1[] = "set";
parse_pgm_token_string_t cmd_circle_coef_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_circle_coef_result, arg1, str_circle_coef_arg1);
parse_pgm_token_num_t cmd_circle_coef_val = TOKEN_NUM_INITIALIZER(struct cmd_circle_coef_result, circle_coef, FLOAT);

//prog_char help_circle_coef[] = "Set circle coef";
parse_pgm_inst_t cmd_circle_coef = {
    .f = cmd_circle_coef_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_circle_coef,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_circle_coef_arg0,
        (prog_void *) & cmd_circle_coef_arg1,
        (prog_void *) & cmd_circle_coef_val,
        NULL,
    },
};

/* show */

prog_char str_circle_coef_show_arg[] = "show";
parse_pgm_token_string_t cmd_circle_coef_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_circle_coef_result, arg1, str_circle_coef_show_arg);

//prog_char help_circle_coef_show[] = "Show circle coef";
parse_pgm_inst_t cmd_circle_coef_show = {
    .f = cmd_circle_coef_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_circle_coef_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_circle_coef_arg0,
        (prog_void *) & cmd_circle_coef_show_arg,
        NULL,
    },
};
#endif /* TRAJECTORY_MANAGER_V3 */

/**********************************************************/
/* trajectory window configuration */

/* this structure is filled when cmd_trajectory is parsed successfully */
struct cmd_trajectory_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float d_win;
    float a_win;
    float a_start;
};

/* function called when cmd_trajectory is parsed successfully */
static void cmd_trajectory_parsed(void * parsed_result, void * data)
{
    struct cmd_trajectory_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("set")))
    {
        trajectory_set_windows(&mainboard.traj, res->d_win,
                               res->a_win, res->a_start);
    }

    printf_P(PSTR("trajectory %2.2f %2.2f %2.2f\r\n"), mainboard.traj.d_win,
             DEG(mainboard.traj.a_win_rad), DEG(mainboard.traj.a_start_rad));
}

prog_char str_trajectory_arg0[] = "trajectory";
parse_pgm_token_string_t cmd_trajectory_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg0, str_trajectory_arg0);
prog_char str_trajectory_arg1[] = "set";
parse_pgm_token_string_t cmd_trajectory_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg1, str_trajectory_arg1);
parse_pgm_token_num_t cmd_trajectory_d = TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, d_win, FLOAT);
parse_pgm_token_num_t cmd_trajectory_a = TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, a_win, FLOAT);
parse_pgm_token_num_t cmd_trajectory_as = TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, a_start, FLOAT);

//prog_char help_trajectory[] = "Set trajectory windows (distance, angle, angle_start)";
parse_pgm_inst_t cmd_trajectory = {
    .f = cmd_trajectory_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_trajectory,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_trajectory_arg0,
        (prog_void *) & cmd_trajectory_arg1,
        (prog_void *) & cmd_trajectory_d,
        (prog_void *) & cmd_trajectory_a,
        (prog_void *) & cmd_trajectory_as,
        NULL,
    },
};

/* show */

prog_char str_trajectory_show_arg[] = "show";
parse_pgm_token_string_t cmd_trajectory_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg1, str_trajectory_show_arg);

//prog_char help_trajectory_show[] = "Show trajectory window configuration";
parse_pgm_inst_t cmd_trajectory_show = {
    .f = cmd_trajectory_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_trajectory_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_trajectory_arg0,
        (prog_void *) & cmd_trajectory_show_arg,
        NULL,
    },
};



/**********************************************************/
/* rs_gains configuration */

/* this structure is filled when cmd_rs_gains is parsed successfully */
struct cmd_rs_gains_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float ed;
};

/* function called when cmd_rs_gains is parsed successfully */
static void cmd_rs_gains_parsed(void * parsed_result, void * data)
{
#ifdef HOST_VERSION
    printf("not implemented\n");
#else
    struct cmd_rs_gains_result * res = parsed_result;
    double cl = 0.0, cr = 0.0, ed = 0.0;

    if (!strcmp_P(res->arg1, PSTR("set")))
    {
        ed = res->ed;
        cl = (2.0 / (ed + 1.0));
        cr = (2.0 / ((1.0 / ed) + 1.0));

        /* increase gain to decrease dist, increase left and it will turn more left */
        rs_set_left_ext_encoder(&mainboard.rs, encoders_dspic_get_value,
                                LEFT_ENCODER, IMP_COEF * cl);
        rs_set_right_ext_encoder(&mainboard.rs, encoders_dspic_get_value,
                                 RIGHT_ENCODER, IMP_COEF * cr);
    }

    printf_P(PSTR("rs_gains set ed = %f, cr = %f, cl = %f\r\n"), ed, cr, cl);
#endif
}

prog_char str_rs_gains_arg0[] = "rs_gains";
parse_pgm_token_string_t cmd_rs_gains_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg0, str_rs_gains_arg0);
prog_char str_rs_gains_arg1[] = "set";
parse_pgm_token_string_t cmd_rs_gains_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg1, str_rs_gains_arg1);
parse_pgm_token_num_t cmd_rs_gains_ed = TOKEN_NUM_INITIALIZER(struct cmd_rs_gains_result, ed, FLOAT);

//prog_char help_rs_gains[] = "Set rs_gains (left, right)";
parse_pgm_inst_t cmd_rs_gains = {
    .f = cmd_rs_gains_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_rs_gains,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_rs_gains_arg0,
        (prog_void *) & cmd_rs_gains_arg1,
        (prog_void *) & cmd_rs_gains_ed,
        NULL,
    },
};

/* show */
prog_char str_rs_gains_show_arg[] = "show";
parse_pgm_token_string_t cmd_rs_gains_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg1, str_rs_gains_show_arg);

//prog_char help_rs_gains_show[] = "Show rs_gains";
parse_pgm_inst_t cmd_rs_gains_show = {
    .f = cmd_rs_gains_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_rs_gains_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_rs_gains_arg0,
        (prog_void *) & cmd_rs_gains_show_arg,
        NULL,
    },
};


/**********************************************************/
/* track configuration */

/* this structure is filled when cmd_track is parsed successfully */
struct cmd_track_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float val;
};

/* function called when cmd_track is parsed successfully */
static void cmd_track_parsed(void * parsed_result, void * data)
{
    struct cmd_track_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("set")))
    {
        position_set_physical_params(&mainboard.pos, res->val, DIST_IMP_MM);
    }
    printf_P(PSTR("track set %f\r\n"), mainboard.pos.phys.track_mm);
}

prog_char str_track_arg0[] = "track";
parse_pgm_token_string_t cmd_track_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg0, str_track_arg0);
prog_char str_track_arg1[] = "set";
parse_pgm_token_string_t cmd_track_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg1, str_track_arg1);
parse_pgm_token_num_t cmd_track_val = TOKEN_NUM_INITIALIZER(struct cmd_track_result, val, FLOAT);

//prog_char help_track[] = "Set track in mm";
parse_pgm_inst_t cmd_track = {
    .f = cmd_track_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_track,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_track_arg0,
        (prog_void *) & cmd_track_arg1,
        (prog_void *) & cmd_track_val,
        NULL,
    },
};

/* show */

prog_char str_track_show_arg[] = "show";
parse_pgm_token_string_t cmd_track_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg1, str_track_show_arg);

//prog_char help_track_show[] = "Show track";
parse_pgm_inst_t cmd_track_show = {
    .f = cmd_track_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_track_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_track_arg0,
        (prog_void *) & cmd_track_show_arg,
        NULL,
    },
};



/**********************************************************/
/* centrifugal configuration */

/* this structure is filled when cmd_centrifugal is parsed successfully */
struct cmd_centrifugal_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float val;
};

/* function called when cmd_centrifugal is parsed successfully */
static void cmd_centrifugal_parsed(void * parsed_result, void * data)
{
    struct cmd_centrifugal_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("set")))
    {
        position_set_centrifugal_coef(&mainboard.pos, res->val);
    }
    printf_P(PSTR("centrifugal set %f\r\n"), mainboard.pos.centrifugal_coef);
}

prog_char str_centrifugal_arg0[] = "centrifugal";
parse_pgm_token_string_t cmd_centrifugal_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_centrifugal_result, arg0, str_centrifugal_arg0);
prog_char str_centrifugal_arg1[] = "set";
parse_pgm_token_string_t cmd_centrifugal_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_centrifugal_result, arg1, str_centrifugal_arg1);
parse_pgm_token_num_t cmd_centrifugal_val = TOKEN_NUM_INITIALIZER(struct cmd_centrifugal_result, val, FLOAT);

//prog_char help_centrifugal[] = "Set centrifugal coef";
parse_pgm_inst_t cmd_centrifugal = {
    .f = cmd_centrifugal_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_centrifugal,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_centrifugal_arg0,
        (prog_void *) & cmd_centrifugal_arg1,
        (prog_void *) & cmd_centrifugal_val,
        NULL,
    },
};

/* show */

prog_char str_centrifugal_show_arg[] = "show";
parse_pgm_token_string_t cmd_centrifugal_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_centrifugal_result, arg1, str_centrifugal_show_arg);

//prog_char help_centrifugal_show[] = "Show centrifugal";
parse_pgm_inst_t cmd_centrifugal_show = {
    .f = cmd_centrifugal_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_centrifugal_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_centrifugal_arg0,
        (prog_void *) & cmd_centrifugal_show_arg,
        NULL,
    },
};


/**********************************************************/
/* Pt_Lists for testing traj */

#define PT_LIST_SIZE 10
static struct xy_point pt_list[PT_LIST_SIZE];
static uint16_t pt_list_len = 0;

/* this structure is filled when cmd_pt_list is parsed successfully */
struct cmd_pt_list_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    uint16_t arg2;
    int16_t arg3;
    int16_t arg4;
};

/* function called when cmd_pt_list is parsed successfully */
static void cmd_pt_list_parsed(void * parsed_result, void * data)
{
    struct cmd_pt_list_result * res = parsed_result;
    uint8_t i, why = 0;

    if (!strcmp_P(res->arg1, PSTR("append")))
    {
        res->arg2 = pt_list_len;
    }

    if (!strcmp_P(res->arg1, PSTR("insert")) ||
            !strcmp_P(res->arg1, PSTR("append")))
    {
        if (res->arg2 > pt_list_len)
        {
            printf_P(PSTR("Index too large\r\n"));
            return;
        }
        if (pt_list_len >= PT_LIST_SIZE)
        {
            printf_P(PSTR("List is too large\r\n"));
            return;
        }
        memmove(&pt_list[res->arg2 + 1], &pt_list[res->arg2],
                PT_LIST_SIZE - 1 - res->arg2);
        pt_list[res->arg2].x = res->arg3;
        pt_list[res->arg2].y = res->arg4;
        pt_list_len++;
    }
    else if (!strcmp_P(res->arg1, PSTR("del")))
    {
        if (pt_list_len <= 0)
        {
            printf_P(PSTR("Error: list empty\r\n"));
            return;
        }
        if (res->arg2 > pt_list_len)
        {
            printf_P(PSTR("Index too large\r\n"));
            return;
        }
        memmove(&pt_list[res->arg2], &pt_list[res->arg2 + 1],
                (PT_LIST_SIZE - 1 - res->arg2) * sizeof (struct xy_point));
        pt_list_len--;
    }
    else if (!strcmp_P(res->arg1, PSTR("reset")))
    {
        pt_list_len = 0;
    }

    /* else it is a "show" or a "start" */
    if (pt_list_len == 0)
    {
        printf_P(PSTR("List empty\r\n"));
        return;
    }
    for (i = 0; i < pt_list_len; i++)
    {
        printf_P(PSTR("%d: x=%d y=%d\r\n"), i, pt_list[i].x, pt_list[i].y);
        if (!strcmp_P(res->arg1, PSTR("start")))
        {
            trajectory_goto_xy_abs(&mainboard.traj, pt_list[i].x, pt_list[i].y);
            why = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER); /* all */
        }
        else if (!strcmp_P(res->arg1, PSTR("avoid_start")))
        {
            while (1)
            {
                why = goto_and_avoid(pt_list[i].x, pt_list[i].y, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR_NO_TIMER);
                printf("next point\r\n");
                if (why != END_OBSTACLE)
                    break;
            }
        }
        if (why & (~(END_TRAJ | END_NEAR)))
            trajectory_stop(&mainboard.traj);
    }
}

prog_char str_pt_list_arg0[] = "pt_list";
parse_pgm_token_string_t cmd_pt_list_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg0, str_pt_list_arg0);
prog_char str_pt_list_arg1[] = "insert";
parse_pgm_token_string_t cmd_pt_list_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_arg1);
parse_pgm_token_num_t cmd_pt_list_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg2, UINT16);
parse_pgm_token_num_t cmd_pt_list_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg3, INT16);
parse_pgm_token_num_t cmd_pt_list_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg4, INT16);

//prog_char help_pt_list[] = "Insert point in pt_list (idx,x,y)";
parse_pgm_inst_t cmd_pt_list = {
    .f = cmd_pt_list_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_pt_list,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_pt_list_arg0,
        (prog_void *) & cmd_pt_list_arg1,
        (prog_void *) & cmd_pt_list_arg2,
        (prog_void *) & cmd_pt_list_arg3,
        (prog_void *) & cmd_pt_list_arg4,
        NULL,
    },
};

/* append */

prog_char str_pt_list_arg1_append[] = "append";
parse_pgm_token_string_t cmd_pt_list_arg1_append = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_arg1_append);

//prog_char help_pt_list_append[] = "Append point in pt_list (x,y)";
parse_pgm_inst_t cmd_pt_list_append = {
    .f = cmd_pt_list_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_pt_list_append,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_pt_list_arg0,
        (prog_void *) & cmd_pt_list_arg1_append,
        (prog_void *) & cmd_pt_list_arg3,
        (prog_void *) & cmd_pt_list_arg4,
        NULL,
    },
};

/* del */

prog_char str_pt_list_del_arg[] = "del";
parse_pgm_token_string_t cmd_pt_list_del_arg = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_del_arg);

//prog_char help_pt_list_del[] = "Del or insert point in pt_list (num)";
parse_pgm_inst_t cmd_pt_list_del = {
    .f = cmd_pt_list_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_pt_list_del,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_pt_list_arg0,
        (prog_void *) & cmd_pt_list_del_arg,
        (prog_void *) & cmd_pt_list_arg2,
        NULL,
    },
};
/* show */

prog_char str_pt_list_show_arg[] = "show#reset#start#avoid_start";
parse_pgm_token_string_t cmd_pt_list_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_show_arg);

//prog_char help_pt_list_show[] = "Show, start or reset pt_list";
parse_pgm_inst_t cmd_pt_list_show = {
    .f = cmd_pt_list_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_pt_list_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_pt_list_arg0,
        (prog_void *) & cmd_pt_list_show_arg,
        NULL,
    },
};

#endif /* COMPILE_COMMANDS_TRAJ_OPTIONALS ------------------------------------*/


/**********************************************************/
/* Goto function */

/* this structure is filled when cmd_goto is parsed successfully */
struct cmd_goto_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    int32_t arg2;
    int32_t arg3;
    int32_t arg4;
};

/* function called when cmd_goto is parsed successfully */
static void cmd_goto_parsed(void * parsed_result, void * data)
{
    struct cmd_goto_result * res = parsed_result;
    uint8_t err;
    microseconds t1, t2;

    interrupt_traj_reset();

    if (!strcmp_P(res->arg1, PSTR("a_rel")))
    {
        trajectory_a_rel(&mainboard.traj, res->arg2);
    }
    else if (!strcmp_P(res->arg1, PSTR("d_rel")))
    {
        trajectory_d_rel(&mainboard.traj, res->arg2);
    }
    else if (!strcmp_P(res->arg1, PSTR("a_abs")))
    {
        trajectory_a_abs(&mainboard.traj, res->arg2);
    }
    else if (!strcmp_P(res->arg1, PSTR("a_to_xy")))
    {
        trajectory_turnto_xy(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("a_behind_xy")))
    {
        trajectory_turnto_xy_behind(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("xy_rel")))
    {
        trajectory_goto_xy_rel(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("xy_abs")))
    {
        trajectory_goto_xy_abs(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("avoid")))
    {
        err = goto_and_avoid(res->arg2, res->arg3, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
        if (err != END_TRAJ && err != END_NEAR)
            strat_hardstop();

		printf_P(PSTR("returned %s\r\n"), get_err(err));
		return;
    }
    else if (!strcmp_P(res->arg1, PSTR("avoid_fw")))
    {
        err = goto_and_avoid_forward(res->arg2, res->arg3, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
        if (err != END_TRAJ && err != END_NEAR)
            strat_hardstop();

		printf_P(PSTR("returned %s\r\n"), get_err(err));
		return;
    }
    else if (!strcmp_P(res->arg1, PSTR("avoid_bw")))
    {
        err = goto_and_avoid_backward(res->arg2, res->arg3, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
        if (err != END_TRAJ && err != END_NEAR)
            strat_hardstop();

		printf_P(PSTR("returned %s\r\n"), get_err(err));
		return;
    }
    else if (!strcmp_P(res->arg1, PSTR("xy_abs_fow")))
    {
        trajectory_goto_forward_xy_abs(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("xy_abs_back")))
    {
        trajectory_goto_backward_xy_abs(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("da_rel")))
    {
        trajectory_d_a_rel(&mainboard.traj, res->arg2, res->arg3);
    }


	t1 = time_get_us2();
	while ((err = test_traj_end(TRAJ_FLAGS_NO_NEAR)) == 0 || !cmdline_keypressed())
	{
	    t2 = time_get_us2();
	    if (t2 - t1 >= 10000)
	    {
	        dump_cs_debug("angle", &mainboard.angle.cs);
	        dump_cs_debug("distance", &mainboard.distance.cs);
	        t1 = t2;
	    }

		if (err!=0 && err != END_TRAJ && err != END_NEAR) {
			strat_hardstop();
			break;
		}
    }

	printf_P(PSTR("returned %s\r\n"), get_err(err));

}

prog_char str_goto_arg0[] = "goto";
parse_pgm_token_string_t cmd_goto_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg0, str_goto_arg0);
prog_char str_goto_arg1_a[] = "d_rel#a_rel#a_abs";
parse_pgm_token_string_t cmd_goto_arg1_a = TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg1, str_goto_arg1_a);
parse_pgm_token_num_t cmd_goto_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_goto_result, arg2, INT32);

/* 1 params */
//prog_char help_goto1[] = "Change orientation of the mainboard";
parse_pgm_inst_t cmd_goto1 = {
    .f = cmd_goto_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_goto1,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_goto_arg0,
        (prog_void *) & cmd_goto_arg1_a,
        (prog_void *) & cmd_goto_arg2,
        NULL,
    },
};

prog_char str_goto_arg1_b[] = "xy_rel#xy_abs#xy_abs_fow#xy_abs_back#da_rel#a_to_xy#avoid#avoid_fw#avoid_bw#a_behind_xy";
parse_pgm_token_string_t cmd_goto_arg1_b = TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg1, str_goto_arg1_b);
parse_pgm_token_num_t cmd_goto_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_goto_result, arg3, INT32);

/* 2 params */
//prog_char help_goto2[] = "Go to a (x,y) or (d,a) position";
parse_pgm_inst_t cmd_goto2 = {
    .f = cmd_goto_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_goto2,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_goto_arg0,
        (prog_void *) & cmd_goto_arg1_b,
        (prog_void *) & cmd_goto_arg2,
        (prog_void *) & cmd_goto_arg3,
        NULL,
    },
};

/**********************************************************/
/* Position tests */

/* this structure is filled when cmd_position is parsed successfully */
struct cmd_position_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    int32_t arg2;
    int32_t arg3;
    int32_t arg4;
};



void auto_position(void)
{
#define AUTOPOS_SPEED_FAST 	500
#define TRESPA_BAR		17
#define HOME_X_EDGE		70
//#define HOME_Y_UP_EDGE	1200

#define HOME_Y_UP_EDGE_YELLOW	1200
#define HOME_Y_UP_EDGE_GREEN	1200

    uint8_t err;
    uint16_t old_spdd, old_spda;

    /* save & set speeds */
    interrupt_traj_reset();
    strat_get_speed(&old_spdd, &old_spda);
    strat_set_speed(AUTOPOS_SPEED_FAST, AUTOPOS_SPEED_FAST);

	/* set position of the robot aligned to home edges */
	mainboard.our_color == I2C_COLOR_YELLOW?
	strat_reset_pos(COLOR_X(HOME_X_EDGE + ROBOT_CENTER_TO_BACK),
					COLOR_Y(HOME_Y_UP_EDGE_YELLOW - TRESPA_BAR - (ROBOT_WIDTH/2.0)),
					COLOR_A_ABS(0)):
	strat_reset_pos(COLOR_X(HOME_X_EDGE + ROBOT_CENTER_TO_BACK),
					COLOR_Y(HOME_Y_UP_EDGE_GREEN - TRESPA_BAR - (ROBOT_WIDTH/2.0)),
					COLOR_A_ABS(0));


	/* goto in line with the center cup */
	trajectory_d_rel(&mainboard.traj, 350-ROBOT_CENTER_TO_BACK);
    err = wait_traj_end(END_INTR | END_TRAJ);
    if (err == END_INTR)
        goto intr;

	trajectory_turnto_xy(&mainboard.traj, COLOR_X(MY_CUP_3_X), MY_CUP_3_Y);
    err = wait_traj_end(END_INTR | END_TRAJ);
    if (err == END_INTR)
        goto intr;


#if 0
    /* goto blocking to x axis */
    trajectory_d_rel(&mainboard.traj, -300);
    err = wait_traj_end(END_INTR | END_TRAJ | END_BLOCKING);
    if (err == END_INTR)
        goto intr;
    wait_ms(100);

    /* set x and angle */
    strat_reset_pos(COLOR_X(ROBOT_CENTER_TO_BACK), DO_NOT_SET_POS, COLOR_A_ABS(0));

    /* prepare to y axis */
    trajectory_d_rel(&mainboard.traj, 90);
    err = wait_traj_end(END_INTR | END_TRAJ);
    if (err == END_INTR)
        goto intr;

    trajectory_a_rel(&mainboard.traj, COLOR_A_REL(90));
    err = wait_traj_end(END_INTR | END_TRAJ);
    if (err == END_INTR)
        goto intr;

    /* goto blocking to y axis */
    trajectory_d_rel(&mainboard.traj, -800);
    err = wait_traj_end(END_INTR | END_TRAJ | END_BLOCKING);
    if (err == END_INTR)
        goto intr;
    wait_ms(100);

    /* set y */
    strat_reset_pos(DO_NOT_SET_POS, COLOR_Y(ROBOT_CENTER_TO_BACK), 90);
#endif
    /* restore speeds */
    strat_set_speed(old_spdd, old_spda);
    return;

intr:
    strat_hardstop();
    strat_set_speed(old_spdd, old_spda);
}

/* function called when cmd_position is parsed successfully */
static void cmd_position_parsed(void * parsed_result, void * data)
{
    struct cmd_position_result * res = parsed_result;

    /* display raw position values */
    if (!strcmp_P(res->arg1, PSTR("reset")))
    {
        position_set(&mainboard.pos, 0, 0, 0);
    }
    else if (!strcmp_P(res->arg1, PSTR("set")))
    {
        position_set(&mainboard.pos, res->arg2, res->arg3, res->arg4);
    }
    else if (!strcmp_P(res->arg1, PSTR("autoset_yellow")))
    {
        mainboard.our_color = I2C_COLOR_YELLOW;
        auto_position();
    }
    else if (!strcmp_P(res->arg1, PSTR("autoset_green")))
    {
        mainboard.our_color = I2C_COLOR_GREEN;
        auto_position();
    }

    /* else it's just a "show" */
    printf_P(PSTR("x=%.2f y=%.2f a=%.2f\r\n"),
             position_get_x_double(&mainboard.pos),
             position_get_y_double(&mainboard.pos),
             DEG(position_get_a_rad_double(&mainboard.pos)));
}

prog_char str_position_arg0[] = "position";
parse_pgm_token_string_t cmd_position_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg0, str_position_arg0);
prog_char str_position_arg1[] = "show#reset#autoset_yellow#autoset_green";
parse_pgm_token_string_t cmd_position_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg1, str_position_arg1);

//prog_char help_position[] = "Show/reset (x,y,a) position";
parse_pgm_inst_t cmd_position = {
    .f = cmd_position_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_position,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_position_arg0,
        (prog_void *) & cmd_position_arg1,
        NULL,
    },
};


prog_char str_position_arg1_set[] = "set";
parse_pgm_token_string_t cmd_position_arg1_set = TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg1, str_position_arg1_set);
parse_pgm_token_num_t cmd_position_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg2, INT32);
parse_pgm_token_num_t cmd_position_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg3, INT32);
parse_pgm_token_num_t cmd_position_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg4, INT32);

//prog_char help_position_set[] = "Set (x,y,a) position";
parse_pgm_inst_t cmd_position_set = {
    .f = cmd_position_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_position_set,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_position_arg0,
        (prog_void *) & cmd_position_arg1_set,
        (prog_void *) & cmd_position_arg2,
        (prog_void *) & cmd_position_arg3,
        (prog_void *) & cmd_position_arg4,
        NULL,
    },
};



/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_infos is parsed successfully */
struct cmd_strat_infos_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

/* function called when cmd_strat_infos is parsed successfully */
static void cmd_strat_infos_parsed(void *parsed_result, void *data)
{
    struct cmd_strat_infos_result *res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("reset")))
    {
        strat_reset_infos();
    }

    strat_infos.dump_enabled = 1;
    strat_dump_infos(__FUNCTION__);
}

prog_char str_strat_infos_arg0[] = "strat_infos";
parse_pgm_token_string_t cmd_strat_infos_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_infos_result, arg0, str_strat_infos_arg0);
prog_char str_strat_infos_arg1[] = "show#reset";
parse_pgm_token_string_t cmd_strat_infos_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_infos_result, arg1, str_strat_infos_arg1);

//prog_char help_strat_infos[] = "reset/show strat infos";
parse_pgm_inst_t cmd_strat_infos = {
    .f = cmd_strat_infos_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_strat_infos,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_strat_infos_arg0,
        (prog_void *) & cmd_strat_infos_arg1,
        NULL,
    },
};

/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_conf is parsed successfully */
struct cmd_strat_conf_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

/* function called when cmd_strat_conf is parsed successfully */
static void cmd_strat_conf_parsed(void *parsed_result, void *data)
{
    struct cmd_strat_conf_result *res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("base")))
    {
	   strat_infos.match_strategy=STR_BASE;
    }
    if (!strcmp_P(res->arg1, PSTR("qualification")))
    {
	   strat_infos.match_strategy=STR_QUALIFICATION;
    }
    if (!strcmp_P(res->arg1, PSTR("homologation")))
    {
	   strat_infos.match_strategy=STR_HOMOLOGATION;

    }
    else if (!strcmp_P(res->arg1, PSTR("do_tower")))
		strat_infos.conf.flags |= CONF_FLAG_DO_TOWER;

    else if (!strcmp_P(res->arg1, PSTR("do_fast_g1")))
		strat_infos.conf.flags |= CONF_FLAG_DO_STAND_FAST_GROUP_1;
	
    else if (!strcmp_P(res->arg1, PSTR("do_escape")))
		strat_infos.conf.flags |= CONF_FLAG_DO_ESCAPE_UPPER_ZONE;


	/* flags */
	strat_smart[MAIN_ROBOT].current_strategy=0;
	strat_smart[SEC_ROBOT].current_strategy=0;
	strat_set_next_sec_strategy();
	strat_set_next_main_strategy();

	strat_infos.dump_enabled = 1;
	strat_dump_conf();
}

prog_char str_strat_conf_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf_result, arg0, str_strat_conf_arg0);
prog_char str_strat_conf_arg1[] = "show#base#homologation#qualification#do_tower#do_fast_g1#do_escape";
parse_pgm_token_string_t cmd_strat_conf_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf_result, arg1, str_strat_conf_arg1);

//prog_char help_strat_conf[] = "configure specific strat for a match";
parse_pgm_inst_t cmd_strat_conf = {
    .f = cmd_strat_conf_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_strat_conf,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_strat_conf_arg0,
        (prog_void *) & cmd_strat_conf_arg1,
        NULL,
    },
};

#ifdef COMPILE_COMMANDS_TRAJ_OPTIONALS

/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_conf2 is parsed successfully */
struct cmd_strat_conf2_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    fixed_string_t arg2;
};

/* function called when cmd_strat_conf2 is parsed successfully */
static void cmd_strat_conf2_parsed(void *parsed_result, void *data)
{
    struct cmd_strat_conf2_result *res = parsed_result;
    uint8_t on, bit = 0;

    if (!strcmp_P(res->arg2, PSTR("on")))
        on = 1;
    else
        on = 0;

    /* contruct here the bit mask */

    if (on)
        strat_infos.conf.flags |= bit;
    else
        strat_infos.conf.flags &= (~bit);

    strat_infos.dump_enabled = 1;
    strat_dump_conf();
}

prog_char str_strat_conf2_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg0, str_strat_conf2_arg0);
prog_char str_strat_conf2_arg1[] = "opp_tracking";
parse_pgm_token_string_t cmd_strat_conf2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg1, str_strat_conf2_arg1);
prog_char str_strat_conf2_arg2[] = "on#off";
parse_pgm_token_string_t cmd_strat_conf2_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg2, str_strat_conf2_arg2);


//prog_char help_strat_conf2[] = "enable/disable strat option";
parse_pgm_inst_t cmd_strat_conf2 = {
    .f = cmd_strat_conf2_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_strat_conf2,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_strat_conf2_arg0,
        (prog_void *) & cmd_strat_conf2_arg1,
        (prog_void *) & cmd_strat_conf2_arg2,
        NULL,
    },
};


/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_conf3 is parsed successfully */
struct cmd_strat_conf3_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    uint8_t arg2;
};

/* function called when cmd_strat_conf3 is parsed successfully */
static void cmd_strat_conf3_parsed(void *parsed_result, void *data)
{
    struct cmd_strat_conf3_result *res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("something")))
    {
        /* set value of a strat parameter */
			printf ("not implemented");
    }

    strat_infos.dump_enabled = 1;
    strat_dump_conf();
}

prog_char str_strat_conf3_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf3_result, arg0, str_strat_conf3_arg0);
prog_char str_strat_conf3_arg1[] = "something";
parse_pgm_token_string_t cmd_strat_conf3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf3_result, arg1, str_strat_conf3_arg1);
parse_pgm_token_num_t cmd_strat_conf3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_strat_conf3_result, arg2, UINT16);

//prog_char help_strat_conf3[] = "set strat param value";
parse_pgm_inst_t cmd_strat_conf3 = {
    .f = cmd_strat_conf3_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_strat_conf3,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_strat_conf3_arg0,
        (prog_void *) & cmd_strat_conf3_arg1,
        (prog_void *) & cmd_strat_conf3_arg2,
        NULL,
    },
};

#endif /* COMPILE_COMMANDS_TRAJ_OPTIONALS */

/**********************************************************/
/* Subtraj 1 */

/* this structure is filled when cmd_subtraj1 is parsed successfully */
struct cmd_subtraj1_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    int32_t arg2;
    //int32_t arg3;
};

/* function called when cmd_subtraj1 is parsed successfully */
static void cmd_subtraj1_parsed(void *parsed_result, void *data)
{
    struct cmd_subtraj1_result *res = parsed_result;
    uint8_t err = 0;
    uint8_t zone_num = ZONES_MAX;

	strat_infos.debug_step = 1;

   	/**
	 *	strat several zones
	 */
    if (strcmp_P(res->arg1, PSTR("begining")) == 0) {
        //err = strat_begining();
		printf_P(PSTR("subtraj returned %s\r\n"), get_err(err));
    }

	/**
     * 	go & work on zone
	 */

	/* go and work on zone */
   	else if (!strcmp_P(res->arg1, PSTR("home_spotlight"))) {
	   	zone_num = ZONE_MY_HOME_SPOTLIGHT;
		time_reset();
	}

   	else if (!strcmp_P(res->arg1, PSTR("home_timeout"))) {
        zone_num = ZONE_MY_HOME_POPCORNS;
		time_set(STAND_RELEASE_TIME-1, 0);
	}

   	else if (!strcmp_P(res->arg1, PSTR("home_popcorns"))) {
        zone_num = ZONE_MY_HOME_POPCORNS;
		time_reset();
	}

   	else if (!strcmp_P(res->arg1, PSTR("machine")))
        zone_num = ZONE_MY_POPCORNMAC;

   	else if (!strcmp_P(res->arg1, PSTR("machine_opp")))
        zone_num = ZONE_OPP_POPCORNMAC;

    else if (!strcmp_P(res->arg1, PSTR("cinema_up"))) {
        zone_num = ZONE_MY_CINEMA_UP;
	}
    else if (!strcmp_P(res->arg1, PSTR("cinema_down_sec"))) {
        zone_num = ZONE_MY_CINEMA_DOWN_SEC;
	}
    else if (!strcmp_P(res->arg1, PSTR("cinema_down_main"))) {
        zone_num = ZONE_MY_CINEMA_DOWN_MAIN;
	}
    else if (!strcmp_P(res->arg1, PSTR("stairs_ways"))) {
        zone_num = ZONE_MY_STAIRWAY;
	}
    else if (!strcmp_P(res->arg1, PSTR("outside"))) {
        zone_num = ZONE_MY_HOME_OUTSIDE;
	}
    else if (!strcmp_P(res->arg1, PSTR("platform"))) {
        zone_num = ZONE_MY_PLATFORM;
	}
    else if (!strcmp_P(res->arg1, PSTR("stairs"))) {
        zone_num = ZONE_MY_STAIRS;
	}


	/* go and work */
    if (zone_num < ZONES_MAX)
	{
		if (strat_infos.zones[zone_num].robot==MAIN_ROBOT)
		{
            /* goto */
            err = strat_goto_zone(MAIN_ROBOT, zone_num);
		    printf_P(PSTR("goto returned %s\r\n"), get_err(err));
			if (!TRAJ_SUCCESS(err))
			   ERROUT(err);

       		/* work */
            err = strat_work_on_zone(MAIN_ROBOT, zone_num);
            printf_P(PSTR("work returned %s\r\n"), get_err(err));
		}
		else {
            /* goto */
            err = strat_goto_zone(SEC_ROBOT, zone_num);
			if (bt_robot_2nd_wait_ack()!=0) {
				printf_P(PSTR("bt cmd ERROR\n\r"));
				ERROUT(END_ERROR);
			}
			err = bt_robot_2nd_wait_end();
		    printf_P(PSTR("goto returned %s\r\n"), get_err(err));
			if (!TRAJ_SUCCESS(err))
			   ERROUT(err);

            /* XXX debug */
            //strat_debug_wait_key_pressed (SEC_ROBOT);

            /* work */
            err = strat_work_on_zone(SEC_ROBOT, zone_num);
			if (bt_robot_2nd_wait_ack()!=0) {
				printf_P(PSTR("bt cmd ERROR\n\r"));
				ERROUT(END_ERROR);
			}
			err = bt_robot_2nd_wait_end();
		    printf_P(PSTR("work returned %s\r\n"), get_err(err));
			if (!TRAJ_SUCCESS(err))
			   ERROUT(err);
		}

    }
end:
    trajectory_hardstop(&mainboard.traj);
}

prog_char str_subtraj1_arg0[] = "subtraj";
parse_pgm_token_string_t cmd_subtraj1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_subtraj1_result, arg0, str_subtraj1_arg0);
prog_char str_subtraj1_arg1[] = "begining#home_popcorns#home_spotlight#home_timeout#machine#machine_opp#cinema_up#cinema_down_sec#cinema_down_main#platform#stairs#stairs_ways#outside";
parse_pgm_token_string_t cmd_subtraj1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_subtraj1_result, arg1, str_subtraj1_arg1);
//parse_pgm_token_num_t cmd_subtraj1_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj1_result, arg2, INT32);
//parse_pgm_token_num_t cmd_subtraj1_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj1_result, arg3, INT32);
//prog_char help_subtraj1[] = "Test sub-trajectories"; //" (a,b,c: specific params)";
parse_pgm_inst_t cmd_subtraj1 = {
    .f = cmd_subtraj1_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_subtraj1,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) &cmd_subtraj1_arg0,
        (prog_void *) &cmd_subtraj1_arg1,
        //(prog_void *) &cmd_subtraj1_arg2,
        //(prog_void *)&cmd_subtraj1_arg3,
        NULL,
    },
};


/**********************************************************/
/* Subtraj 2 */

/* this structure is filled when cmd_subtraj2 is parsed successfully */
struct cmd_subtraj2_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    uint8_t arg2;
    //	int32_t arg2;
    //	int32_t arg3;
    //	int32_t arg4;
    //	int32_t arg5;
};

/* function called when cmd_subtraj2 is parsed successfully */
static void cmd_subtraj2_parsed(void *parsed_result, void *data)
{

    struct cmd_subtraj2_result *res = parsed_result;
    uint8_t err = 0;
    uint8_t zone_num = ZONES_MAX;

	strat_infos.debug_step = 1;

	/**
	 *	strat smart
	 */
    if (!strcmp_P(res->arg1, PSTR("strat_ptinto"))) {


		/* play */
		strat_secondary_robot_disable ();
		err = strat_smart_main_robot();
		printf_P(PSTR("strat smart returned %s\r\n"), get_err(err));
		ERROUT(err);
    }
    else if (!strcmp_P(res->arg1, PSTR("strat_tirantes"))) {


		/* play */
		strat_secondary_robot_disable ();
		do {
			time_wait_ms(200);
			err = strat_smart_secondary_robot();
		} while (!err);
		printf_P(PSTR("strat smart returned %s\r\n"), get_err(err));
		ERROUT(err);
    }

	/**
     * 	go & work on zone
	 */

	/* get zone number */
	if (res->arg2 == 0) {
		zone_num = ZONES_MAX;
	}
    else if (!strcmp_P(res->arg1, PSTR("stand_group"))) {
		if (res->arg2 <= 4)
        	zone_num = ZONE_MY_STAND_GROUP_1 + res->arg2 - 1;
	}
    else if (!strcmp_P(res->arg1, PSTR("clapper"))) {
		if (res->arg2 <= 3)
        	zone_num = ZONE_MY_CLAP_1 + res->arg2 - 1;
	}
  	else if (!strcmp_P(res->arg1, PSTR("cup"))) {

		if (res->arg2 <= 3)
        	zone_num = ZONE_POPCORNCUP_1 + res->arg2 - 1;
	}

	/* go and work*/
    if (zone_num < ZONES_MAX) {

		if (strat_infos.zones[zone_num].robot==MAIN_ROBOT) {

            /* goto */
            err = strat_goto_zone(MAIN_ROBOT, zone_num);
		    printf_P(PSTR("goto returned %s\r\n"), get_err(err));
			if (!TRAJ_SUCCESS(err))
			   ERROUT(err);

       		/* work */
            err = strat_work_on_zone(MAIN_ROBOT, zone_num);
            printf_P(PSTR("work returned %s\r\n"), get_err(err));
		}
		else {
            /* goto */
            err = strat_goto_zone(SEC_ROBOT, zone_num);
			if (bt_robot_2nd_wait_ack()!=0) {
				printf_P(PSTR("bt cmd ERROR\n\r"));
				ERROUT(END_ERROR);
			}
			err = bt_robot_2nd_wait_end();
		    printf_P(PSTR("goto returned %s\r\n"), get_err(err));
			if (!TRAJ_SUCCESS(err))
			   ERROUT(err);

            /* XXX debug */
            //strat_debug_wait_key_pressed (SEC_ROBOT);

            /* work */
            err = strat_work_on_zone(SEC_ROBOT, zone_num);
			if (bt_robot_2nd_wait_ack()!=0) {
				printf_P(PSTR("bt cmd ERROR\n\r"));
				ERROUT(END_ERROR);
			}
			err = bt_robot_2nd_wait_end();
		    printf_P(PSTR("work returned %s\r\n"), get_err(err));
			if (!TRAJ_SUCCESS(err))
			   ERROUT(err);
		}
    }
	else
		printf_P(PSTR("invalid element number\r\n"));

end:
    trajectory_hardstop(&mainboard.traj);
}

prog_char str_subtraj2_arg0[] = "subtraj";
parse_pgm_token_string_t cmd_subtraj2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_subtraj2_result, arg0, str_subtraj2_arg0);
prog_char str_subtraj2_arg1[] = "strat_ptinto#strat_tirantes#stand_group#clapper#cup";
parse_pgm_token_string_t cmd_subtraj2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_subtraj2_result, arg1, str_subtraj2_arg1);
parse_pgm_token_num_t cmd_subtraj2_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj2_result, arg2, UINT8);
//parse_pgm_token_num_t cmd_subtraj2_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj2_result, arg3, INT32);
//parse_pgm_token_num_t cmd_subtraj2_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj2_result, arg4, INT32);
//parse_pgm_token_num_t cmd_subtraj2_arg5 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj2_result, arg5, INT32);

//prog_char help_subtraj2[] = "Test sub-trajectories (a,b,c,d: specific params)";
parse_pgm_inst_t cmd_subtraj2 = {
    .f = cmd_subtraj2_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_subtraj2,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) &cmd_subtraj2_arg0,
        (prog_void *) &cmd_subtraj2_arg1,
		(prog_void *) &cmd_subtraj2_arg2,
        //		(prog_void *)&cmd_subtraj2_arg3,
        //		(prog_void *)&cmd_subtraj2_arg4,
        //		(prog_void *)&cmd_subtraj2_arg5,
        NULL,
    },
};
