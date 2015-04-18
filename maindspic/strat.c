/*
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2012)
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
 *  Javier Balinas Santos <javier@arc-robots.org> and Silvia Santano
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>
#include <clock_time.h>

#ifndef HOST_VERSION_OA_TEST
#include <uart.h>
#include <dac_mc.h>
#include <pwm_servo.h>

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

#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "bt_protocol.h"

#else

/**
   XXX
   strat.c is used for OA test for both robots.
   We need define some robot dimensions here, depending on robot
 */

#ifndef IM_SECONDARY_ROBOT

/* XXX keep synchronized with maindspic/main.h */

/* robot dimensions */
#define ROBOT_LENGTH            288.5
#define ROBOT_WIDTH             330.0
#define ROBOT_CENTER_TO_FRONT   167.0
#define ROBOT_CENTER_TO_BACK    121.5
#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/* XXX obstacle clerance */
#define OBS_CLERANCE            (235.+10.)

#else

/* XXX keep synchronized with secondary_robot/main.h */

/* robot dimensions */
#define ROBOT_LENGTH      	    163.
#define ROBOT_WIDTH 	    	210.
#define ROBOT_CENTER_TO_BACK    105.0
#define ROBOT_CENTER_TO_FRONT   (ROBOT_LENGTH-ROBOT_CENTER_TO_BACK)
#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/* XXX obstacle clearance */
#define OBS_CLERANCE            (149.+10.)

#endif /* ! IM_SECONDARY_ROBOT */


#endif

#define LIMIT_BBOX_Y_UP			(2000 - OBS_CLERANCE-70)
#define LIMIT_BBOX_Y_DOWN		OBS_CLERANCE+100
#define LIMIT_BBOX_X_UP			3000 - OBS_CLERANCE
#define LIMIT_BBOX_X_DOWN		OBS_CLERANCE

struct strat_infos strat_infos = {
    /* conf */
    .conf = {
        .flags = 0,
    },

	/* stands */
    .zones[ZONE_MY_STAND_GROUP_1] =
	{
		/* type */
		ZONE_TYPE_STAND,
		/* target: x,y, */
		MY_STAND_4_X, MY_STAND_4_Y,
		/* boundinbox: x_down, x_up,  y_down,  y_up, */
		700, 1400, 150, 700,
		/* init_x, init_y */
	  	0, 0,
		/* priority, flags */
		0, 0,
		/* statistics: opp_time_zone_us, last_time_opp_here */
//		0, (9000*1000L),
		/* robot in charge */
		MAIN_ROBOT
	},
    .zones[ZONE_MY_STAND_GROUP_2] =
	{
		ZONE_TYPE_STAND,
		MY_STAND_7_X, MY_STAND_7_Y,
		0, 100, 0, 300,
		0, 0, //MY_CUP_2_X+OBS_CLERANCE, MY_CUP_2_Y,
		0, 0,
//		0, (9000*1000L),
		MAIN_ROBOT
	},
    .zones[ZONE_MY_STAND_GROUP_3] =
	{
		ZONE_TYPE_STAND,
		MY_STAND_2_X+50, MY_STAND_2_Y,              /* XXX: aprox. to be tested */
		650, 950, 1700, 1950,
		MY_STAND_2_X-OBS_CLERANCE, AREA_Y-330, /* TODO: xy closed to stands */ 	
		0, 0,
//		0, (9000*1000L),
		MAIN_ROBOT
	},
    .zones[ZONE_MY_STAND_GROUP_4] =
	{
		ZONE_TYPE_STAND,
		MY_STAND_1_X, MY_STAND_1_Y,
		0, 150, 0, 300,
		MY_STAND_1_X+OBS_CLERANCE, AREA_Y-330, /* TODO: xy closed to stands */ 
		0, 0,
//		0, (9000*1000L),
		MAIN_ROBOT
	},
	/* popcorn machines */
	.zones[ZONE_MY_POPCORNMAC] =
	{
		ZONE_TYPE_POPCORNMAC,
		MY_POPCORNMAC_X, MY_POPCORNMAC_Y,
		150, 750, 1700, 2000,
		MY_POPCORNMAC_X, AREA_Y-330,
		0, 0,
//		0, (9000*1000L),
		MAIN_ROBOT
	},
	/* XXX AVOID for the moment */
	.zones[ZONE_OPP_POPCORNMAC] =
	{
		ZONE_TYPE_POPCORNMAC,
		OPP_POPCORNMAC_X, OPP_POPCORNMAC_Y,
		2250, 2850, 1700, 2000,
		OPP_POPCORNMAC_X, AREA_Y-330,
		0, ZONE_AVOID,
//		0, (9000*1000L),
		MAIN_ROBOT
	},
	/* popcorn cups */
	.zones[ZONE_POPCORNCUP_1] =
	{
		ZONE_TYPE_POPCORNCUP,
		MY_CUP_1_X, MY_CUP_1_Y,
		760, 1060, 1020, 1320,
		MY_CUP_1_X-ROBOT_SEC_OBS_CLERANCE,	MY_CUP_1_Y-ROBOT_SEC_OBS_CLERANCE,
		0, 0,
//		0, (9000*1000L),
		SEC_ROBOT
	},
	.zones[ZONE_POPCORNCUP_2] =
	{
		ZONE_TYPE_POPCORNCUP,
		MY_CUP_2_X, MY_CUP_2_Y,
		100, 400, 100, 400,
		MY_CUP_2_X+OBS_CLERANCE+30+80, MY_CUP_2_Y,
		0, 0,
//		0, (9000*1000L),
		MAIN_ROBOT
	},
	.zones[ZONE_POPCORNCUP_3] =
	{
		ZONE_TYPE_POPCORNCUP,
	 	MY_CUP_3_X, MY_CUP_3_Y,
		1350, 1650, 200, 500,
		MY_CUP_3_X-OBS_CLERANCE-50, MY_CUP_3_Y,
		0, 0,
//		0, (9000*1000L),
		MAIN_ROBOT
	},
	/* cinemas */
 	.zones[ZONE_MY_CINEMA_UP] =
	{
		ZONE_TYPE_CINEMA,
		MY_CINEMA_UP_X, MY_CINEMA_UP_Y,
		2600, 3000, 1200, 1600,
		AREA_X-400-ROBOT_SEC_OBS_CLERANCE, MY_CINEMA_UP_Y,
		0, 0,
//	 	0, (9000*1000L),
		SEC_ROBOT
	},
	.zones[ZONE_MY_CINEMA_DOWN] =
	{
		ZONE_TYPE_CINEMA,
		MY_CINEMA_DOWN_X, MY_CINEMA_DOWN_Y,
		2600, 3000, 400, 800,
		AREA_X-400-ROBOT_SEC_OBS_CLERANCE, MY_CINEMA_DOWN_Y,
		0, 0,
//		0, (9000*1000L),
		SEC_ROBOT
	},
	/* stairs */
  	.zones[ZONE_MY_STAIRS] =
	{
		ZONE_TYPE_STAIRS,
		MY_STAIRS_X, MY_STAIRS_Y,
		1000, 1500, 1400, 2000,
		MY_STAIRS_X, 1150,
		0, 0,
//		0, (9000*1000L),
		SEC_ROBOT
	},
	/* stair ways */
   	.zones[ZONE_MY_STAIRWAY] =
	{
		ZONE_TYPE_STAIRWAY,
		MY_STAIRS_X, MY_STAIRS_Y,
		1000, 1500, 1400, 2000,
        MY_STAIRS_X, 1150,
		0, 0,
//		0, (9000*1000L),
		SEC_ROBOT
	},
	/* clapper boards */
   	.zones[ZONE_MY_CLAP_1] =
	{
		ZONE_TYPE_CLAP,
		MY_CLAP_1_X,  MY_CLAP_Y,
		180, 480, 0, 300,
		MY_CLAP_1_X, MY_CUP_2_Y,
		0, 0,
//		0, (9000*1000L),
		MAIN_ROBOT
	},
    .zones[ZONE_MY_CLAP_2] =
	{
		ZONE_TYPE_CLAP,
		MY_CLAP_2_X, MY_CLAP_Y,
		780, 1080, 0, 300,
		MY_CLAP_2_X, MY_CUP_2_Y,
		0, 0,
//		0, (9000*1000L),
		MAIN_ROBOT
	},
    .zones[ZONE_MY_CLAP_3] =
	{
		ZONE_TYPE_CLAP,
		MY_CLAP_3_X,  MY_CLAP_Y,
		2230, 2530, 0, 300,
		MY_CLAP_3_X, MY_CUP_2_Y,
		0, 0,
//		0, (9000*1000L),
		SEC_ROBOT
	},
	/* home */
  	.zones[ZONE_MY_HOME] =
	{
		ZONE_TYPE_HOME,
		MY_HOME_SPOTLIGHT_X, MY_HOME_SPOTLIGHT_Y,
		90, 650, 800, 1200,
		670, MY_HOME_SPOTLIGHT_Y,
		0, 0,
//		0, (9000*1000L),
		MAIN_ROBOT
	},
	/* home */
  	.zones[ZONE_MY_HOME_OUTSIDE] =
	{
		ZONE_TYPE_STRAT,
		0, 0,
		90, 650, 800, 1200,	 /* not matter xy init is 0 */
		402+ROBOT_SEC_OBS_CLERANCE, 800+(ROBOT_SEC_WIDTH/2),
		0, 0,
//		0, (9000*1000L),
		SEC_ROBOT
	},
};

struct strat_smart strat_smart[ROBOT_MAX];

/* points we get from each zone */
/* TODO: implement a function to access to array. XXX Range must be assserted
uint8_t strat_zones_points[ZONES_MAX];/
*/

/*************************************************************/

/*                  INIT                                     */

/*************************************************************/
#define BOUNDINBOX_INCLUDES_PLAFORM 0
#define BOUNDINBOX_WITHOUT_PLATFORM 1

void strat_set_bounding_box(uint8_t which)
{

    strat_infos.area_bbox.x1 = OBS_CLERANCE;
    strat_infos.area_bbox.x2 = 3000 - OBS_CLERANCE;

	if (which == BOUNDINBOX_INCLUDES_PLAFORM)
	    strat_infos.area_bbox.y1 = OBS_CLERANCE+100;
	else
	    strat_infos.area_bbox.y1 = OBS_CLERANCE;

    strat_infos.area_bbox.y2 = 2000 - OBS_CLERANCE-70;

    polygon_set_boundingbox(strat_infos.area_bbox.x1,
                strat_infos.area_bbox.y1,
                strat_infos.area_bbox.x2,
                strat_infos.area_bbox.y2);
#ifdef HOST_VERSION_OA_TEST

  printf("boundingbox at: %d %d %d %d\n",
        strat_infos.area_bbox.x1,
                strat_infos.area_bbox.y1,
                strat_infos.area_bbox.x2,
                strat_infos.area_bbox.y2);
#endif
}

#ifndef HOST_VERSION_OA_TEST


/* zone names */
const char zone_name[ZONES_MAX][14]= {
    [ZONE_MY_STAND_GROUP_1]="STAND GROUP 1\0",
    [ZONE_MY_STAND_GROUP_2]="STAND GROUP 2\0",
    [ZONE_MY_STAND_GROUP_3]="STAND GROUP 3\0",
    [ZONE_MY_STAND_GROUP_4]="STAND GROUP 4\0",
    [ZONE_MY_POPCORNMAC]="MACHINE\0",
    [ZONE_OPP_POPCORNMAC]="OPP MACHINE\0",
    [ZONE_POPCORNCUP_1]="CUP 1\0",
    [ZONE_POPCORNCUP_2]="CUP 2\0",
    [ZONE_POPCORNCUP_3]="CUP 3\0",
    [ZONE_MY_CINEMA_UP]="CINEMA UP\0",
    [ZONE_MY_CINEMA_DOWN]="CINEMA DOWN\0",
    [ZONE_MY_STAIRS]="STAIRS\0",
    [ZONE_MY_HOME]="HOME\0",
    [ZONE_MY_CLAP_1]="CLAPPER 1\0",
    [ZONE_MY_CLAP_2]="CLAPPER 2\0",
    [ZONE_MY_CLAP_3]="CLAPPER 3\0",
    [ZONE_MY_STAIRWAY]="STAIRWAY\0",
    [ZONE_MY_HOME_OUTSIDE]="OUTSIDE\0",
};

/* return string with the zone name */
const char *get_zone_name(uint8_t zone_num)
{
    if (zone_num >= ZONES_MAX)
    	return "END_UNKNOWN";

    return &zone_name[zone_num][0];
}

/* display curret strat configuration */
void strat_dump_conf(void)
{
    if (!strat_infos.dump_enabled)
        return;

    printf_P(PSTR("-- conf --\r\n"));

    /* add here configuration dump */
}


/* display current information about the state of the game */
void strat_dump_infos(const char *caller)
{
	//int8_t zone_opp;
    if (!strat_infos.dump_enabled)
        return;

    printf(PSTR("%s() dump strat infos:\r\n"), caller);

    /* add here print infos */
	printf_P("%d %d\n", opponent1_is_infront(),opponent2_is_infront());

}

/* init current area state before a match. Dump update user conf
 * here */
void strat_reset_infos(void)
{
    /* bounding box */
    strat_set_bounding_box(BOUNDINBOX_INCLUDES_PLAFORM);
    strat_smart[MAIN_ROBOT].current_zone = ZONES_MAX;
    strat_smart[MAIN_ROBOT].goto_zone = ZONES_MAX;
    strat_smart[MAIN_ROBOT].last_zone = ZONES_MAX;

    strat_smart[SEC_ROBOT].current_zone = ZONES_MAX;
    strat_smart[SEC_ROBOT].goto_zone = ZONES_MAX;
    strat_smart[SEC_ROBOT].last_zone = ZONES_MAX;

    /* add here other infos resets */
}

/* called before each strat, and before the start switch */
void strat_preinit(void)
{
    time_reset();
    interrupt_traj_reset();
    mainboard.flags |=  DO_ENCODERS | DO_CS | DO_RS | DO_POS | DO_BD | DO_POWER;

    strat_dump_conf();
    strat_dump_infos(__FUNCTION__);
}

/* called it just before launching the strat */
void strat_init(void)
{
    strat_reset_infos();

    /* we consider that the color is correctly set */

    strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
    time_reset();
    interrupt_traj_reset();

    /* used in strat_base for END_TIMER */
	if (!strat_infos.debug_step)
    	mainboard.flags |=  DO_ENCODERS | DO_CS | DO_RS | DO_POS | DO_BD | DO_POWER | DO_TIMER;
	else
    	mainboard.flags |=  DO_ENCODERS | DO_CS | DO_RS | DO_POS | DO_BD | DO_POWER;
}


/* call it after each strat */
void strat_exit(void)
{
    uint8_t flags;

    /* stop robot, disable timer */
    mainboard.flags &= ~(DO_TIMER);
    strat_hardstop();
    time_reset();

    /* disable CS, and motors */
#ifndef HOST_VERSION
    IRQ_LOCK(flags);
    mainboard.flags &= ~(DO_CS);
    IRQ_UNLOCK(flags);
    dac_mc_set(LEFT_MOTOR, 0);
    dac_mc_set(RIGHT_MOTOR, 0);
#endif

    /* TODO slavespic exit */

    /* stop beacon */
    bt_beacon_set_off();
	
	strat_infos.match_ends = 1;
}

/* called periodically */
void strat_event(void *dummy) {

	/* stop beacon */
	if (strat_infos.match_ends) {
	    bt_beacon_set_off();
	}

    /* limit speed when opponent are close */
    strat_limit_speed();

    /* smart strategy of secondary robot */
	if (strat_secondary_robot_is_enabled())
		strat_smart_secondary_robot();
}


/* dump state (every 5 s max) XXX */
#define DUMP_RATE_LIMIT(dump, last_print)        \
    do {                        \
        if (time_get_s() - last_print > 5) {    \
            dump();                \
            last_print = time_get_s();    \
        }                    \
    } while (0)


#define ERROUT(e) do {\
        err = e;             \
        goto end;         \
    } while(0)


/* Strat main loop */
uint8_t strat_main(void)
{
    uint8_t err;
    strat_limit_speed_enable ();

	/* init time for secondary robot */
	//bt_robot_2nd_start_matchtimer ();

	/* XXX enable smart_trat of secondary robot */
	//strat_secondary_robot_enable ();

    /* play */
    do{
        err = strat_smart_main_robot();
    } while((err & END_TIMER) == 0);

   strat_exit();
   return 0;
}



#endif /* HOST_VERSION_OA_TEST */
