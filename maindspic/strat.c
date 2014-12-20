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
#include <trajectory_manager_utils.h>
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
#include "beacon.h"

#else

/** 
   XXX 
   strat.c is used for OA test for both robots.
   We need define some robot dimensions here, depending on robot 
 */

#ifndef IM_SECONDARY_ROBOT

/* XXX synchronized with maindspic/main.h */

/* robot dimensions */
#define ROBOT_LENGTH            281.5
#define ROBOT_WIDTH             330.0
#define ROBOT_CENTER_TO_FRONT   162.5
#define ROBOT_CENTER_TO_BACK    119.0
#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/* XXX obstacle clerance */
#define OBS_CLERANCE            (232.+10.)

#else

/* XXX synchronized with secondary_robot/main.h */

/* robot dimensions */
#define ROBOT_LENGTH      	    150.
#define ROBOT_WIDTH 	    	230.
#define ROBOT_CENTER_TO_FRONT   75.
#define ROBOT_CENTER_TO_BACK    75.
#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/* XXX obstacle clearance */
#define OBS_CLERANCE            (137.+10.)

#endif /* ! IM_SECONDARY_ROBOT */


#endif

#define LIMIT_BBOX_Y_UP			(2000 - OBS_CLERANCE-70)
#define LIMIT_BBOX_Y_DOWN		OBS_CLERANCE+100
#define LIMIT_BBOX_X_UP			3000 - OBS_CLERANCE
#define LIMIT_BBOX_X_DOWN		OBS_CLERANCE

static volatile uint8_t strat_running = 0;


struct strat_infos strat_infos = { 
    /* conf */
    .conf = {
        .flags = 0,
    },

	/* TODO: init points (depends on strategy) */
   /*zones[W] =                 {type,            x,            y,            x_down,   x_up,  y_down,    y_up,      init_x,                 init_y,                       prio,             flags,        opp_time_zone_us,	last_time_opp_here,	robot };                            */
    .zones[ZONE_MY_STAND_GROUP_1]=        {ZONE_TYPE_STAND,  0,         300,    1750,    1950,	MY_STAND_1_X+300,	LIMIT_BBOX_Y_DOWN,	40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_MY_STAND_GROUP_2]=        {ZONE_TYPE_STAND,	700,     1000,    1700,    2000,		MY_STAND_2_X+300,	LIMIT_BBOX_Y_DOWN,	40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_MY_STAND_GROUP_3]=        {ZONE_TYPE_STAND,  700,     1000,    1750,    1950, 	MY_STAND_3_X,	MY_STAND_3_Y,	40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_MY_STAND_GROUP_4]=        {ZONE_TYPE_STAND,   720,      1020,    495,    795,		MY_STAND_4_X-300,	MY_STAND_4_Y,	40,     0,            0,					(9000*1000L),					MAIN_ROBOT},

   /*zones[W] =                 {type,             x,         y,         					x_down,    x_up,   y_down, y_up,  init_x,       init_y, prio,         flags,        opp_time_zone_us,	last_time_opp_here,	robot };  */
	.zones[ZONE_MY_POPCORNMAC]=        {ZONE_TYPE_POPCORNMAC,  	150,         	  750,    1700,    2000,      0,            0,                    40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
	.zones[ZONE_OPP_POPCORNMAC]=        {ZONE_TYPE_POPCORNMAC,   2250,         2850,    1700,    2000,      0,            0,                    40,     0,            0,					(9000*1000L),					MAIN_ROBOT},

   /*zones[W] =                 {type,             x,         y,         					x_down,    x_up,   y_down, y_up,  init_x,       init_y, prio,         flags,        opp_time_zone_us,	last_time_opp_here,	robot };  */
	.zones[ZONE_POPCORNCUP_1]=        {ZONE_TYPE_POPCORNCUP,   	 760,         1060,    1020,    1320,      0,            0,                    40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
	.zones[ZONE_POPCORNCUP_2]=        {ZONE_TYPE_POPCORNCUP,	100,         400,    100,    400,      0,            0,                    40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
	.zones[ZONE_POPCORNCUP_5]=        {ZONE_TYPE_POPCORNCUP, 1940,     2240,    1020,    1320,      0,            0,                    40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
	.zones[ZONE_POPCORNCUP_4]=        {ZONE_TYPE_POPCORNCUP,	2600,       2900,    100,    400,      0,            0,                    40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
	.zones[ZONE_POPCORNCUP_3]=        {ZONE_TYPE_POPCORNCUP,	1350,       1650,    200,    500,      0,            0,                    40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
	
   /*zones[W] =                 	{type, 				 x,         					y,	x_down,    x_up,   y_down,		 y_up,  init_x,       init_y, prio,         flags,        opp_time_zone_us	,	last_time_opp_here,	robot };  */
	.zones[ZONE_MY_CINEMA_UP]=		{ZONE_TYPE_CINEMA,	 2600,        3000,    1200,    1600,	2350,    MY_CINEMA_UP_Y,                    40,     0,            0,					(9000*1000L),					SEC_ROBOT},
	.zones[ZONE_MY_CINEMA_DOWN]= 	{ZONE_TYPE_CINEMA,	2600,         3000,    400,    800,2350,  MY_CINEMA_DOWN_Y,                    40,     0,            0,					(9000*1000L),					SEC_ROBOT},
  .zones[ZONE_MY_STAIRS]=        {ZONE_TYPE_STAIRS, 	1000,         1500,    1400,    2000,		MY_STAIRS_X,	1200,                    40,     0,            0,					(9000*1000L),					SEC_ROBOT},

  .zones[ZONE_MY_HOME]=        {ZONE_TYPE_HOME,	90,         650,    800,    1200,	650,	MY_HOME_Y,                    40,     0,            0,					(9000*1000L),					MAIN_ROBOT},

   /*zones[W] =                 {type,             x,        			y,        x_down,    x_up,   y_down, y_up,  init_x,       init_y, prio,         flags,        opp_time_zone_us,	last_time_opp_here,	robot };  */
	.zones[ZONE_MY_CLAP_1]=		{ZONE_TYPE_CLAP,	180,	480,	0,	300,	MY_CLAP_1_X,	LIMIT_BBOX_Y_DOWN, 	40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
	.zones[ZONE_MY_CLAP_2]=		{ZONE_TYPE_CLAP,	780,	1080,	0,	300,	MY_CLAP_2_X,    LIMIT_BBOX_Y_DOWN,	40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_MY_CLAP_3]=		{ZONE_TYPE_CLAP,	2230,	2530,	0,	300,	MY_CLAP_3_X,    LIMIT_BBOX_Y_DOWN,	70,     0,            0,					(9000*1000L),					SEC_ROBOT},

   .zones[ZONE_MY_STAIRWAY]=     {ZONE_TYPE_STAIRWAY,	1000,         1500,    1400,    2000,      0,            0,                    40,     0,            0,					(9000*1000L),					MAIN_ROBOT},
   
	};

/* points we get from each zone */
/* TODO*/
uint8_t strat_zones_points[ZONES_MAX]= {3,3,3,3,0,0,0,1,1,1,1,1,1,1,1,1,1,2,2,0,0,6,6,6};

/*************************************************************/

/*                  INIT                                     */

/*************************************************************/
void strat_set_bounding_box(uint8_t type)
{

    strat_infos.area_bbox.x1 = OBS_CLERANCE;
    strat_infos.area_bbox.x2 = 3000 - OBS_CLERANCE;

    strat_infos.area_bbox.y1 = OBS_CLERANCE+100;
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

/* called before each strat, and before the start switch */
void strat_preinit(void)
{
    time_reset();
    interrupt_traj_reset();
    mainboard.flags =  DO_ENCODERS | DO_CS | DO_RS |
                             DO_POS | DO_BD | DO_POWER | DO_BEACON | DO_ROBOT_2ND;

    /* XXX default conf */
    //strat_infos.conf.flags |= ENABLE_R2ND_POS;


    strat_dump_conf();
    strat_dump_infos(__FUNCTION__);
}

/* display curret strat configuration */
void strat_dump_conf(void)
{
    if (!strat_infos.dump_enabled)
        return;

    printf_P(PSTR("-- conf --\r\n"));

    /* add here configuration dump */
}

char numzone2name[ZONES_MAX +1][5]= {
[ZONE_MY_STAND_GROUP_1]="ms1\0",
[ZONE_MY_STAND_GROUP_2]="ms2\0",
[ZONE_MY_STAND_GROUP_3]="ms3\0",
[ZONE_MY_STAND_GROUP_4]="ms4\0",
[ZONE_MY_POPCORNMAC]="mm\0",
[ZONE_OPP_POPCORNMAC]="om\0",
[ZONE_POPCORNCUP_1]="mcf\0",
[ZONE_POPCORNCUP_2]="mcs\0",
[ZONE_POPCORNCUP_5]="ocf\0",
[ZONE_POPCORNCUP_4]="ocs\0",
[ZONE_POPCORNCUP_3]="cc\0",
[ZONE_MY_CINEMA_UP]="mcu\0",
[ZONE_MY_CINEMA_DOWN]="mcd\0",
[ZONE_MY_STAIRS]="ms\0",
[ZONE_MY_HOME]="mh\0",
[ZONE_MY_CLAP_1]="mb1\0",
[ZONE_MY_CLAP_2]="mb2\0",
[ZONE_MY_CLAP_3]="mb3\0",
[ZONE_MY_STAIRWAY]="mw\0",
[ZONES_MAX] = "nll\0",
};

/* display current information about the state of the game */
void strat_dump_infos(const char *caller)
{
	int8_t zone_opp;
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
    strat_set_bounding_box(mainboard.our_color);
    strat_infos.current_zone = ZONES_MAX;
    strat_infos.goto_zone = ZONES_MAX;
    strat_infos.last_zone = ZONES_MAX;

    /* add here other infos resets */
}

void strat_event_enable(void)
{
    strat_running = 1;
}

void strat_event_disable(void)
{
    strat_running = 0;
}

/* call it just before launching the strat */
void strat_init(void)
{
    strat_reset_infos();

    /* we consider that the color is correctly set */

    strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
    time_reset();
    interrupt_traj_reset();

    /* init other devices (lasers...) */
    i2c_slavedspic_mode_init();

    /* used in strat_base for END_TIMER */
    mainboard.flags = DO_ENCODERS | DO_CS | DO_RS |
        DO_POS | DO_BD | DO_TIMER | DO_POWER | DO_BEACON | DO_ROBOT_2ND;
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

    /* stop beacon */
    /* TODO beacon_cmd_beacon_off(); */

    /* slavespic exit TODO 2014 */
    //i2c_slavedspic_mode_turbine_blow(0);
    //i2c_slavedspic_wait_ready();

    /* turn off other devices (lasers...) */

    /* TODO stop beacon
    beacon_cmd_beacon_off();
    beacon_cmd_beacon_off();
    beacon_cmd_beacon_off();
    beacon_cmd_beacon_off();
    */
}

/* called periodically */
void strat_event(void *dummy)
{
    /* XXX in parallel with main strat,
     *    disable/enable events depends on case or protect with IRQ_LOCK.
     */

    /* ignore when strat is not running */
    //if (strat_running == 0)
    //    return;

    /* limit speed when opponent are close */
    strat_limit_speed();

    /* tracking of zones where opp has been working */
    strat_opp_tracking();

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
    uint8_t err, i;
    //strat_begin();
    //strat_limit_speed_enable ();
	
    /* auto-play  */
	set_strat_sec_1();
	strat_infos.current_sec_strategy = 1;
    printf_P(PSTR("\r\n\r\nStrat smart\r\n"));
    do{
        err = strat_smart();
    }while((err & END_TIMER) == 0);
	

   strat_exit();
   return 0;
}



#endif /* HOST_VERSION_OA_TEST */


