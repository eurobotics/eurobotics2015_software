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
 *  Javier Bali�as Santos <javier@arc-robots.org> and Silvia Santano
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#ifndef HOST_VERSION_OA_TEST
#include <uart.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <clock_time.h>

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
#include "../maindspic/strat_avoid.h"
#include "../maindspic/strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"
#include "cmdline.h"

#else
/* robot dimensions */
#define ROBOT_LENGTH    281.5
#define ROBOT_WIDTH 	   330.0
#define ROBOT_CENTER_TO_FRONT 162.5
#define ROBOT_CENTER_TO_BACK 119.0

#endif


static volatile uint8_t strat_running = 0;


struct strat_infos strat_infos = { 
	/* conf */
	.conf = {
		.flags = 0,
	},

   /* TODO: init points (depends on strategy) */
	
   /*zones[W] =                 {type,            x,            y,            x_down,   x_up,  y_down,    y_up,      init_x,                 init_y,                       prio,             flags,       robot };                            */
    .zones[ZONE_MY_STAND_1]=        {ZONE_TYPE_STAND,  MY_STAND_1_X,     MY_STAND_1_Y,     0,         300,    1750,    1950,      0,            0,                    40,     					MAIN_ROBOT},
    .zones[ZONE_MY_STAND_2]=        {ZONE_TYPE_STAND,  MY_STAND_2_X,     MY_STAND_2_Y,     700,     1000,    1700,    2000,      0,            0,                    40,    					MAIN_ROBOT},
    .zones[ZONE_MY_STAND_3]=        {ZONE_TYPE_STAND,  MY_STAND_3_X,     MY_STAND_3_Y,     700,     1000,    1750,    1950,      0,            0,                    40,    					MAIN_ROBOT},
    .zones[ZONE_MY_STAND_4]=        {ZONE_TYPE_STAND,  MY_STAND_4_X,     MY_STAND_4_Y,     720,      1020,    495,    795,      0,            0,                    40,     					MAIN_ROBOT},
    .zones[ZONE_MY_STAND_5]=        {ZONE_TYPE_STAND,  MY_STAND_5_X,     MY_STAND_5_Y,     1150,    1450,    455,    755,      0,            0,                    40,     					MAIN_ROBOT},
    .zones[ZONE_MY_STAND_6]=        {ZONE_TYPE_STAND,  MY_STAND_6_X,     MY_STAND_6_Y,     950,      1250,    80,    380,      0,            0,                    40,     					MAIN_ROBOT},
    .zones[ZONE_MY_STAND_7]=        {ZONE_TYPE_STAND,  MY_STAND_7_X,     MY_STAND_7_Y,     0,         300,       0,    300,      0,            0,                    40,     					MAIN_ROBOT},
    .zones[ZONE_MY_STAND_8]=        {ZONE_TYPE_STAND,  MY_STAND_8_X,     MY_STAND_8_Y,     0,         300,       100,    400,      0,            0,                    40,     					MAIN_ROBOT},

    .zones[ZONE_OPP_STAND_1]=        {ZONE_TYPE_STAND,  OPP_STAND_1_X,     OPP_STAND_1_Y,     2700,        3000,    1750,    1950,      0,            0,                    40,    					MAIN_ROBOT},
    .zones[ZONE_OPP_STAND_2]=        {ZONE_TYPE_STAND,  OPP_STAND_2_X,     OPP_STAND_2_Y,     2000,        2300,    1700,    2000,      0,            0,                    40,    					MAIN_ROBOT},
    .zones[ZONE_OPP_STAND_3]=        {ZONE_TYPE_STAND,  OPP_STAND_3_X,     OPP_STAND_3_Y,     2000,        2300,    1750,    1950,      0,            0,                    40,     					MAIN_ROBOT},
    .zones[ZONE_OPP_STAND_4]=        {ZONE_TYPE_STAND,  OPP_STAND_4_X,     OPP_STAND_4_Y,     1980,        2280,    495,    795,      0,            0,                    40,    					MAIN_ROBOT},
    .zones[ZONE_OPP_STAND_5]=        {ZONE_TYPE_STAND,  OPP_STAND_5_X,     OPP_STAND_5_Y,     1550,        1850,    455,    755,      0,            0,                    40,   					MAIN_ROBOT},
    .zones[ZONE_OPP_STAND_6]=        {ZONE_TYPE_STAND,  OPP_STAND_6_X,     OPP_STAND_6_Y,     1750,        1950,    80,    380,      0,            0,                    40,     					MAIN_ROBOT},
    .zones[ZONE_OPP_STAND_7]=        {ZONE_TYPE_STAND,  OPP_STAND_7_X,     OPP_STAND_7_Y,     2700,        3000,      0,         300,      0,            0,                    40,    					MAIN_ROBOT},
    .zones[ZONE_OPP_STAND_8]=        {ZONE_TYPE_STAND,  OPP_STAND_8_X,     OPP_STAND_8_Y,     2700,        3000,    100,     400,      0,            0,                    40,    					MAIN_ROBOT},
	
   /*zones[W] =                 {type,             x,         y,         					x_down,    x_up,   y_down, y_up,  init_x,       init_y, prio,         flags,        robot };  */
   .zones[ZONE_MY_LIGHTBULB_HOME]=        {ZONE_TYPE_LIGHTBULB,  MY_LIGHTBULB_HOME_X,     MY_LIGHTBULB_HOME_Y,     						      70,         	370,    850,    1150,      0,            0,                    40,    					MAIN_ROBOT},
   .zones[ZONE_MY_LIGHTBULB_PLATFORM]=        {ZONE_TYPE_LIGHTBULB,  MY_LIGHTBULB_PLATFORM_X,     MY_LIGHTBULB_PLATFORM_Y,            1100,         1400,    0,    300,      0,            0,                    40,     					MAIN_ROBOT},
   .zones[ZONE_OPP_LIGHTBULB_HOME]=        {ZONE_TYPE_LIGHTBULB,  OPP_LIGHTBULB_HOME_X,     OPP_LIGHTBULB_HOME_Y,    		 				  2630,         2930,    850,    1150,      0,            0,                    40,    					MAIN_ROBOT},
   .zones[ZONE_OPP_LIGHTBULB_PLATFORM]=        {ZONE_TYPE_LIGHTBULB,  OPP_LIGHTBULB_PLATFORM_X,     OPP_LIGHTBULB_PLATFORM_Y,     1600,         1900,    0,    300,      0,            0,                    40,     					MAIN_ROBOT},

   /*zones[W] =                 {type,             x,         y,         					x_down,    x_up,   y_down, y_up,  init_x,       init_y, prio,         flags,        robot };  */
   .zones[ZONE_MY_POPCORNMAC_1]=        {ZONE_TYPE_POPCORNMAC,  MY_POPCORNMAC_1_X,     MY_POPCORNMAC_1_Y,     150,         	  450,    1700,    2000,      0,            0,                    40,    					MAIN_ROBOT},
   .zones[ZONE_MY_POPCORNMAC_2]=        {ZONE_TYPE_POPCORNMAC,  MY_POPCORNMAC_2_X,     MY_POPCORNMAC_2_Y,     450,         	  750,    1700,    2000,      0,            0,                    40,     					MAIN_ROBOT},
   .zones[ZONE_OPP_POPCORNMAC_1]=        {ZONE_TYPE_POPCORNMAC,  OPP_POPCORNMAC_1_X,     OPP_POPCORNMAC_1_Y,     2550,         2850,    1700,    2000,      0,            0,                    40,     					MAIN_ROBOT},
   .zones[ZONE_OPP_POPCORNMAC_2]=        {ZONE_TYPE_POPCORNMAC,  OPP_POPCORNMAC_2_X,     OPP_POPCORNMAC_2_Y,     2250,         2550,    1700,    2000,      0,            0,                    40,     					MAIN_ROBOT},
  
   /*zones[W] =                 {type,             x,         y,         					x_down,    x_up,   y_down, y_up,  init_x,       init_y, prio,         flags,       robot };  */
	.zones[ZONE_MY_POPCORNCUP_FRONT]=        {ZONE_TYPE_POPCORNCUP,  MY_POPCORNCUP_FRONT_X,     MY_POPCORNCUP_FRONT_Y,    	 760,         1060,    1020,    1320,      0,            0,                    40,     					MAIN_ROBOT},
	.zones[ZONE_MY_POPCORNCUP_SIDE]=        {ZONE_TYPE_POPCORNCUP,  MY_POPCORNCUP_SIDE_X,     MY_POPCORNCUP_SIDE_Y,     			 100,         400,    100,    400,      0,            0,                    40,     					MAIN_ROBOT},
	.zones[ZONE_OPP_POPCORNCUP_FRONT]=        {ZONE_TYPE_POPCORNCUP,  OPP_POPCORNCUP_FRONT_X,     OPP_POPCORNCUP_FRONT_Y,    1940,     2240,    1020,    1320,      0,            0,                    40,     					MAIN_ROBOT},
	.zones[ZONE_OPP_POPCORNCUP_SIDE]=        {ZONE_TYPE_POPCORNCUP,  OPP_POPCORNCUP_SIDE_X,     OPP_POPCORNCUP_SIDE_Y,     		2600,       2900,    100,    400,      0,            0,                    40,    					MAIN_ROBOT},
	.zones[ZONE_POPCORNCUP_CENTRE]=        {ZONE_TYPE_POPCORNCUP,  POPCORNCUP_CENTRE_X,     POPCORNCUP_CENTRE_Y,     		1350,       1650,    200,    500,      0,            0,                    40,    					MAIN_ROBOT},
	
   /*zones[W] =                 {type,             x,         y,         					x_down,    x_up,   y_down, y_up,  init_x,       init_y, prio,         flags,        robot };  */
   .zones[ZONE_MY_CINEMA_UP]=        {ZONE_TYPE_CINEMA,  MY_CINEMA_UP_X,    MY_CINEMA_UP_Y,     					     0,         400,    1200,    1600,      0,            0,                    40,    					MAIN_ROBOT},
   .zones[ZONE_MY_CINEMA_DOWN]=        {ZONE_TYPE_CINEMA,  MY_CINEMA_DOWN_X,    MY_CINEMA_DOWN_Y,     		 0,         400,    400,    800,      0,            0,                    40,     					MAIN_ROBOT},
   .zones[ZONE_OPP_CINEMA_UP]=        {ZONE_TYPE_CINEMA,  OPP_CINEMA_UP_X,    OPP_CINEMA_UP_Y,    					 2600,        3000,    1200,    1600,      0,            0,                    40,     					MAIN_ROBOT},
   .zones[ZONE_OPP_CINEMA_DOWN]=        {ZONE_TYPE_CINEMA,  OPP_CINEMA_DOWN_X,    OPP_CINEMA_DOWN_Y,     2600,         3000,    400,    800,      0,            0,                    40,     					MAIN_ROBOT},

  .zones[ZONE_MY_STAIRS]=        {ZONE_TYPE_STAIRS,  MY_STAIRS_X,    MY_STAIRS_Y,     	  1000,         1500,    1400,    2000,      0,            0,                    40,    					MAIN_ROBOT},
  .zones[ZONE_OPP_STAIRS]=        {ZONE_TYPE_STAIRS,  OPP_STAIRS_X,    OPP_STAIRS_Y,     1500,    2000,    1400,    2000,      0,            0,                    40,     					MAIN_ROBOT},
 
  .zones[ZONE_MY_HOME]=        {ZONE_TYPE_HOME,  MY_HOME_X,    MY_HOME_Y,     		90,         650,    800,    1200,      0,            0,                    40,    					MAIN_ROBOT},
  .zones[ZONE_OPP_HOME]=        {ZONE_TYPE_HOME,  OPP_HOME_X,    OPP_HOME_Y,     2350,         2910,    800,    1200,      0,            0,                    40,    					MAIN_ROBOT},
 
   /*zones[W] =                 {type,             x,         y,         					x_down,    x_up,   y_down, y_up,  init_x,       init_y, prio,         flags,        robot };  */
   .zones[ZONE_MY_CLAP_1]=        {ZONE_TYPE_CLAP,  MY_CLAP_1_X,    CLAP_Y,     	180,      480,    0,    300,      0,            0,                    40,    					MAIN_ROBOT},
    .zones[ZONE_MY_CLAP_2]=        {ZONE_TYPE_CLAP,  MY_CLAP_2_X,     CLAP_Y,    	780,        1080,    0,    300,      0,            0,                    40,    					MAIN_ROBOT},
    .zones[ZONE_MY_CLAP_3]=        {ZONE_TYPE_CLAP,  MY_CLAP_3_X,     CLAP_Y,    	 2230,    2530,    0,    300,      0,            0,                    40,     					MAIN_ROBOT},
    .zones[ZONE_OPP_CLAP_1]=        {ZONE_TYPE_CLAP,  OPP_CLAP_1_X,     CLAP_Y,     2520,         2820,    0,    300,      0,            0,                    40,     					MAIN_ROBOT},
    .zones[ZONE_OPP_CLAP_2]=        {ZONE_TYPE_CLAP,  OPP_CLAP_2_X,     CLAP_Y,     1920,         2220,    0,    300,      0,            0,                    40,    					MAIN_ROBOT},
    .zones[ZONE_OPP_CLAP_3]=        {ZONE_TYPE_CLAP,  OPP_CLAP_3_X,     CLAP_Y,     470,         770,    0,    300,      0,            0,                    40,    					MAIN_ROBOT},

   .zones[ZONE_MY_STAIRWAY_1]=        {ZONE_TYPE_STAIRWAY,  MY_STAIRS_X,    MY_STAIRS_Y,     1000,         1100,    1400,    2000,      0,            0,                    40,    					MAIN_ROBOT},
   .zones[ZONE_MY_STAIRWAY_2]=        {ZONE_TYPE_STAIRWAY,  MY_STAIRS_X,    MY_STAIRS_Y,     1400,         1500,    1400,    2000,      0,            0,                    40,   					MAIN_ROBOT},
   .zones[ZONE_OPP_STAIRWAY_1]=        {ZONE_TYPE_STAIRWAY,  OPP_STAIRS_X,    OPP_STAIRS_Y,    1500,     1600,    1400,    2000,      0,            0,                    40,   					MAIN_ROBOT},
   .zones[ZONE_OPP_STAIRWAY_1]=        {ZONE_TYPE_STAIRWAY,  OPP_STAIRS_X,    OPP_STAIRS_Y,     1900,     2000,    1400,    2000,      0,            0,                    40,     					MAIN_ROBOT},
};


/*************************************************************/

/*                  INIT                                     */

/*************************************************************/
void strat_set_bounding_box(uint8_t type)
{

	if (type == 0)
	{

		strat_infos.area_bbox.x1 = OBS_CLERANCE;
		strat_infos.area_bbox.x2 = 3000 - OBS_CLERANCE;
	
		strat_infos.area_bbox.y1 = OBS_CLERANCE + 300;
		strat_infos.area_bbox.y2 = 2000 - OBS_CLERANCE;
	}
	else
	{
		strat_infos.area_bbox.x1 = 0;
		strat_infos.area_bbox.x2 = 3000;
	
		strat_infos.area_bbox.y1 = 0;
		strat_infos.area_bbox.y2 = 2000;

	}

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
							 DO_POS | DO_BD | DO_POWER | DO_BEACON;

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

	printf(" ENABLE_R2ND_POS is %s\n\r", strat_infos.conf.flags & ENABLE_R2ND_POS? "ON":"OFF");
	printf(" ENABLE_DOWN_SIDE_ZONES is %s\n\r", strat_infos.conf.flags & ENABLE_DOWN_SIDE_ZONES? "ON":"OFF");

	/* add here configuration dump */
}


char numzone2name[ZONES_MAX + 1][3] = {
[ZONE_MY_STAND_1]="ms1",
[ZONE_MY_STAND_2]="ms2",
[ZONE_MY_STAND_3]="ms3",
[ZONE_MY_STAND_4]="ms4",
[ZONE_MY_STAND_5]="ms5",
[ZONE_MY_STAND_6]="ms6",
[ZONE_MY_STAND_7]="ms7",
[ZONE_MY_STAND_8]="ms8",
[ZONE_OPP_STAND_1]="os1",
[ZONE_OPP_STAND_2]="os2",
[ZONE_OPP_STAND_3]="os3",
[ZONE_OPP_STAND_4]="os4",
[ZONE_OPP_STAND_5]="os5",
[ZONE_OPP_STAND_6]="os6",
[ZONE_OPP_STAND_7]="os7",
[ZONE_OPP_STAND_8]="os8",
[ZONE_MY_LIGHTBULB_HOME]="mlh",
[ZONE_MY_LIGHTBULB_PLATFORM]="mlp",
[ZONE_OPP_LIGHTBULB_HOME]="olh",
[ZONE_OPP_LIGHTBULB_PLATFORM]="olp",
[ZONE_MY_POPCORNMAC_1]="mm1",
[ZONE_MY_POPCORNMAC_2]="mm2",
[ZONE_OPP_POPCORNMAC_1]="mm1",
[ZONE_OPP_POPCORNMAC_2]="mm2",
[ZONE_MY_POPCORNCUP_FRONT]="mcf",
[ZONE_MY_POPCORNCUP_SIDE]="mcs",
[ZONE_OPP_POPCORNCUP_FRONT	]="ocf",
[ZONE_OPP_POPCORNCUP_SIDE]="ocs",
[ZONE_POPCORNCUP_CENTRE]="cc",
[ZONE_MY_CINEMA_UP]="mcu",
[ZONE_MY_CINEMA_DOWN]="mcd",
[ZONE_OPP_CINEMA_UP]="ocu",
[ZONE_OPP_CINEMA_DOWN]="ocd",
[ZONE_MY_STAIRS]="ms",
[ZONE_OPP_STAIRS]="os",
[ZONE_MY_HOME]="mh",
[ZONE_OPP_HOME]="oh",
[ZONE_MY_CLAP_1]="mb1",
[ZONE_MY_CLAP_2]="mb2",
[ZONE_MY_CLAP_3]="mb3",
[ZONE_OPP_CLAP_1]="ob1",
[ZONE_OPP_CLAP_2]="ob2",
[ZONE_OPP_CLAP_3]="ob3",
[ZONE_MY_STAIRWAY_1]="mw1",
[ZONE_MY_STAIRWAY_2]="mw2",
[ZONE_OPP_STAIRWAY_1]="ow1",
[ZONE_OPP_STAIRWAY_2]="ow2",
[ZONES_MAX] = "nll",
};

/* display current information about the state of the game */
void strat_dump_infos(const char *caller)
{
	if (!strat_infos.dump_enabled)
		return;

	printf(PSTR("%s() dump strat infos:\r\n"), caller);


	/* add here print infos */
}

/* init current area state before a match. Dump update user conf
 * here */
void strat_reset_infos(void)
{
	/* bounding box */
	strat_set_bounding_box(0);
		
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
	/* we consider that the color is correctly set */

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
	time_reset();
	interrupt_traj_reset();

	/* init other devices (lasers...) */

	/* used in strat_base for END_TIMER */
	mainboard.flags = DO_ENCODERS | DO_CS | DO_RS | 
		DO_POS | DO_BD | DO_TIMER | DO_POWER | DO_BEACON;
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
	pwm_mc_set(LEFT_MOTOR, 0);
	pwm_mc_set(RIGHT_MOTOR, 0);
#endif

	/* stop beacon */
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_BEACON);
	IRQ_UNLOCK(flags);
#ifndef HOST_VERSION
	beacon_stop();
#endif
}

/* called periodically */
void strat_event(void *dummy)
{
	/* XXX in parallel with main strat, 
	 *	disable/enable events depends on case or protect with IRQ_LOCK.
	 */

	/* ignore when strat is not running */
	//if (strat_running == 0)
	//	return;

	/* limit speed when opponent are close */
	strat_limit_speed();

}

/* dump state (every 5 s max) XXX */
#define DUMP_RATE_LIMIT(dump, last_print)		\
	do {						\
		if (time_get_s() - last_print > 5) {	\
			dump();				\
			last_print = time_get_s();	\
		}					\
	} while (0)


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)	

/* debug state machines step to step */
void state_debug_wait_key_pressed(void)
{
	if (!strat_infos.debug_step)
		return;

	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* strat main loop */
uint8_t strat_main(void)
{
#define DEBUG_STRAT_HARVEST_FRUITS
#ifdef DEBUG_STRAT_HARVEST_FRUITS 
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif

    uint8_t err = 0;
    goto end;

end:
	time_wait_ms (500);
	return err;	
}

/* start a match debugging or not */
void strat_start_match(uint8_t debug)
{
	uint8_t old_level = gen.log_level;

	time_wait_ms(1000);

	/* logs */
	gen.logs[NB_LOGS] = E_USER_STRAT;

	if (debug) {
		strat_infos.dump_enabled = 1;
		gen.log_level = 5;
	}
	else {
		strat_infos.dump_enabled = 0;
		gen.log_level = 0;
	}	

	/* get color */
	mainboard.our_color = sensor_get(S_COLOR);
	printf_P(PSTR("COLOR is %s\r\n"), mainboard.our_color == I2C_COLOR_RED? "RED" : "YELLOW");

	/* set x,y and angle */
#define TRESPA_BAR 17
	strat_reset_pos(COLOR_X(ROBOT_WIDTH/2 + TRESPA_BAR), TRESPA_BAR+(ROBOT_LENGTH/2), COLOR_A_ABS(90));	

	printf_P(PSTR("x=%.2f y=%.2f a=%.2f\r\n"), 
		 position_get_x_double(&mainboard.pos),
		 position_get_y_double(&mainboard.pos),
		 DEG(position_get_a_rad_double(&mainboard.pos)));

	/* strat start */
	strat_start();

	/* restore logs */
	gen.log_level = old_level;
}

#endif /* HOST_VERSION_OA_TEST */


