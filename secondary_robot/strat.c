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
 *  Javier Baliñas Santos <javier@arc-robots.org> and Silvia Santano
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#ifndef HOST_VERSION
#include <uart.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <time.h>

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
#include "beacon.h"

#endif

struct strat_infos strat_infos = { 
	/* conf */
	.conf = {
		.flags = 0,
	},

   /*zones[W] =                          { x_up, y_up, x_down, y_down, init_x,init_y, prio,     flags };                            */
   .zones[ZONE_TOTEM_1_SIDE_1]=          { 1400, 1000, 700,    500   , 1100,  500,   ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_TOTEM_1_SIDE_2]=          { 1400, 1500, 700,    1000  , 1100,  1500,  ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_TOTEM_2_SIDE_1]=          { 2300, 1000, 1600,   500   , 1900,  400,   ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_TOTEM_2_SIDE_2]=          { 2300, 1500, 1600,   1000  , 1900,  1500,  ZONE_PRIO_2, ZONE_WITH_TREASURE },
	
	.zones[ZONE_PURPLE_FLOOR_COIN_1]=     { 1400, 650,  700,    250   , 2000,  200,  ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_PURPLE_FLOOR_COIN_2]=     { 2450, 1200, 2100,   800   , 2400,  1000, ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_PURPLE_FLOOR_COIN_3]=     { 2650, 1850, 2250,   1550  , 2250,  1700, ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_PURPLE_FLOOR_GOLDBAR]=    { 2700, 1000, 2300,   600   , 2300,  785,  ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_RED_FLOOR_COIN_1]=        { 2300, 650,  1600,   250   , 1000,  200,  ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_RED_FLOOR_COIN_2]=        { 550,  1200, 900,    800   , 600,   1000, ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_RED_FLOOR_COIN_3]=        { 350,  1850, 750,   1550   , 750,   1700, ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_RED_FLOOR_GOLDBAR]=       { 300,  1000, 700,    600   , 700,   785,  ZONE_PRIO_2, ZONE_WITH_TREASURE },

	.zones[ZONE_PURPLE_BOTTLE_1]=         { 2550, 2000, 2170,   1600  , 1900,  1600, ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_PURPLE_BOTTLE_2]=         { 1307, 2000, 927,    1600  , 600,   1600, ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_RED_BOTTLE_1]=            { 830,  2000, 450,    1600  , 640,   1700, ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_RED_BOTTLE_2]=            { 2073, 2000, 1693,   1600  , 1883,  1700, ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_PURPLE_MAP]=              { 2000, 500,  1500,      0  , 1600,  400,  ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_RED_MAP]=                 { 1500, 500,  1000,      0  , 1400,  400,  ZONE_PRIO_2, ZONE_WITH_TREASURE },

   /*XXX To pickup floor coins group the init position should depend on which color we are playing*/
	.zones[ZONE_FLOOR_COINS_GROUP]=                 { 1780, 1950, 1220,   1450  , 1782,  1470, ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_MIDDLE_FLOOR_GOLDBAR]=              { 1780, 1156, 1220,   1550  , 1500,  1500, ZONE_PRIO_2, ZONE_WITH_TREASURE },
	
   /* ship zones */
   .zones[ZONE_SHIP_PURPLE_CAPTAINS_BEDRROM]=      { 3000,     500,    2500,       0     , 2100 ,  250,  ZONE_PRIO_2, ZONE_WITH_TREASURE },
   .zones[ZONE_SHIP_PURPLE_DECK]=                  { 3000,     1400,   2600,     500     , 2250,   950,  ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_SHIP_PURPLE_HOLD]=                  { 3000,     2000,   2600,     1400    , 2250,   1700, ZONE_PRIO_2, ZONE_WITH_TREASURE },
   .zones[ZONE_SHIP_RED_CAPTAINS_BEDRROM]=         { 500,      500,    0,          0     , 900,    250,  ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_SHIP_RED_DECK]=                     { 400,      1400,   0,         500    , 750,    950,  ZONE_PRIO_2, ZONE_WITH_TREASURE },
	.zones[ZONE_SHIP_RED_HOLD]=                     { 400,      2000,   0,        1400    , 750,    1700, ZONE_PRIO_2, ZONE_WITH_TREASURE },
};


/*************************************************************/

/*                  INIT                                     */

/*************************************************************/
void strat_set_bounding_box(void)
{
	strat_infos.area_bbox.x1 = 400 + OBS_CLERANCE;
	strat_infos.area_bbox.y1 = OBS_CLERANCE;
	strat_infos.area_bbox.x2 = 3000 - 400 - OBS_CLERANCE;
	strat_infos.area_bbox.y2 = 2000 - 44 - OBS_CLERANCE;

	polygon_set_boundingbox(strat_infos.area_bbox.x1,
				strat_infos.area_bbox.y1,
				strat_infos.area_bbox.x2,
				strat_infos.area_bbox.y2);
}

#ifndef HOST_VERSION

/* called before each strat, and before the start switch */
void strat_preinit(void)
{
	time_reset();
	interrupt_traj_reset();
	mainboard.flags =  DO_ENCODERS | DO_CS | DO_RS |
							 DO_POS | DO_BD | DO_POWER | DO_OPP;

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
	strat_set_bounding_box();

	/* add here other infos resets */
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
	time_wait_ms(100);
	comb_set_pos(COMB_POS_CLOSE);
	time_wait_ms(100);
	arm_set_pos(ARM_POS_HIDE);
	time_wait_ms(100);
	teeth_set_pos(TEETH_POS_CLOSE);
	time_wait_ms(100);

	/* used in strat_base for END_TIMER */
	mainboard.flags = DO_ENCODERS | DO_CS | DO_RS | 
		DO_POS | DO_BD | DO_TIMER | DO_POWER | DO_OPP;
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
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_CS);
	pwm_mc_set(LEFT_MOTOR, 0);
	pwm_mc_set(RIGHT_MOTOR, 0);
	IRQ_UNLOCK(flags);

	/* turn off other devices (lasers...) */
}

/* called periodically */
void strat_event(void *dummy)
{
	/* XXX in parallel with main strat, 
	 *	disable/enable events depends on case or protect with IRQ_LOCK.
	 */

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


/* start a match debuging or not */
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
	mainboard.our_color = sensor_get(S_COLOR_SWITCH);
	printf_P(PSTR("COLOR is %s\r\n"), mainboard.our_color == I2C_COLOR_RED? "RED" : "PURPLE");

	/* reset position */
	strat_position_color();

	printf_P(PSTR("x=%.2f y=%.2f a=%.2f\r\n"), 
		 position_get_x_double(&mainboard.pos),
		 position_get_y_double(&mainboard.pos),
		 DEG(position_get_a_rad_double(&mainboard.pos)));

	/* strat start */
	strat_start();

	/* restore logs */
	gen.log_level = old_level;
}


/* strat main loop */
uint8_t strat_main(void)
{
#define STRAT_GOTO_EMPTY_TOTEM			0
#define STRAT_GOTO_EMPTY_TOTEM_RETRY	1
#define STRAT_DO_EMPTY_TOTEM				2
#define STRAT_DO_SAVE_TREASURE			3
#define STRAT_GOTO_DISCOVER_MAP			4
#define STRAT_GOTO_DISCOVER_MAP_AVOID	5
#define STRAT_DO_DISCOVER_MAP				6
#define STRAT_WAIT_FOR_MAIN_ROBOT		7
#define STRAT_GOTO_PROTECT					8
#define STRAT_ESCAPE_DISCOVER_MAP		9
#define STRAT_GOTO_BLOCKING				10
#define STRAT_END								11

	uint8_t state = STRAT_GOTO_DISCOVER_MAP;
   uint8_t err;

	time_wait_ms(250);

	strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);

	while(1)
  	{

#ifdef HOMOLOGATION
		//time_wait_ms(2800);
#endif

		switch(state)
		{
			case STRAT_GOTO_EMPTY_TOTEM:
			
				/* TODO goto with curve traj */

				/* go near orphan coin */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1000), 357);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err)) {
					time_wait_ms(2800);
					state = STRAT_GOTO_EMPTY_TOTEM_RETRY;
					break;
				}

				/* go near totem */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1460), 710);
				err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					time_wait_ms(2800);
					state = STRAT_GOTO_EMPTY_TOTEM_RETRY;
					break;
				}

				state = STRAT_DO_EMPTY_TOTEM;
				break;

			case STRAT_GOTO_EMPTY_TOTEM_RETRY:
		
				/* TODO goto with avoidance */

				/* go near totem */
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1460), 710);
				err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					time_wait_ms(2800);

					/* XXX number of times or timeout? */

					break;
				}

				state = STRAT_DO_EMPTY_TOTEM;
				break;

			case STRAT_DO_EMPTY_TOTEM:
				err = strat_empty_totem();
				if (!TRAJ_SUCCESS(err)) {
					trajectory_goto_xy_rel(&mainboard.traj, COLOR_SIGN(100), -10);
					err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
					state = STRAT_GOTO_EMPTY_TOTEM_RETRY;
					time_wait_ms(2800);
					break;
				}
				
				state = STRAT_DO_SAVE_TREASURE;
				break;

			case STRAT_DO_SAVE_TREASURE:
				err = strat_save_treasure_on_ship();
				if (!TRAJ_SUCCESS(err)) {

					time_wait_ms(2800);

					comb_set_pos(COMB_POS_CLOSE);

					trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(700), 600);
					err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
					if (!TRAJ_SUCCESS(err)) {
						time_wait_ms(2800);	
						break;
					}

					state = STRAT_GOTO_DISCOVER_MAP;
					break;
				}
				
				state = STRAT_GOTO_DISCOVER_MAP;
				break;

			case STRAT_WAIT_FOR_MAIN_ROBOT:
				if(time_get_s() > WAIT_FOR_MAIN_ROBOT_TIMEOUT)
					state = STRAT_GOTO_DISCOVER_MAP;
				break;
#if 0
			case STRAT_GOTO_BLOCKING:
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1665), 260);
				err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					//trajectory_d_rel(&mainboard.traj, -100);
					err = wait_traj_end(TRAJ_FLAGS_STD);
					time_wait_ms(2800);
					strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
					break;
				}

				time_wait_ms(15);

				state = STRAT_GOTO_DISCOVER_MAP;
				break;
#endif
			case STRAT_GOTO_DISCOVER_MAP:
				//trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1370), 260);
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1440), 260);

				time_wait_ms(1000);

				err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					trajectory_d_rel(&mainboard.traj, -100);
					err = wait_traj_end(TRAJ_FLAGS_STD);
					time_wait_ms(2800);
					strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
					break;
				}

				state = STRAT_DO_DISCOVER_MAP;
				break;

			case STRAT_GOTO_DISCOVER_MAP_AVOID:
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(700), 235);
				err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					state = STRAT_GOTO_DISCOVER_MAP;
					break;
				}

				state = STRAT_GOTO_DISCOVER_MAP;
				break;

			case STRAT_DO_DISCOVER_MAP:
				err = strat_pickup_map();
				//if (!TRAJ_SUCCESS(err)) {
				//	time_wait_ms(2800);
				//	break;
				//}

				strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
				//trajectory_a_rel(&mainboard.traj, COLOR_A_REL(-360));
				//err = wait_traj_end(END_TRAJ);

				trajectory_a_rel(&mainboard.traj, COLOR_A_REL(-90));
				err = wait_traj_end(END_TRAJ);
				
				arm_set_pos(ARM_POS_PICKUP_MAP2);

				state = STRAT_END;
				break;
				
			case STRAT_ESCAPE_DISCOVER_MAP:
				if(time_get_s() < 80)
					break;

				strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);

				//trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1370), 400);
				trajectory_d_rel(&mainboard.traj, -200);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err)) {
					state = STRAT_END;
					break;
				}
				
				state = STRAT_END;
				break;
		
			case STRAT_GOTO_PROTECT:			
				trajectory_goto_xy_abs(&mainboard.traj, COLOR_X(1000), 500);
				err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
				if (!TRAJ_SUCCESS(err)) {
					time_wait_ms(2800);
					break;
				}

				state = STRAT_END;
				break;
		
			case STRAT_END:
				return END_TRAJ;
			
			default:
				break;
		}

	}

	return END_TRAJ;
}

#endif /* HOST_VERSION */


