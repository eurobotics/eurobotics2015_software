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
#include <dac_mc.h>
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

   /*zones[W] =                          {type, x, 											x_down, y_up, x_up, y_down, init_x,init_y, prio, flags };                            */
   .zones[ZONE_TOTEM_OUR_SIDE_1]=        {ZONE_TYPE_TOTEM, OUR_TOTEM_X, OUR_TOTEM_Y, 1300, 800,  900,    200   , 1100,  400,   ZONE_PRIO_40, 0 },
	.zones[ZONE_TOTEM_OUR_SIDE_2]=        {ZONE_TYPE_TOTEM, OUR_TOTEM_X, OUR_TOTEM_Y, 1300, 1800, 900,    1200  , 1100,  1600,  ZONE_PRIO_70, 0 },
	.zones[ZONE_TOTEM_OPP_SIDE_1]=     	  {ZONE_TYPE_TOTEM, OPP_TOTEM_X, OPP_TOTEM_Y, 2100, 800,  1700,   200   , 1900,  400,   ZONE_PRIO_30, 0 },
	.zones[ZONE_TOTEM_OPP_SIDE_2]=        {ZONE_TYPE_TOTEM, OPP_TOTEM_X, OPP_TOTEM_Y, 2100, 1800, 1700,   1200  , 1900,  1600,  ZONE_PRIO_100,0 },
	
	.zones[ZONE_OUR_FLOOR_COIN_1]=        {ZONE_TYPE_COIN,  OUR_FLOOR_COIN_1_X, OUR_FLOOR_COIN_1_Y, 2300, 650,  1600,   250   , 1000,  200,  ZONE_PRIO_0, ZONE_CHECKED },
	.zones[ZONE_OUR_FLOOR_COIN_2]=        {ZONE_TYPE_COIN,  OUR_FLOOR_COIN_2_X, OUR_FLOOR_COIN_2_Y, 550,  1200, 900,    800   , 600,   1000, ZONE_PRIO_0, ZONE_CHECKED },
	.zones[ZONE_OUR_FLOOR_COIN_3]=        {ZONE_TYPE_COIN,  OUR_FLOOR_COIN_3_X, OUR_FLOOR_COIN_3_Y, 350,  1850, 750,   1550   , 750,   1700, ZONE_PRIO_0, ZONE_CHECKED },

	.zones[ZONE_OPP_FLOOR_COIN_1]=        {ZONE_TYPE_COIN,  OPP_FLOOR_COIN_1_X, OPP_FLOOR_COIN_1_Y, 1400, 650,  700,    250   , 2000,  200,  ZONE_PRIO_0, ZONE_CHECKED },
	.zones[ZONE_OPP_FLOOR_COIN_2]=        {ZONE_TYPE_COIN,  OPP_FLOOR_COIN_2_X, OPP_FLOOR_COIN_2_Y, 2450, 1200, 2100,   800   , 2400,  1000, ZONE_PRIO_0, ZONE_CHECKED },
	.zones[ZONE_OPP_FLOOR_COIN_3]=        {ZONE_TYPE_COIN,  OPP_FLOOR_COIN_3_X, OPP_FLOOR_COIN_3_Y, 2650, 1850, 2250,   1550  , 2250,  1700, ZONE_PRIO_0, ZONE_CHECKED },

	.zones[ZONE_OUR_FLOOR_GOLDBAR]=       {ZONE_TYPE_GOLDBAR, OUR_FLOOR_GOLDBAR_X, OUR_FLOOR_GOLDBAR_Y, 300,  1000, 700,    600   , 700,   785,  ZONE_PRIO_0, ZONE_CHECKED },
	.zones[ZONE_OPP_FLOOR_GOLDBAR]=       {ZONE_TYPE_GOLDBAR, OPP_FLOOR_GOLDBAR_X, OPP_FLOOR_GOLDBAR_Y, 2700, 1000, 2300,   600   , 2300,  785,  ZONE_PRIO_0, ZONE_CHECKED },

	.zones[ZONE_OUR_BOTTLE_1]=            {ZONE_TYPE_BOTTLE, OUR_BOTTLE_1_X, BOTTLES_Y, 840,  2000, 440,    1450  , 640,   1700, ZONE_PRIO_60, 0 },
	.zones[ZONE_OUR_BOTTLE_2]=            {ZONE_TYPE_BOTTLE, OUR_BOTTLE_2_X, BOTTLES_Y, 2083, 2000, 1683,   1450  , 1883,  1700, ZONE_PRIO_90, 0 },
	.zones[ZONE_OPP_BOTTLE_1]=            {ZONE_TYPE_BOTTLE, OPP_BOTTLE_1_X, BOTTLES_Y, 2560, 2000, 2160,   1450  , 2000,  1700, ZONE_PRIO_0,  ZONE_AVOID },
	.zones[ZONE_OPP_BOTTLE_2]=            {ZONE_TYPE_BOTTLE, OPP_BOTTLE_2_X, BOTTLES_Y, 1317, 2000, 917,    1450  , 1477,  1700, ZONE_PRIO_0,  ZONE_AVOID },

	.zones[ZONE_OUR_MAP]=                 {ZONE_TYPE_MAP, OUR_MAP_X, OUR_MAP_Y, 1500, 500,  1000,      0  , 1400,  400,  ZONE_PRIO_0, ZONE_CHECKED },
	.zones[ZONE_OPP_MAP]=                 {ZONE_TYPE_MAP, OUR_MAP_X, OUR_MAP_Y, 2000, 500,  1500,      0  , 1600,  400,  ZONE_PRIO_0, ZONE_AVOID },

	.zones[ZONE_MIDDLE_COINS_GROUP]=      {ZONE_TYPE_COINS_GROUP, MIDDLE_COINS_GROUP_X, MIDDLE_COINS_GROUP_Y, 1780, 1950, 1220,   1450  , 1218,  1470, ZONE_PRIO_0, ZONE_CHECKED },
	.zones[ZONE_MIDDLE_FLOOR_GOLDBAR]=    {ZONE_TYPE_GOLDBAR, MIDDLE_FLOOR_GOLDBAR_X, MIDDLE_FLOOR_GOLDBAR_Y, 1780, 1156, 1220,   1550  , 1500,  1700, ZONE_PRIO_0, ZONE_CHECKED },
	
    /* ship zones */
   .zones[ZONE_SHIP_OUR_CAPTAINS_BEDRROM]= {ZONE_TYPE_CAPTAINS_BEDROOM, OUR_CAPTAINS_BEDROOM_X, OUR_CAPTAINS_BEDROOM_Y, 500, 500, 0,    0 , 800,   250,  ZONE_PRIO_0, ZONE_AVOID },
   .zones[ZONE_SHIP_OPP_CAPTAINS_BEDRROM]= {ZONE_TYPE_CAPTAINS_BEDROOM, OPP_CAPTAINS_BEDROOM_X, OPP_CAPTAINS_BEDROOM_Y, 3000,500, 2500, 0 , 2200,  250,  ZONE_PRIO_0, ZONE_AVOID },

	.zones[ZONE_SHIP_OUR_DECK_1]= {ZONE_TYPE_DECK, OUR_SHIP_DECK_1_X, OUR_SHIP_DECK_1_Y,  400,     1100,   0,        500     , 640,    900,  ZONE_PRIO_80, ZONE_AVOID },
   .zones[ZONE_SHIP_OPP_DECK_1]= {ZONE_TYPE_DECK, OPP_SHIP_DECK_1_X, OPP_SHIP_DECK_1_Y, 3000,     1100,   2600,     500     , 2360,   900,  ZONE_PRIO_0, ZONE_AVOID },

	.zones[ZONE_SHIP_OUR_DECK_2]=   {ZONE_TYPE_DECK, OUR_SHIP_DECK_2_X, OUR_SHIP_DECK_2_Y,  400,     1400,   0,        900    , 640,    900,  ZONE_PRIO_80, 0 },
   .zones[ZONE_SHIP_OPP_DECK_2]=   {ZONE_TYPE_DECK, OPP_SHIP_DECK_2_X, OPP_SHIP_DECK_2_Y, 3000,     1400,   2600,     900    , 2360,   900,  ZONE_PRIO_0, ZONE_AVOID },

	.zones[ZONE_SHIP_OUR_HOLD]=  {ZONE_TYPE_HOLD, 750,    1700, 400,      2000,   0,        1400    , 750,    1700, ZONE_PRIO_0, ZONE_AVOID },
	.zones[ZONE_SHIP_OPP_HOLD]=  {ZONE_TYPE_HOLD, 2250,   1700, 3000,     2000,   2600,     1400    , 2250,   1700, ZONE_PRIO_0, ZONE_AVOID },

	/* zone to save/restore treasure temporaly */
	.zones[ZONE_SAVE_TREASURE]=  {ZONE_TYPE_SAVE, SAVE_TREASURE_X, SAVE_TREASURE_Y, 900,      1600,   500,      1200    , 900,    1600, ZONE_PRIO_0, ZONE_AVOID },
};


/*************************************************************/

/*                  INIT                                     */

/*************************************************************/
void strat_set_bounding_box(uint8_t type)
{

	strat_infos.area_bbox.x1 = 400 + OBS_CLERANCE -10;
	strat_infos.area_bbox.x2 = 3000 - 400 - OBS_CLERANCE + 10;
	
	strat_infos.area_bbox.y1 = OBS_CLERANCE;
	strat_infos.area_bbox.y2 = 2000 - 44 - OBS_CLERANCE;


   switch(type) {
      case I2C_COLOR_RED:
      	strat_infos.area_bbox.x2 = 3000 - 400 - OBS_CLERANCE - 100;
         break;
      case I2C_COLOR_PURPLE:
	      strat_infos.area_bbox.x1 = 400 + OBS_CLERANCE + 100;
         break;
      default:
         break;
   }

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

	/* XXX default conf */
	//strat_infos.conf.flags |= ENABLE_R2ND_POS;

	strat_infos.step_our_totem_1 = 1;
	strat_infos.step_our_totem_2 = 1;
	strat_infos.step_opp_totem_1 = 1;
	strat_infos.step_opp_totem_2 = 1;

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


char numzone2name[ZONES_MAX + 1][30] = {
[ZONE_TOTEM_OUR_SIDE_1] = "TOTEM_OUR_SIDE_1",
[ZONE_TOTEM_OUR_SIDE_2] = "TOTEM_OUR_SIDE_2",
[ZONE_TOTEM_OPP_SIDE_1] = "TOTEM_OPP_SIDE_1",
[ZONE_TOTEM_OPP_SIDE_2] = "TOTEM_OPP_SIDE_2",
[ZONE_OUR_FLOOR_COIN_1] = "OUR_FLOOR_COIN_1",
[ZONE_OUR_FLOOR_COIN_2] = "OUR_FLOOR_COIN_2",
[ZONE_OUR_FLOOR_COIN_3] = "OUR_FLOOR_COIN_3",
[ZONE_OUR_FLOOR_GOLDBAR] = "OUR_FLOOR_GOLDBAR",
[ZONE_OPP_FLOOR_COIN_1] = "OPP_FLOOR_COIN_1",
[ZONE_OPP_FLOOR_COIN_2] = "_OPP_FLOOR_COIN_2",
[ZONE_OPP_FLOOR_COIN_3] = "OPP_FLOOR_COIN_3",
[ZONE_OPP_FLOOR_GOLDBAR] = "OPP_FLOOR_GOLDBAR",
[ZONE_OUR_BOTTLE_1] = "OUR_BOTTLE_1",
[ZONE_OUR_BOTTLE_2] = "OUR_BOTTLE_2",
[ZONE_OPP_BOTTLE_1] = "OPP_BOTTLE_1",
[ZONE_OPP_BOTTLE_2] = "OPP_BOTTLE_2",
[ZONE_OUR_MAP] = "OUR_MAP",
[ZONE_OPP_MAP] = "OPP_MAP",
[ZONE_MIDDLE_COINS_GROUP] = "MIDDLE_COINS_GROUP",
[ZONE_MIDDLE_FLOOR_GOLDBAR] = "MIDDLE_FLOOR_GOLDBAR",
[ZONE_SHIP_OUR_CAPTAINS_BEDRROM] = "OUR_CAPTAINS_BEDRROM",
[ZONE_SHIP_OUR_HOLD] = "OUR_HOLD",
[ZONE_SHIP_OUR_DECK_2] = "OUR_DECK_2",
[ZONE_SHIP_OUR_DECK_1] = "OUR_DECK_1",
[ZONE_SHIP_OPP_CAPTAINS_BEDRROM] = "OPP_CAPTAINS_BEDRROM",
[ZONE_SHIP_OPP_HOLD] = "OPP_HOLD",
[ZONE_SHIP_OPP_DECK_1] = "OPP_DECK_1",
[ZONE_SHIP_OPP_DECK_2] = "OPP_DECK_2",
[ZONE_SAVE_TREASURE] = "SAVE_TREASURE",
[ZONES_MAX] = "NULL",
};

/* display current information about the state of the game */
void strat_dump_infos(const char *caller)
{
	if (!strat_infos.dump_enabled)
		return;

	printf(PSTR("%s() dump strat infos:\r\n"), caller);

	printf("treasure_in_mouth = %d\r\n", strat_infos.treasure_in_mouth);
	printf("treasure_in_boot = %d\r\n", strat_infos.treasure_in_boot);

	printf("curretn_zone is %s\r\n", numzone2name[strat_infos.current_zone]);
	printf("goto_zone is %s\r\n", numzone2name[strat_infos.goto_zone]);
	printf("last_zone is %s\r\n", numzone2name[strat_infos.last_zone]);

	/* add here print infos */
}

/* init current area state before a match. Dump update user conf
 * here */
void strat_reset_infos(void)
{
	/* bounding box */
	strat_set_bounding_box(mainboard.our_color);

	strat_infos.treasure_in_mouth = 0;
	strat_infos.treasure_in_boot = 0;
		
	strat_infos.current_zone = ZONES_MAX;
	strat_infos.goto_zone = ZONES_MAX;
	strat_infos.last_zone = ZONES_MAX;

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
	dac_mc_set(LEFT_MOTOR, 0);
	dac_mc_set(RIGHT_MOTOR, 0);
	IRQ_UNLOCK(flags);

	/* stop beacon */
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_OPP);
	IRQ_UNLOCK(flags);
	beacon_cmd_beacon_off();

	/* slavespic exit */
   i2c_slavedspic_mode_turbine_blow(0);
   i2c_slavedspic_wait_ready();

	/* turn off other devices (lasers...) */

	/* stop beacon */
	beacon_cmd_beacon_off();
	beacon_cmd_beacon_off();
	beacon_cmd_beacon_off();
	beacon_cmd_beacon_off();
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


/* strat main loop */
uint8_t strat_main(void)
{
	uint8_t err;

#ifdef HOMOLOGATION

	trajectory_d_rel(&mainboard.traj, 300);
	err = wait_traj_end(TRAJ_FLAGS_STD);

	/* */
	err = goto_and_avoid(COLOR_X(strat_infos.zones[ZONE_TOTEM_OUR_SIDE_2].init_x), 
								strat_infos.zones[ZONE_TOTEM_OUR_SIDE_2].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	if (TRAJ_SUCCESS(err))
	err = strat_empty_totem_side(COLOR_X(strat_infos.zones[ZONE_TOTEM_OUR_SIDE_2].x),
									strat_infos.zones[ZONE_TOTEM_OUR_SIDE_2].y, STORE_BOOT, 0);

	/* */
	err = goto_and_avoid(COLOR_X(strat_infos.zones[ZONE_SHIP_OUR_DECK_2].init_x), 
								strat_infos.zones[ZONE_SHIP_OUR_DECK_2].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	if (TRAJ_SUCCESS(err))
	err = strat_save_treasure_generic(COLOR_X(strat_infos.zones[ZONE_SHIP_OUR_DECK_2].x), 
												 strat_infos.zones[ZONE_SHIP_OUR_DECK_2].y);

	/* */

	err = goto_and_avoid(COLOR_X(strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].init_x), 
								strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	if (TRAJ_SUCCESS(err))
	err = strat_empty_totem_side(COLOR_X(strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].x),
									strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].y, STORE_BOOT, 0);
	/* */
	err = goto_and_avoid(COLOR_X(strat_infos.zones[ZONE_SHIP_OUR_DECK_2].init_x), 
								strat_infos.zones[ZONE_SHIP_OUR_DECK_2].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

	if (TRAJ_SUCCESS(err))
	err = strat_save_treasure_generic(COLOR_X(strat_infos.zones[ZONE_SHIP_OUR_DECK_2].x), 
												 strat_infos.zones[ZONE_SHIP_OUR_DECK_2].y);
	/* */
	err = goto_and_avoid(COLOR_X(strat_infos.zones[ZONE_MIDDLE_COINS_GROUP].init_x), 
								strat_infos.zones[ZONE_MIDDLE_COINS_GROUP].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

	if (TRAJ_SUCCESS(err))
	err = strat_pickup_coins_floor(COLOR_X(strat_infos.zones[ZONE_MIDDLE_COINS_GROUP].x), 
											strat_infos.zones[ZONE_MIDDLE_COINS_GROUP].y, GROUP);

	/* */
	err = goto_and_avoid(COLOR_X(strat_infos.zones[ZONE_SHIP_OUR_DECK_2].init_x), 
								strat_infos.zones[ZONE_SHIP_OUR_DECK_2].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

	if (TRAJ_SUCCESS(err))
	err = strat_save_treasure_generic(COLOR_X(strat_infos.zones[ZONE_SHIP_OUR_DECK_2].x), 
												 strat_infos.zones[ZONE_SHIP_OUR_DECK_2].y);

	while(time_get_s() < 89);
	strat_exit();
	return 0;

#else

	strat_begin();

	/* try to empty opp totem */
   trajectory_goto_xy_abs(&mainboard.traj,
								COLOR_X(strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].init_x),
								strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].init_y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

 	err = strat_work_on_zone(ZONE_TOTEM_OPP_SIDE_2);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT, "Work on zone %d fails", ZONE_TOTEM_OPP_SIDE_2);

			strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].flags |= ZONE_CHECKED;
         i2c_slavedspic_mode_turbine_angle(0,200);
         i2c_slavedspic_wait_ready();
         i2c_slavedspic_mode_lift_height(30);
			i2c_slavedspic_wait_ready();
         i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM,I2C_FINGERS_MODE_HOLD,0);
			i2c_slavedspic_wait_ready();
         i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR,I2C_FINGERS_MODE_CLOSE,0);
         i2c_slavedspic_wait_ready();
	}

	strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].flags |= ZONE_CHECKED;

	/* auto-play */
	do{
		err = strat_smart();
	}while((err & END_TIMER) == 0);


	/* end */
   strat_exit();
   return 0;

#endif
}

#endif /* HOST_VERSION */


