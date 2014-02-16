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

#ifndef HOST_VERSION_OA_TEST
#include <uart.h>
#include <dac_mc.h>
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
#include "strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"

#else

#ifndef IM_SECONDARY_ROBOT
/* robot dimensions */
#define ROBOT_LENGTH     281.5
#define ROBOT_WIDTH 	   330.0

#define ROBOT_CENTER_TO_FRONT 162.5
#define ROBOT_CENTER_TO_BACK  119.0

#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/* XXX obstacle clerance */
#define OBS_CLERANCE       242.

#else
/* robot dimensions */
#define IM_SECONDARY_ROBOT
#define ROBOT_LENGTH      150.
#define ROBOT_WIDTH 	    230.

#define ROBOT_CENTER_TO_FRONT 75.0
#define ROBOT_CENTER_TO_BACK  75.0

#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/* XXX obstacle clerance */
#define OBS_CLERANCE       147.

#endif /* ! IM_SECONDARY_ROBOT */


#endif


static volatile uint8_t strat_running = 0;


struct strat_infos strat_infos = { 
	/* conf */
	.conf = {
		.flags = 0,
	},

   /*zones[W] =			     			{type, x, x_down, x_up, y_down, y_up, init_x,init_y, prio, flags };                            */
   .zones[ZONE_TREE_1]=        {ZONE_TYPE_TREE, TREE_1_X, TREE_1_Y, 2850, 3150,  1150,    1450   , 2600,  1300,   ZONE_PRIO_40, 0 },
   .zones[ZONE_TREE_2]=        {ZONE_TYPE_TREE, TREE_2_X, TREE_2_Y, 2150, 2450,  1850,    2150   , 2300,  1600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_TREE_3]=        {ZONE_TYPE_TREE, TREE_3_X, TREE_3_Y, 550, 850,  1850,    2150   , 700,  1600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_TREE_4]=        {ZONE_TYPE_TREE, TREE_4_X, TREE_4_Y, -150, 150,  1150,    1450   , 400,  1300,   ZONE_PRIO_40, 0 },
    .zones[ZONE_HEART_FIRE_1]=  {ZONE_TYPE_HEART_FIRE, HEART_FIRE_1_X, HEART_FIRE_1_Y, 2750, 3000,  1750, 2000   , 2600,  1500,   ZONE_PRIO_40, 0 },
    .zones[ZONE_HEART_FIRE_2]=  {ZONE_TYPE_HEART_FIRE, HEART_FIRE_2_X, HEART_FIRE_2_Y, 0, 250,  1750,   2000   , 1100,  1050,   ZONE_PRIO_40, 0 },
     .zones[ZONE_HEART_FIRE_3]= {ZONE_TYPE_HEART_FIRE, HEART_FIRE_3_X, HEART_FIRE_3_Y, 1350, 1650,  900,    1200   , 400,  1500,   ZONE_PRIO_40, 0 },
     .zones[ZONE_FIRE_1]=       {ZONE_TYPE_FIRE, FIRE_1_X, FIRE_1_Y, 2530, 2670,  1085,  1115   , 2600,  700,   ZONE_PRIO_40, 0 },
    .zones[ZONE_FIRE_2]=        {ZONE_TYPE_FIRE, FIRE_2_X, FIRE_2_Y, 2085, 2115,  530,    670   , 2500,  600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_FIRE_3]=        {ZONE_TYPE_FIRE, FIRE_3_X, FIRE_3_Y, 2085, 2115,  1530,   1670   , 2500,  1600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_FIRE_4]=        {ZONE_TYPE_FIRE, FIRE_4_X, FIRE_4_Y, 885, 915,  530,    670   , 500,  600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_FIRE_5]=        {ZONE_TYPE_FIRE, FIRE_5_X, FIRE_5_Y, 885, 915,  1530,    1670   , 500,  1600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_FIRE_6]=        {ZONE_TYPE_FIRE, FIRE_6_X, FIRE_6_Y, 330, 470,  1085,    1115   , 400,  700,   ZONE_PRIO_40, 0 },
    .zones[ZONE_TORCH_1]=        {ZONE_TYPE_TORCH, TORCH_1_X, TORCH_1_Y, 2985, 3000,  730,    870   , 2600,  800,   ZONE_PRIO_40, 0 },
    .zones[ZONE_TORCH_2]=        {ZONE_TYPE_TORCH, TORCH_2_X, TORCH_2_Y, 1630, 1770,  1985,    2000   , 1700,  1600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_TORCH_3]=        {ZONE_TYPE_TORCH, TORCH_3_X, TORCH_3_Y, 1230, 1370,  1985,    2000   , 1300,  1600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_TORCH_4]=        {ZONE_TYPE_TORCH, TORCH_4_X, TORCH_4_Y, 0, 15,  900,    730   , 400,  800,   ZONE_PRIO_40, 0 },
    .zones[ZONE_MOBILE_TORCH_1]= {ZONE_TYPE_MOBILE_TORCH, MOBILE_TORCH_1_X, MOBILE_TORCH_1_Y, 820, 980,  1020,   1180   , 2100,  700,   ZONE_PRIO_40, 0 },
    .zones[ZONE_MOBILE_TORCH_2]= {ZONE_TYPE_MOBILE_TORCH, MOBILE_TORCH_2_X, MOBILE_TORCH_2_Y, 2020, 2180,  1020, 1180   , 900,  700,   ZONE_PRIO_40, 0 },
    .zones[ZONE_BASKET_1]=        {ZONE_TYPE_BASKET, BASKET_1_X, BASKET_1_Y, 1900, 2600,  0,    300   , 2300,  600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_BASKET_2]=        {ZONE_TYPE_BASKET, BASKET_2_X, BASKET_2_Y, 400, 1100,  0,    300   , 700,  600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_MAMOOTH_1]=       {ZONE_TYPE_MAMOOTH, MAMOOTH_1_X, MAMOOTH_1_Y, 1900, 2600,  0,    300   , 2300,  600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_MAMOOTH_2]=       {ZONE_TYPE_MAMOOTH, MAMOOTH_2_X, MAMOOTH_2_Y, 400, 1100,  0,    300   , 700,  600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_FRESCO]=          {ZONE_TYPE_FRESCO, FRESCO_X, FRESCO_Y, 1100, 1900,  0,    300   , 1500,  300,   ZONE_PRIO_40, 0 },
    .zones[ZONE_HOME_RED]=        {ZONE_TYPE_HOME, HOME_RED_X, HOME_RED_Y, 0, 400,  0,    690   , 2800,  600,   ZONE_PRIO_40, 0 },
    .zones[ZONE_HOME_YELLOW]=     {ZONE_TYPE_HOME, HOME_YELLOW_X, HOME_YELLOW_Y, 2600, 3000,  0,   690   , 200,  600,   ZONE_PRIO_40, 0 },
};


/*************************************************************/

/*                  INIT                                     */

/*************************************************************/
void strat_set_bounding_box(uint8_t type)
{

	strat_infos.area_bbox.x1 = OBS_CLERANCE;
	strat_infos.area_bbox.x2 = 3000 - OBS_CLERANCE;
	
	strat_infos.area_bbox.y1 = OBS_CLERANCE + 300;
	strat_infos.area_bbox.y2 = 2000 - OBS_CLERANCE;

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
							 DO_POS | DO_BD | DO_POWER | DO_OPP;

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


char numzone2name[ZONES_MAX + 1][30] = {
[ZONE_TREE_1]="TREE_1",
[ZONE_TREE_2]="TREE_2",
[ZONE_TREE_3]="TREE_3",
[ZONE_TREE_4]="TREE_4",
[ZONE_HEART_FIRE_1]="HEART_FIRE_1",
[ZONE_HEART_FIRE_2]="HEART_FIRE_2",
[ZONE_HEART_FIRE_3]="HEART_FIRE_3",
[ZONE_FIRE_1]="FIRE_1",
[ZONE_FIRE_2]="FIRE_2",
[ZONE_FIRE_3]="FIRE_3",
[ZONE_FIRE_4]="FIRE_4",
[ZONE_FIRE_5]="FIRE_5",
[ZONE_FIRE_6]="FIRE_6",
[ZONE_TORCH_1]="TORCH_1",
[ZONE_TORCH_2]="TORCH_2",
[ZONE_TORCH_3]="TORCH_3",
[ZONE_TORCH_4]="TORCH_4",
[ZONE_MOBILE_TORCH_1]="MOBILE_TORCH_1",
[ZONE_MOBILE_TORCH_2]="MOBILE_TORCH_2",
[ZONE_MOBILE_TORCH_3]="MOBILE_TORCH_3",
[ZONE_BASKET_1]="BASKET_1",
[ZONE_BASKET_2]="BASKET_2",
[ZONE_MAMOOTH_1]="MAMOOTH_1",
[ZONE_MAMOOTH_2]="MAMOOTH_2",
[ZONE_FRESCO]="FRESCO",
[ZONE_HOME_RED]="HOME_RED",
[ZONE_HOME_YELLOW]="HOME_YELLOW",
[ZONES_MAX] = "NULL",
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
#ifndef HOST_VERSION
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_CS);
	IRQ_UNLOCK(flags);
	dac_mc_set(LEFT_MOTOR, 0);
	dac_mc_set(RIGHT_MOTOR, 0);
#endif

	/* stop beacon */
	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_OPP);
	IRQ_UNLOCK(flags);
	beacon_cmd_beacon_off();

	/* slavespic exit TODO 2014 */
  //i2c_slavedspic_mode_turbine_blow(0);
  //i2c_slavedspic_wait_ready();

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

	/* ignore when strat is not running */
	if (strat_running == 0)
		return;

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
#if notyet /* TODO 2014 */
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



#endif /* HOMOLOGATION */

#endif /* notyet TODO 2014 */

	/* end */
   strat_exit();
   return 0;
}

#endif /* HOST_VERSION_OA_TEST */


