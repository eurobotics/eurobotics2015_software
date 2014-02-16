/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
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

#include <uart.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <trajectory_manager_utils.h>
//#include <trajectory_manager_core.h>
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
#include "cmdline.h"


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)


/* Add here the main strategic, the inteligence of robot */


/* return 1 if is a valid zone and 0 otherwise */
uint8_t strat_is_valid_zone(uint8_t zone_num)
{
#define OPP_WAS_IN_ZONE_TIMES	

	//static uint16_t opp_times[ZONES_MAX];
	//static microseconds opp_time_us = 0;

	/* discard actual zone */
	if(strat_infos.current_zone == zone_num)
		return 0;

	/* discard down side zones depends on strat config */
	if((strat_infos.conf.flags & ENABLE_DOWN_SIDE_ZONES) == 0 
		&& strat_infos.zones[zone_num].init_y < (AREA_Y/2)
		&& zone_num != ZONE_SHIP_OUR_DECK_2
		&& zone_num != ZONE_SHIP_OUR_DECK_1 )
		return 0;
/*
	else if((strat_infos.conf.flags & ENABLE_DOWN_SIDE_ZONES) 
		&& strat_infos.zones[zone_num].init_y > (AREA_Y/2) 
		&& zone_num != ZONE_SHIP_OUR_DECK_2
		&& zone_num != ZONE_SHIP_OUR_DECK_1 )
		return 0;
*/
	/* discard if opp is in zone */
	if(opponent_is_in_area(	COLOR_X(strat_infos.zones[zone_num].x_up),strat_infos.zones[zone_num].y_up,
								COLOR_X(strat_infos.zones[zone_num].x_down),	strat_infos.zones[zone_num].y_down)) {

#if 0
		if(time_get_us2() - opp_time_us < 100000UL)
		{
			opp_time_us = time_get_us2();

			opp_times[zone_num]++;
			if(opp_times[zone_num] > OPP_WAS_IN_ZONE_TIMES)
				strat_infos.zones[zone_num].flags |= ZONE_CHECKED_OPP;
		}
#endif
		DEBUG(E_USER_STRAT, "Discarded zone %s, opp inside", numzone2name[zone_num]);
		return 0;
	}

	/* discard our checked zones TODO*/
	//if(strat_infos.zones[zone_num].flags & ZONE_CHECKED)
	//	return 0;	

	/* discard opp checked zones TODO*/
	/*if(strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP)
		return 0;*/	

	/* discard avoid zones */
	if(strat_infos.zones[zone_num].flags & ZONE_AVOID)
		return 0;	
		
	return 1;
}

/* return new work zone, -1 if any zone is found */
int8_t strat_get_new_zone(void)
{
	uint16_t prio_max = 0;
	int8_t zone_num = -1;
	int8_t i;
	
	/* evaluate zones */
	for(i=0; i < ZONES_MAX; i++) 
	{
		/* check if is a valid zone */
		if(!strat_is_valid_zone(i))	
			continue;

		/* check priority */
		if(strat_infos.zones[i].prio > prio_max) {

			/* update zone candidate params */
			prio_max = strat_infos.zones[i].prio;
			zone_num = i;
		}
	}

	return zone_num;
}

/* return END_TRAJ if zone is reached, err otherwise */
uint8_t strat_goto_zone(uint8_t zone_num)
{
	int8_t err;

	err = goto_and_avoid(COLOR_X(strat_infos.zones[zone_num].init_x), 
				  strat_infos.zones[zone_num].init_y,  TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	
	return err;
}


/* return END_TRAJ if the work is done, err otherwise */
uint8_t strat_work_on_zone(uint8_t zone_num)
{
	uint8_t err = END_TRAJ;
	int16_t x = strat_infos.zones[zone_num].x;
	int16_t y = strat_infos.zones[zone_num].y;

	static uint8_t first_time = 1;

#ifdef DEBUG_STRAT_SMART
	return END_TRAJ;
#endif
	
#ifdef HOST_VERSION
	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
#else
	/* TODO */
#endif 
	
	return err;

}

/* debug state machines step to step */
void state_debug_wait_key_pressed(void)
{
	if (!strat_infos.debug_step)
		return;

	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* smart play */
uint8_t strat_smart(void)
{
	int8_t zone_num;
	uint8_t err;

	/* XXX DEBUG STEP BY STEP */
	//state_debug_wait_key_pressed();


	// If last seconds of the match
	/*if(time_get_s() > LAST_SECONDS_TIME)
	...
	*/
	
	
	/* get new zone */
	zone_num = strat_get_new_zone();

		
	if(zone_num == -1) {
		DEBUG(E_USER_STRAT, "No zone is found");
		DEBUG(E_USER_STRAT, "Down side zones enabled");	
		strat_infos.conf.flags |= ENABLE_DOWN_SIDE_ZONES;	
		strat_infos.conf.flags |= ENABLE_R2ND_POS;

		/* Enable all paths */
		DEBUG(E_USER_STRAT, "All possible paths enabled");	
		strat_set_bounding_box(-1);	
		return END_TRAJ;
	}

	/* goto zone */
	strat_infos.goto_zone = zone_num;
	strat_dump_infos(__FUNCTION__);
	
	err = strat_goto_zone(strat_infos.goto_zone);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT, "Can't reach zone %d", strat_infos.goto_zone);
		return END_TRAJ;
	}

	/* work on zone */
	strat_infos.last_zone = strat_infos.current_zone;
	strat_infos.current_zone = strat_infos.goto_zone;
	strat_dump_infos(__FUNCTION__);

	err = strat_work_on_zone(strat_infos.current_zone);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT, "Work on zone %d fails", strat_infos.current_zone);

		// Switch off devices, go back to normal state if anything was deployed
		return err;
	}

	/* mark the zone as checked */
		strat_infos.zones[strat_infos.current_zone].flags |= ZONE_CHECKED;
	}

	DEBUG(E_USER_STRAT, "Work on zone %d successed!", strat_infos.current_zone);
	return END_TRAJ;
}


