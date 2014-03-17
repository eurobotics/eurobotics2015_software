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
 *  Javier Balias Santos <javier@arc-robots.org> and Silvia Santano
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



uint8_t strat_begin(void)
{   
	uint8_t err;
	uint8_t zone_num=0;
	#define ZONES_SEQUENCE_LENGTH 6
	uint8_t zones_sequence[ZONES_SEQUENCE_LENGTH] = 						
	{ZONE_TORCH_1,ZONE_FIRE_1,ZONE_FIRE_2,ZONE_MOBILE_TORCH_1,ZONE_FIRE_3,ZONE_TORCH_3};
	
	if(mainboard.our_color==I2C_COLOR_RED)
	{
		zones_sequence[0]=ZONE_TORCH_4; zones_sequence[1]=ZONE_FIRE_6; zones_sequence[2]=ZONE_FIRE_4; 
		zones_sequence[3]=ZONE_MOBILE_TORCH_2; zones_sequence[4]=ZONE_FIRE_5; zones_sequence[5]=ZONE_TORCH_2; 
	}
	
	for(zone_num=0; zone_num<ZONES_SEQUENCE_LENGTH; zone_num++)
	{
		/* goto zone */
		printf_P(PSTR("Going to zone %s.\r\n"),numzone2name[zones_sequence[zone_num]]);
		strat_dump_infos(__FUNCTION__);
		strat_infos.current_zone=-1;
		strat_infos.goto_zone=zones_sequence[zone_num];
		err = goto_and_avoid(strat_infos.zones[zones_sequence[zone_num]].init_x, strat_infos.zones[zones_sequence[zone_num]].init_y,  TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
		trajectory_a_abs(&mainboard.traj, strat_infos.zones[zones_sequence[zone_num]].init_a);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		strat_infos.last_zone=strat_infos.current_zone;
		strat_infos.goto_zone=-1;
		if (!TRAJ_SUCCESS(err)) {
			strat_infos.current_zone=-1;
			printf_P(PSTR("Can't reach zone %s.\r\n"),numzone2name[zones_sequence[zone_num]]);
		}
		else{
			strat_infos.current_zone=zones_sequence[zone_num];
		}

		/* work on zone */
		strat_dump_infos(__FUNCTION__);
		err = strat_work_on_zone(zones_sequence[zone_num]);
		if (!TRAJ_SUCCESS(err)) {
			printf_P(PSTR("Work on zone %s fails.\r\n"),numzone2name[zones_sequence[zone_num]]);
		}
		else
		{
			// Switch off devices, go back to normal state if anything was deployed
		}

		/* mark the zone as checked */
		strat_infos.zones[zones_sequence[zone_num]].flags |= ZONE_CHECKED;
		printf_P(PSTR("Work on zone %s succeeded!\r\n"),numzone2name[zones_sequence[zone_num]]);
	}
	return err;
}














