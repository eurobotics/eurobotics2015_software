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
 *  Javier Bali�as Santos <javier@arc-robots.org> and Silvia Santano
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

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
#include "bt_protocol.h"


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)


/* Add here the main strategy, the intelligence of robot */

void strat_update_priorities(int num_args, ...)
{
	va_list arg_list;
	int num_zone,i;
	uint8_t priority;

	va_start(arg_list, num_args);
	DEBUG(E_USER_STRAT,"num_args %d",num_args);
	for(i=0,priority=200;  i < num_args;  i++,priority-=10)
	//for (num_zone = num_args, priority=10; num_zone != 0; )
	{
		num_zone = va_arg(arg_list, int);
		strat_infos.zones[(uint8_t) num_zone].prio = priority;

		DEBUG(E_USER_STRAT,"zone %s : %d", get_zone_name((uint8_t) num_zone),strat_infos.zones[(uint8_t) num_zone].prio);
		//strat_infos.zones[num_zone].robot=robot;
	}

	va_end(arg_list);
}
/* set next SEC_ROBOT strategy */
void strat_set_next_sec_strategy(void)
{
	switch(strat_infos.match_strategy){
		case STR_HOMOLOGATION:
			strat_change_sequence_homologation(SEC_ROBOT);
			break;
		case STR_BASE:
			strat_change_sequence_base(SEC_ROBOT);
			break;
		default:
			break;
	}
	DEBUG(E_USER_STRAT,"R2, NEW sequence #%d", strat_smart[MAIN_ROBOT].current_strategy);
}

/* set next MAIN_ROBOT strategy */
void strat_set_next_main_strategy(void)
{
	switch(strat_infos.match_strategy){
		case STR_HOMOLOGATION:
			strat_change_sequence_homologation(MAIN_ROBOT);
			break;
		case STR_BASE:
			strat_change_sequence_base(MAIN_ROBOT);
			break;
		default:
			break;
	}
	DEBUG(E_USER_STRAT,"R1, NEW sequence #%d", strat_smart[MAIN_ROBOT].current_strategy);
}

void strat_change_sequence_homologation(uint8_t robot){
	if(robot == MAIN_ROBOT){
		switch(strat_smart[robot].current_strategy){
				default:
					strat_update_priorities(6,ZONE_MY_STAND_GROUP_1, ZONE_MY_CLAP_2,ZONE_POPCORNCUP_2,ZONE_MY_STAND_GROUP_2,
											ZONE_MY_CLAP_1,ZONE_MY_HOME_SPOTLIGHT);
					break;
			}
	}
	else{
		switch(strat_smart[robot].current_strategy){
				default:
					strat_update_priorities(3,ZONE_MY_HOME_OUTSIDE,ZONE_POPCORNCUP_1,ZONE_MY_CLAP_3);
					break;
			}
	}
}

void strat_change_sequence_base(uint8_t robot){
	if(robot == MAIN_ROBOT){

		strat_smart[robot].current_strategy ++;
		switch(strat_smart[robot].current_strategy){
				case 1:
					strat_update_priorities(9,ZONE_MY_STAND_GROUP_1, ZONE_MY_CLAP_2, ZONE_POPCORNCUP_2, 
											  ZONE_MY_STAND_GROUP_2, ZONE_MY_CLAP_1, ZONE_MY_STAND_GROUP_4,
											  ZONE_MY_STAND_GROUP_3, ZONE_MY_HOME_POPCORNS, ZONE_MY_HOME_SPOTLIGHT);
					break;
#if 0
				case 2:
					strat_update_priorities(9,ZONE_MY_STAND_GROUP_1,ZONE_MY_CLAP_2,ZONE_POPCORNCUP_2,ZONE_MY_STAND_GROUP_2,ZONE_MY_CLAP_1,ZONE_MY_STAND_GROUP_3,
											ZONE_MY_POPCORNMAC,ZONE_MY_STAND_GROUP_4,ZONE_MY_HOME);
#endif

				default:
					break;
			}
	}
	else{
		strat_smart[robot].current_strategy ++;
		switch(strat_smart[robot].current_strategy){
				case 1:
					strat_update_priorities(6,ZONE_MY_HOME_OUTSIDE, ZONE_POPCORNCUP_1, ZONE_MY_CLAP_3, ZONE_MY_CINEMA_DOWN, ZONE_MY_CINEMA_UP, ZONE_MY_STAIRWAY);
					break;
				case 2:
					strat_update_priorities(6,ZONE_MY_HOME_OUTSIDE, ZONE_POPCORNCUP_1, ZONE_MY_CINEMA_UP, ZONE_MY_CLAP_3, ZONE_MY_CINEMA_DOWN, ZONE_MY_STAIRWAY);
					break;
				case 3:
					strat_update_priorities(6,ZONE_MY_HOME_OUTSIDE, ZONE_POPCORNCUP_1, ZONE_MY_CINEMA_DOWN, ZONE_MY_CINEMA_UP, ZONE_MY_CLAP_3, ZONE_MY_STAIRWAY);;
					strat_smart[robot].current_strategy=0;
					break;
				default:
					break;
			}
	}
}
