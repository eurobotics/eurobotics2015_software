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



#ifdef 2012
uint8_t strat_begin(void)
{   
   uint8_t err;
	uint16_t old_spdd, old_spda;
   uint32_t old_var_2nd_ord_pos, old_var_2nd_ord_neg;
	int16_t posx, posy, posa;
#ifdef CATCH_GOLDBAR
	uint8_t n=0;
	int8_t nb_tries;
#endif

   #define LONG_DISTANCE        5000
   #define Y_BEGIN_CURVE        842
   #define X_BEGIN_CURVE_HOME   325
   #define X_END_CURVE          1150


   DEBUG(E_USER_STRAT, "Start");
   strat_limit_speed_disable();

	/* save speed and aceleration values */
	strat_get_speed(&old_spdd, &old_spda);
   old_var_2nd_ord_pos = mainboard.angle.qr.var_2nd_ord_pos;
   old_var_2nd_ord_neg = mainboard.angle.qr.var_2nd_ord_neg;

   /* modify speed and quadramp */
	strat_set_speed(4000,2000);
   quadramp_set_2nd_order_vars(&mainboard.angle.qr, 20, 20);

#ifdef CATCH_GOLDBAR
	/* prepare fingers */
	i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_GOLDBAR_FLOOR);
#endif

	/* robot describes curve trajectory until goldbar */
   trajectory_d_rel(&mainboard.traj, LONG_DISTANCE);
   err = WAIT_COND_OR_TRAJ_END(x_is_more_than(X_BEGIN_CURVE_HOME), TRAJ_FLAGS_STD);
   if(err) {
   	if (!TRAJ_SUCCESS(err))
			ERROUT(err);   	} 
	else DEBUG(E_USER_STRAT, "X is more than X_BEGIN_CURVE_HOME");
		
#ifdef CATCH_GOLDBAR
	(mainboard.our_color==I2C_COLOR_YELLOW)?
		i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR_RIGHT,I2C_FINGERS_MODE_OPEN,-50) :
		i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR_LEFT,I2C_FINGERS_MODE_OPEN,-50);
#endif

   trajectory_only_a_abs(&mainboard.traj, COLOR_A_ABS(90));
   err = WAIT_COND_OR_TRAJ_END(y_is_more_than(Y_BEGIN_CURVE), END_OBSTACLE|END_BLOCKING|END_INTR);
   if(err) {
   	if (!TRAJ_SUCCESS(err))
			ERROUT(err);  	}
	else DEBUG(E_USER_STRAT, "Y is more than Y_BEGIN_CURVE");

#ifdef CATCH_GOLDBAR
	/* prepare fingers */
	(mainboard.our_color==I2C_COLOR_YELLOW)?
		i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR_LEFT,I2C_FINGERS_MODE_OPEN,120):
		i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR_RIGHT,I2C_FINGERS_MODE_OPEN,120);
#endif

   trajectory_only_a_abs(&mainboard.traj, COLOR_A_ABS(0));
   err=WAIT_COND_OR_TRAJ_END(x_is_more_than(X_END_CURVE), TRAJ_FLAGS_NO_NEAR);
	strat_hardstop();
   if(err) {
   	if (!TRAJ_SUCCESS(err))
			ERROUT(err);   	}  
	else DEBUG(E_USER_STRAT, "X is more than X_END_CURVE");


	DEBUG(E_USER_STRAT, "Correct position error");
	//trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(0));
	//err=wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
  	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);   	
	posx=position_get_x_s16(&mainboard.pos);
	posy=position_get_y_s16(&mainboard.pos);
	posa=position_get_a_deg_s16(&mainboard.pos);
	strat_reset_pos(posx+COLOR_SIGN(-20),posy+20,posa+COLOR_SIGN(3));

	//trajectory_goto_xy_abs(&mainboard.traj,COLOR_X(1900), 1610);
	//err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);		


#ifdef CATCH_GOLDBAR
	/*catch goldbar */
	DEBUG(E_USER_STRAT, "Catch goldbar");
	(mainboard.our_color==I2C_COLOR_YELLOW)?
		i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR_LEFT,I2C_FINGERS_MODE_OPEN,0):
		i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR_RIGHT,I2C_FINGERS_MODE_OPEN,0);

	trajectory_a_rel(&mainboard.traj, COLOR_A_REL(-40));
	err=wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
  	if (!TRAJ_SUCCESS(err))
		ERROUT(err);   	
	trajectory_d_rel(&mainboard.traj, 50);
	err=wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
  	if (!TRAJ_SUCCESS(err))
		ERROUT(err);   	

	i2c_slavedspic_wait_ready();
	DEBUG(E_USER_STRAT, "blockings: %d %d", slavedspic.fingers_floor_blocked, slavedspic.fingers_totem_blocked);

	nb_tries = 2;

retry:
	DEBUG(E_USER_STRAT, "First finger");
	(mainboard.our_color==I2C_COLOR_YELLOW)?
		i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR_LEFT,I2C_FINGERS_MODE_HOLD,0):
		i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR_RIGHT,I2C_FINGERS_MODE_HOLD,0);

	i2c_slavedspic_wait_ready();
	DEBUG(E_USER_STRAT, "blockings: %d %d", slavedspic.fingers_floor_blocked, slavedspic.fingers_totem_blocked);

	//time_wait_ms(100);
	DEBUG(E_USER_STRAT, "Second finger");
	(mainboard.our_color==I2C_COLOR_YELLOW)?
		i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR_RIGHT,I2C_FINGERS_MODE_CLOSE,0):
		i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR_LEFT,I2C_FINGERS_MODE_CLOSE,0);

	do{
		DEBUG(E_USER_STRAT, "Harvest goldbar floor");
		i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_GOLDBAR_FLOOR);
	   i2c_slavedspic_wait_ready();
		DEBUG(E_USER_STRAT, "Store goldbar in boot");
		i2c_slavedspic_mode_store(1,I2C_STORE_MODE_GOLDBAR_IN_BOOT);
	   i2c_slavedspic_wait_ready();
		//time_wait_ms(2000);
		n++;
		DEBUG(E_USER_STRAT, "Nb of goldbars in boot: %d", slavedspic.nb_goldbars_in_boot);
	}while((slavedspic.nb_goldbars_in_boot==0) && (n<2));

	if((slavedspic.fingers_floor_blocked || slavedspic.fingers_totem_blocked) && nb_tries) {

		nb_tries --;

		trajectory_d_rel(&mainboard.traj, -50);
		err=wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	  	if (!TRAJ_SUCCESS(err))
			ERROUT(err);   	

		i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_GOLDBAR_FLOOR);
		i2c_slavedspic_wait_ready();

		trajectory_a_rel(&mainboard.traj, COLOR_A_REL(-40));
		err=wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	  	if (!TRAJ_SUCCESS(err))
			ERROUT(err);   	

		goto retry;
	}

	if(!slavedspic.nb_goldbars_in_boot) {
		err = strat_save_treasure_generic(COLOR_X(1500), 1700);
	  	if (!TRAJ_SUCCESS(err))
			ERROUT(err);   	
	}
#endif /* CATCH_GOLDBAR */

end:
	if(slavedspic.nb_goldbars_in_boot)	
		strat_infos.treasure_in_boot =1;

   DEBUG(E_USER_STRAT, "End (%d goldbars catched)", slavedspic.nb_goldbars_in_boot);
   /* restore speed and quadramp values */
	strat_set_speed(old_spdd, old_spda);
   strat_limit_speed_enable();
   quadramp_set_2nd_order_vars(&mainboard.angle.qr,old_var_2nd_ord_pos,old_var_2nd_ord_neg);

   return err;
}
#endif 2012

