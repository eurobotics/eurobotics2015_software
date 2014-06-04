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


/* goto orphan fire */
uint8_t strat_goto_orphan_fire (uint8_t zone_num) 
{
    int16_t robot_x, robot_y, x, y;

    /* depending on robot and fire position */
    position_get_x_s16(&robot_x),
	position_get_y_s16(&robot_y),
    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

    if ((zone_num == ZONE_FIRE_1) || (zone_num == ZONE_FIRE_6)) {
        if (robot_y > y) {
            
            /* infront color */
            switch (zone_num) {
                case ZONE_FIRE_1: color = I2C_COLOR_RED; break;
                case ZONE_FIRE_6: color = I2C_COLOR_YELLOW; break;
                default: color = I2C_COLOR_YELLOW;
            }

            /* y correction */  
            if (color == maindspic.our_color)
                y += DIST_ORPHAN_FIRE_PUSH;
            else
                y += DIST_ORPHAN_FIRE_PULL;

        }
        else {  

            /* infront color */
            switch (zone_num) {
                case ZONE_FIRE_1: color = I2C_COLOR_YELLOW; break;
                case ZONE_FIRE_6: color = I2C_COLOR_RED; break;
                default: color = I2C_COLOR_YELLOW;
            }

            /* y correction */   
            if (color == maindspic.our_color)         
                y -= DIST_ORPHAN_FIRE_PUSH;
            else
                y -= DIST_ORPHAN_FIRE_PULL;
        } 
    }
    else {
        if (robot_x > x) {

            /* infront color */
            switch (zone_num) {
                case ZONE_FIRE_2: color = I2C_COLOR_RED; break;
                case ZONE_FIRE_3: color = I2C_COLOR_YELLOW; break;
                case ZONE_FIRE_4: color = I2C_COLOR_RED; break;
                case ZONE_FIRE_5: color = I2C_COLOR_YELLOW; break;
                default: color = I2C_COLOR_YELLOW;
            }

            /* x correction */  
            if (color == maindspic.our_color)
                x += DIST_ORPHAN_FIRE_PUSH;
            else
                x += DIST_ORPHAN_FIRE_PULL;
        }
        else {
            /* infront color */
            switch (zone_num) {
                case ZONE_FIRE_2: color = I2C_COLOR_YELLOW; break;
                case ZONE_FIRE_3: color = I2C_COLOR_RED; break;
                case ZONE_FIRE_4: color = I2C_COLOR_YELLOW; break;
                case ZONE_FIRE_5: color = I2C_COLOR_RED; break;
                default: color = I2C_COLOR_YELLOW;
            }

            /* x correction */  
            if (color == maindspic.our_color)
                x -= DIST_ORPHAN_FIRE_PUSH;
            else
                x -= DIST_ORPHAN_FIRE_PULL;
        }
    }

    DEBUG (E_USER_STRAT, "fire is %s", color==I2C_COLOR_RED? "R":"Y");

    /* select level of sucker depending on infront color */
    if (color == maindspic.our_color)
        level = I2C_SLAVEDSPIC_LEVEL_FIRE_PUSH_PULL;
    else
        level = I2C_SLAVEDSPIC_LEVEL_FIRE_STANDUP;

    
    /* ready for pickup */
    i2c_slavedspic_wait_ready();
    i2c_slavedspic_mode_ready_for_pickup_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG, level);

    /* go near */
    err = goto_and_avoid_forward (x, y, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
}

/* harvest orphan fires  */
uint8_t strat_harvest_orphan_fire (int16_t x, int16_t y)
{
//#define DEBUG_STRAT_FIRES
#ifdef DEBUG_STRAT_FIRES
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif

    uint8_t err = 0;
	uint16_t old_spdd, old_spda, temp_spdd, temp_spda;
    int16_t d, clean_floor_a_rel;
	uint8_t stick_type;

	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_FAST);
   

	/* turn to infront of fire */
    trajectory_turnto_xy (&mainboard.traj, x, y);
	err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

    /* ready for push/pull */
    i2c_slavedspic_wait_ready();
    i2c_slavedspic_mode_ready_for_pickup_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG, 
                                              I2C_SLAVEDSPIC_LEVEL_FIRE_PUSH_PULL);
    i2c_slavedspic_wait_ready();

    /* push/pull fire */
    if (distance_from_robot (x,y) < DIST_ORPHAN_FIRE_PUSH) {
        d = -DIST_ORPHAN_FIRE_OVER;
        level = I2C_SLAVEDSPIC_LEVEL_FIRE_GROUND_PULL;
    }
    else {
        d = DIST_ORPHAN_FIRE_OVER;
        level = I2C_SLAVEDSPIC_LEVEL_FIRE_GROUND_PUSH;
    }
    trajectory_d_rel(&mainboard.traj, d);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* pickup fire */
    i2c_slavedspic_mode_pickup_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG, level);
    i2c_slavedspic_wait_ready();


    /* XXX set color of fire */
    mainboard.stored_fire_color[slavedspic.nb_stored_fires+1] = mainboard.our_color;

	/* store fire */
    i2c_slavedspic_mode_store_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);

end:
	strat_set_speed(old_spdd, old_spda);	
    strat_limit_speed_enable();
    return err;
}


/* goto torch */
uint8_t strat_goto_torch (uint8_t zone_num)
{
    int16_t x, y;

    /*  position */
    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

    /* init x,y correction */
    if (zone_num == ZONE_TORCH_1)       { x += DIST_ORPHAN_FIRE_PUSH ; y+=5 }
    else if (zone_num == ZONE_TORCH_2)  { x -= DIST_ORPHAN_FIRE_PUSH ; y-=5 }

    else if ((zone_num == ZONE_TORCH_3) ||
             (zone_num == ZONE_TORCH_4))  { x +=5 ; y-=DIST_ORPHAN_FIRE_PUSH }

    /* ready for pickup */
    i2c_slavedspic_wait_ready();
    i2c_slavedspic_mode_ready_for_pickup_torch(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);

    /* go near */
    err = goto_and_avoid (x, y, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
}


/* harvest torch  */
uint8_t strat_harvest_torch (uint8_t zone_num)
{
//#define DEBUG_STRAT_FIRES
#ifdef DEBUG_STRAT_FIRES
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif

    uint8_t err = 0;
	uint16_t old_spdd, old_spda, temp_spdd, temp_spda;
    int16_t d, clean_floor_a_rel;
	uint8_t stick_type;
    int16_t x, int16_t y;

    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_FAST);
   

	/* turn to infront of torch */
    trajectory_turnto_xy (&mainboard.traj, x, y);
	err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

    /* ready for pickup */
    i2c_slavedspic_wait_ready();
    i2c_slavedspic_mode_ready_for_pickup_torch(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);
    i2c_slavedspic_wait_ready();

    /* push/pull fire */
    trajectory_d_rel(&mainboard.traj, DIST_TORCH_OVER);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* pickup fire */
    i2c_slavedspic_mode_pickup_torch(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG, level);
    i2c_slavedspic_wait_ready();

    /* XXX set color of fire */
    if (zone_num == ZONE_TORCH_1 || zone_num == ZONE_TORCH_3)
        mainboard.stored_fire_color[slavedspic.nb_stored_fires+1] = I2C_COLOR_RED;
    else
       mainboard.stored_fire_color[slavedspic.nb_stored_fires+1] = I2C_COLOR_YELLOW;
        
	/* store fire */
    i2c_slavedspic_mode_store_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);

    /* XXX return inside boundinbox */
    trajectory_d_rel(&mainboard.traj, -DIST_TORCH_OVER);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);
    
end:
	strat_set_speed(old_spdd, old_spda);	
    strat_limit_speed_enable();
    return err;
}

/* goto mobile torch */
uint8_t strat_goto_mobile_torch (uint8_t zone_num)
{
    int16_t robot_x, robot_y, x, y;

    /* depending on robot and mobiel torch position */
    position_get_x_s16(&robot_x),
	position_get_y_s16(&robot_y),
    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

    /* 3 init points depending on robot position */

    /* top */
    x -= MOBIL_TORCH_X_OFFSET;
    y += DIST_MOBIL_TORCH_OVER;

    /* bottom-right */
    if ((robot_y < y) && (robot_x > x))
        rotate(x, y, RAD(-120));

    /* bottom-left */
    else
        rotate(x, y, RAD(120));

    /* ready for pickup */
    i2c_slavedspic_wait_ready();
    i2c_slavedspic_mode_ready_for_pickup_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG,
                                              I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_TOP);

    /* go near */
    err = goto_and_avoid (x, y, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
}

/* pickup mobile torch fires */
static uint8_t __strat_pickup_mobile_torch (uint8_t zone_num, uint8_t level)
{
//#define DEBUG_STRAT_FIRES
#ifdef DEBUG_STRAT_FIRES
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif

    uint8_t err = 0;
	uint16_t old_spdd, old_spda, temp_spdd, temp_spda;
    int16_t d, clean_floor_a_rel;
	uint8_t stick_type;
    int16_t x, int16_t y;

    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_FAST);
   

	/* turn to infront of torch */
    trajectory_turnto_xy (&mainboard.traj, x, y);
	err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

    /* ready for pickup */
    i2c_slavedspic_wait_ready();

    /* pickup fire */
    i2c_slavedspic_mode_pickup_torch(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG, level);
    i2c_slavedspic_wait_ready();

    /* XXX set color of fire */
    if (zone_num == ZONE_M_TORCH_1) 
    {
        if (level == I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_MID) 
             color = (mainboard.our_color == I2C_COLOR_YELLOW? I2C_COLOR_RED:I2C_COLOR_YELLOW);
        else color = (mainboard.our_color == I2C_COLOR_YELLOW? I2C_COLOR_YELLOW:I2C_COLOR_RED);
    }
    else {
        if (level == I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_MID) 
             color = (mainboard.our_color == I2C_COLOR_YELLOW? I2C_COLOR_YELLOW:I2C_COLOR_RED);
        else color = (mainboard.our_color == I2C_COLOR_YELLOW? I2C_COLOR_RED:I2C_COLOR_YELLOW);
    }

    mainboard.stored_fire_color[slavedspic.nb_stored_fires+1] = fire_color;

	/* store fire */
    i2c_slavedspic_mode_store_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);

end:
	strat_set_speed(old_spdd, old_spda);	
    strat_limit_speed_enable();
    return err;
}

/* pickup mobile torch, top fire */
inline uint8_t strat_pickup_mobile_torch_top (uint8_t zone_num) {
    return __strat_pickup_mobile_torch (zone_num, I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_TOP);
}

/* pickup mobile torch, middle fire */
inline uint8_t strat_pickup_mobile_torch_mid (uint8_t zone_num) {
    return __strat_pickup_mobile_torch (zone_num, I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_MIDDLE);
}

/* pickup mobile torch, bottom fire */
inline uint8_t strat_pickup_mobile_torch_bot (uint8_t zone_num) {
    return __strat_pickup_mobile_torch (zone_num, I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_DOWN);
}


