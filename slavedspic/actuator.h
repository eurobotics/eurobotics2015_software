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
 *  Javier Baliñas Santos <javier@arc-robots.org>
 */


#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include <aversive.h>
#include <aversive/error.h>
#include <clock_time.h>


#define old_version

#define STANDS_EXCHANGER_SPEED				10000
#define STANDS_EXCHANGER_ACCEL				0
#define STANDS_EXCHANGER_K_IMP_mm			1
#define STANDS_EXCHANGER_CALIB_IMP_MAX		0
#define STANDS_EXCHANGER_POSITION_MAX_mm	16000L
#define STANDS_EXCHANGER_POSITION_MIN_mm	-16000L

#define POS_POPCORN_TRAY_OPEN		0
#define POS_POPCORN_TRAY_CLOSE		980

#define POS_STANDS_CLAMP_L_OPEN		700
#define POS_STANDS_CLAMP_L_CLOSE	500

#define POS_STANDS_CLAMP_R_OPEN		310
#define POS_STANDS_CLAMP_R_CLOSE	550

#define POS_STANDS_TOWER_CLAMPS_UP_LOCK				500
#define POS_STANDS_TOWER_CLAMPS_UP_UNLOCK_LEFT		680
#define POS_STANDS_TOWER_CLAMPS_UP_UNLOCK_RIGHT		370


#define POS_STANDS_TOWER_CLAMPS_DOWN_LOCK			443
#define POS_STANDS_TOWER_CLAMPS_DOWN_UNLOCK_LEFT	600
#define POS_STANDS_TOWER_CLAMPS_DOWN_UNLOCK_RIGHT	310

#define POS_STANDS_ELEVATOR_L_UP	300
#define POS_STANDS_ELEVATOR_L_DOWN	860

#define POS_STANDS_ELEVATOR_R_UP	800
#define POS_STANDS_ELEVATOR_R_DOWN	190

#define ANG_STANDS_BLADE_L_HIDE_LEFT			-80
#define ANG_STANDS_BLADE_L_PUSH_STAND_LEFT		-38
#define ANG_STANDS_BLADE_L_CENTER				0
#define ANG_STANDS_BLADE_L_PUSH_STAND_RIGHT		38
#define ANG_STANDS_BLADE_L_HIDE_RIGHT			80
#define POS_OFFSET_STANDS_BLADE_L				356

#define ANG_STANDS_BLADE_R_HIDE_LEFT			-80
#define ANG_STANDS_BLADE_R_PUSH_STAND_LEFT		-38
#define ANG_STANDS_BLADE_R_CENTER				0
#define ANG_STANDS_BLADE_R_PUSH_STAND_RIGHT		38
#define ANG_STANDS_BLADE_R_HIDE_RIGHT			80
#define POS_OFFSET_STANDS_BLADE_R				435

#define POS_CUP_CLAMP_L_HIDE			729
#define POS_CUP_CLAMP_L_LOCKED			512
#define POS_CUP_CLAMP_L_OPEN			240
#define POS_POPCORN_DOOR_L_CLOSE		POS_CUP_CLAMP_L_HIDE
#define POS_POPCORN_DOOR_L_OPEN			140

#define POS_CUP_CLAMP_R_HIDE			366
#define POS_CUP_CLAMP_R_LOCKED			508
#define POS_CUP_CLAMP_R_OPEN			780
#define POS_POPCORN_DOOR_R_CLOSE		POS_CUP_CLAMP_R_HIDE
#define POS_POPCORN_DOOR_R_OPEN			886

#define POS_POPCORN_RAMP_L_HIDE			200
#define POS_POPCORN_RAMP_L_HARVEST		517
#define POS_POPCORN_RAMP_L_OPEN			640

#define POS_POPCORN_RAMP_R_HIDE			815
#define POS_POPCORN_RAMP_R_HARVEST		505
#define POS_POPCORN_RAMP_R_OPEN			385

#define POS_CUP_CLAMP_FRONT_OPEN		350
#define POS_CUP_CLAMP_FRONT_CUP_LOCKED	200

#define POS_CUP_HOLDER_FRONT_CUP_HOLD	184
#define POS_CUP_HOLDER_FRONT_READY		512
#define POS_CUP_HOLDER_FRONT_HIDE		564


/* test end traj */
#define END_TRAJ   		1
#define END_NEAR   		2
#define END_TIME   		4
#define END_BLOCKING   	8



/* init actuators */
void actuator_init(void);

/**** stands_exchanger functions ********************************************************/

/* stop without rampe */
void stands_exchanger_hard_stop(void);

/* calibrate initial position */
void stands_exchanger_calibrate(void);

/* set position in mm */
void stands_exchanger_set_position(int32_t position_mm);

/* return position in mm */
int32_t stands_exchanger_get_position(void);

/* return END_TRAJ or END_BLOCKING */
int8_t stands_exchanger_check_position_reached(void);

/* return END_TRAJ or END_BLOCKING */
uint8_t stands_exchanger_test_traj_end();

/* return END_TRAJ or END_BLOCKING */
uint8_t stands_exchanger_wait_end();



/**** popcorn_tray functions *********************************************************/

typedef struct {
	uint8_t mode;

#define POPCORN_TRAY_MODE_OPEN 		0	
#define POPCORN_TRAY_MODE_CLOSE 	1		
#define POPCORN_TRAY_MODE_MAX		2

#define POPCORN_TRAY_MODE_POS_MIN	0
#define POPCORN_TRAY_MODE_POS_MAX	1

	uint16_t servo_pos;
} popcorn_tray_t;

/* set popcorn_tray position depends on mode */
int8_t popcorn_tray_set_mode(popcorn_tray_t *popcorn_tray, uint8_t mode, int16_t pos_offset);



/****stands_clamps functions *********************************************************/
typedef struct {
	uint8_t type;
#define STANDS_CLAMP_TYPE_LEFT		0
#define STANDS_CLAMP_TYPE_RIGHT		1
#define STANDS_CLAMP_TYPE_MAX		2

	uint8_t mode;
#define STANDS_CLAMP_MODE_OPEN		0
#define STANDS_CLAMP_MODE_CLOSE		1
#define STANDS_CLAMP_MODE_MAX		2

#define STANDS_CLAMP_MODE_L_POS_MAX		0	
#define STANDS_CLAMP_MODE_L_POS_MIN		1
#define STANDS_CLAMP_MODE_R_POS_MAX		1
#define STANDS_CLAMP_MODE_R_POS_MIN		0

	uint16_t servo_pos;
} stands_clamp_t;

/* set stands_clamp position depends on mode */
int8_t stands_clamp_set_mode(stands_clamp_t *stands_clamp, uint8_t mode, int16_t pos_offset);



/**** stands_tower_clamps functions *********************************************************/
typedef struct {
	uint8_t mode;
#define STANDS_TOWER_CLAMPS_MODE_UNLOCK_LEFT		0
#define STANDS_TOWER_CLAMPS_MODE_LOCK				1
#define STANDS_TOWER_CLAMPS_MODE_UNLOCK_RIGHT		2
#define STANDS_TOWER_CLAMPS_MODE_MAX				3

#define STANDS_TOWER_CLAMPS_MODE_UP_POS_MIN			2
#define STANDS_TOWER_CLAMPS_MODE_UP_POS_MAX			0
#define STANDS_TOWER_CLAMPS_MODE_DOWN_POS_MIN		2
#define STANDS_TOWER_CLAMPS_MODE_DOWN_POS_MAX		0

	uint16_t ax12_pos_up;
	uint16_t ax12_pos_down;
} stands_tower_clamps_t;

/* set stands_tower_clamps position depends on mode */
int8_t stands_tower_clamps_set_mode(stands_tower_clamps_t *stands_tower_clamps, uint8_t mode, int16_t pos_offset);

/* return stands_tower_clamps traj flag */
uint8_t stands_tower_clamps_test_traj_end(stands_tower_clamps_t *stands_tower_clamps);

/* return END_TRAJ or END_TIMER */
uint8_t stands_tower_clamps_wait_end(stands_tower_clamps_t *stands_tower_clamps);



/****stands_elevators functions *********************************************************/
typedef struct {
	uint8_t type;
#define STANDS_ELEVATOR_TYPE_LEFT		0
#define STANDS_ELEVATOR_TYPE_RIGHT		1
#define STANDS_ELEVATOR_TYPE_MAX		2	

	uint8_t mode;
#define STANDS_ELEVATOR_MODE_UP			0
#define STANDS_ELEVATOR_MODE_DOWN		1
#define STANDS_ELEVATOR_MODE_MAX		2

#define STANDS_ELEVATOR_MODE_L_POS_MIN		0
#define STANDS_ELEVATOR_MODE_L_POS_MAX		1	
#define STANDS_ELEVATOR_MODE_R_POS_MIN		1
#define STANDS_ELEVATOR_MODE_R_POS_MAX		0

	uint16_t ax12_pos;
} stands_elevator_t;

/* set stands_elevator position depends on mode */
int8_t stands_elevator_set_mode(stands_elevator_t *stands_elevator, uint8_t mode, int16_t pos_offset);

/* return stands_elevator traj flag */
uint8_t stands_elevator_test_traj_end(stands_elevator_t *stands_elevator);

/* return END_TRAJ or END_TIMER */
uint8_t stands_elevator_wait_end(stands_elevator_t *stands_elevator);



/****stands_blade functions *********************************************************/
typedef struct {
	uint8_t type;
#define STANDS_BLADE_TYPE_LEFT		0
#define STANDS_BLADE_TYPE_RIGHT		1
#define STANDS_BLADE_TYPE_MAX		2	

	uint8_t mode;
#define STANDS_BLADE_MODE_HIDE_LEFT				0
#define STANDS_BLADE_MODE_PUSH_STAND_LEFT		1
#define STANDS_BLADE_MODE_CENTER				2
#define STANDS_BLADE_MODE_PUSH_STAND_RIGHT		3
#define STANDS_BLADE_MODE_HIDE_RIGHT			4
#define STANDS_BLADE_MODE_SET_ANGLE				5
#define STANDS_BLADE_MODE_MAX					6

#define STANDS_BLADE_MODE_L_ANG_MIN		0
#define STANDS_BLADE_MODE_L_ANG_MAX		4	
#define STANDS_BLADE_MODE_R_ANG_MIN		0
#define STANDS_BLADE_MODE_R_ANG_MAX		4

	int8_t ax12_ang;
} stands_blade_t;

/* set stands_blade angle depends on mode */
int8_t stands_blade_set_mode(stands_blade_t *stands_blade, uint8_t mode, int8_t ang_offset);

/* return stands_blade traj flag */
uint8_t stands_blade_test_traj_end(stands_blade_t *stands_blade);

/* return END_TRAJ or END_TIMER */
uint8_t stands_blade_wait_end(stands_blade_t *stands_blade);



/****cup_clamp_popcorn_door functions *********************************************************/
typedef struct {
	uint8_t type;
#define CUP_CLAMP_POPCORN_DOOR_TYPE_LEFT	0
#define CUP_CLAMP_POPCORN_DOOR_TYPE_RIGHT	1
#define CUP_CLAMP_POPCORN_DOOR_TYPE_MAX		2	

	uint8_t mode;
#define CUP_CLAMP_MODE_HIDE					0
#define CUP_CLAMP_MODE_LOCKED				1
#define CUP_CLAMP_MODE_OPEN					2
#define POPCORN_DOOR_MODE_OPEN				3
#define CUP_CLAMP_POPCORN_DOOR_MODE_MAX		4

#define CUP_CLAMP_POPCORN_DOOR_MODE_L_POS_MIN		3
#define CUP_CLAMP_POPCORN_DOOR_MODE_L_POS_MAX		0	
#define CUP_CLAMP_POPCORN_DOOR_MODE_R_POS_MIN		0
#define CUP_CLAMP_POPCORN_DOOR_MODE_R_POS_MAX		3

	uint16_t ax12_pos;
} cup_clamp_popcorn_door_t;

/* set cup_clamp_popcorn_door position depends on mode */
int8_t cup_clamp_popcorn_door_set_mode(cup_clamp_popcorn_door_t *cup_clamp_popcorn_door, uint8_t mode, int16_t pos_offset);

/* set cup_clamp_position depends on mode */
int8_t cup_clamp_set_mode(cup_clamp_popcorn_door_t *cup_clamp_popcorn_door, uint8_t mode, int16_t pos_offset);

/* set popcorn_door position depends on mode */
int8_t popcorn_door_set_mode(cup_clamp_popcorn_door_t *cup_clamp_popcorn_door, uint8_t mode, int16_t pos_offset);

/* return cup_clamp_popcorn_door traj flag */
uint8_t cup_clamp_popcorn_door_test_traj_end(cup_clamp_popcorn_door_t *cup_clamp_popcorn_door);

/* return END_TRAJ or END_TIMER */
uint8_t cup_clamp_popcorn_door_wait_end(cup_clamp_popcorn_door_t *cup_clamp_popcorn_door);



/**** popcorn_ramps functions *********************************************************/
typedef struct {
	uint8_t mode;
#define POPCORN_RAMPS_MODE_HIDE			0
#define POPCORN_RAMPS_MODE_HARVEST		1
#define POPCORN_RAMPS_MODE_OPEN			2
#define POPCORN_RAMPS_MODE_MAX			3

#define POPCORN_RAMPS_MODE_L_POS_MIN		0
#define POPCORN_RAMPS_MODE_L_POS_MAX		2
#define POPCORN_RAMPS_MODE_R_POS_MIN		2
#define POPCORN_RAMPS_MODE_R_POS_MAX		0	

	uint16_t ax12_pos_l;
	uint16_t ax12_pos_r;
} popcorn_ramps_t;

/* set popcorn_ramps position depends on mode */
int8_t popcorn_ramps_set_mode(popcorn_ramps_t *popcorn_ramps, uint8_t mode, int16_t pos_offset);

/* return popcorn_ramps traj flag */
uint8_t popcorn_ramps_test_traj_end(popcorn_ramps_t *popcorn_ramps);

/* return END_TRAJ or END_TIMER */
uint8_t popcorn_ramps_wait_end(popcorn_ramps_t *popcorn_ramps);



/**** cup_clamp_front functions *********************************************************/
typedef struct {
	uint8_t mode;
#define CUP_CLAMP_FRONT_MODE_OPEN			0
#define CUP_CLAMP_FRONT_MODE_CUP_LOCKED		1
#define CUP_CLAMP_FRONT_MODE_MAX			2

#define CUP_CLAMP_FRONT_MODE_POS_MIN		1
#define CUP_CLAMP_FRONT_MODE_POS_MAX		0

	uint16_t ax12_pos;

} cup_clamp_front_t;

/* set cup_clamp_front position depends on mode */
int8_t cup_clamp_front_set_mode(cup_clamp_front_t *cup_clamp_front, uint8_t mode, int16_t pos_offset);

/* return cup_clamp_front traj flag */
uint8_t cup_clamp_front_test_traj_end(cup_clamp_front_t *cup_clamp_front);

/* return END_TRAJ or END_TIMER */
uint8_t cup_clamp_front_wait_end(cup_clamp_front_t *cup_clamp_front);



/**** cup_holder_front functions *********************************************************/
typedef struct {
	uint8_t mode;
#define CUP_HOLDER_FRONT_MODE_CUP_HOLD		0
#define CUP_HOLDER_FRONT_MODE_READY			1
#define CUP_HOLDER_FRONT_MODE_HIDE			2
#define CUP_HOLDER_FRONT_MODE_MAX			3

#define CUP_HOLDER_FRONT_MODE_POS_MIN		0
#define CUP_HOLDER_FRONT_MODE_POS_MAX		2

	uint16_t ax12_pos;

} cup_holder_front_t;

/* set cup_holder_front position depends on mode */
int8_t cup_holder_front_set_mode(cup_holder_front_t *cup_holder_front, uint8_t mode, int16_t pos_offset);

/* return cup_holder_front traj flag */
uint8_t cup_holder_front_test_traj_end(cup_holder_front_t *cup_holder_front);

/* return END_TRAJ or END_TIMER */
uint8_t cup_holder_front_wait_end(cup_holder_front_t *cup_holder_front);



#endif /* _ACTUATOR_H_ */
