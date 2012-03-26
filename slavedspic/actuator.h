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

#include <aversive.h>#include <aversive/error.h>
#include <time.h>

#define LIFT_SPEED						100
#define LIFT_ACCEL						1
#define LIFT_K_IMP_mm					-1.0
#define LIFT_CALIB_IMP_MAX				0
#define LIFT_HEIGHT_MAX_mm				42000
#define LIFT_HEIGHT_MIN_mm				500

#define TURBINE_POS_ANGLE_MAX			1080
#define TURBINE_POS_ANGLE_MIN			10
#define TURBINE_POS_ANGLE_ZERO 		670
#define TURBINE_POS_ANGLE_90			130
#define TURBINE_K_POS_DEG		 		(double)((TURBINE_POS_ANGLE_90 - TURBINE_POS_ANGLE_ZERO)  / 90.0)
#define TURBINE_BLOW_SPEED_OFF		200

#define POS_FINGER_TOTEM_R_HUG		738
#define POS_FINGER_TOTEM_R_OPEN		POS_FINGER_TOTEM_R_HUG
#define POS_FINGER_TOTEM_R_HOLD		492
#define POS_FINGER_TOTEM_R_CLOSE		384
#define POS_FINGER_TOTEM_R_PUSHIN	290
#define POS_FINGER_TOTEM_R_MAX		POS_FINGER_TOTEM_R_HUG
#define POS_FINGER_TOTEM_R_MIN		POS_FINGER_TOTEM_R_PUSHIN

#define POS_FINGER_TOTEM_L_HUG		463
#define POS_FINGER_TOTEM_L_OPEN		POS_FINGER_TOTEM_L_HUG
#define POS_FINGER_TOTEM_L_HOLD		704
#define POS_FINGER_TOTEM_L_CLOSE		809
#define POS_FINGER_TOTEM_L_PUSHIN	893
#define POS_FINGER_TOTEM_L_MAX		POS_FINGER_TOTEM_L_PUSHIN
#define POS_FINGER_TOTEM_L_MIN		POS_FINGER_TOTEM_L_HUG


#define POS_FINGER_FLOOR_R_HUG		807
#define POS_FINGER_FLOOR_R_OPEN		602
#define POS_FINGER_FLOOR_R_HOLD		387
#define POS_FINGER_FLOOR_R_CLOSE		269
#define POS_FINGER_FLOOR_R_PUSHIN	175

#define POS_FINGER_FLOOR_L_HUG 		254
#define POS_FINGER_FLOOR_L_OPEN 		488
#define POS_FINGER_FLOOR_L_HOLD		700
#define POS_FINGER_FLOOR_L_CLOSE		805
#define POS_FINGER_FLOOR_L_PUSHIN	882


#define POS_ARM_R_HIDE				533
#define POS_ARM_R_SHOW				294
#define POS_ARM_R_PUSH_GOLDBAR	278
#define POS_ARM_R_PUSH_FLOOR		250

#define POS_ARM_L_HIDE				238
#define POS_ARM_L_SHOW				480
#define POS_ARM_L_PUSH_GOLDBAR	491
#define POS_ARM_L_PUSH_FLOOR		522

#define POS_BOOT_OPEN_FULL		700 /* TODO set position */
#define POS_BOOT_OPEN_HOLD		700
#define POS_BOOT_CLOSE			835

#define POS_HOOK_HIDE			140
#define POS_HOOK_SHOW			950
#define POS_HOOK_FUCKYOU		500
#define POS_HOOK_OPEN_HOLD		720

#define POS_TRAY_RECEPTION_UP		1000	
#define POS_TRAY_RECEPTION_DOWN	140

#define POS_TRAY_STORE_UP			400	
#define POS_TRAY_STORE_DOWN		20  	

#define TRAY_BOOT_VIBRATE_PWM				(2000)
#define TRAY_STORE_VIBRATE_PERIOD_us	(300000L) 
#define TRAY_STORE_PERIOD_EVENT_us		(50000) 	
#define TRAY_STORE_PERIOD_PRESCALER		(uint16_t)(TRAY_STORE_VIBRATE_PERIOD_us / TRAY_STORE_PERIOD_EVENT_us)

#define END_TRAJ		1
#define END_BLOCKING	2


/* init actuators */
void actuator_init(void);

/**** lift functions ********************************************************/

/* calibrate initial position */
void lift_calibrate(void);

/* set height in mm */
void lift_set_height(int32_t height_mm);

/* return heigh in mm */
int32_t lift_get_height(void);

/* return 1 if height reached, -1 if blocking and zero if no ends yet */
int8_t lift_check_height_reached(void);

/**** turbine funcions *********************************************************/

/* turbine structure */
typedef struct {
	uint8_t power;
#define TURBINE_POWER_ON 					RELE_OUT_PIN_ON
#define TURBINE_POWER_OFF 					RELE_OUT_PIN_OFF
#define TURBINE_ANGLE_SPEED_CONTROL_us	25000

	uint16_t angle_pos;
	uint16_t angle_consign;
	uint16_t angle_speed;
	int8_t 	angle_event_handler;

	uint16_t blow_speed;

} turbine_t;

void turbine_power_on(turbine_t *turbine);
void turbine_power_off(turbine_t *turbine);

void turbine_set_angle(turbine_t *turbine, int16_t angle_deg, uint16_t wait_ms);
int8_t turbine_check_angle_reached(turbine_t *turbine);

void turbine_set_blow_speed(turbine_t *turbine, uint16_t speed);

uint16_t turbine_get_angle(turbine_t *turbine);
uint16_t turbine_get_blow_speed(turbine_t *turbine);

/**** fingers funcions *********************************************************/

typedef struct {
	uint8_t type;
#define FINGERS_TYPE_FLOOR		0
#define FINGERS_TYPE_TOTEM		1
#define FINGERS_TYPE_MAX		2

	uint8_t mode;
#define FINGERS_MODE_HUG		0
#define FINGERS_MODE_OPEN		1
#define FINGERS_MODE_HOLD		2
#define FINGERS_MODE_CLOSE		3
#define FINGERS_MODE_PUSHIN	4
#define FINGERS_MODE_MAX		5

#define FINGERS_MODE_R_POS_MAX	FINGERS_MODE_HUG
#define FINGERS_MODE_R_POS_MIN	FINGERS_MODE_PUSHIN
#define FINGERS_MODE_L_POS_MAX	FINGERS_MODE_PUSHIN
#define FINGERS_MODE_L_POS_MIN	FINGERS_MODE_HUG

	microseconds time_us;

	uint16_t ax12_pos_l;
	uint16_t ax12_pos_r;
} fingers_t;

/* set finger position depends on mode */
int8_t fingers_set_mode(fingers_t *fingers, uint8_t mode, int16_t pos_offset);

/* return 1 if mode is done, -1 if error */
int8_t fingers_check_mode_done(fingers_t *fingers);

/**** arms funcions *********************************************************/
typedef struct {
	uint8_t type;
#define ARM_TYPE_RIGHT	0
#define ARM_TYPE_LEFT	1
#define ARM_TYPE_MAX		2	

	uint8_t mode;
#define ARM_MODE_HIDE				0
#define ARM_MODE_SHOW				1
#define ARM_MODE_PUSH_GOLDBAR		2
#define ARM_MODE_PUSH_FLOOR		3
#define ARM_MODE_MAX					4

	uint16_t ax12_pos;
	microseconds time_us;
} arm_t;

/* set finger position depends on mode */
uint8_t arm_set_mode(arm_t *arm, uint8_t mode, int16_t pos_offset);

/* return 1 if mode is done */
int8_t arm_check_mode_done(arm_t *arm);


/**** boot funcions *********************************************************/
typedef struct {
	uint8_t mode;
#define BOOT_MODE_OPEN_FULL		0
#define BOOT_MODE_OPEN_HOLD		1
#define BOOT_MODE_CLOSE				2
#define BOOT_MODE_MAX				3

	uint16_t ax12_pos;
} boot_t;

/* set finger position depends on mode */
uint8_t boot_set_mode(boot_t *boot, uint8_t mode);

/* return 1 if mode is done */
uint8_t boot_check_mode_done(boot_t *boot);


/**** hook funcions *********************************************************/
typedef struct {
	uint8_t mode;
#define HOOK_MODE_HIDE			0
#define HOOK_MODE_SHOW			1
#define HOOK_MODE_FUCKYOU		2
#define HOOK_MODE_OPEN_HOLD	3
#define HOOK_MODE_MAX			4

	uint16_t ax12_pos;
} hook_t;

/* set finger position depends on mode */
uint8_t hook_set_mode(hook_t *hook, uint8_t mode);

/* return 1 if mode is done */
uint8_t hook_check_mode_done(hook_t *hook);

/**** trays funcions *********************************************************/

typedef struct {
	uint8_t type;
#define TRAY_TYPE_RECEPTION	0
#define TRAY_TYPE_STORE			1
#define TRAY_TYPE_BOOT			2
#define TRAY_TYPE_MAX			3

	uint8_t mode;
#define TRAY_MODE_DOWN		0 /* it means off in case of boot tray */
#define TRAY_MODE_UP			1 /* only reception and store tray */
#define TRAY_MODE_VIBRATE	2 /* only boot and store tray */
#define TRAY_MODE_MAX		3	

	uint16_t servo_pos;
	int16_t motor_pwm;
	int8_t event_handler;

} tray_t;

void tray_set_mode(tray_t *tray, uint8_t mode);

#endif
