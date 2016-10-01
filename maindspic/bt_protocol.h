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

#ifndef __BT_PROTOCOL_H__
#define __BT_PROTOCOL_H__

/* number of bt devices, maximun 4 */
#define BT_PROTO_NUM_DEVICES 2


/* BT_TASKS */

#define BT_SET_COLOR		1
#define BT_AUTOPOS			2
#define BT_GOTO_XY_ABS		3
#define BT_GOTO_XY_REL		4
#define BT_GOTO_AVOID		5
#define BT_GOTO_AVOID_FW	6
#define BT_GOTO_AVOID_BW	7
#define BT_PICK_CUP						8
#define BT_CARPET						9
#define BT_STAIRS						10
#define BT_BRING_CUP_CINEMA				11
#define BT_CLAPPERBOARD					12
#define BT_GOTO_FW_XY_ABS				13
#define BT_GOTO_BW_XY_ABS				14
#define BT_MATCHTIMER					15


/* send and receive commands to/from bt devices, periodic dev status pulling */
void bt_protocol (void * dummy);


/************************************************************
 * BEACON COMMANDS
 ***********************************************************/

/* set color */
void bt_beacon_set_color (void);

/* beacon on */
void bt_beacon_set_on (void);

/* beacon on with watchdog */
void bt_beacon_set_on_watchdog (void);

/* beacon off*/
void bt_beacon_set_off (void);

/* request opponent position */
void bt_beacon_req_status(void);

/************************************************************
 * ROBOT 2ND COMMANDS
 ***********************************************************/

/* send command, and return after received ack */
void bt_robot_2nd_cmd_no_wait_ack (uint8_t cmd_id, int16_t arg0, int16_t arg1, int16_t arg2);

/* send command, and return after received ack */
uint8_t bt_robot_2nd_cmd (uint8_t cmd_id, int16_t arg0, int16_t arg1);

/* auto set possition */
void bt_robot_2nd_autopos (void);

/* start macth timer */
void bt_robot_2nd_start_matchtimer (void);

/* set color */
void bt_robot_2nd_set_color (void);

/* goto xy_abs */
void bt_robot_2nd_goto_xy_abs (int16_t x, int16_t y);
void bt_robot_2nd_goto_forward_xy_abs (int16_t x, int16_t y);
void bt_robot_2nd_goto_backward_xy_abs (int16_t x, int16_t y);

/* goto xy_rel */
void bt_robot_2nd_goto_xy_rel (int16_t x, int16_t y);

/* wait for robot 2nd ends */
uint8_t bt_robot_2nd_wait_end (void);
/* check if for robot 2nd ended */
uint8_t bt_robot_2nd_test_end (void);


/* request opponent position */
void bt_robot_2nd_req_status(void);


/* ACK */
uint8_t bt_robot_2nd_wait_ack (void);
uint8_t bt_robot_2nd_is_ack_received (void);

/* return 1 if ret is received */
uint8_t bt_robot_2nd_is_ret_received (void);

void bt_robot_2nd_bt_task_pick_cup (int16_t x, int16_t y, uint8_t side);
void bt_robot_2nd_bt_task_carpet(void);
void bt_robot_2nd_bt_task_stairs(void);
void bt_robot_2nd_bt_task_bring_cup_cinema(int16_t x, int16_t y, uint8_t side);
void bt_robot_2nd_bt_task_clapperboard(int16_t x, int16_t y);

void bt_robot_2nd_autopos (void);

void bt_robot_2nd_goto_and_avoid (int16_t x, int16_t y);
void bt_robot_2nd_goto_and_avoid_forward (int16_t x, int16_t y);
void bt_robot_2nd_goto_and_avoid_backward (int16_t x, int16_t y);

#endif
