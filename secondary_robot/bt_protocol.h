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

#ifndef __BT_PROTOCOL_H__
#define __BT_PROTOCOL_H__


/******************* BT STATUS MANAGE FUNCTIONS ******************************/


/* set bt cmd id and args checksum */
void bt_status_set_cmd_ret (uint8_t ret);

void bt_status_set_cmd_ack (uint8_t val);

void bt_send_status (void);


/******************* BT PROTOCOL COMMANDS *************************************/

void bt_strat_exit (void);
void bt_strat_init (void);
void bt_auto_position (void);
void bt_set_color (uint8_t color);

void bt_trajectory_goto_xy_abs (int16_t x, int16_t y, int16_t args_checksum);
void bt_trajectory_goto_backward_xy_abs (int16_t x, int16_t y, int16_t args_checksum);
void bt_trajectory_goto_xy_rel(int16_t x, int16_t y, int16_t args_checksum);
void bt_trajectory_goto_forward_xy_abs (int16_t x, int16_t y, int16_t args_checksum);
void bt_goto_and_avoid (int16_t x, int16_t y, int16_t args_checksum);
void bt_goto_and_avoid_forward (int16_t x, int16_t y, int16_t args_checksum);
void bt_goto_and_avoid_backward (int16_t x, int16_t y, int16_t args_checksum);

void bt_task_pick_cup(int16_t x, int16_t y, uint8_t side, int16_t args_checksum);
void bt_task_carpet(void);
void bt_task_stairs(void);
void bt_task_bring_cup(int16_t x, int16_t y, uint8_t side, int16_t args_checksum);
void bt_task_clap(int16_t x, int16_t y, int16_t args_checksum);

#endif /* __BT_PROTOCOL_H__ */
