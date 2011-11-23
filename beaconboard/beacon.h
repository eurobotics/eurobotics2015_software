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

struct beacon {
	int32_t beacon_speed;
	uint8_t angle_offset;
	
	int32_t opponent_angle;
	int32_t opponent_dist;
	int32_t prev_opponent_angle;
	int32_t prev_opponent_dist;
	int32_t robot_x;
	int32_t robot_y;
	int32_t robot_a;
	int32_t opponent_x;
	int32_t opponent_y;
};

extern struct beacon beacon;

void beacon_init(void);
void beacon_start(void);
void beacon_stop(void);
void beacon_calc(void *dummy);

void beacon_angle_dist_to_x_y(int32_t angle, int32_t dist, int32_t *x, int32_t *y);
