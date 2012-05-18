/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2010)
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

#ifndef _BEACON_H_
#define _BEACON_H_

#define BEACON_UART		MUX_UART

void beacon_cmd_wt11_local_reset(void);
void beacon_cmd_wt11_call(void);
void beacon_cmd_wt11_close(void);

void beacon_init(void);
void beacon_protocol(void * dummy);

void beacon_cmd_color(void);
void beacon_cmd_opponent(void);
void beacon_cmd_beacon_on(void);
void beacon_cmd_beacon_on_watchdog(void);
void beacon_cmd_beacon_off(void);

void beacon_opponent_pulling(void);

/* pull opponent position */
void beacon_pull_opponent(void);

#endif
