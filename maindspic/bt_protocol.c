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

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <clock_time.h>

#include "main.h"
#include "bt_protocol.h"

#ifdef HOST_VERSION
#include "robotsim.h"
#endif

void bt_robot2_send_command(const char * format, ...)
{
  char buffer[256];
  va_list args;
  uint16_t i, n;

  va_start (args, format);
  n = vsprintf (buffer,format, args);

#ifdef HOST_VERSION
  for(i=0; i<n; i++)
    robotsim_uart_send_BT(buffer[i]);  
  robotsim_uart_send_BT('\n');
#endif

  va_end (args);
}

