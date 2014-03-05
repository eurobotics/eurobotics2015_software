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

/* send data in multiplexing mode thru a link */
void bt_wt11_mux_send (uint8_t link_id, uint8_t *data, uint16_t length)
{
#define WT11_MUX_SOF        0xBF
#define WT11_MUX_LENGTH_MAX 128
#define WT11_MUX_CTRL_CMD   0xFF

#ifndef HOST_VERSION
#define __uart_send(x)  uart_send(BT_UART,x)
#else
#define __uart_send(x)  robotsim_uart_send_BT((uint8_t)x);
#endif

  uint16_t i=0;

  /* check length */
  if (length > WT11_MUX_LENGTH_MAX) {
    ERROR(E_USER_BT_PROTO, "BT_PROTO: ERROR, length > max");
    return;
  }

  /* start of frame */
  __uart_send(WT11_MUX_SOF);

  /* link ID */
  __uart_send(link_id);

  /* flags and length */
  __uart_send(length >> 8);
  __uart_send(length & 0xFF);

  /* data */
  for(i=0; i<length; i++)
    __uart_send(data[i]);

  /* nLINK */
  __uart_send(link_id ^ 0xFF);  
}

/* receive data in multiplexing mode, 
   returns data length, -1 if no data received */
int16_t bt_wt11_mux_recv (uint8_t *link_id, uint8_t *data)
{
  volatile int16_t c = 0;
  static uint8_t state = 0;
  static uint8_t __link_id = 0;
  static uint16_t __length, i = 0;

  do {

    /* get byte */
#ifndef HOST_VERSION
    c = uart_recv_nowait(BT_UART);
#else
		c = robotsim_uart_recv_BT();  
#endif
  
    if (c == -1)
      return c;

    switch (state) {

      /* start of frame */
      case 0:
        if (c == WT11_MUX_SOF)
          state ++;
        break;

      /* link ID */
      case 1:
        if (((c >= 0) && (c <=8)) || (c == WT11_MUX_CTRL_CMD)) {
          __link_id = c;
          state ++;
        }
        else
          state = 0;
        break;
          
      /* flags and length */
      case 2:
        __length = ((uint16_t)c << 8);
        state ++;
        break;

      case 3:
        __length |= ((uint16_t)c & 0x00FF);
        state ++;
        i = 0;
        break;

      /* data */
      case 4:
        data[i++] = c;
        
        if (i == __length)
          state ++;
        break;

      /* nLINK */    
      case 5:
        if ((c ^ 0xFF) == (__link_id)) {
          *link_id = __link_id;
          state = 0;
          i = 0;

          DEBUG (E_USER_BT_WT11, "received %s from link %d\n", data, *link_id);
          return __length;
        }
        else {
          state = 0;
          i = 0;

          ERROR (E_USER_BT_WT11, "BT_WT11: link ID ERROR");
          return -1;
        }
        break;
        
      default:
        state = 0;
        i = 0;
        break;
    }

  } while (c != -1);

  return -1;
}

#if 0

/* return the sum of length datum */
uint16_t checksum(uint8_t *data, uint16_t length) {
  uint16_t sum=0, i=0;

  for (i=0; i<length; i++)
    sum += data[i];

  return sum;
}

/* processes received data thru bluetooth UART */
void bt_receive_command (void)
{
  uint8_t data[WT11_MUX_LENGTH_MAX];

  /* received cmd */
  length = bt_wt11_mux_recv (&link_id, data);

  if (length != -1) {

    /* parse received data */
    switch (data[0]) {

        case BEACON_STATUS:
			    struct bt_beacon_status * ans = 
				    (struct bt_beacon_status *)buf;
			
			    if (size != sizeof (*ans))
				    goto error_size;

          if (ans->checksum != checksum(data, length-2))
            goto error_chechsum;

#if 0
	        /* opponent pos */
	        beaconboard.opponent_x = ans->;
	        beaconboard.opponent_y;
	        beaconboard.opponent_a;
	        beaconboard.opponent_d;

        #ifdef TWO_OPPONENTS
	        beaconboard.opponent2_x;
	        beaconboard.opponent2_y;
	        beaconboard.opponent2_a;
	        beaconboard.opponent2_d;
        #endif
          break;
#endif


        case ROBOT_2ND_STATUS:
			    struct bt_robot_2nd_status * ans = 
				    (struct bt_robot_2nd_status *)buf;
			
			    if (size != sizeof (*ans))
				    goto error_size;

          if (ans->checksum != checksum(data, length-2))
            goto error_chechsum;

          break;

        default:
          DEBUG (E_USER_BT_PROTO, "BT_PROTO: unknow received command");
          break;
    }

  }

  return;

 /* received errors */	
error_checksum:
  bt_errors_checksum ++;
	NOTICE(E_USER_BT_PROTO, "recv CHECKSUM error (%d)", bt_errors_checksum);
  return;

error_size:
	bt_errors_size ++;
	NOTICE(E_USER_BT_PROTO, "recv SIZE error (%d)", bt_errors_size);
  return;
 
}

void bt_protocol_send_event (void)
{
  /* send command if is the case */
  if (cmd_size)


  else
  /* pulling status */

}
#endif

void bt_robot_sec_send_ascii_cmd(const char * format, ...)
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

