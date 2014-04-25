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
#include "wt11.h"
#include "bt_protocol.h"
#include "../common/bt_commands.h"

#ifdef HOST_VERSION
#include "robotsim.h"
#endif


#ifdef notyet
static uint8_t cmd_data[BT_PROTO_NB_LINKS][WT11_MUX_LENGTH_MAX];
static uint8_t cmd_size[BT_PROTO_NB_LINKS];


/* fill a link id buffer which has to be send */
uint8_t bt_send_cmd (uint8_t link_id, uint8_t *data, uint16_t size) 
{
	int16_t i;
  uint8_t flags;

	/* check length */
	if(size > WT11_MUX_LENGTH_MAX){
		ERROR(E_USER_BT_PROTO, "Command size is too large");	
		return 1;
	}

  /* check if there is any pendant cmd */
  if (cmd_data[link_id])
    return 1;
		
	/* fill buffer */
	for(i=0; i<size; i++){
		cmd_data[link_id][i] = data[i];	
	}

	/* command size != 0 indicate 
   * that there is a command to send */	
	IRQ_LOCK(flags);
	cmd_size[link_id] = size;
  	IRQ_UNLOCK(flags);

  return 0;
}

/* fill a link id buffer with formated data to be send */
int8_t bt_send_ascii_cmd (uint8_t link_id, const char * format, ...)
{
  char buffer[100];
  va_list args;
  uint16_t i, n;

  va_start (args, format);
  n = vsprintf (buffer,format, args);

  ret = bt_send_cmd (link_id, data, n);

  va_end (args);
}

/* TODO */
void bt_protocol (void)
{
  /* receive commands */

  /* send commands */
  if (cmd_size)


  /* pulling commands */

}

/* processes received data thru bluetooth UART */
void bt_recv_cmd (void)
{
  uint8_t data[WT11_MUX_LENGTH_MAX];
	uint8_t flags;

  /* received data */
  length = wt11_recv_mux (&link_id, data);
  if (length == -1)
    return;

  /* parse data links */
  if (link_id != WT11_MUX_CTRL_CMD) 
  {
    switch (data[0]) {

        case BT_BEACON_STATUS_ANS:
		      struct bt_beacon_status * ans = (struct bt_beacon_status *)data;
          double x, y, a, d;
		
		      if (size != sizeof (*ans))
			      goto error_size;

          if (ans->checksum != checksum(data, length-1))
            goto error_chechsum;

          /* beacon correction */
          x = bt_struct_read_int(ans->opponent_x);
          y = bt_struct_read_int(ans->opponent_y);
          abs_xy_to_rel_da(x, y, &d, &a);

          IRQ_LOCK(flags);
          beaconboard.opponent_x = (int16_t)x;
          beaconboard.opponent_y = (int16_t)y;
          beaconboard.opponent_a = (int16_t)a;
          beaconboard.opponent_d = (int16_t)d;       
          IRQ_UNLOCK(flags);


          #ifdef TWO_OPPONENTS
          x = bt_struct_read_int(ans->opponent2_x);
          y = bt_struct_read_int(ans->opponent2_y);
          abs_xy_to_rel_da(x, y, &d, &a);

          IRQ_LOCK(flags);
          beaconboard.opponent2_x = (int16_t)x;
          beaconboard.opponent2_y = (int16_t)y;
          beaconboard.opponent2_a = (int16_t)a;
          beaconboard.opponent2_d = (int16_t)d;       
          IRQ_UNLOCK(flags);
          #endif

          break;

        case BT_ROBOT_2ND_STATUS_ANS:
		      struct bt_robot_2nd_status * ans = (struct bt_robot_2nd_status *)data;
          double x, y, a_abs, a, d;
		
		      if (size != sizeof (*ans))
			      goto error_size;

          if (ans->checksum != checksum(data, length-1))
            goto error_chechsum;

          /* save data */

          /* status */
          robot_2nd.status = ans->status;

          /* end_traj */
          robot_2nd.end_traj = ans->end_traj;

          /* robot pos */
          x =  = bt_struct_read_int(ans->x);
          y =  = bt_struct_read_int(ans->y);
          a_abs = bt_struct_read_int(ans->a_ab);
          abs_xy_to_rel_da(x, y, &d, &a);

          IRQ_LOCK(flags);
          robot_2nd.x = (int16_t)x;
          robot_2nd.y = (int16_t)y;
          robot_2nd.a = (int16_t)a;
          robot_2nd.d = (int16_t)d;       
          IRQ_UNLOCK(flags);

          /* opponents pos with beacon correction */
          x = bt_struct_read_int(ans->opponent_x);
          y = bt_struct_read_int(ans->opponent_y);
          abs_xy_to_rel_da(x, y, &d, &a);

          IRQ_LOCK(flags);
          robot_2nd.opponent_x = (int16_t)x;
          robot_2nd.opponent_y = (int16_t)y;
          robot_2nd.opponent_a = (int16_t)a;
          robot_2nd.opponent_d = (int16_t)d;       
          IRQ_UNLOCK(flags);


          #ifdef TWO_OPPONENTS
          x = bt_struct_read_int(ans->opponent2_x);
          y = bt_struct_read_int(ans->opponent2_y);
          abs_xy_to_rel_da(x, y, &d, &a);

          IRQ_LOCK(flags);
          robot_2nd.opponent2_x = (int16_t)x;
          robot_2nd.opponent2_y = (int16_t)y;
          robot_2nd.opponent2_a = (int16_t)a;
          robot_2nd.opponent2_d = (int16_t)d;       
          IRQ_UNLOCK(flags);
          #endif

          break;

        default:
          DEBUG (E_USER_BT_PROTO, "BT_PROTO: unknow received command");
          break;
    }
  } 
#if 0
  /* parse control wt11 answers */
  else  {
    
	  /* beacon wt11 open link connection pass */
   	ret = sscanf(data, "CONNECT %d RFCOMM %d", (int*)&link_id, (int*)&rfcomm);
	  if(ret == 1){
		  beacon_connected = 1;
		  NOTICE(E_USER_STRAT, "beacon wt11 link open PASS (%d)", link_id);						
	  }

	  /* beacon wt11 open link connection fails */
   	ret = sscanf(data, "NO CARRIER %d ERROR %d RFC_CONNECTION_FAILED", (int*)&link_id, (int*)&error_id);
	  if(ret == 2){
		  beacon_connected = 0;
		  ERROR(E_USER_STRAT, "beacon wt11 link open FAIL(%d,%d)", error_id, link_id);						
	  }

  }
#endif
  /* return */
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




/************************************************************
 * BEACON COMMANDS 
 ***********************************************************/

/* set color */
void bt_beacon_set_color (void)
{
	if(mainboard.our_color == I2C_COLOR_YELLOW)
    bt_send_ascii_cmd (beacon.link_id, "\ncolor yellow\n");
  else
    bt_send_ascii_cmd (beacon.link_id, "\ncolor red\n");  
}


/* beacon on */
void bt_beacon_set_on (void) {
  bt_send_ascii_cmd (beacon.link_id, "\nbeacon on\n");
}

/* beacon on with watchdog */
void beacon_cmd_beacon_on_watchdog (void) {
  bt_send_ascii_cmd (beacon.link_id, "\nbeacon watchdog_on\n");
}

/* beacon off*/
void beacon_cmd_beacon_off (void) 
{
	uint8_t flags;

	IRQ_LOCK(flags);
	mainboard.flags &= ~(DO_OPP);
	IRQ_UNLOCK(flags);

  bt_send_ascii_cmd (beacon.link_id, "\nbeacon off\n");
}


/* request opponent position */
void bt_beacon_rqst_opponent(void)
{
	char buff[32];
	uint16_t size;
	int16_t robot_x, robot_y, robot_a;
	uint8_t flags;
	uint8_t i;
	
	IRQ_LOCK(flags);
	robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);
	robot_a = position_get_a_deg_s16(&mainboard.pos);
	IRQ_UNLOCK(flags);


  rel_da_to_abs_xy(BEACON_OFFSET_D, BEACON_OFFSET_A, 
                  &beacon_x, &beacon_y);    

  opp_x += beacon_x;
  opp_y += beacon_y;


  bt_send_ascii_cmd (beacon.link_id, "\nopponent %d %d %d\n",
								      robot_x, robot_y, robot_a);
}


#endif







