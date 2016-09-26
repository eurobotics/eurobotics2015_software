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

#ifndef __BT_COMMANDS_H__
#define __BT_COMMANDS_H__


#define BT_SIDE_FRONT 0
#define BT_SIDE_REAR  1
#define BT_SIDE_ALL   2


struct bt_cmd_hdr {
	uint16_t cmd;
} __attribute__ ((aligned (2)));

/************************************************************
 * BEACON COMMANDS 
 ***********************************************************/

#define BT_BEACON_SYNC_HEADER	"beacon_sync"
#define BT_BEACON_STATUS_ANS	0x01
struct bt_beacon_status_ans 
{
	struct bt_cmd_hdr hdr;

  /* opp pos */
	int16_t opponent1_x;
	int16_t opponent1_y;
	int16_t opponent1_a;
	int16_t opponent1_d;

#ifdef TWO_OPPONENTS
	int16_t opponent2_x;
	int16_t opponent2_y;
	int16_t opponent2_a;
	int16_t opponent2_d;
#endif

  	uint16_t checksum;
} __attribute__ ((aligned (2)));

/************************************************************
 * ROBOT_2ND COMMANDS 
 ***********************************************************/

#define BT_ROBOT_2ND_SYNC_HEADER	"robot_2nd_sync"
#define BT_ROBOT_2ND_STATUS_ANS 	0x02
struct bt_robot_2nd_status_ans 
{
	struct bt_cmd_hdr hdr;

	/* running command info */
	//uint8_t cmd_id;


	uint8_t cmd_ret; 		/* ACK and command return value, 
							   END_TRAJ flags rules, see strat_base.h */
	//uint8_t cmd_args_checksum;

	/* strat info */
	uint8_t color;
	uint8_t done_flags;
#define BT_MAMOOTH_DONE		1
#define BT_OPP_FIRES_DONE	2
#define BT_FRESCO_DONE		4

	/* robot position */
	int16_t x;
	int16_t y;
	int16_t a_abs;

	/* opponent pos */
	int16_t opponent1_x;
	int16_t opponent1_y;

	int16_t opponent2_x;
	int16_t opponent2_y;

	uint16_t checksum;

} __attribute__ ((aligned (2)));



/* return the sum of length datum */
inline uint16_t bt_checksum(uint8_t *data, uint16_t length) {
  uint16_t sum=0, i=0;

  for (i=0; i<length; i++)
    sum += data[i];

  return sum;
}

#define BT_WAIT_COND_OR_TIMEOUT(cond, timeout)                   \
({                                                            \
        microseconds __us = time_get_us2();                   \
        uint8_t __ret = 1;                                    \
        while(! (cond)) {                                     \
                if (time_get_us2() - __us > (timeout)*1000L) {\
                        __ret = 0;                            \
                        break;                                \
                }                                             \
        }                                                     \
							      \
        __ret;                                                \
})

#endif /* __BT_COMMANDS__ */

