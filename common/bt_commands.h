/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Bali�as Santos <javier@arc-robots.org>
 *
 *  $Id$
 */

#ifndef __BT_COMMANDS_H__
#define __BT_COMMANDS_H__


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
	int16_t opponent_x;
	int16_t opponent_y;
	int16_t opponent_a;
	int16_t opponent_d;

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
	uint8_t cmd_id;
#define BT_SET_COLOR			1

#define BT_GOTO_XY_ABS		2
#define BT_GOTO_XY_REL		3
#define BT_GOTO_AVOID		4
#define BT_GOTO_AVOID_FW	5
#define BT_GOTO_AVOID_BW	6

#define BT_DO_FRESCO_INIT	7
#define BT_DO_MAMMUT_1		8
#define BT_DO_MAMMUT_2		9
#define BT_DO_NET				10
#define BT_DO_OPP_FIRES		11

	uint8_t cmd_ret; 		/* END_TRAJ flags rules, see strat_base.h */
	uint8_t cmd_args_checksum;

	/* strat info */
	uint8_t color;
	uint16_t done_flags;
#define BT_DONE_MAMMUT_1	1
#define BT_DONE_MAMMUT_2	2
#define BT_DONE_FRESCO		4
#define BT_DONE_NET			8
#define BT_DONE_OPP_FIRES	16

	/* robot position */
	int16_t x;
	int16_t y;
	int16_t a_abs;

	/* opponent pos */
	int16_t opponent_x;
	int16_t opponent_y;

#ifdef TWO_OPPONENTS
	uint16_t opponent2_x;
	uint16_t opponent2_y;
#endif

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
