/*  
 *  Copyright Droids Corporation (2009)
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
 *  Revision : $Id: state.h,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*   *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011) *  Javier Baliñas Santos <javier@arc-robots.org> * *  Code ported to family of microcontrollers dsPIC from *  state.h,v 1.4 2009/05/27 20:04:07 zer0 Exp. */

#ifndef _STATE_H_
#define _STATE_H_

typedef struct {
#define TS_STATE_IDLE				0
#define TS_STATE_TAKE				1
#define TS_STATE_WAITING_STOP		2
#define TS_STATE_EJECT				3
#define TS_STATE_WAITING_FREE		4
#define TS_STATE_STOP				5
#define TS_STATE_SHOW				6
#define TS_STATE_PUSH_L				7
#define TS_STATE_PUSH_R				8
#define TS_STATE_OUT					9

	uint8_t state;
	uint8_t state_rqst;
	uint8_t state_changed;

	uint16_t speed;
	uint16_t speed_rqst;

	/* info */
	uint8_t belts_blocked;
	uint8_t token_catched;

	/* conf */
	uint8_t sensor_stop;
	uint8_t sensor_catched;
	uint8_t belts_side;

}token_system_t;

/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_slavedspic_set_mode *cmd);

/* get mode */
uint8_t state_get_mode(void);

/* run state machines */
void state_machines(void);

/* init state machines */
void state_init(void);

#endif
