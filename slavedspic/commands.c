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
 *  Revision : $Id$
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

/*   *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011) *  Javier Baliñas Santos <javier@arc-robots.org> * *  Code ported to family of microcontrollers dsPIC from *  commands.c,v 1.5 2009/05/27 20:04:07 zer0 Exp   */

#include <stdlib.h>
#include <aversive/pgmspace.h>
#include <parse.h>

/* commands_gen.c */
extern parse_pgm_inst_t cmd_reset;
//extern parse_pgm_inst_t cmd_bootloader;
extern parse_pgm_inst_t cmd_sensor;
extern parse_pgm_inst_t cmd_log;
extern parse_pgm_inst_t cmd_log_show;
extern parse_pgm_inst_t cmd_log_type;
extern parse_pgm_inst_t cmd_scheduler;

/* commands_ax12.c */
//extern parse_pgm_inst_t cmd_baudrate;
extern parse_pgm_inst_t cmd_uint16_read;
extern parse_pgm_inst_t cmd_uint16_write;
extern parse_pgm_inst_t cmd_uint8_read;
extern parse_pgm_inst_t cmd_uint8_write;
extern parse_pgm_inst_t cmd_ax12_stress;
extern parse_pgm_inst_t cmd_ax12_dump_stats;

/* commands_slavedspic.c */
extern parse_pgm_inst_t cmd_event;
extern parse_pgm_inst_t cmd_color;
extern parse_pgm_inst_t cmd_belts;
extern parse_pgm_inst_t cmd_state_debug;
extern parse_pgm_inst_t cmd_state1;
extern parse_pgm_inst_t cmd_state2;
extern parse_pgm_inst_t cmd_state3;
//extern parse_pgm_inst_t cmd_state_machine;
//extern parse_pgm_inst_t cmd_test;


/* in progmem */
parse_pgm_ctx_t main_ctx[] = {

	/* commands_gen.c */
	(parse_pgm_inst_t *)&cmd_reset,
	//(parse_pgm_inst_t *)&cmd_bootloader,
	(parse_pgm_inst_t *)&cmd_sensor,
	(parse_pgm_inst_t *)&cmd_log,
	(parse_pgm_inst_t *)&cmd_log_show,
	(parse_pgm_inst_t *)&cmd_log_type,
	(parse_pgm_inst_t *)&cmd_scheduler,

	/* commands_ax12.c */
	//(parse_pgm_inst_t *)&cmd_baudrate,
	(parse_pgm_inst_t *)&cmd_uint16_read,
	(parse_pgm_inst_t *)&cmd_uint16_write,
	(parse_pgm_inst_t *)&cmd_uint8_read,
	(parse_pgm_inst_t *)&cmd_uint8_write,
	(parse_pgm_inst_t *)&cmd_ax12_stress,
	(parse_pgm_inst_t *)&cmd_ax12_dump_stats,

	/* commands_slavedspic.c */
	(parse_pgm_inst_t *)&cmd_event,
	(parse_pgm_inst_t *)&cmd_color,
	(parse_pgm_inst_t *)&cmd_belts,
	(parse_pgm_inst_t *)&cmd_state_debug,
	(parse_pgm_inst_t *)&cmd_state1,
	(parse_pgm_inst_t *)&cmd_state2,	
	(parse_pgm_inst_t *)&cmd_state3,
	//(parse_pgm_inst_t *)&cmd_state_machine,
	//(parse_pgm_inst_t *)&cmd_test,

	NULL,
};
