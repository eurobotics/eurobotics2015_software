/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2012)
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

#include <aversive.h>
#include <aversive/error.h>

#include <ax12.h>
#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <timer.h>
#include <scheduler.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "actuator.h"

/* called every 5 ms */
void do_cs(__attribute__((unused)) void *dummy) 
{
	/* read encoders */
	if (slavedspic.flags & DO_ENCODERS) {
		encoders_dspic_manage(NULL);
	}
	/* control system */
	if (slavedspic.flags & DO_CS) {
		if (slavedspic.lift.on)
			cs_manage(&slavedspic.lift.cs);
	}
	if (slavedspic.flags & DO_BD) {
		bd_manage_from_cs(&slavedspic.lift.bd, &slavedspic.lift.cs);
	}
	if (slavedspic.flags & DO_POWER)
		BRAKE_OFF();
	else
		BRAKE_ON();
}

void dump_cs_debug(const char *name, struct cs *cs)
{
	DEBUG(E_USER_CS, "%s cons=% .5ld fcons=% .5ld err=% .5ld "
	      "in=% .5ld out=% .5ld", 
	      name, cs_get_consign(cs), cs_get_filtered_consign(cs),
	      cs_get_error(cs), cs_get_filtered_feedback(cs),
	      cs_get_out(cs));
}

void dump_cs(const char *name, struct cs *cs)
{
	printf_P(PSTR("%s cons=% .5ld fcons=% .5ld err=% .5ld "
		      "in=% .5ld out=% .5ld\r\n"), 
		 name, cs_get_consign(cs), cs_get_filtered_consign(cs),
		 cs_get_error(cs), cs_get_filtered_feedback(cs),
		 cs_get_out(cs));
}

void dump_pid(const char *name, struct pid_filter *pid)
{
	printf_P(PSTR("%s P=% .8ld I=% .8ld D=% .8ld out=% .8ld\r\n"),
		 name,
		 pid_get_value_in(pid) * pid_get_gain_P(pid),
		 pid_get_value_I(pid) * pid_get_gain_I(pid),
		 pid_get_value_D(pid) * pid_get_gain_D(pid),
		 pid_get_value_out(pid));
}

void slavedspic_cs_init(void)
{
	/* ---- CS */
	/* PID */
	pid_init(&slavedspic.lift.pid);
	pid_set_gains(&slavedspic.lift.pid, 3000, 16, 10000);
	pid_set_maximums(&slavedspic.lift.pid, 0, 500000, 60000);
	pid_set_out_shift(&slavedspic.lift.pid, 8);
	pid_set_derivate_filter(&slavedspic.lift.pid, 1);

	/* QUADRAMP */
	quadramp_init(&slavedspic.lift.qr);
	quadramp_set_1st_order_vars(&slavedspic.lift.qr, LIFT_SPEED, LIFT_SPEED); 	/* 4000 set speed */
	quadramp_set_2nd_order_vars(&slavedspic.lift.qr, LIFT_ACCEL, LIFT_ACCEL); 		/* 10 set accel */

	/* CS */
	cs_init(&slavedspic.lift.cs);
	cs_set_consign_filter(&slavedspic.lift.cs, quadramp_do_filter, &slavedspic.lift.qr);
	cs_set_correct_filter(&slavedspic.lift.cs, pid_do_filter, &slavedspic.lift.pid);
	cs_set_process_in(&slavedspic.lift.cs, dac_mc_set, LIFT_DAC_MC);
	cs_set_process_out(&slavedspic.lift.cs, encoders_dspic_get_value, LIFT_ENCODER);
	cs_set_consign(&slavedspic.lift.cs, 0);

	/* Blocking detection */
	bd_init(&slavedspic.lift.bd);
	//bd_set_speed_threshold(&slavedspic.lift.bd, 150);
	//bd_set_current_thresholds(&slavedspic.lift.bd, 500, 8000, 1000000, 40);
	bd_set_speed_threshold(&slavedspic.lift.bd, 100);
	bd_set_current_thresholds(&slavedspic.lift.bd, 20, 8000, 1000000, 50);


	/* set on!! */
	slavedspic.lift.on = 1;
}
