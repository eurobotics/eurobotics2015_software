/*  
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
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
 *  Revision : $Id: cs.c,v 1.8 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  cs.c,v 1.8 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/error.h>

#include <encoders_dspic.h>
#include <dac_mc.h>
#include <pwm_servo.h>
//#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "robotsim.h"
#include "strat.h"
#include "actuator.h"
#include "i2c_protocol.h"

void dump_cs(const char *name, struct cs *cs);

/* called periodically */
static void do_cs(void *dummy) 
{
	static uint16_t cpt = 0;
	static int32_t old_a = 0, old_d = 0;

#ifdef HOST_VERSION
	robotsim_update();
#else
	/* read encoders */
	if (mainboard.flags & DO_ENCODERS) {
		encoders_dspic_manage(NULL);
	}
#endif

	/* robot system, conversion to angle, distance */
	if (mainboard.flags & DO_RS) {
		int16_t a,d;

		/* Read the encoders, and update internal virtual counters. */
		
		/* takes about 0.5 ms on AVR@16MHz */
		rs_update(&mainboard.rs); 

		/* process and store current speed */
		a = rs_get_angle(&mainboard.rs);
		d = rs_get_distance(&mainboard.rs);

		mainboard.speed_a = a - old_a;
		mainboard.speed_d = d - old_d;

		old_a = a;
		old_d = d;
	}

	/* control system */
	if (mainboard.flags & DO_CS) {

		if (mainboard.angle.on)
			cs_manage(&mainboard.angle.cs);

		if (mainboard.distance.on)
			cs_manage(&mainboard.distance.cs);

#if 0
		if (cs_get_consign(&mainboard.distance.cs) != cs_get_filtered_consign(&mainboard.distance.cs) || ) 
		{
			t1 = time_get_us2();
        	dump_cs_debug("distance", &mainboard.distance.cs);
		}
		else if (/*t1 != 0 &&*/ (time_get_us2() - t1 < 2000000))
		{
        	dump_cs_debug("distance", &mainboard.distance.cs);
		}
#endif
	}

	/* position calculus */
	if ((cpt & 1) && (mainboard.flags & DO_POS)) {

		/* about 1.5ms 
       		 * (worst case without centrifugal force compensation) */
		position_manage(&mainboard.pos);
	}

	/* blocking detection */
	if (mainboard.flags & DO_BD) {
		bd_manage_from_cs(&mainboard.angle.bd, &mainboard.angle.cs);
		bd_manage_from_cs(&mainboard.distance.bd, &mainboard.distance.cs);
	}

#ifndef HOST_VERSION
	/* take a look to match time */
	if (mainboard.flags & DO_TIMER) {
		uint8_t second;

		/* the robot should stop correctly in the strat, but
		 * in some cases, we must force the stop from an
		 * interrupt */

		second = time_get_s();

		if ((second >= MATCH_TIME + 2)) {

			/* stop motors */
			dac_mc_set(LEFT_MOTOR, 0);
			dac_mc_set(RIGHT_MOTOR, 0);

			/* kill strat */
			strat_exit();

			printf_P(PSTR("END OF TIME\r\n"));
		}
	}
#endif	
	/* motors brakes */
	if (mainboard.flags & DO_POWER)
		BRAKE_OFF();
	else
		BRAKE_ON();
	
	cpt++;

#ifdef HOST_VERSION
	if ((cpt & 7) == 0) {
		robotsim_dump();
	}
#endif
}


void dump_cs_debug(const char *name, struct cs *cs)
{
	DEBUG(E_USER_CS, "%s cons= %+.5"PRIi32" fcons= %+.5"PRIi32" err= %+.5"PRIi32" "
	      "in= %+.5"PRIi32" out= %+.5"PRIi32"",
	      name, cs_get_consign(cs), cs_get_filtered_consign(cs),
	      cs_get_error(cs), cs_get_filtered_feedback(cs),
	      cs_get_out(cs));
}

void dump_cs(const char *name, struct cs *cs)
{
	printf_P(PSTR("%s cons= %+.5"PRIi32" fcons= %+.5"PRIi32" err= %+.5"PRIi32" "
		      "in= %+.5"PRIi32" out= %+.5"PRIi32"\r\n"),
		 name, cs_get_consign(cs), cs_get_filtered_consign(cs),
		 cs_get_error(cs), cs_get_filtered_feedback(cs),
		 cs_get_out(cs));
}

void dump_pid(const char *name, struct pid_filter *pid)
{
	printf_P(PSTR("%s P= %+.8"PRIi32" I= %+.8"PRIi32" D= %+.8"PRIi32" out= %+.8"PRIi32"\r\n"),
		 name,
		 pid_get_value_in(pid) * pid_get_gain_P(pid),
		 pid_get_value_I(pid) * pid_get_gain_I(pid),
		 pid_get_value_D(pid) * pid_get_gain_D(pid),
		 pid_get_value_out(pid));
}

//void dac_set_and_save(void *dac, int32_t val);

/* cs init */
void maindspic_cs_init(void)
{
	/* ROBOT_SYSTEM */
	rs_init(&mainboard.rs);
	rs_set_left_pwm(&mainboard.rs, dac_set_and_save, LEFT_MOTOR);
	rs_set_right_pwm(&mainboard.rs,  dac_set_and_save, RIGHT_MOTOR);

#define Ed	0.9981761809228520 //1.0 //1.00375
#define Cl	(2.0/(Ed + 1.0))
#define Cr  (2.0 /((1.0 / Ed) + 1.0))

	/* increase gain to decrease dist, increase left and it will turn more left */
#ifdef HOST_VERSION
	rs_set_left_ext_encoder(&mainboard.rs, robotsim_encoder_get,
				LEFT_ENCODER, IMP_COEF * 1.);
	rs_set_right_ext_encoder(&mainboard.rs, robotsim_encoder_get,
				 RIGHT_ENCODER, IMP_COEF * 1.);
#else
	rs_set_left_ext_encoder(&mainboard.rs, encoders_dspic_get_value, 
				LEFT_ENCODER, IMP_COEF * Cl); 
	rs_set_right_ext_encoder(&mainboard.rs, encoders_dspic_get_value, 
				 RIGHT_ENCODER, IMP_COEF * Cr);
#endif

	/* rs will use external encoders */
	rs_set_flags(&mainboard.rs, RS_USE_EXT);

	/* POSITION MANAGER */
	position_init(&mainboard.pos);
	position_set_physical_params(&mainboard.pos, VIRTUAL_TRACK_MM, DIST_IMP_MM * 1.0022958057395100); //0.9875567845
	position_set_related_robot_system(&mainboard.pos, &mainboard.rs);
	position_set_centrifugal_coef(&mainboard.pos, 0.0); // 0.000016
	position_use_ext(&mainboard.pos);

	/* TRAJECTORY MANAGER */
	trajectory_init(&mainboard.traj);
	trajectory_set_cs(&mainboard.traj, &mainboard.distance.cs,
			  &mainboard.angle.cs);
	trajectory_set_robot_params(&mainboard.traj, &mainboard.rs, &mainboard.pos); /* d, a */
	trajectory_set_speed(&mainboard.traj, SPEED_DIST_FAST, SPEED_ANGLE_FAST);	
	
	/* distance window, angle window, angle start */
  	trajectory_set_windows(&mainboard.traj, 100., 5.0, 30.0); //50., 5.0, 5.0

	/* ---- CS angle */
	/* PID */
	pid_init(&mainboard.angle.pid);
#ifndef HOST_VERSION
	pid_set_gains(&mainboard.angle.pid, 350, 0, 4000); // real
#else
//	pid_set_gains(&mainboard.angle.pid, 40, 0, 1200); // robotsim tunning
//	pid_set_gains(&mainboard.angle.pid, 300, 0, 3500); // robotsim tunning
	pid_set_gains(&mainboard.angle.pid, 60, 0, 2800); // robotsim tunning
#endif
	pid_set_maximums(&mainboard.angle.pid, 0, 30000, 65500);
	pid_set_out_shift(&mainboard.angle.pid, 6);	
	pid_set_derivate_filter(&mainboard.angle.pid, 1);

	/* QUADRAMP */
	quadramp_init(&mainboard.angle.qr);
	quadramp_set_1st_order_vars(&mainboard.angle.qr, SPEED_ANGLE_VERY_FAST, SPEED_ANGLE_VERY_FAST); 	/* set speed */
	quadramp_set_2nd_order_vars(&mainboard.angle.qr, ACC_ANGLE, ACC_ANGLE); 		/* set accel */

	/* CS */
	cs_init(&mainboard.angle.cs);
	cs_set_consign_filter(&mainboard.angle.cs, quadramp_do_filter, &mainboard.angle.qr);
	cs_set_correct_filter(&mainboard.angle.cs, pid_do_filter, &mainboard.angle.pid);
	cs_set_process_in(&mainboard.angle.cs, rs_set_angle, &mainboard.rs);
	cs_set_process_out(&mainboard.angle.cs, rs_get_angle, &mainboard.rs);
	cs_set_consign(&mainboard.angle.cs, 0);

	/* Blocking detection */
	bd_init(&mainboard.angle.bd);
	bd_set_speed_threshold(&mainboard.angle.bd, 100);
	bd_set_current_thresholds(&mainboard.angle.bd, 20, 8000, 1000000, 25);

	/* ---- CS distance */
	/* PID */
	pid_init(&mainboard.distance.pid);
#ifndef HOST_VERSION
	pid_set_gains(&mainboard.distance.pid, 250, 0, 2800); // real
#else
//	pid_set_gains(&mainboard.distance.pid, 40, 0, 1200); // robotsim tunning
//	pid_set_gains(&mainboard.distance.pid, 300, 0, 3500); // robotsim tunning
	pid_set_gains(&mainboard.distance.pid, 60, 0, 2800); // robotsim tunning
#endif
	pid_set_maximums(&mainboard.distance.pid, 0, 30000, 65500);
	pid_set_out_shift(&mainboard.distance.pid, 6);
	pid_set_derivate_filter(&mainboard.distance.pid, 1);

	/* QUADRAMP */
	quadramp_init(&mainboard.distance.qr);
	quadramp_set_1st_order_vars(&mainboard.distance.qr, SPEED_DIST_VERY_FAST, SPEED_DIST_VERY_FAST); 	/* set speed */
	quadramp_set_2nd_order_vars(&mainboard.distance.qr, ACC_DIST, ACC_DIST); 	/* set accel */

	/* CS */
	cs_init(&mainboard.distance.cs);
	cs_set_consign_filter(&mainboard.distance.cs, quadramp_do_filter, &mainboard.distance.qr);
	cs_set_correct_filter(&mainboard.distance.cs, pid_do_filter, &mainboard.distance.pid);
	cs_set_process_in(&mainboard.distance.cs, rs_set_distance, &mainboard.rs);
	cs_set_process_out(&mainboard.distance.cs, rs_get_distance, &mainboard.rs);
	cs_set_consign(&mainboard.distance.cs, 0);

	/* Blocking detection */
	bd_init(&mainboard.distance.bd);
	bd_set_speed_threshold(&mainboard.distance.bd, 100);
	bd_set_current_thresholds(&mainboard.distance.bd, 100, 2000, 1000000, 30); //20, 8000, 1000000, 50);

	/* set them on !! */
	mainboard.angle.on = 1;
	mainboard.distance.on = 1;

	/* EVENT CS */
	scheduler_add_periodical_event_priority(do_cs, NULL,
						EVENT_PERIOD_CS / SCHEDULER_UNIT, EVENT_PRIORITY_CS);
}

