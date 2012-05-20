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

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <configuration_bits_config.h>

#include <uart.h>
#include <i2c_mem.h>

#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <ax12.h>

#include <timer.h>
#include <scheduler.h>
#include <time.h>

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

#include "../common/i2c_commands.h"

#include "main.h"
#include "strat.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "cs.h"
#include "i2c_protocol.h"
#include "beacon.h"
#include "ax12_user.h"


struct genboard gen;
struct mainboard mainboard;
struct beaconboard beaconboard;

#define HACK_EUROBOT_2012

/***************************************************************/

#ifdef notuse
void do_led_blink(void *dummy) {
}
#endif

static void main_timer_interrupt(void)
{
	/* scheduler tasks */
	sei();
	scheduler_interrupt();	
}

/* main timer */
void main_timer_init(void)
{
	/* use timer 1 */
	T1CON = 0;              
	IFS0bits.T1IF = 0;      
	IEC0bits.T1IE = 1;      
	TMR1 = 0x0000;  	
	PR1 = SCHEDULER_UNIT * (unsigned long)((double)FCY / 1000000.0);
	T1CONbits.TON = 1;  
}

/* timer 1 interrupt */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
  _T1IF=0;
  main_timer_interrupt();
}

void io_pins_init(void)
{
	/*************************************** 	*  IO portmap and config 	*/
	/* XXX: after reset all pins are inputs */
	/* XXX: after reset all ANALOG pins are analog
	 *		  and has disabled the read operation	 */

	/* analog inputs */
	/* by default all analog pins are digital */
	AD1PCFGL = 0xFF;	

	/* L6203 H bridges (outputs) */
#ifdef HACK_EUROBOT_2012
	_TRISC7 		= 0;	// M_DER_IN1
	_TRISC0  	= 0;	// M_DER_IN2
	_TRISC6 	   = 0;	// M_DER_EN

	_TRISB15 	= 0;	// M_IZQ_IN1
	_TRISC1  	= 0;	// M_IZQ_IN2
	_TRISB14 	= 0;	// M_IZQ_EN
#else
	_TRISB13 	= 0;	// M_DER_IN1
	_TRISC7  	= 0;	// M_DER_IN2
	_TRISB12 	= 0;	// M_DER_EN

	_TRISB15 	= 0;	// M_IZQ_IN1
	_TRISC6  	= 0;	// M_IZQ_IN2
	_TRISB14 	= 0;	// M_IZQ_EN
#endif	

	
	// encoders (inputs)
	_QEA1R 	= 21;	// QEA1 <- RP21
	_TRISC5  = 1;	// QEA1
	_QEB1R 	= 20;	// QEB1 <- RP20
	_TRISC4	= 1;	// QEB1

	_QEA2R 	= 19;	// QEA2 <- RP19
	_TRISC3  = 1;	// QEA2
	_QEB2R 	= 4;	// QEB2 <- RP4
	_TRISB4	= 1;	// QEB2
	
	// sensors
	_TRISA4 = 1;	// COLOR
	_TRISA9 = 1;	// START	
	_TRISC8 = 1;	// OPPONENT FRONT
	_TRISC9 = 1;   // OPPONENT FRONT
	_TRISB3 = 1; 	// OPPONENT REAR

	// wt11 reset (output)
	_TRISA8	= 0;	// RESET_BLUE
	_LATA8	= 0;	// RESET_BLUE OFF
	
	// i2c
		
	// uarts
	_U1RXR 	= 8;	// U1RX <- RP8
	_TRISB8  = 1;	// U1RX is input
  	_RP7R 	= 3;	// U1TX -> RP7
	_TRISB7	= 0;	// U1TX is output

	_U2RXR 	= 9;	// U2RX <- RP9 <- SERVOS_AX12_UART
  	_RP9R 	= 5;	// U2TX -> RP9 -> SERVOS_AX12_UART
	_TRISB9	= 0;	// U2TX is output
 	_ODCB9 	= 1;	// For half-duplex mode RP9 is open collector
}


int main(void)
{	
	/* disable interrupts */
	cli();

	/* TODO: eeprom magic number */

	/* remapeable pins */
	io_pins_init();

	/* brake motors */
	BRAKE_ON();
	
	/* oscillator */
	oscillator_init();

	/* reset data structures */
	memset(&gen, 0, sizeof(gen));
	memset(&mainboard, 0, sizeof(mainboard));

	/* init flags */
	mainboard.flags = DO_ENCODERS | DO_RS | DO_BD
							| DO_POS | DO_POWER;
	
	beaconboard.opponent_x = I2C_OPPONENT_NOT_THERE;
	beaconboard.opponent2_x = I2C_OPPONENT_NOT_THERE;

	/* UART */
	uart_init();
	uart_register_rx_event(CMDLINE_UART, emergency);

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

	/* ENCODERS */
	encoders_dspic_init();

	/* I2C */
#ifdef notuse
	i2c_init();
	i2c_register_read_event(i2c_read_event);
	i2c_register_write_event(i2c_write_event);
	i2c_protocol_init();
#endif

	/* DAC_MC */
#ifdef HACK_EUROBOT_2012
	pwm_mc_channel_init(LEFT_MOTOR,
	                    PWM_MC_MODE_SIGNED | PWM_MC_MODE_SIGN_INVERTED, 
	                    2, 1, &PORTC, 7, &PORTC, 0);

	pwm_mc_channel_init(RIGHT_MOTOR,
	                    PWM_MC_MODE_SIGNED, 
	                    1, 1, &PORTB, 15, &PORTC, 1);

	pwm_mc_init(LEFT_MOTOR,  15000, CH1_IND&PEN1H&PDIS1L);
	pwm_mc_init(RIGHT_MOTOR, 15000, CH1_IND&PEN1H&PDIS1L);
#else
	pwm_mc_channel_init(LEFT_MOTOR,
	                    PWM_MC_MODE_SIGNED, 
	                    1, 1, &PORTB, 15, &PORTC, 6);
	pwm_mc_channel_init(RIGHT_MOTOR,
	                    PWM_MC_MODE_SIGNED, 
	                   1, 2, &PORTB, 13, &PORTC, 7);

	pwm_mc_init(LEFT_MOTOR, 15000, CH1_IND&PEN1H&PDIS1L &
											 CH2_IND&PEN2H&PDIS2L);
	pwm_mc_init(RIGHT_MOTOR, 15000, CH1_IND&PEN1H&PDIS1L &
											  CH2_IND&PEN2H&PDIS2L);
#endif
                    												 
	pwm_mc_set(LEFT_MOTOR, 0);
	pwm_mc_set(RIGHT_MOTOR, 0);


	/* MAIN TIMER */
	main_timer_init();

	/* SCHEDULER */
	scheduler_init();

	/* EVENTS OR INIT MODULES THAT INCLUDE EVENTS */

	/* time */
	time_init(EVENT_PRIORITY_TIME);

	/* all cs management */
	maindspic_cs_init();

	/* sensors, will also init hardware adc */
	sensor_init();

	/* i2c slaves polling (gpios and slavedspic) */
#ifdef notuse
	scheduler_add_periodical_event_priority(i2c_poll_slaves, NULL,
						EVENT_PERIOD_I2C_POLL / SCHEDULER_UNIT, EVENT_PRIORITY_I2C_POLL);

	/* beacon commnads and polling */
	scheduler_add_periodical_event_priority(beacon_protocol, NULL,
					EVENT_PERIOD_BEACON_PULL / SCHEDULER_UNIT, EVENT_PRIORITY_BEACON_POLL);

	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
						1000000 / SCHEDULER_UNIT, EVENT_PRIORITY_LED);#endif

	/* strat-related event */
	//scheduler_add_periodical_event_priority(strat_event, NULL,
	//					EVENT_PERIOD_STRAT / SCHEDULER_UNIT, EVENT_PRIORITY_STRAT);


	/* SERVOS AX12 */
	ax12_user_init();

	/* log setup */
 	gen.logs[0] = E_USER_STRAT;
 	//gen.logs[1] = E_USER_I2C_PROTO;
 	//gen.logs[2] = E_USER_BEACON;
 	//gen.logs[2] = E_OA;
 	gen.log_level = 5;
	
	/* reset strat infos */
	//strat_reset_infos();

	/* enable interrupts */
	sei();

	/* wait to init of slavedspic */
	wait_ms(2000);
	
	/* say hello */
	printf("\r\n");
	printf("Siempre falta tiempo para hacer pruebas!!\r\n");

	/* init cmdline */
	cmdline_init();


	/* start */
	strat_start_match(0);

	/* main loop */
	while(1)	{
		cmdline_interact_nowait();
		
		/* add here main program */
	}

	return 0;
}










