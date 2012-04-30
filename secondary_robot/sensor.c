/*  
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
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
 *  Revision : $Id: sensor.c,v 1.7 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  sensor.c,v 1.7 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdlib.h>

#include <aversive.h>
#include <aversive/error.h>
#include <scheduler.h>

#include "main.h"
#include "sensor.h"
#include "strat.h"
#include "strat_utils.h"

/************ ADC */

#ifdef notuse

/* config init */
static void adc_init(void)
{
	/* adc off */
	_ADON = 0;

	/* 3V external reference  */
	_VCFG = 0b011;


	/* by default: 10 bit mode and ADCLK from TCY */
	
	/* Adquisition and conversion time (TSAM, TAD, TCONV):

		_SAMC =	11111 = 31 TAD = TSAM
					•
					•
					•
					00001 = 1 TAD
					00000 = 0 TAD

		_ADCS =	00111111 = TCY · (ADCS<7:0> + 1) = 64 · TCY = TAD
					.
					.
					. 	
					00000010 = TCY · (ADCS<7:0> + 1) = 3 · TCY = TAD 
					00000001 = TCY · (ADCS<7:0> + 1) = 2 · TCY = TAD
					00000000 = TCY · (ADCS<7:0> + 1) = 1 · TCY = TAD

		TADmin = 76 ns

		TCONV = 12 • TAD

		FCONVmax = 1.1 Msps 

	*/

	_SSRC = 0b111;			/* TSAM auto with internal counter */
	_SAMC = 0b11111;		/* TSAM = 31· TCY = 775 ns */
	_ADCS = 0b00111111;	/* TAD = 64· TCY = 1.6 us, TCONV = 19.2 us (50 Ksps max) */ 

	_ASAM = 0;
	
	/* interrupt */
	_AD1IF = 0;
	_AD1IE = 1;

	/* adc on */
	_ADON = 1;
}


/* launch new adquisition */
void adc_launch(uint16_t conf)
{
	AD1CHS0 = conf;
	
	/* lauch conversion */
	_SAMP = 1;
}


/* adc interrupt */
static void adc_event(uint16_t result);
void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void)
{
  	_AD1IF=0;

	/* XXX suposse _DONE = 1 */
	adc_event(ADCBUF0);
}

struct adc_infos {
	uint16_t config;
	int16_t value;
	int16_t prev_val;
   int16_t (*filter)(struct adc_infos *, int16_t);
};


/* reach 90% of the value in 4 samples */
int16_t rii_light(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + (int32_t)adc->prev_val / 2;
	return adc->prev_val / 2;
}

/* reach 90% of the value in 8 samples */
int16_t rii_medium(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + ((int32_t)adc->prev_val * 3) / 4;
	return adc->prev_val / 4;
}

/* reach 90% of the value in 16 samples */
int16_t rii_strong(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + ((int32_t)adc->prev_val * 7) / 8;
	return adc->prev_val / 8;
}

#define ADC_CONF(x) (x)

/* define which ADC to poll, see in sensor.h */
static struct adc_infos adc_infos[ADC_MAX] = { 
	[ADC_LASER_1] = { .config = ADC_CONF(7), .filter = NULL },
	[ADC_LASER_2] = { .config = ADC_CONF(6), .filter = NULL },
};

/* call on adc interrupt */
static void adc_event(uint16_t result)
{
	static uint8_t i = 0;

	/* filter value if needed */
	if (adc_infos[i].filter)
		adc_infos[i].value = adc_infos[i].filter(&adc_infos[i], result);
	else
		adc_infos[i].value = result;

	i ++;
	if (i >= ADC_MAX)
		i = 0;
	else
		adc_launch(adc_infos[i].config);
}

/* called every 10 ms, see init below */
static void do_adc(void *dummy) 
{
	/* launch first conversion */
	adc_launch(adc_infos[0].config);
}

/* get analog sensor value */
int16_t sensor_get_adc(uint8_t i)
{
	int16_t tmp;
	uint8_t flags;

	IRQ_LOCK(flags);
	tmp = adc_infos[i].value;
	IRQ_UNLOCK(flags);
	return tmp;
}

/* get laser distance in mm */

typedef struct {
	int16_t offset_code;
	int16_t offset_mm;
	double gain_mm_code;
} laser_calib_t;


#define LASER_L_D_MIN_MM 		120

#define LASER_L_D_CAL_MM 		2080
#define LASER_L_D_MIN_CODE		1
#define LASER_L_D_CAL_CODE		468
#define LASER_L_G_MM_CODE		4.4540

#define LASER_R_D_MIN_MM 		130
#define LASER_R_D_CAL_MM 		2087
#define LASER_R_D_MIN_CODE		1
#define LASER_R_D_CAL_CODE		850
#define LASER_R_G_MM_CODE		2.4581

#define LASER_D_CENTER		 	13
#define LASER_D_OUT_OF_RANGE	5000

const laser_calib_t laser_calib[2] = {
	[ADC_LASER_R] = { LASER_R_D_MIN_CODE, LASER_R_D_MIN_MM, LASER_R_G_MM_CODE },
	[ADC_LASER_L] = { LASER_L_D_MIN_CODE, LASER_L_D_MIN_MM, LASER_L_G_MM_CODE },
};

int16_t sensor_get_laser_distance(uint8_t i)
{
	double d = LASER_D_OUT_OF_RANGE;
	int16_t value_code;
	
	/* get code */
	value_code = sensor_get_adc(i);

	/* if code is more than code of minimun distance */
	if(value_code > (3*laser_calib[i].offset_code)) {

		/* convert to distance */
		d = (value_code - laser_calib[i].offset_code) * laser_calib[i].gain_mm_code;
		d += laser_calib[i].offset_mm;
		d += (ROBOT_WIDTH/2.0);
	}

	return (int16_t)d;
}

/* get distance and angle (+/- PI) of laser point */
int16_t sensor_get_laser_point_da(uint8_t i, int16_t *d, double *a_rad)
{
	double d_laser = sensor_get_laser_distance(i);
	double d_pt, a_pt_rad;

	/* return if no valid laser distance */
	if(((int16_t) d_laser ) == LASER_D_OUT_OF_RANGE)
		return 0;

#ifdef LASER_POINT_WITH_PRECISSION
	/* distance and relative angle to laser point */
	d_pt = norm(d_laser, LASER_D_CENTER);
	a_pt_rad = acos(d_laser/d_pt);	
	
	if(i == ADC_LASER_R)
		a_pt_rad = -(M_PI/2) - a_pt_rad;
	else
		a_pt_rad = 	(M_PI/2) + a_pt_rad;
#else

	d_pt = d_laser;
	if(i == ADC_LASER_R)
		a_pt_rad = -(M_PI/2);
	else
		a_pt_rad = 	(M_PI/2);
	
#endif
	
	*d = (int16_t)d_pt;
	*a_rad = a_pt_rad;

	return 1;
}

#endif

/************ boolean sensors */


struct sensor_filter {
	uint8_t filter;
	uint8_t prev;
	uint8_t thres_off;
	uint8_t thres_on;
	uint8_t cpt;
	uint8_t invert;
};

static struct sensor_filter sensor_filter[SENSOR_MAX] = {
	[S_START_SWITCH] 	= { 1, 0, 0, 1, 0, 1 }, /* 0 */
	[S_COLOR_SWITCH] 	= { 1, 0, 0, 1, 0, 1 }, /* 1 */
	[S_OPP_FRONT_R] 	= { 1, 0, 0, 1, 0, 1 }, /* 2 */
	[S_OPP_FRONT_L] 	= { 1, 0, 0, 1, 0, 1 }, /* 3 */
	[S_OPP_REAR] 		= { 1, 0, 0, 1, 0, 1 }, /* 4 */

#ifdef notuse	
	[S_GP0_0] = { 1, 0, 0, 1, 0, 1 }, /* 8 */
	[S_GP0_1] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_2] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_3] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_4] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_5] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_6] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_7] = { 1, 0, 0, 1, 0, 1 }, /* 15 */

	[S_GP1_0] = { 1, 0, 0, 1, 0, 1 }, /* 16 */
	[S_GP1_1] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_2] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_3] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_4] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_5] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_6] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_7] = { 1, 0, 0, 1, 0, 1 }, /* 23 */

	[S_GP2_0] = { 1, 0, 0, 1, 0, 0 }, /* 24 */
	[S_GP2_1] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP2_2] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP2_3] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP2_4] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP2_5] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP2_6] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP2_7] = { 1, 0, 0, 1, 0, 0 }, /* 31 */

	[S_GP3_0] = { 1, 0, 0, 1, 0, 1 }, /* 32 */
	[S_GP3_1] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP3_2] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP3_3] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP3_4] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP3_5] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP3_6] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP3_7] = { 1, 0, 0, 1, 0, 0 }, /* 39 */
#endif
};

/* value of filtered sensors */
static uint64_t sensor_filtered = 0;

/* sensor mapping : 
 * 0:  	 PORTA 9 (START)
 * 1:  	 PORTA 4 (COLOR)
 * 2:  	 PORTC 8 (OPP_FRONT_R)
 * 3:  	 PORTC 9 (OPP_FRONT_L)
 * 4:  	 PORTB 3 (OPP_REAR)
 * 5-7:   reserved
 * 8-15:  i2c GP0
 * 16-23: i2c GP1
 * 24-31: i2c GP2
 * 32-39: i2c GP3
 */

uint64_t sensor_get_all(void)
{
	uint64_t tmp;
	uint8_t flags;
	IRQ_LOCK(flags);
	tmp = sensor_filtered;
	IRQ_UNLOCK(flags);
	return tmp;
}

uint8_t sensor_get(uint8_t i)
{
	uint64_t tmp = sensor_get_all();
	return (uint8_t)((tmp & ((uint64_t)1 << i))>>i);
}

/* get the physical value of pins */
static uint64_t sensor_read(void)
{
	uint64_t tmp = 0;

	tmp |= (uint64_t)((PORTA & (_BV(9))) >> 9) << 0;
	tmp |= (uint64_t)((PORTA & (_BV(4))) >> 4) << 1;
	tmp |= (uint64_t)((PORTC & (_BV(8))) >> 8) << 2;
	tmp |= (uint64_t)((PORTC & (_BV(9))) >> 9) << 3;
	tmp |= (uint64_t)((PORTB & (_BV(3))) >> 3) << 4;

	/* 5 to 7 reserved */

#if notuse
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio0))<< 8;
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio1))<< 16;
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio2))<< 24;
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio3))<< 32;
#endif

	/* add reserved sensors here */
	return tmp;
}

/* called every 10 ms, see init below */
static void do_boolean_sensors(void *dummy)
{
	uint8_t i;
	uint8_t flags;
	uint64_t sensor = sensor_read();
	uint64_t tmp = 0;

	for (i=0; i<SENSOR_MAX; i++) {
		
		if(sensor_filter[i].filter == 0)
			continue;
		
		if (((uint64_t)1 << i) & sensor) {
			if (sensor_filter[i].cpt < sensor_filter[i].filter)
				sensor_filter[i].cpt++;
			if (sensor_filter[i].cpt >= sensor_filter[i].thres_on)
				sensor_filter[i].prev = 1;
		}
		else {
			if (sensor_filter[i].cpt > 0)
				sensor_filter[i].cpt--;
			if (sensor_filter[i].cpt <= sensor_filter[i].thres_off)
				sensor_filter[i].prev = 0;
		}
		
		if (sensor_filter[i].prev && !sensor_filter[i].invert) {
			tmp |= ((uint64_t)1 << i);
		}
		else if (!sensor_filter[i].prev && sensor_filter[i].invert) {
			tmp |= ((uint64_t)1 << i);
		}

	}
	IRQ_LOCK(flags);
	sensor_filtered = tmp;
	IRQ_UNLOCK(flags);
}

/* virtual obstacle */
#define DISABLE_CPT_MAX 500
static uint16_t disable = 0; 	/* used to disable obstacle detection 
			   				 		 * during some time */

/* called every 10 ms */
void sensor_obstacle_update(void)
{
	if (disable > 0) {
		disable --;
		if (disable == 0)
			DEBUG(E_USER_STRAT, "re-enable sensor");
	}
}

void sensor_obstacle_disable(void)
{
	DEBUG(E_USER_STRAT, "disable sensor");
	disable = DISABLE_CPT_MAX;
}

void sensor_obstacle_enable(void)
{
	disable = 0;
}

uint8_t sensor_obstacle_is_disabled(void)
{
	return disable;
}


/************ global sensor init */

/* called every 10 ms, see init below */
static void do_sensors(void *dummy)
{
#ifdef notuse
	do_adc(NULL);
#endif

	do_boolean_sensors(NULL);
	sensor_obstacle_update();
}

void sensor_init(void)
{
#ifdef notuse
	adc_init();
#endif

	/* CS EVENT */
	scheduler_add_periodical_event_priority(do_sensors, NULL, 
						EVENT_PERIOD_SENSORS / SCHEDULER_UNIT, EVENT_PRIORITY_SENSORS);
}

