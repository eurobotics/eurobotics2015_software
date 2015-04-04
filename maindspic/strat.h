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
 *  Javier Bali�as Santos <javier@arc-robots.org> and Silvia Santano
 */

#ifndef _STRAT_H_
#define _STRAT_H_

#ifndef HOST_VERSION_OA_TEST
 #include <clock_time.h>
#endif

/* compilation flavours */
//#define HOMOLOGATION

/* area */
#define AREA_X 3000
#define AREA_Y 2000

#define LIMIT_BBOX_Y_UP			(2000 - OBS_CLERANCE-70)
#define LIMIT_BBOX_Y_DOWN		OBS_CLERANCE+100
#define LIMIT_BBOX_X_UP			3000 - OBS_CLERANCE
#define LIMIT_BBOX_X_DOWN		OBS_CLERANCE

/* position of the elements */

#define MY_STAND_1_X			90
#define MY_STAND_1_Y			2000-200
#define MY_STAND_2_X			850
#define MY_STAND_2_Y			2000-100
#define MY_STAND_3_X			850
#define MY_STAND_3_Y			2000-200
#define MY_STAND_4_X			870
#define MY_STAND_4_Y			2000-1355
#define MY_STAND_5_X			1300
#define MY_STAND_5_Y			2000-1400
#define MY_STAND_6_X			1100
#define MY_STAND_6_Y			2000-1770
#define MY_STAND_7_X			90
#define MY_STAND_7_Y			2000-1850
#define MY_STAND_8_X			90
#define MY_STAND_8_Y			2000-1750	

#define OPP_STAND_1_X			3000-90
#define OPP_STAND_1_Y			2000-200
#define OPP_STAND_2_X			3000-850
#define OPP_STAND_2_Y			2000-100
#define OPP_STAND_3_X			3000-850
#define OPP_STAND_3_Y			2000-200
#define OPP_STAND_4_X			3000-870
#define OPP_STAND_4_Y			2000-1355
#define OPP_STAND_5_X			3000-1300
#define OPP_STAND_5_Y			2000-1400
#define OPP_STAND_6_X			3000-1100
#define OPP_STAND_6_Y			2000-1770
#define OPP_STAND_7_X			3000-90
#define OPP_STAND_7_Y			2000-1850
#define OPP_STAND_8_X			3000-90
#define OPP_STAND_8_Y			2000-1750

#define MY_LIGHTBULB_HOME_X				40
#define MY_LIGHTBULB_HOME_Y					1000
#define MY_LIGHTBULB_PLATFORM_X		1250
#define MY_LIGHTBULB_PLATFORM_Y			50
#define OPP_LIGHTBULB_HOME_X			2960
#define OPP_LIGHTBULB_HOME_Y				1000
#define OPP_LIGHTBULB_PLATFORM_X	1750
#define OPP_LIGHTBULB_PLATFORM_Y		50

#define MY_POPCORNMAC_1_X				300
#define MY_POPCORNMAC_1_Y				2000-35
#define MY_POPCORNMAC_2_X				600
#define MY_POPCORNMAC_2_Y				2000-35
#define OPP_POPCORNMAC_1_X				2400
#define OPP_POPCORNMAC_1_Y				2000-35
#define OPP_POPCORNMAC_2_X				2700
#define OPP_POPCORNMAC_2_Y		        2000-35

#define MY_POPCORNCUP_FRONT_X				910
#define MY_POPCORNCUP_FRONT_Y				2000-830
#define MY_POPCORNCUP_SIDE_X				250
#define MY_POPCORNCUP_SIDE_Y				250
#define OPP_POPCORNCUP_FRONT_X				2090
#define OPP_POPCORNCUP_FRONT_Y				2000-830
#define OPP_POPCORNCUP_SIDE_X				    2750
#define OPP_POPCORNCUP_SIDE_Y				    2000-1750
#define POPCORNCUP_CENTRE_X				1500
#define POPCORNCUP_CENTRE_Y				2000-1650

#define MY_CINEMA_UP_X					2800
#define MY_CINEMA_UP_Y					1400
#define MY_CINEMA_DOWN_X				2800
#define MY_CINEMA_DOWN_Y				600
#define OPP_CINEMA_UP_X					200
#define OPP_CINEMA_UP_Y					1400
#define OPP_CINEMA_DOWN_X				200
#define OPP_CINEMA_DOWN_Y				600

#define MY_STAIRS_X						1250
#define MY_STAIRS_Y						1400
#define OPP_STAIRS_X					1750
#define OPP_STAIRS_Y					1400

#define MY_HOME_X				250
#define MY_HOME_Y				1000
#define OPP_HOME_X				2750
#define OPP_HOME_Y				1000

#define MY_CLAP_1_X				320
#define MY_CLAP_2_X				920
#define MY_CLAP_3_X				2320
#define OPP_CLAP_1_X				2680
#define OPP_CLAP_2_X				620
#define OPP_CLAP_3_X				2080
#define CLAP_Y						0


/* convert coords according to our color */
#define COLOR_Y(y)     (y)
#define COLOR_X(x)     ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (AREA_X-(x)))

#define COLOR_A_REL(a) ((mainboard.our_color==I2C_COLOR_YELLOW)? (a) : (-a))
#define COLOR_A_ABS(a) ((mainboard.our_color==I2C_COLOR_YELLOW)? (a) : (180-a))

#define COLOR_SIGN(x)  ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (-x))
#define COLOR_INVERT(x)((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (!x))

#define COLOR_I(x)	  ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) :  ((NB_SLOT_X-1)-x))

#define START_X 200
#define START_Y COLOR_Y(200)
#define START_A COLOR_A(45)

#define CENTER_X 1500
#define CENTER_Y 1000

#define SIDE_REAR		I2C_SIDE_REAR
#define SIDE_FRONT 	    I2C_SIDE_FRONT 
#define SIDE_MAX		I2C_SIDE_MAX

#define OPPOSITE_SIDE(side) ((side==I2C_SIDE_FRONT)? (I2C_SIDE_REAR) : (I2C_SIDE_FRONT))	

#define GO_FORWARD	    0
#define GO_BACKWARD	    1

/* useful traj flags */
#define TRAJ_SUCCESS(f) 			(f & (END_TRAJ|END_NEAR))
#define TRAJ_BLOCKING(f) 			(f & (END_BLOCKING))

#define TRAJ_FLAGS_STD 				(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER 		(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR 			(END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST 		(END_TRAJ|END_BLOCKING|END_INTR)

#define LAST_SECONDS_TIME	80

//#define CALIBRATION
#ifdef CALIBRATION

/* default acc */
#define ACC_DIST  10
#define ACC_ANGLE 10

/* default speeds */
#define SPEED_DIST_VERY_FAST 	1000.
#define SPEED_ANGLE_VERY_FAST 	1000.
#define SPEED_DIST_FAST 		1000.
#define SPEED_ANGLE_FAST 		1000.
#define SPEED_DIST_SLOW 		1000.
#define SPEED_ANGLE_SLOW 		1000.
#define SPEED_DIST_VERY_SLOW 	1000.
#define SPEED_ANGLE_VERY_SLOW	1000.

#else

/* default acc */
#define ACC_DIST  75
#define ACC_ANGLE 75

/* default speeds */
#ifdef HOMOLOGATION
#define SPEED_DIST_FAST 		2000
#define SPEED_ANGLE_FAST 		2000
#else

//#define SPEED_DIST_VERY_FAST 	4500 /* XXX very clean wheels */
//#define SPEED_ANGLE_VERY_FAST 4500
#define SPEED_DIST_VERY_FAST 	4000
#define SPEED_ANGLE_VERY_FAST 	4000
#define SPEED_DIST_FAST 		3000
#define SPEED_ANGLE_FAST 		3000
#endif

//Do not change
#define SPEED_DIST_SLOW 		2000
#define SPEED_ANGLE_SLOW 		2000
#define SPEED_DIST_VERY_SLOW 	500
#define SPEED_ANGLE_VERY_SLOW   500

#endif

/* zones */
#define ZONE_MY_STAND_GROUP_1			0
#define ZONE_MY_STAND_GROUP_2			1
#define ZONE_MY_STAND_GROUP_3			2
#define ZONE_MY_STAND_GROUP_4			3
#define ZONE_MY_POPCORNMAC				4
#define ZONE_OPP_POPCORNMAC				5
#define ZONE_POPCORNCUP_1				6
#define ZONE_POPCORNCUP_2				7
#define ZONE_POPCORNCUP_3				8
#define ZONE_POPCORNCUP_4				9
#define ZONE_POPCORNCUP_5				10
#define ZONE_MY_CINEMA_UP				11
#define ZONE_MY_CINEMA_DOWN				12
#define ZONE_MY_STAIRS					13
#define ZONE_MY_HOME					14
#define ZONE_MY_CLAP_1					15
#define ZONE_MY_CLAP_2					16
#define ZONE_MY_CLAP_3					17
#define ZONE_MY_STAIRWAY				18
#define ZONES_MAX		    			19



/************************************************************* 
 * Strat data structures 
 ************************************************************/

/* boulding box */
struct bbox {
	int32_t x1;
	int32_t y1;
	int32_t x2;
	int32_t y2;
};

/* configuration */
struct conf {

/* depends on flags the robot
 * will do different things */
	uint8_t flags;
#define CONF_FLAG_XXX   1
};


/* strat structure */
typedef struct {
	/* type */
	uint16_t type;
	#define ZONE_TYPE_STAND							0
	#define ZONE_TYPE_LIGHTBULB					1
	#define ZONE_TYPE_POPCORNMAC			2
	#define ZONE_TYPE_POPCORNCUP				3
	#define ZONE_TYPE_HOME							4
	#define ZONE_TYPE_CINEMA						5
	#define ZONE_TYPE_STAIRS						6
	#define ZONE_TYPE_STAIRWAY					7
	#define ZONE_TYPE_CLAP							8
	#define ZONE_TYPE_BAREA							9
	#define ZONE_TYPE_MAX								10

        /* target point */
	int16_t x;
	int16_t y;
	/* boundinbox */
	int16_t x_down;
	int16_t x_up;
	int16_t y_down;
	int16_t y_up;

	/* init point */
	int16_t init_x;
	int16_t init_y;

	/* priority */
	uint8_t prio;
	#define ZONE_PRIO_0		0
	#define ZONE_PRIO_10		10
	#define ZONE_PRIO_20		20
	#define ZONE_PRIO_30		30
	#define ZONE_PRIO_40		40
	#define ZONE_PRIO_50		50
	#define ZONE_PRIO_60		60
	#define ZONE_PRIO_70		70
	#define ZONE_PRIO_80		80
	#define ZONE_PRIO_90		90
	#define ZONE_PRIO_100	   100
	#define ZONE_PRIO_MAX	   100

	uint16_t flags;
	#define ZONE_CHECKED	 	1
	#define ZONE_CHECKED_OPP	2
	#define ZONE_SEC_ROBOT	   	4
	#define ZONE_AVOID		   	8
  
#ifndef HOST_VERSION_OA_TEST
	/* opponent statistics */
	microseconds opp_time_zone_us;
	microseconds last_time_opp_here; 	/*in us, since beginning of the match*/
#endif
	
	/* which robots can perform this action */
	uint8_t robot;
	#define MAIN_ROBOT  0
	#define SEC_ROBOT   1
	#define BOTH_ROBOTS 2
	
} strat_zones_t;

/* information about strat stuff */
struct strat_infos {
	uint8_t dump_enabled;
	uint8_t debug_step;
	uint8_t current_sec_strategy;
	struct bbox area_bbox;

    /* strat configuration */
	struct conf conf;

	/* points areas */
	strat_zones_t zones[ZONES_MAX];
	
	/* our zone position */
	uint8_t current_zone;
	uint8_t goto_zone;
	uint8_t last_zone;

	/* state of the robot */
	uint8_t lightbulbs;
	uint8_t stands;
	uint8_t carpets;
	uint8_t popcornbaskets;

	/* opponent zone position */
	uint8_t opp_current_zone;
	uint8_t opp2_current_zone;

	/* opponent statistics */
	uint8_t opp_score;
};

extern struct strat_infos strat_infos;

/* get zone struct index */
extern char numzone2name[ZONES_MAX+1][5];

/* points we get from each zone */
extern uint8_t strat_zones_points[ZONES_MAX];

#ifndef HOST_VERSION_OA_TEST

/************************************************************* 
 * Functions headers of strat files
 ************************************************************/

/********************************************
 * in strat.c 
 *******************************************/
void strat_set_bounding_box(uint8_t type);

void strat_dump_infos(const char *caller);
void strat_dump_conf(void);
void strat_reset_infos(void);

void strat_preinit(void);
void strat_init(void);
void strat_exit(void);

uint8_t strat_main(void);
uint8_t strat_begin(void);
uint8_t strat_begin_alcabot (void);

void strat_event(void *dummy);
void strat_event_enable(void);
void strat_event_disable(void);

/********************************************
 * in strat_spotlight.c 
 *******************************************/


/********************************************
 * in strat_popcorn.c 
 *******************************************/

 
/********************************************
 * in strat_main.c 
 *******************************************/

uint8_t strat_main_loop(void);

/* return new work zone, -1 if any zone is found */
int8_t strat_get_new_zone(void);

/* return END_TRAJ if zone is reached */
uint8_t strat_goto_zone(uint8_t zone_num);

/* return END_TRAJ if the work is done */
uint8_t strat_work_on_zone(uint8_t zone_num);

/* debug state machines step to step */
void state_debug_wait_key_pressed(void);

/* smart play */
//#define DEBUG_STRAT_SMART
uint8_t strat_smart(void);
void recalculate_priorities(void);

/* tracking of zones where opp has been working */
void strat_opp_tracking (void);


/* Sets of  strategies for secondary robot*/
void set_strat_sec_1(void);
void set_strat_sec_2(void);
void set_strat_sec_3(void);
void set_strat_main_1(void);
void set_next_sec_strategy(void);

/* homologation */
void strat_homologation(void);
void strat_initial_move(void);

#else /* HOST_VERSION_OA_TEST */

void strat_set_bounding_box(uint8_t type);

#endif /* HOST_VERSION_OA_TEST */


#endif
