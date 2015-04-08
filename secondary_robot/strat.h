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
 *  Javier Balias Santos <javier@arc-robots.org> and Silvia Santano
 */

#ifndef _STRAT_H_
#define _STRAT_H_

/* compilation flavours */
//#define HOMOLOGATION
//#define DEMO_MODE

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
#define MY_POPCORNCUP_SIDE_Y				2000-1750
#define OPP_POPCORNCUP_FRONT_X				2090
#define OPP_POPCORNCUP_FRONT_Y				2000-830
#define OPP_POPCORNCUP_SIDE_X				    2750
#define OPP_POPCORNCUP_SIDE_Y				    2000-1750
#define POPCORNCUP_CENTRE_X				1500
#define POPCORNCUP_CENTRE_Y				2000-1650

#define MY_CINEMA_UP_X						200
#define MY_CINEMA_UP_Y						1400
#define MY_CINEMA_DOWN_X					200
#define MY_CINEMA_DOWN_Y					600
#define OPP_CINEMA_UP_X						2800
#define OPP_CINEMA_UP_Y						1400
#define OPP_CINEMA_DOWN_X				2800
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
#define MY_CLAP_2_X				620
#define MY_CLAP_3_X				920
#define OPP_CLAP_1_X				2680
#define OPP_CLAP_2_X				2320
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
#define SIDE_FRONT 	I2C_SIDE_FRONT
#define SIDE_MAX		I2C_SIDE_MAX

#define OPPOSITE_SIDE(side) ((side==I2C_SIDE_FRONT)? (I2C_SIDE_REAR) : (I2C_SIDE_FRONT))

#define GO_FORWARD	0
#define GO_BACKWARD	1

/* useful traj flags */
#define TRAJ_SUCCESS(f) 				(f & (END_TRAJ|END_NEAR))
#define TRAJ_BLOCKING(f) 				(f & (END_BLOCKING))

#define TRAJ_FLAGS_STD 				(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER 		(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR 			(END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST 		(END_TRAJ|END_BLOCKING|END_INTR)

#define LAST_SECONDS_TIME	80


//#define CALIBRATION
#ifdef CALIBRATION

/* default acc */
#define ACC_DIST  5.
#define ACC_ANGLE 5.

/* default speeds */
#define SPEED_DIST_FAST 			1000.
#define SPEED_ANGLE_FAST 			1000.
#define SPEED_DIST_SLOW 		  	1000.
#define SPEED_ANGLE_SLOW 		  	1000.
#define SPEED_DIST_VERY_SLOW 		1000.
#define SPEED_ANGLE_VERY_SLOW 	    1000.

#else

/* default acc */
#define ACC_DIST  30 //40.
#define ACC_ANGLE 30 //90.

/* default speeds */
//#define HOMOLOGATION_SPEED
#ifdef HOMOLOGATION_SPEED
#define SPEED_DIST_FAST 			2000.
#define SPEED_ANGLE_FAST 			2000.
#else

#define SPEED_DIST_FAST 			5000.
#define SPEED_ANGLE_FAST 			5000.
#endif

//Do not change
#define SPEED_DIST_SLOW 		  	2500.
#define SPEED_ANGLE_SLOW 		  	2500.
#define SPEED_DIST_VERY_SLOW 		1000.
#define SPEED_ANGLE_VERY_SLOW 		1000.

#endif

/* zones */
#define ZONE_MY_STAND_1			0
#define ZONE_MY_STAND_2			1
#define ZONE_MY_STAND_3			2
#define ZONE_MY_STAND_4			3
#define ZONE_MY_STAND_5			4
#define ZONE_MY_STAND_6			5
#define ZONE_MY_STAND_7			6
#define ZONE_MY_STAND_8			7
#define ZONE_OPP_STAND_1			8
#define ZONE_OPP_STAND_2			9
#define ZONE_OPP_STAND_3			10
#define ZONE_OPP_STAND_4			11
#define ZONE_OPP_STAND_5			12
#define ZONE_OPP_STAND_6			13
#define ZONE_OPP_STAND_7			14
#define ZONE_OPP_STAND_8			15
#define ZONE_MY_LIGHTBULB_HOME					16
#define ZONE_MY_LIGHTBULB_PLATFORM			17
#define ZONE_OPP_LIGHTBULB_HOME				18
#define ZONE_OPP_LIGHTBULB_PLATFORM		19
#define ZONE_MY_POPCORNMAC_1				20
#define ZONE_MY_POPCORNMAC_2				21
#define ZONE_OPP_POPCORNMAC_1				22
#define ZONE_OPP_POPCORNMAC_2				23
#define ZONE_MY_POPCORNCUP_FRONT				24
#define ZONE_MY_POPCORNCUP_SIDE				25
#define ZONE_OPP_POPCORNCUP_FRONT				26
#define ZONE_OPP_POPCORNCUP_SIDE				    27
#define ZONE_POPCORNCUP_CENTRE				28
#define ZONE_MY_CINEMA_UP						29
#define ZONE_MY_CINEMA_DOWN					30
#define ZONE_OPP_CINEMA_UP						31
#define ZONE_OPP_CINEMA_DOWN				32
#define ZONE_MY_STAIRS						33
#define ZONE_OPP_STAIRS					34
#define ZONE_MY_HOME				35
#define ZONE_OPP_HOME				36
#define ZONE_MY_CLAP_1				37
#define ZONE_MY_CLAP_2				38
#define ZONE_MY_CLAP_3				39
#define ZONE_OPP_CLAP_1				40
#define ZONE_OPP_CLAP_2				41
#define ZONE_OPP_CLAP_3				42
#define ZONE_MY_STAIRWAY_1			43
#define ZONE_MY_STAIRWAY_2			44
#define ZONE_OPP_STAIRWAY_1			45
#define ZONE_OPP_STAIRWAY_2			46
#define ZONES_MAX		    			47

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
  #define ENABLE_R2ND_POS				  1 /* TODO: set by command */
	#define ENABLE_DOWN_SIDE_ZONES	2
};



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
   #define ZONE_PRIO_100	100
   #define ZONE_PRIO_MAX	100

	uint16_t flags;
   #define ZONE_CHECKED		    	1
   #define ZONE_CHECKED_OPP		2
   #define ZONE_SEC_ROBOT	   	4
   #define ZONE_AVOID		    	8

	/* which robots can perform this action */
	uint8_t robot;
	#define MAIN_ROBOT  0
	#define SEC_ROBOT   1
	#define BOTH_ROBOTS 2
} strat_zones;


struct strat_infos {
	uint8_t dump_enabled;
	uint8_t debug_step;
	struct conf conf;
	struct bbox area_bbox;

	/* points areas */
	strat_zones zones[ZONES_MAX];

	/* our zone position */
	uint8_t current_zone;
	uint8_t goto_zone;
	uint8_t last_zone;

	/* state of the robot */
	uint8_t lightbulbs;
	uint8_t stands;
	uint8_t carpets;
	uint8_t popcornbaskets;
};

extern struct strat_infos strat_infos;

extern char numzone2name[ZONES_MAX + 1][3];

#ifndef HOST_VERSION_OA_TEST

/*************************************************************
 * Functions headers of strat files
 ************************************************************/

/********************************************
 * in strat.c
 *******************************************/
#define AREA_BBOX_6X5	0
#define AREA_BBOX_4X4	1
void strat_set_bounding_box(uint8_t type);

void strat_dump_infos(const char *caller);
void strat_dump_conf(void);
void strat_reset_infos(void);

void strat_preinit(void);
void strat_init(void);
void strat_exit(void);

uint8_t strat_main(void);
uint8_t strat_beginning(void);
void strat_event(void *dummy);
void strat_event_enable(void);
void strat_event_disable(void);
void strat_start_match(uint8_t debug);


/********************************************
 * in strat_main.c
 *******************************************/

/* auto possition depending on color */
void strat_auto_position (void);

/* match tasks */
void strat_initial_move(void);

/* bt_tasks */
uint8_t pick_popcorn_cup(void);
uint8_t extend_carpet(void);
uint8_t climb_stairs(void);
uint8_t bring_cup_to_cinema(void);
uint8_t close_clapperboard(void);

/********************************************
 * in strat_event.c
 *******************************************/

/*************** STRAT EVENT SCHEDULE FUNCTIONS *******************************/

/* schedule a single strat tevent */
void strat_event_schedule_single (void (*f)(void *), void * data);

/* schedule a periodical strat tevent */
void strat_event_schedule_periodical(void (*f)(void *), void * data);


/*************** STRAT EVENT WRAPPER FUNCTIONS  *******************************/

/* auto position event */
void strat_auto_position_event (void *data);

/* trajectory functions */
void strat_goto_xy_abs_event (void *data);
void strat_goto_forward_xy_abs_event (void *data);
void strat_goto_backward_xy_abs_event (void *data);
void strat_goto_xy_rel_event (void *data);
/* match tasks */



#else /* HOST_VERSION_OA_TEST */

#define AREA_BBOX_6X5	0
#define AREA_BBOX_4X4	1
void strat_set_bounding_box(uint8_t type);

#endif /* HOST_VERSION_OA_TEST */


#endif
