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
#define SIDE_FRONT 		I2C_SIDE_FRONT
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
#define ACC_DIST  5 //40.
#define ACC_ANGLE 5 //90.

/* default speeds */

//Do not change
#define SPEED_DIST_FAST 			2500.
#define SPEED_ANGLE_FAST 			2500.
#define SPEED_DIST_SLOW 		  	1000.
#define SPEED_ANGLE_SLOW 		  	1000.
#define SPEED_DIST_VERY_SLOW 		500.
#define SPEED_ANGLE_VERY_SLOW 		500.

#endif



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
	//strat_zones zones[ZONES_MAX];

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

//extern char numzone2name[ZONES_MAX + 1][3];

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

/* set current bt task */
void strat_bt_task_rqst (uint8_t task_id, 
						int16_t a, int16_t b, 
						int16_t c, int16_t d, int16_t e);

/* never returns */
void strat_bt_task_scheduler (void);


#else /* HOST_VERSION_OA_TEST */

#define AREA_BBOX_6X5	0
#define AREA_BBOX_4X4	1
void strat_set_bounding_box(uint8_t type);

#endif /* HOST_VERSION_OA_TEST */


#endif
