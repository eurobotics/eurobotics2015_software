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
 *  Javier Baliñas Santos <javier@arc-robots.org> and Silvia Santano
 */

#ifndef _STRAT_H_
#define _STRAT_H_

/* homologation compilation */
//#define HOMOLOGATION

/* area */
#define AREA_X 3000
#define AREA_Y 2000

/* XXX obstacle clerance */
#define OBS_CLERANCE   240.0

/* position of the tokens */
#define TOTEM_1_X               1100
#define TOTEM_2_X               1900
#define TOTEM_1_Y               AREA_Y/2
#define TOTEM_2_Y               TOTEM_1_Y
#define TOTEM_WIDE              250
#define PURPLE_FLOOR_COIN_1_X      2000 
#define PURPLE_FLOOR_COIN_1_Y      500 
#define PURPLE_FLOOR_COIN_2_X      2150 
#define PURPLE_FLOOR_COIN_2_Y      1000 
#define PURPLE_FLOOR_COIN_3_X      2550 
#define PURPLE_FLOOR_COIN_3_Y      1700 
#define RED_FLOOR_COIN_1_X   1000 
#define RED_FLOOR_COIN_1_Y   500 
#define RED_FLOOR_COIN_2_X   850 
#define RED_FLOOR_COIN_2_Y   1000 
#define RED_FLOOR_COIN_3_X   450
#define RED_FLOOR_COIN_3_Y   1700  
#define BOTTLE_Y                2000 
#define PURPLE_BOTTLE_1_X          2360   
#define PURPLE_BOTTLE_2_X          1117   
#define RED_BOTTLE_1_X       640  
#define RED_BOTTLE_2_X       1883  
#define FLOOR_COINS_GROUP_X     1500
#define FLOOR_COINS_GROUP_Y     1700
#define MIDDLE_FLOOR_GOLDBAR_X  1353
#define MIDDLE_FLOOR_GOLDBAR_Y  1500
#define RED_FLOOR_GOLDBAR_X  420
#define RED_FLOOR_GOLDBAR_Y  785
#define PURPLE_FLOOR_GOLDBAR_X     2580
#define PURPLE_FLOOR_GOLDBAR_Y     785

/* infos about strat */
#define NB_SLOT_X				8
#define NB_SLOT_Y				6
#define NB_SLOT_GREEN		5
#define NB_GRID_LINES_X 	9
#define NB_GRID_LINES_Y 	7


/* convert coords according to our color */
#define COLOR_Y(y)     (y)
#define COLOR_X(x)     ((mainboard.our_color==I2C_COLOR_RED)? (x) : (AREA_X-(x)))

#define COLOR_A_REL(a) ((mainboard.our_color==I2C_COLOR_RED)? (a) : (-a))
#define COLOR_A_ABS(a) ((mainboard.our_color==I2C_COLOR_RED)? (a) : (180-a))

#define COLOR_SIGN(x)  ((mainboard.our_color==I2C_COLOR_RED)? (x) : (-x))
#define COLOR_INVERT(x)((mainboard.our_color==I2C_COLOR_RED)? (x) : (!x))

#define COLOR_I(x)	  ((mainboard.our_color==I2C_COLOR_RED)? (x) :  ((NB_SLOT_X-1)-x))

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

#define TRAJ_FLAGS_STD 					(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER 			(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR 			(END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)

#ifndef HOMOLOGATION
#define TRAJ_FLAGS_SMALL_DIST 		(END_TRAJ|END_BLOCKING|END_INTR)
#else
#define TRAJ_FLAGS_SMALL_DIST 		(END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#endif

/* default speeds */
#ifdef HOMOLOGATION
#define SPEED_DIST_FAST 		1400
#define SPEED_ANGLE_FAST 		1400
#else
#define SPEED_DIST_FAST 		2800
#define SPEED_ANGLE_FAST 		2800
#endif

//Do not change
#define SPEED_DIST_SLOW 		1400
#define SPEED_ANGLE_SLOW 		1400
#define SPEED_DIST_VERY_SLOW 	350
#define SPEED_ANGLE_VERY_SLOW 350


#define PLACE_D_SAFE				  140

/* zones */
#define ZONE_TOTEM_1_SIDE_1			      0
#define ZONE_TOTEM_1_SIDE_2       	      1
#define ZONE_TOTEM_2_SIDE_1			      2
#define ZONE_TOTEM_2_SIDE_2			      3
#define ZONE_RED_FLOOR_COIN_1			      4
#define ZONE_RED_FLOOR_COIN_2  	         5
#define ZONE_RED_FLOOR_COIN_3 	       	6
#define ZONE_RED_FLOOR_GOLDBAR   	      7
#define ZONE_PURPLE_FLOOR_COIN_1	         8
#define ZONE_PURPLE_FLOOR_COIN_2	         9
#define ZONE_PURPLE_FLOOR_COIN_3	         10
#define ZONE_PURPLE_FLOOR_GOLDBAR			11
#define ZONE_RED_BOTTLE_1			         12
#define ZONE_RED_BOTTLE_2   			      13
#define ZONE_PURPLE_BOTTLE_1			      14
#define ZONE_PURPLE_BOTTLE_2			      15
#define ZONE_RED_MAP          	       	16
#define ZONE_PURPLE_MAP          	      17
#define ZONE_FLOOR_COINS_GROUP          	18
#define ZONE_MIDDLE_FLOOR_GOLDBAR         19

/* place zones */
#define ZONE_SHIP_RED_CAPTAINS_BEDRROM	   20
#define ZONE_SHIP_RED_HOLD	               21
#define ZONE_SHIP_RED_DECK                22
#define ZONE_SHIP_PURPLE_CAPTAINS_BEDRROM 23
#define ZONE_SHIP_PURPLE_HOLD             24
#define ZONE_SHIP_PURPLE_DECK             25

/* Store goldbars and coins or not and where */
#define STORE_FRONT                       1
#define STORE_BACK                        2
#define DONT_STORE                        0

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
 * do one things or anothers */
	uint8_t flags;
   #define ZONE_IS_CLOSE		         1
   #define ZONE_IS_CLOSE_TO_OPP			2

	/* thresholds */
	uint8_t th_place_prio;
	uint8_t th_token_score;
};

/* scores */
#define BLACKCOIN_SCORE 0
#define WHITECOIN_SCORE 1
#define BOTTLE_SCORE 5
#define MAP_SCORE 5
#define GOLDBAR_SCORE 3


typedef struct {

	/* boundinbox */
	int16_t x_up;
	int16_t y_up;
	int16_t x_down;
	int16_t y_down;

   /*init points*/
	int16_t init_x;
	int16_t init_y;
	
	uint8_t prio;
   #define ZONE_PRIO_0		0
   #define ZONE_PRIO_1		10
   #define ZONE_PRIO_2		20
   #define ZONE_PRIO_3		30
   #define ZONE_PRIO_4		40
   #define ZONE_PRIO_MAX	100

	uint16_t flags;
   #define ZONE_WITH_TREASURE		2
   #define ZONE_AVOID		    	4
   #define ZONE_CHECKED		    	8
   #define ZONE_CHECKED_OPP		16
   #define ZONE_OPPONENT	   	32
   #define ZONE_OPPONENT_2	   	64
   #define ZONE_ROBOT    	   	128
   #define ZONE_SEC_ROBOT	   	256
} strat_zones;


struct strat_infos {
	uint8_t dump_enabled;
	struct conf conf;
	struct bbox area_bbox;

	/* points areas */
	strat_zones zones[26];
	/* our zone position */
	uint8_t current_zone;
	/* opponent zone position */
	uint8_t opp_current_zone;

	/* opponent stadistics */
	uint32_t opp_time_zone_ms;
};
extern struct strat_infos strat_infos;


#ifndef HOST_VERSION

/************************************************************* 
 * Functions headers of strat files
 ************************************************************/

/********************************************
 * in strat.c 
 *******************************************/
void strat_set_bounding_box(void);

void strat_dump_infos(const char *caller);
void strat_dump_conf(void);
void strat_reset_infos(void);

void strat_preinit(void);
void strat_init(void);
void strat_exit(void);

uint8_t strat_main(void);
uint8_t strat_beginning(void);
void strat_event(void *dummy);



/********************************************
 * in strat_treasure.c 
 *******************************************/

/* pick up the piece of fabric that discover the map */
uint8_t strat_pickup_map(void);

/* empty our totem down side and save treasure on ship */
uint8_t strat_empty_totem(void);

/* save treasure after empty totem */
uint8_t strat_save_treasure_on_ship(void);

/* start a match debuging or not */
void strat_start_match(uint8_t debug);

/* add here more strat functions in files */

#else

void strat_set_bounding_box(void);

#endif /* HOST_VERSION */

#endif
