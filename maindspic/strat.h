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

/* area */
#define AREA_X 3000
#define AREA_Y 2000

/* XXX obstacle clerance */
#define OBS_CLERANCE       230.0
//#define OBS_CLERANCE  205.0    /* when goes backwards */

/* position of the tokens */
#define TOTEM_1_X            1100
#define TOTEM_1_Y            (AREA_Y/2)
#define TOTEM_2_X            1900
#define TOTEM_2_Y            (AREA_Y/2)
#define TOTEM_WIDE           250

#define OUR_TOTEM_X				TOTEM_1_X
#define OUR_TOTEM_Y 				TOTEM_1_Y
#define OPP_TOTEM_X 				TOTEM_2_X
#define OPP_TOTEM_Y 				TOTEM_2_Y

#define OPP_FLOOR_COIN_1_X   2000 
#define OPP_FLOOR_COIN_1_Y   500 
#define OPP_FLOOR_COIN_2_X   2150 
#define OPP_FLOOR_COIN_2_Y   1000 
#define OPP_FLOOR_COIN_3_X   2550 
#define OPP_FLOOR_COIN_3_Y   1700
 
#define OUR_FLOOR_COIN_1_X   1000 
#define OUR_FLOOR_COIN_1_Y   500 
#define OUR_FLOOR_COIN_2_X   850 
#define OUR_FLOOR_COIN_2_Y   1000 
#define OUR_FLOOR_COIN_3_X   450
#define OUR_FLOOR_COIN_3_Y   1700  

#define BOTTLES_Y            2000
#define OUR_BOTTLE_1_X       640  
#define OUR_BOTTLE_2_X       1883  
#define OPP_BOTTLE_1_X       2360   
#define OPP_BOTTLE_2_X       1117   

#define MIDDLE_COINS_GROUP_X  	1500
#define MIDDLE_COINS_GROUP_Y     1700

#define MIDDLE_FLOOR_GOLDBAR_X  1500
#define MIDDLE_FLOOR_GOLDBAR_Y  1353

#define OUR_FLOOR_GOLDBAR_X  		420
#define OUR_FLOOR_GOLDBAR_Y  		785

#define OPP_FLOOR_GOLDBAR_X     	2580
#define OPP_FLOOR_GOLDBAR_Y     	785

#define OUR_SHIP_DECK_1_X        200
#define OUR_SHIP_DECK_1_Y        (500+360)
#define OUR_SHIP_DECK_2_X        200
#define OUR_SHIP_DECK_2_Y        900

#define OPP_SHIP_DECK_1_X        (3000-200)
#define OPP_SHIP_DECK_1_Y        (500+360)
#define OPP_SHIP_DECK_2_X        (3000-200)
#define OPP_SHIP_DECK_2_Y        900

#define OUR_CAPTAINS_BEDROOM_X	250
#define OUR_CAPTAINS_BEDROOM_Y	250
#define OPP_CAPTAINS_BEDROOM_X	2750
#define OPP_CAPTAINS_BEDROOM_Y	250

#define SAVE_TREASURE_X				700
#define SAVE_TREASURE_Y				1400

#define OUR_MAP_X						1500
#define OUR_MAP_Y							0



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
#define TRAJ_FLAGS_SMALL_DIST 		(END_TRAJ|END_BLOCKING|END_INTR)

/* homologation compilation */
//#define HOMOLOGATION

/* default speeds */
#ifdef HOMOLOGATION
#define SPEED_DIST_FAST 		2000
#define SPEED_ANGLE_FAST 		2000
#else

#define SPEED_DIST_FAST 		4000
#define SPEED_ANGLE_FAST 		4000
#endif

//Do not change
#define SPEED_DIST_SLOW 		2000
#define SPEED_ANGLE_SLOW 		2000
#define SPEED_DIST_VERY_SLOW 	500
#define SPEED_ANGLE_VERY_SLOW 500

#define PLACE_D_SAFE	 140

/*for coins */
#define ONE       0
#define GROUP     1


/* zones */
#define ZONE_TOTEM_OUR_SIDE_1			   0
#define ZONE_TOTEM_OUR_SIDE_2       	1
#define ZONE_TOTEM_OPP_SIDE_1			   2
#define ZONE_TOTEM_OPP_SIDE_2			   3
#define ZONE_OUR_FLOOR_COIN_1			   4
#define ZONE_OUR_FLOOR_COIN_2  	      5
#define ZONE_OUR_FLOOR_COIN_3 	      6
#define ZONE_OUR_FLOOR_GOLDBAR   	   7
#define ZONE_OPP_FLOOR_COIN_1	         8
#define ZONE_OPP_FLOOR_COIN_2	         9
#define ZONE_OPP_FLOOR_COIN_3	         10
#define ZONE_OPP_FLOOR_GOLDBAR			11
#define ZONE_OUR_BOTTLE_1			      12
#define ZONE_OUR_BOTTLE_2   			   13
#define ZONE_OPP_BOTTLE_1			      14
#define ZONE_OPP_BOTTLE_2			      15
#define ZONE_OUR_MAP          	      16
#define ZONE_OPP_MAP          	      17
#define ZONE_MIDDLE_COINS_GROUP        18
#define ZONE_MIDDLE_FLOOR_GOLDBAR      19

/* place zones */
#define ZONE_SHIP_OUR_CAPTAINS_BEDRROM	20
#define ZONE_SHIP_OUR_HOLD	            21
#define ZONE_SHIP_OUR_DECK_2           22
#define ZONE_SHIP_OUR_DECK_1        	23

#define ZONE_SHIP_OPP_CAPTAINS_BEDRROM	24
#define ZONE_SHIP_OPP_HOLD             25
#define ZONE_SHIP_OPP_DECK_1          	26
#define ZONE_SHIP_OPP_DECK_2        	27

#define ZONE_SAVE_TREASURE		         28
#define ZONES_MAX								29

/* Store goldbars and coins or not and where */
#define STORE_MOUTH    1
#define STORE_BOOT     2
#define DONT_STORE     0

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
   #define ENABLE_R2ND_POS				1 /* TODO: set by command */
	#define ENABLE_DOWN_SIDE_ZONES	2
};



typedef struct {
	/* type */
	uint16_t type;
	#define ZONE_TYPE_TOTEM					0
	#define ZONE_TYPE_GOLDBAR				1
	#define ZONE_TYPE_MAP					2
	#define ZONE_TYPE_BOTTLE				3
	#define ZONE_TYPE_COIN					4
	#define ZONE_TYPE_COINS_GROUP			5
	#define ZONE_TYPE_HOLD					6
	#define ZONE_TYPE_DECK					7
	#define ZONE_TYPE_CAPTAINS_BEDROOM	8
	#define ZONE_TYPE_SAVE					9
	#define ZONE_TYPE_MAX					10

	/* target point */
	int16_t x;
	int16_t y;	

	/* boundinbox */
	int16_t x_down;
	int16_t y_up;
	int16_t x_up;
	int16_t y_down;


	//int16_t x_up;
	//int16_t y_up;
	//int16_t x_down;
	//int16_t y_down;

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

	/* step of each empty totem */
	uint8_t step_our_totem_1;
	uint8_t step_our_totem_2;
	uint8_t step_opp_totem_1;
	uint8_t step_opp_totem_2;

	/* opponent zone position */
	uint8_t opp_current_zone;
	uint8_t opp2_current_zone;

	/* opponent stadistics */
	uint32_t opp_time_zone_ms;

	uint8_t treasure_in_mouth;
	uint8_t treasure_in_boot;
};

extern struct strat_infos strat_infos;

extern char numzone2name[ZONES_MAX + 1][30];

#ifndef HOST_VERSION

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



/********************************************
 * in strat_treasure.c 
 *******************************************/
uint8_t strat_empty_totem_side(int16_t x, int16_t y, uint8_t store_goldbar, uint8_t step);
uint8_t strat_pickup_coins_floor(int16_t x, int16_t y, uint8_t group);
uint8_t strat_pickup_goldbar_floor(int16_t x, int16_t y, uint8_t store);
uint8_t strat_send_message_bottle(int16_t x, int16_t y);
uint8_t strat_save_treasure_generic(int16_t x, int16_t y);
uint8_t strat_save_treasure_arms(int16_t x, int16_t y, uint8_t arm_side);
uint8_t strat_save_treasure_in_deck_back(int16_t x, int16_t y);
uint8_t strat_save_treasure_in_deck_back_blowing(int16_t x, int16_t y);
uint8_t strat_save_treasure_in_hold_back(int16_t x, int16_t y);
uint8_t strat_raise_window(uint8_t window);
uint8_t strat_steal_treasure_hold(void);
uint8_t strat_game(void);



/********************************************
 * in strat_begin.c 
 *******************************************/
uint8_t strat_begin(void);



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

/* smart play */
//#define DEBUG_STRAT_SMART
void strat_smart(void);

/* add here more strat functions in files */

#else /* HOST_VERSION */

#define AREA_BBOX_6X5	0
#define AREA_BBOX_4X4	1
void strat_set_bounding_box(uint8_t type);

#endif /* HOST_VERSION */

#endif
