/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2010)
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

#ifndef _STRAT_H_
#define _STRAT_H_

/* area */
#define AREA_X 3000
#define AREA_Y 2000

/* obstacle clerance */
#define OBS_CLERANCE   260.0

/* totems */
#define TOTEM_1_X    1100
#define TOTEM_2_X    1900
#define TOTEM_1_Y    AREA_Y/2
#define TOTEM_2_Y    TOTEM_1_Y

/* infos about strat */
#define NB_SLOT_X				8
#define NB_SLOT_Y				6
#define NB_SLOT_GREEN		5
#define NB_GRID_LINES_X 	9
#define NB_GRID_LINES_Y 	7


/* convert coords according to our color */
#define COLOR_Y(y)     (y)
#define COLOR_X(x)     ((mainboard.our_color==I2C_COLOR_BLUE)? (x) : (AREA_X-(x)))

#define COLOR_A_REL(a) ((mainboard.our_color==I2C_COLOR_BLUE)? (a) : (-a))
#define COLOR_A_ABS(a) ((mainboard.our_color==I2C_COLOR_BLUE)? (a) : (180-a))

#define COLOR_SIGN(x)  ((mainboard.our_color==I2C_COLOR_BLUE)? (x) : (-x))
#define COLOR_INVERT(x)((mainboard.our_color==I2C_COLOR_BLUE)? (x) : (!x))

#define COLOR_I(x)	  ((mainboard.our_color==I2C_COLOR_BLUE)? (x) :  ((NB_SLOT_X-1)-x))

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

#define TOKEN_DIAMETER	200

/* useful traj flags */
#define TRAJ_SUCCESS(f) 				(f & (END_TRAJ|END_NEAR))

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
#define SPEED_DIST_SLOW 		1000
#define SPEED_ANGLE_SLOW 		1000
#define SPEED_DIST_VERY_SLOW 	1000
#define SPEED_ANGLE_VERY_SLOW 1000

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
#define LINE1_TOKENS_ON_BONUS_OLD		1
#define LINE1_TOKENS_NEAR_WALL			2
#define LINE1_OPP_TOKEN_BEFORE_PLACE	4
#define LINE1_OPP_TOKEN_AFTER_PLACE		8
#define GREEN_OPP_ZONE_FIRST			  16
//#define STRAT_CONF_PLACE_ONLYEXT		  16


	/* thresholds */
	uint8_t th_place_prio;
	uint8_t th_token_score;

};

/* scores */
#define NOPLACE_SCORE	 0
#define PION_SCORE		10
#define FIGURE_SCORE		20
#define TOWER1H_SCORE	40
#define TOWER2H_SCORE	60
#define PLACE_ALL_SCORE	70


/* slot dimensions */
#define SLOT_SIZE 		350
#define SLOT_SIZE_HALF	175
#define SLOT_DIAGONAL	495
#define SLOT_GREEN_SIZE 280

/* slot of token */
struct slot_info {
	int16_t x;
	int16_t y;

	uint8_t color;
#define SLOT_BLUE			I2C_COLOR_BLUE
#define SLOT_RED			I2C_COLOR_RED
#define SLOT_GREEN_BLUE	I2C_COLOR_MAX
#define SLOT_GREEN_RED	(I2C_COLOR_MAX+1)

	
	uint8_t prio;
#define SLOT_PRIO_0		0
#define SLOT_PRIO_1		10
#define SLOT_PRIO_2		20
#define SLOT_PRIO_3		30
#define SLOT_PRIO_4		40
#define SLOT_PRIO_5		50
#define SLOT_PRIO_6		60
#define SLOT_PRIO_7		70
#define SLOT_PRIO_MAX	80
#define SLOT_PRIOR_INC	10

/* strat areas priorities */
#define SLOT_PRIO_GREEN			SLOT_PRIO_0	
#define SLOT_PRIO_CENTER		SLOT_PRIO_1
#define SLOT_PRIO_PATH			SLOT_PRIO_2
#define SLOT_PRIO_NEAR_GREEN	SLOT_PRIO_3
#define SLOT_PRIO_NEAR_SAFE	SLOT_PRIO_4
#define SLOT_PRIO_WALL			SLOT_PRIO_5
#define SLOT_PRIO_CORNER		(SLOT_PRIO_WALL + 5)
#define SLOT_PRIO_SAFE			SLOT_PRIO_6
#define SLOT_PRIO_BONUS_WALL	SLOT_PRIO_7
#define SLOT_PRIO_BONUS			SLOT_PRIO_MAX


	uint16_t flags;
//#define SLOT_BONUS			1
#define SLOT_SAFE				2
#define SLOT_AVOID			4
#define SLOT_CHECKED			8
#define SLOT_BUSY				16
#define SLOT_FIGURE			32
#define SLOT_OPPONENT		64
#define SLOT_ROBOT			128

	uint8_t flags_poly;
#define SLOT_POLY_SQUARE	1

};

typedef struct {
	int8_t i;
	int8_t j;
}slot_index_t;


#define NB_TOWER_MAX	6
typedef struct {
	int8_t i;
	int8_t j;
	int16_t x;
	int16_t y;
	int16_t w;
	int8_t c;		
} tower_t;

typedef struct {
//	/* work pt */
//	int16_t x;
//	int16_t y;

	/* boundinbox */
	int16_t x_up;
	int16_t y_up;
	int16_t x_down;
	int16_t y_down;

	/* stadistics */
	uint8_t num_visits;
	uint32_t total_time_ms;

#ifdef ZONES_HAS_FNCS
	/* taskt to do before and after work */
	uint8_t (*do_before)(void);
	uint8_t (*do_after)(void);
#endif
} zone_t;

struct strat_infos {
	uint8_t dump_enabled;
	struct conf conf;
	struct bbox area_bbox;

	/* playing areas */
	struct slot_info slot[NB_SLOT_X][NB_SLOT_Y];

	/* grid lines */
	uint16_t grid_line_x[NB_GRID_LINES_X];
	uint16_t grid_line_y[NB_GRID_LINES_Y];

	/* our slot position */
	slot_index_t slot_actual;
	slot_index_t slot_before;

	/* opponent slot position */
	slot_index_t opp_slot_actual;
	slot_index_t opp_slot_before;

	/* towers found */
	uint8_t num_towers;
	tower_t towers[NB_TOWER_MAX];

	/* "pickup and place" zones */
#define ZONE_OPP_NEAR_HOME	0
#define ZONE_OPP_NEAR_SAFE	1
#define ZONE_NEAR_HOME		2
#define ZONE_NEAR_SAFE		3
#define ZONE_WALL_BONUS		4
#define NB_ZONES_MAX			5

	zone_t zones[NB_ZONES_MAX];

	/* opponent stadistics */
	uint8_t opp_actual_zone;
	uint8_t opp_before_zone;
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


/*********************************************
 * in strat_tokens.c 
 ********************************************/

/* update number of token inside */
uint8_t strat_get_num_tokens(void);

/* pick up a token */
/* use it in short distance ranges */
uint8_t strat_pickup_token(int16_t x, int16_t y, uint8_t side);

/* pick up token chossing side automaticaly */
/* we suppose that at least one side is empty */
uint8_t strat_pickup_token_auto(int16_t x, int16_t y, uint8_t *side);

/* place a token */	
/* use it in near range distance */
uint8_t strat_place_token(int16_t x, int16_t y, uint8_t side, uint8_t go);

/* place token automaticaly */
/* we suppose that there is at least one token catched */
uint8_t strat_place_token_auto(int16_t x, int16_t y, uint8_t *side, uint8_t go);

/* push a token in order to get out an opponent token from slot */
uint8_t strat_push_slot_token(int8_t i, int8_t j);

/* pickup near slots on an area 3x3 with center the robot */
#define MODE_PICKUP	1
#define MODE_PUSH		2
#define MODE_ALL		3
uint8_t strat_pickup_or_push_near_slots(uint8_t mode);

/* place tokens on near 3x3 area slots, return 0 if there aren't more slots */
uint8_t strat_place_near_slots(uint8_t only_one, uint8_t token_score);

/* pickup and place near tokens in 3x3 area where is the robot */
//uint8_t strat_pickup_and_place_near_slots(void);

/* enable/disable look for towers */
void strat_look_for_towers_enable(void);
void strat_look_for_towers_disable(void);

/* look for tower of 2 or 3 levels, if find any tower, it's added to infos */
void strat_look_for_towers(void);

/* return 1 if a new tower is added succesfully */ 
uint8_t strat_info_add_tower(int16_t x, int16_t y, int16_t w);

/* return 1 if an exist tower is deleted succesfully */
uint8_t strat_info_del_tower(int8_t i, int8_t j);

/* enable/disable look for figures */
void strat_look_for_figures_enable(void);
void strat_look_for_figures_disable(void);

/* try to find figures from line 1 */
void strat_look_for_figures(void);

/* get a valid ij tower coordenade */
uint8_t strat_get_best_tower_ij(int8_t *i, int8_t *j);

/* get slot to pickup a token at ij position */
void strat_get_slot_to_place(int8_t i, int8_t j, int8_t *i_place, int8_t *j_place);


/**************************************************
 * in strat_static.c 
 *************************************************/
uint8_t strat_harvest_line1(void);
uint8_t strat_harvest_line2(void);
uint8_t strat_harvest_green_area(void);

#define TYPE_PION		0
#define TYPE_FIGURE	1
uint8_t strat_pickup_green_token(uint8_t type, uint8_t color);

/* harvest green area ending with two figures inside */
uint8_t strat_harvest_green_area_smart(uint8_t color);

/**************************************************
 * in strat_navigation.c 
 *************************************************/

/* strat of Spanish Cup */
uint8_t strat_bonus_point(void);

/* update robot or opponent slot position */
#define TYPE_ROBOT		0
#define TYPE_OPPONENT	1
#define GRID_MARGIN 		10

void strat_update_slot_position(uint8_t type, int16_t margin, 
									     int8_t x_line_init, int8_t x_line_end,
									     int8_t y_line_init, int8_t y_line_end);

/* update opponent zone infos */
void strat_update_zones(void);

uint8_t strat_play_with_opp(void);
uint8_t strat_big_final(void);

///* work on a zone */
//uint8_t strat_work_on_zone(zone_t * z);
//
///* little tasks or not ;) */
//uint8_t strat_place_figure_near_opp_home(void);
//uint8_t strat_place_on_near_opp_safe_slot(void);
//uint8_t strat_place_on_opp_safe_slot(void);
//uint8_t strat_pickup_bonus_near_wall(void);


/* add here more strat functions in files */

#else

#define AREA_BBOX_6X5	0
#define AREA_BBOX_4X4	1
void strat_set_bounding_box(uint8_t type);

#endif /* HOST_VERSION */

#endif
