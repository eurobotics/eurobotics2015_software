/*
 *  Copyright Droids Corporation, Microb Technology (2009)
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
 *  Revision : $Id: obstacle_avoidance_config.h,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  obstacle_avoidance_config.h,v 1.4 2009/05/27 20:04:07 zer0 Exp.
 */

#define MAX_POLY 			8					// 2 opp + 2nd robot + totem area + boundingbox
#define MAX_PTS 			MAX_POLY*4 + 8	// MAX_POLY * 4 (all polys are squares) + 4 points more of totems poly
#define MAX_RAYS 			400
#define MAX_CHKPOINTS 	20
