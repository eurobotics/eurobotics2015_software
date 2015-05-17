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
 *  Revision : $Id: obstacle_avoidance_config.h,v 1.3 2009/05/27 20:04:07 zer0 Exp $
 *
 */

#define MAX_POLY 			8					// 2 opp + 2nd robot + boundingbox + 2 home + stairs +platform
#define MAX_PTS 			MAX_POLY*4 + 8	// MAX_POLY * 4 (all polys are squares) + 4 points more of totems poly
#define MAX_RAYS 			500
#define MAX_CHKPOINTS 	20 
