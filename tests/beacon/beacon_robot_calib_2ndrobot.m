%%
%%   Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
%%
%%  This program is free software; you can redistribute it and/or modify
%%  it under the terms of the GNU General Public License as published by
%%  the Free Software Foundation; either version 2 of the License, or
%%  (at your option) any later version.
%%
%%  This program is distributed in the hope that it will be useful,
%%  but WITHOUT ANY WARRANTY; without even the implied warranty of
%%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%%  GNU General Public License for more details.
%%
%%  You should have received a copy of the GNU General Public License
%%  along with this program; if not, write to the Free Software
%%  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
%%
%%  Revision : $Id$
%%
%%  Javier Baliñas Santos <javier@arc-robots.org> 
%%


%% Load beacon data calibration "robot_calib_4" (see beacon.xls)

clear all
load beacon_robot_calib_2ndrobot_3m;
x = data(:,2);   % (size/period) x 100000 (counts/counts)
y = data(:,1);   % distance(cm)
x_eval_ini = 5443;   % evaluated distance min (in counts)
x_eval_end = 952; % evaluated distance max (in counts)
x_eval_delta = 8;    % x delta increment evaluation

