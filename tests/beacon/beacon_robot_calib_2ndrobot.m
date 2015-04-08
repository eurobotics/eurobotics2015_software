%% Load beacon data calibration "robot_calib_4" (see beacon.xls)

clear all
load beacon_robot_calib_2ndrobot_3m;
x = data(:,2);   % (size/period) x 100000 (counts/counts)
y = data(:,1);   % distance(cm)
x_eval_ini = 5443;   % evaluated distance min (in counts)
x_eval_end = 952; % evaluated distance max (in counts)
x_eval_delta = 8;    % x delta increment evaluation

