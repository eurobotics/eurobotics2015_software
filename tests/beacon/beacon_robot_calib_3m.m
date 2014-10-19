%% Load beacon data calibration "robot_calib_3m" (see beacon.xls)

clear all
load beacon_robot_calib_3m;
x = data(:,2);   % (size/period) x 100000 (counts/counts)
y = data(:,1);   % distance(cm)
x_eval_ini = 5400; % evaluated distance min (in counts)
x_eval_end = 920; % evaluated distance max (in counts)
x_eval_delta = 8;  % x delta increment evaluation
