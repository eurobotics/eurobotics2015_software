%% Load beacon data calibration "robot_calib_4" (see beacon.xls)

clear all
load beacon_robot_calib_4;
x = data(:,2);   % (size/period) x 100000 (counts/counts)
y = data(:,1);   % distance(cm)

