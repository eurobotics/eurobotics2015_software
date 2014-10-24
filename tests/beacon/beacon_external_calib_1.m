%% Load beacon data calibration "external_calib_1" (see beacon.xls)

clear all
load beacon_external_calib_1;
offset_cm = 0;
first_sample = 2;   % aprox. 30 cm
k = 1.6;
x = data(first_sample:end,2).*k;            % size (us)
y = data(first_sample:end,1) + offset_cm;   % distance(cm)

