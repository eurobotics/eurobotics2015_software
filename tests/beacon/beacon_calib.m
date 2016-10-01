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


%% Process to get a cuve that fit with data calibration

%% Scale and interpolation of data
mu = mean(x);
sigma = std(x);
x_scale = (x-mu)./sigma;
x_scale_inter = linspace(x_scale(1), x_scale(2), 100);
x_inter = linspace(x(1),x(end),100);

%% Fit a polynomial
N = 6
p = polyfit(x_scale,y,N);
y_fit = polyval(p,x_scale);

%% Results

% compare curves
close all
figure
subplot(2,1,1), plot(x,y,'ro')
hold on
plot(x,y_fit)
grid on
legend('Calibration samples', ['Polynomial fit N=', num2str(N)]);
ylabel('distance (cm)')
xlabel('pulse measure (counts*10^5/counts)')

% error
error = y-y_fit;
%figure
subplot(2,1,2), plot(x, error)
grid on
legend('fit error (cm)')
ylabel('distance (cm)')
xlabel('pulse measure (counts*10^5/counts)')


%% Table
% Note: array index = (array_size-1) - (medida-x_eval_min)/x_eval_delta
%x_eval = 6560:-5:2415;
%x_eval = 10500:-8:3852; % index = 831- (x_medida-3852)/8)

% evaluate a x range and get y
%x_eval = 4600:-8:1600;
x_eval = x_eval_ini:-x_eval_delta:x_eval_end;
y_eval = polyval(p,(x_eval-mu)./sigma);

% round to integer values
y_int = round(y_eval);

% y increment per x value
y_inc = y_int(2:end)-y_int(1:end-1);

% info table
table = [x_eval; y_int; [0 y_inc]];

% plot y = f(x)
figure
subplot(2,1,1)
plot(x,y,'ro')
hold on
plot(x_eval,y_eval)
grid on
ylabel('distance (cm)')
xlabel('pulse measure (counts*10^5/counts)')
legend('Calibration samples','Lookup table samples')

title([num2str(length(x_eval)), ' samples, pulse measures range = [', ...
         num2str(x_eval_ini),':', ...
         num2str(x_eval_delta),':', ...
         num2str(x_eval_end),']',...
         ', distance range = [',num2str(y_int(1)),',',num2str(y_int(end)),']'])

% plot y increment per x
%figure
subplot(2,1,2)
stem(table(2,:), table(3,:))
grid on
ylabel('distance resolution d[i]-d[i-1] (cm)')
xlabel('distance samples d[i] (cm)')
legend('Lookup table distance resolution')

title([num2str(length(x_eval)), ' samples, pulse measures range = [', ...
         num2str(x_eval_ini),':', ...
         num2str(x_eval_delta),':', ...
         num2str(x_eval_end),']',...
         ', distance range = [',num2str(y_int(1)),',',num2str(y_int(end)),']'])

%% Inverse fit
p_inv = polyfit((y-mean(y))./std(y), x, 6);
y_eval = [30:5:350];
x_eval = polyval(p_inv, (y_eval-mean(y_eval))./std(y_eval));

% figure
% plot(x_eval,y_eval)
% legend('Pulse(us) = f(dist(cm))')


