%% Process to get a cuve that fit with data calibration

%% Scale and interpolation of data
mu = mean(x);
sigma = std(x);
x_scale = (x-mu)./sigma;
x_scale_inter = linspace(x_scale(1), x_scale(2), 100);
x_inter = linspace(x(1),x(end),100);

%% Fit a polynomial
p = polyfit(x_scale,y,6);
y_fit = polyval(p,x_scale);

%% Results

% compare curves
close all
figure
plot(x,y,'ro')
hold on
plot(x,y_fit)
grid on
legend('Calibration samples', 'Polynomial fit');

% error
error = y-y_fit;
figure
plot(x, error);
legend('error (cm)')

%% Table
% Note: array index = (array_size-1) - (medida-x_eval_min)/x_eval_delta
%x_eval = 6560:-5:2415;
%x_eval = 10500:-8:3852; % index = 831- (x_medida-3852)/8)

% evaluate a x range and get y
x_eval = 4600:-8:1600;
y_eval = polyval(p,(x_eval-mu)./sigma);

% round to integer values
y_int = round(y_eval);

% y increment per x value
y_inc = y_int(2:end)-y_int(1:end-1);

% info table
table = [x_eval; y_int; [0 y_inc]];

% plot y increment per x
figure, stem(table(2,:), table(3,:))

% plot y = f(x)
figure
plot(x,y,'ro')
hold on
plot(x_eval,y_eval)
grid on


%% Inverse fit
p_inv = polyfit((y-mean(y))./std(y), x, 6);
y_eval = [30:5:350];
x_eval = polyval(p_inv, (y_eval-mean(y_eval))./std(y_eval));

figure
plot(x_eval,y_eval)
legend('Pulse(us) = f(dist(cm))')


