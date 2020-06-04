clear;
close all;

%% PHYSICAL PARAMETERS AND CONSTANTS

Kv = 3040; % Motor Constant (RPM / Volt)

Kt = inv(Kv / 60 * 2 * pi); % Kt = 1/Kv if Kv is in (rad/s)/Volt

% Kt = 318; % Motor Constant (N * m / A)
D = 1e-20; % Frictional Loss Constant (N * m * s / rad)
R = 5e-3; % Motor Resistance (Ohms)

r_robot_im = 5; % robot radius (in)
w_robot_im = 11.8; % robot weight (lbs)
r_wheel_im = 2.5; % wheel radius (in)
w_wheel_im = 0.2; % wheel weight (lbs)
% the im suffix stands for imperial 

slew_rate = 1e40; % V / s

sys = create1DOFSVM(Kt, D, R, ...
    r_robot_im, w_robot_im, r_wheel_im, w_wheel_im);

%% SIMULATION PARAMETERS

dt = 1e-6; % modeling physics at this rate (seconds)
Ts = 1e-3; % algorithms updating at this rate (seconds)
duration = 8; % simulation duration (seconds)

initial_position = 0; % inital heading (radians)
initial_velocity = 0; % initial angular velocity (radians / second)
initial_voltage = 8; % initial motor voltage (volts fuck u think)
initial_extrestorque = 0; % initial external resistive torque (volts)

%% ALGORITHM PARAMETERS

% shit goes here think of something smart varun i can't do all the heavy
%  lifting jeez

%% SIMULATION RUNNING (THIS IS WHERE THE FUN BEGINS)

sysd = c2d(sys, dt); % ??? discretize ??? maybe ??? faster ???


u0 = [initial_voltage initial_extrestorque]; % initial input state
x0 = [initial_position initial_velocity]; % initial state space state
TT = 0:Ts:duration; % algorithm evaluation times (seconds)
tt = 0:dt:Ts;
steps = numel(TT)+1; % number of times algorithm is run (#)
uu = zeros(steps, 2); % all inputs (steps -> motor voltage (volts), 
                      %              external resistive torque (volts))
yy = zeros(steps, 2); % all states (steps -> angular position in rad, 
                      %                         angular velocity in rad/s)
yy_all = zeros(numel(tt).*(steps-1), 2); % yy but with like wayyy more
uu(1, :) = u0;
yy(1, :) = x0;


for k=2:steps
    
    fprintf("%d/%d\n", k, steps);
    
    % Update the physics 
    if k==2
        [yy(k, :), yy_all_temp] = step1DOFSVM(sys, ...
            yy(k-1, :), uu(k-1, :), ...
            dt, Ts, slew_rate);
    else
        [yy(k, :), yy_all_temp] = step1DOFSVM(sys, ...
            yy(k-1, :), uu(k-1, :), ...
            dt, Ts, slew_rate, uu(k-2, :));    
    end
    yy_all( ((k-2)*numel(tt)+1) : ((k-1)*numel(tt)), : ) = yy_all_temp;
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ALGO GOES HERE BUT HERE'S THE LOGIC FOR NOW %
    uu(k, :) = uu(k-1, :); % CLOSED-EYE KALMAN 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%% PLOTS AND VISUALIZATIONS

figure('units','normalized','outerposition',[0 0 1 1])
Nplot = 3;

TTplot = [TT duration + Ts];
TTplot_all = linspace(0, TTplot(end), size(yy_all, 1));

subplot(Nplot, 1, 1);
plot(TTplot, mod(yy(:, 1).*180./pi, 360), ...
    TTplot_all, mod(yy_all(:, 1).*180./pi, 360), ...
    'LineWidth', 2); 
ylabel("Angular Position (deg%360)");
xlabel("Time (s)");
legend("Discrete @ Ts", "Continuous @ dt");

subplot(Nplot, 1, 2);
plot(TTplot, yy(:, 2).*180./pi, ...
    TTplot_all, yy_all(:, 2).*180/pi, ...
    'LineWidth', 2);
ylabel("Angular Velocity (deg/s)");
xlabel("Time (s)");
legend("Discrete @ Ts", "Continuous @ dt");

subplot(Nplot, 1, 3);
plot(TTplot, uu(:, 1), 'LineWidth', 2);
ylabel("Input Voltage (V)");
xlabel("Time (s)");

sgtitle(sprintf(['Robot Dynamic Modeling: Kt=%.3f, D=%.3f, R=%.3f, ', ...
    'BotR=%.1f, BotW=%.1f, WheelR=%.1f, WheelW=%.1f, SR=%.1g, ', ...
    'Ts=%.1g, dt=%.1g'], ...
    Kt, D, R, r_robot_im, w_robot_im, r_wheel_im, w_wheel_im, ...
    slew_rate, Ts, dt), ...
    "FontSize", 18);

