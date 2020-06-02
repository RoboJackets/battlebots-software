clear;
close all;

%% PHYSICAL PARAMETERS AND CONSTANTS

Kt = 1; % Motor Constant (N * m / A)
D = 0.01; % Frictional Loss Constant (N * m * s / rad)
R = 1; % Motor Resistance (Ohms)

r_robot_im = 6; % robot radius (in)
w_robot_im = 12; % robot weight (lbs)
r_wheel_im = 1.5; % wheel radius (in)
w_wheel_im = 0.2; % wheel weight (lbs)
% the im suffix stands for imperial 

sys = create1DOFSVM(Kt, D, R, ...
    r_robot_im, w_robot_im, r_wheel_im, w_wheel_im);

%% SIMULATION PARAMETERS

dt = 1e-6; % modeling physics at this rate (seconds)
Ts = 1e-3; % algorithms updating at this rate (seconds)
duration = 8; % simulation duration (seconds)

initial_position = 0; % inital heading (radians)
initial_velocity = 0; % initial angular velocity (radians / second)
initial_voltage = 2; % initial motor voltage (volts fuck u think)
initial_extrestorque = 0; % initial external resistive torque (volts)

%% ALGORITHM PARAMETERS

% shit goes here think of something smart varun i can't do all the heavy
%  lifting jeez

%% SIMULATION RUNNING (THIS IS WHERE THE FUN BEGINS)

u0 = [initial_voltage initial_extrestorque]; % initial input state
x0 = [initial_position initial_velocity]; % initial state space state
TT = 0:Ts:duration; % algorithm evaluation times (seconds)
steps = numel(TT)+1; % number of times algorithm is run (#)
uu = zeros(steps, 2); % all inputs (steps -> motor voltage (volts), 
                      %              external resistive torque (volts))
yy = zeros(steps, 2); % all states (steps -> angular position in rad, 
                      %                         angular velocity in rad/s)
uu(1, :) = u0;
yy(1, :) = x0;

for k=2:steps
    
    fprintf("%d/%d\n", k, steps);
    
    % Update the physics 
    yy(k, :) = step1DOFSVM(sys, yy(k-1, :), uu(k-1, :), dt, Ts);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ALGO GOES HERE BUT HERE'S THE LOGIC FOR NOW %
    uu(k, :) = uu(k-1, :); % CLOSED-EYE KALMAN 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%% PLOTS AND VISUALIZATIONS

figure('units','normalized','outerposition',[0 0 1 1])
Nplot = 3;

TTplot = [TT TT(end) + Ts];

subplot(Nplot, 1, 1);
plot(TTplot, mod(yy(:, 1).*180./pi, 360), 'LineWidth', 2); 
ylabel("Angular Position (deg%360)");
xlabel("Time (s)");

subplot(Nplot, 1, 2);
plot(TTplot, yy(:, 2).*180./pi, 'LineWidth', 2);
ylabel("Angular Velocity (deg/s)");
xlabel("Time (s)");

subplot(Nplot, 1, 3);
plot(TTplot, uu(:, 1));
ylabel("Input Voltage (V)");
xlabel("Time (s)");

sgtitle(sprintf("Robot Dynamic Modeling: Kt=%.3f, D=%.3f, R=%.3f, BotR=%.1f, BotW=%.1f, WheelR=%.1f, WheelW=%.1f", ...
    Kt, D, R, r_robot_im, w_robot_im, r_wheel_im, w_wheel_im), ...
    "FontSize", 18);

