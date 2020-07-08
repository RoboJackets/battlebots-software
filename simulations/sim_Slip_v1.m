clear;
close all;
graphics = 1;
%% PHYSICAL PARAMETERS AND CONSTANTS

% using: Scorpion HKIV-4020-860KV
Kv = 860; % Motor Constant (RPM / Volt)
gear_rat = 2; % gear ratio (RPM / RPM)
Kt = inv(2*pi*Kv*gear_rat/60); % Kt = 1/Kv if Kv is in (rad/s)/Volt
% Kt = 318; % Motor Constant (N * m / A)
D = 1e-3; % Frictional Loss Constant (N * m * s / rad)
R = 12e-3; % Motor Resistance (Ohms)

r_robot_im = 5; % robot radius (in)
w_robot_im = 11.8; % robot weight (lbs)
r_wheel_im = 2.5; % wheel radius (in)
w_wheel_im = 0.2; % wheel weight (lbs)
% the im suffix stands for imperial 

slew_rate = 1e40; % V / s

[sys, A, B, ~, ~] = createTorqueSVM(Kt, D, R, ...
    r_robot_im, w_robot_im, r_wheel_im, w_wheel_im);


    
%% SIMULATION PARAMETERS

dt = 1e-6; % modeling physics at this rate (seconds)
Ts = 1/(3.2e3); % algorithms updating at this rate (seconds)
duration = 10; % simulation duration (seconds)

initial_position = 0; % inital heading (radians)
initial_velocity = 0; % initial angular velocity (radians / second)
initial_voltage_left = 22; % initial motor voltage (volts fuck u think)
initial_voltage_right = 22;
initial_extrestorque = 0; % initial external resistive torque (volts)
initial_wheel_velocity = 0;
initial_wheel_force = 0;




%% SIMULATION SETUP

% sysd = c2d(sys, dt); % ??? discretize ??? maybe ??? faster ???
sysd = sys;


u0 = [initial_voltage_left initial_voltage_right]; % initial input state
x0 = [initial_position initial_velocity initial_wheel_velocity initial_wheel_velocity initial_wheel_force initial_wheel_force];% initial state space state

TT = 0:Ts:duration; % algorithm evaluation times (seconds)
tt = 0:dt:Ts;
steps = numel(TT)+1; % number of times algorithm is run (#)
uu = zeros(steps, 2); % all inputs (steps -> motor voltage (volts))
yy = zeros(steps, 6); % all states (steps -> angular position in rad, 
                   %                         angular velocity in rad/s)

yy_all = zeros(numel(tt).*(steps-1), 6); % yy but with like wayyy more                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  l = zeros(numel(tt).*(steps-1), 2); % yy but with like wayyy more
uu(1, :) = u0;
yy(1, :) = x0;


%% SIMULATION EXECUTION 
%% fprintf("%d\n", A);
for k=2:steps
    
    fprintf("%d/%d\n", k, steps);
    % fprintf("%d\n", yy(k-1, 5));
    
    % Update the physics 
    if k==2
        [yy(k, :), yy_all_temp] = step1TorqueSVM(sysd, A, B, ...
            yy(k-1, :), uu(k-1, :), ...
            dt, Ts, slew_rate);
    else
        [yy(k, :), yy_all_temp] = step1TorqueSVM(sysd, A, B, ...
            yy(k-1, :), uu(k-1, :), ...
            dt, Ts, slew_rate, uu(k-2, :));    
    end
    yy_all( ((k-2)*numel(tt)+1) : ((k-1)*numel(tt)), : ) = yy_all_temp;
    
   
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ALGO GOES HERE BUT HERE'S THE LOGIC FOR NOW %
    uu(k, :) = uu(k-1, :); % CLOSED-EYE KALMAN 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%% PHYSICAL QUANTITY PLOTS 
if graphics == 1
    figure('units','normalized','outerposition',[0 0 1 1])
    Nplot = 3;

    TTplot = [TT duration + Ts];
    TTplot_all = linspace(0, TTplot(end), size(yy_all, 1));

    subplot(Nplot, 1, 1);
    axis on; grid on;
    plot(TTplot, mod(yy(:, 1).*180./pi, 360), ...
        TTplot_all, mod(yy_all(:, 1).*180./pi, 360), ...
        'LineWidth', 2); 
    ylabel("Angular Position (deg%360)");
    xlabel("Time (s)");
    legend("Discrete @ Ts", "Continuous @ dt");

    subplot(Nplot, 1, 2);
    axis on; grid on;
    plot(TTplot, yy(:, 2).*180./pi, ...
        TTplot_all, yy_all(:, 2).*180/pi, ...
        'LineWidth', 2);
    hold on;
    yline(18000, "LineWidth", 2);
    hold off;
    ylabel("Angular Velocity (deg/s)");
    xlabel("Time (s)");
    legend("Discrete @ Ts", "Continuous @ dt", "Target Steady-State");

    subplot(Nplot, 1, 3);
    axis on; grid on;
    plot(TTplot, uu(:, 1), 'LineWidth', 2);
    ylabel("Input Voltage (V)");
    xlabel("Time (s)");

    sgtitle(sprintf(['Robot Dynamic Modeling: Kv=%.2e, D=%.3f, R=%.3f, ', ...
        'BotR=%.1f, BotW=%.1f, WheelR=%.1f, WheelW=%.1f, SR=%.1e, ', ...
        'Ts=%.1e, dt=%.1e'], ...
        Kv, D, R, r_robot_im, w_robot_im, r_wheel_im, w_wheel_im, ...
        slew_rate, Ts, dt), ...
        "FontSize", 18);


    %% Wheel position plots %%
    figure('units','normalized','outerposition',[0 0 1 1])
    Nplot = 4;

    TTplot = [TT duration + Ts];
    TTplot_all = linspace(0, TTplot(end), size(yy_all, 1));

    subplot(Nplot, 1, 1);
    axis on; grid on;
    plot(TTplot, yy(:, 3).*180./pi, 'LineWidth', 2); 
    ylabel("Angular Velocity Omega_1 (deg/s)");
    xlabel("Time (s)");

    subplot(Nplot, 1, 2);
    plot(TTplot, yy(:, 4).*180./pi, 'LineWidth', 2); 
    ylabel("Angular Velocity Omega_2 (deg/s)");
    xlabel("Time (s)");

    subplot(Nplot, 1, 3);
    plot(TTplot, yy(:, 5), 'LineWidth', 2); 
    ylabel("Force Tan at Wheel_1 (N)");
    xlabel("Time (s)");

    subplot(Nplot, 1, 4);
    plot(TTplot, yy(:, 6), 'LineWidth', 2); 
    ylabel("Force Tan at Wheel_2 (N)");
    xlabel("Time (s)");


    sgtitle(sprintf(['Robot Dynamic Modeling: Kv=%.2e, D=%.3f, R=%.3f, ', ...
        'BotR=%.1f, BotW=%.1f, WheelR=%.1f, WheelW=%.1f, SR=%.1e, ', ...
        'Ts=%.1e, dt=%.1e'], ...
        Kv, D, R, r_robot_im, w_robot_im, r_wheel_im, w_wheel_im, ...
        slew_rate, Ts, dt), ...
        "FontSize", 18);
end
