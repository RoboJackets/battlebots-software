clear;
close all;

%% PHYSICAL PARAMETERS AND CONSTANTS

% using: Scorpion HKIV-4020-860KV
Kv = 860; % Motor Constant (RPM / Volt)
gear_rat = 8; % gear ratio (RPM / RPM)
Kt = inv(2*pi*Kv*gear_rat/60); % Kt = 1/Kv if Kv is in (rad/s)/Volt
% Kt = 318; % Motor Constant (N * m / A)
D = 1e-3; % Frictional Loss Constant (N * m * s / rad)
R = 12e-3; % Motor Resistance (Ohms)

r_robot_im = 5; % robot radius (in)
w_robot_im = 11.8; % robot weight (lbs)
r_wheel_im = 2.5; % wheel radius (in)
w_wheel_im = 0.2; % wheel weight (lbs)
% the im suffix stands for imperial 

mu_static = 0.8; %Coefficient of static friction between wheel and ground

slew_rate = 1e40; % V / s

[sys, A, B, ~, ~, alpha, beta] = create1DOFSVM(Kt, D, R, ...
    r_robot_im, w_robot_im, r_wheel_im, w_wheel_im);


    
%% SIMULATION PARAMETERS

dt = 1e-5; % modeling physics at this rate (seconds)
Ts = 1/(3.2e3); % algorithms updating at this rate (seconds)
duration = 2; % simulation duration (seconds)

initial_position = 0; % inital heading (radians)
initial_velocity = 0; % initial angular velocity (radians / second)
initial_voltage = 0; % initial motor voltage (volts fuck u think)
initial_extrestorque = 0; % initial external resistive torque (volts)

use_perfect_accs = false;
use_ir_beacons = false;


%% SENSOR PARAMETERS

ir_eps = 1e-1; % epsilon for detecting ir beacon


% all angles are in degrees for defining parameters ok just deal
g = 9.81; % m / s^2
sys_temp = 25; % temperature in celsius
% modeling the ADXL375 accelerometer 
% www.analog.com/media/en/technical-documentation/data-sheets/ADXL375.pdf 
left_acc_pos_im = [-0.75; -0.60];
left_acc_pos_im = sort(left_acc_pos_im, 'ascend');
right_acc_pos_im = [0.60; 0.75];
right_acc_pos_im = sort(right_acc_pos_im, 'ascend');


left_acc_pos = left_acc_pos_im .* 0.0254;
right_acc_pos = right_acc_pos_im .* 0.0254;
all_acc_pos = [left_acc_pos; right_acc_pos];

left_acc_dir = [45; 45];
right_acc_dir = [45; 45];

[acc_pairs1, acc_pairs2] = meshgrid(1:numel(left_acc_pos), 1:numel(right_acc_pos));
acc_pairs = [acc_pairs1(:) acc_pairs2(:)];
acc_dists = right_acc_pos(acc_pairs(:, 2)) - left_acc_pos(acc_pairs(:, 1));

[all_acc_pairs1, all_acc_pairs2] = meshgrid(...
    1:numel(all_acc_pos), 1:numel(all_acc_pos));
all_acc_pairs = [all_acc_pairs1(:) all_acc_pairs2(:)];
all_acc_pairs = all_acc_pairs(all_acc_pairs(:, 1) < all_acc_pairs(:, 2), :);
all_acc_dists = all_acc_pos(all_acc_pairs(:, 2)) ...
    - all_acc_pos(all_acc_pairs(:, 1));


% angle deviation CW of +y on acc from direction of 
% tangential acceleration if robot rotating CCW
left_acc_params = [];
right_acc_params = [];
left_accs = cell(numel(left_acc_pos), 1);
right_accs = cell(numel(right_acc_pos), 1);
for k=1:size(left_acc_pos, 1)
    if use_perfect_accs
        left_acc_params = [left_acc_params ; getAccParams("perfect", 0)];        
    else
        left_acc_params = [left_acc_params ; getAccParams("ADXL375", 0)];
    end

    left_accs{k, 1} = imuSensor("accel-mag", "SampleRate", 1/Ts, ...
        "Temperature", sys_temp, ...
        "Accelerometer", left_acc_params(k));
end
for k=1:size(right_acc_pos, 1)
    if use_perfect_accs
        right_acc_params = [right_acc_params ; getAccParams("perfect", 1)];        
    else
        right_acc_params = [right_acc_params ; getAccParams("ADXL375", 1)];
    end

    right_accs{k, 1} = imuSensor("accel-mag", "SampleRate", 1/Ts, ...
        "Temperature", sys_temp, ...
        "Accelerometer", right_acc_params(k));
end
all_accs = [left_accs; right_accs];

%% CALCULATE MAX ANGULAR ACCELERATION
%The maximum force of static friction on each wheel
%Given by half the weight of the robot (converted to N) * static friction
%coefficient
w_robot = w_robot_im * 0.453592; %Convert to MKS units
r_robot = r_robot_im * 0.0254;
I_robot = 0.5 .* w_robot .* r_robot .^ 2; % robot body inertia

f_max = mu_static * (w_robot / 2) * g; 
torque_max = 2 * f_max * r_robot;
a_max = torque_max / I_robot; %Torque = I * a

%% SIMULATION SETUP

% sysd = c2d(sys, dt); % ??? discretize ??? maybe ??? faster ???
sysd = sys;


u0 = [initial_voltage initial_extrestorque]; % initial input state
x0 = [initial_position initial_velocity]; % initial state space state
TT = 0:Ts:duration; % algorithm evaluation times (seconds)
tt = 0:dt:Ts;
steps = numel(TT)+1; % number of times algorithm is run (#)
uu = zeros(steps, 2); % all inputs (steps -> motor voltage (volts), 
                      %              external resistive torque (volts))
yy = zeros(steps, 2); % all states (steps -> angular position in rad, 
                      %                         angular velocity in rad/s)

left_tang_accel_guess_hist = zeros(steps, numel(left_acc_pos));
right_tang_accel_guess_hist = zeros(steps, numel(right_acc_pos));
all_tang_accel_guess_hist = zeros(steps, numel(all_acc_pos));

left_cent_accel_guess_hist = zeros(steps, numel(left_acc_pos));
right_cent_accel_guess_hist = zeros(steps, numel(right_acc_pos));
all_cent_accel_guess_hist = zeros(steps, numel(all_acc_pos));

ang_accel_guess_hist = zeros(steps, size(acc_pairs, 1));
all_ang_accel_guess_hist = zeros(steps, size(all_acc_pairs, 1));

left_acc_true = zeros(size(left_acc_pos, 1), steps, 3);                      
right_acc_true = zeros(size(right_acc_pos, 1), steps, 3);    
all_acc_true = zeros(numel(all_acc_pos), steps, 3);

left_acc_data = zeros(size(left_acc_pos, 1), steps, 3);
right_acc_data = zeros(size(right_acc_pos, 1), steps, 3);
all_acc_data = zeros(numel(all_acc_pos), steps, 3);

pred_hist = zeros(steps, 2+2*size(acc_pairs, 1));
yy_all = zeros(numel(tt).*(steps-1), 2); % yy but with like wayyy more                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  l = zeros(numel(tt).*(steps-1), 2); % yy but with like wayyy more
uu(1, :) = u0;
yy(1, :) = x0;

for k=1:size(left_acc_pos, 1)
    left_acc_true(k, 1, :) = [0 0 0];
    left_acc_data(k, 1, :) = [0 0 0];
end
for k=1:size(right_acc_pos, 1)
    right_acc_true(k, 1, :) = [0 0 0];
    right_acc_data(k, 1, :) = [0 0 0];
end

%% EKF Setup
targ_angvel = 3000 / 60 * 2 * pi; % target angular velocity
deltaVolt = 1e-1; % max change in volts every Ts
maxVolt = 22; % max voltage we can input
minVolt = -22; % min voltage we can input
% EKF = MeltyBrain_EKF2(Ts, duration, alpha, beta, acc_dists, 0.0254 .* r_wheel_im, 0.0254 .* r_robot_im, ...
%                      size(acc_pairs, 1), 0, nan, nan, use_ir_beacons, ir_eps);
%Constructor arguments:
% MeltyBrain_EKF2(dt, Tsim, alpha, beta, dists, ...
%                 wheelRad, botRad, imus, mags, maxBField, ...
%                 fieldOffset, beacon, beaconRange)
HEKF = MeltyBrain_HEKF2(dt, alpha, beta, acc_dists, ...
    0.0254 .* r_wheel_im, 0.0254 .* r_robot_im, size(acc_pairs, 1));



%% SIMULATION EXECUTION 

for k=2:steps
    
    fprintf("%d/%d\n", k, steps);
    
    % DYNAMICS UPDATE STEP
    
    if k==2
        [yy(k, :), yy_all_temp] = step1DOFSVM(sysd, A, B, ...
            yy(k-1, :), uu(k-1, :), ...
            dt, Ts, slew_rate);
    else
        [yy(k, :), yy_all_temp] = step1DOFSVM(sysd, A, B, ...
            yy(k-1, :), uu(k-1, :), ...
            dt, Ts, slew_rate, uu(k-2, :));    
    end
    
    yy_all( ((k-2)*numel(tt)+1) : ((k-1)*numel(tt)), : ) = yy_all_temp;
    
    
    % GENERATE SENSOR DATA
    
    ang_vel = yy_all_temp(:, 2); % extract fine angular velocity
    ang_accel = diff(ang_vel) ./ dt; % numerical angular acceleration
    
    ang_vel_act = ang_vel(end);
    ang_accel_act = ang_accel(end);
    
    left_tang_accel = ang_accel_act .* left_acc_pos(:, 1);
    right_tang_accel = ang_accel_act .* right_acc_pos(:, 1); 
    all_tang_accel = ang_accel_act .* all_acc_pos(:, 1);
    
    left_cent_accel = (ang_vel_act.^2) * left_acc_pos(:, 1);
    right_cent_accel = (ang_vel_act.^2) * right_acc_pos(:, 1);
    all_cent_accel = (ang_vel_act.^2) * all_acc_pos(:, 1);
    
    kacc2 = 1;
    
    for kacc = 1:size(left_acc_pos, 1)
        ay = left_tang_accel(kacc) .* cosd(left_acc_dir(kacc)) ...
            + -left_cent_accel(kacc) .* sind(left_acc_dir(kacc));
        ax = -left_cent_accel(kacc) .* cosd(left_acc_dir(kacc)) ...
            + -left_tang_accel(kacc) .* sind(left_acc_dir(kacc));
        left_acc_true(kacc, k, :) = [ax ay 0];
        curr_acc = left_accs{kacc, 1};
        [acc_readings, ~] = curr_acc( ...
            reshape(left_acc_true(kacc, k, :), [1 3]), ...
            [0 0 0]);
        left_acc_data(kacc, k, :) = acc_readings;
        left_acc_true(kacc, k, 3) = -g;
        
        all_acc_data(kacc2, k, :) = left_acc_data(kacc, k, :);
        all_acc_true(kacc2, k, :) = left_acc_true(kacc, k, :);
        
        kacc2 = kacc2 + 1;
    end
    
    for kacc = 1:size(right_acc_pos, 1)
        ay = right_tang_accel(kacc) .* cosd(right_acc_dir(kacc)) ...
            + -right_cent_accel(kacc) .* sind(right_acc_dir(kacc));
        ax = -right_cent_accel(kacc) .* cosd(right_acc_dir(kacc)) ...
            + -right_tang_accel(kacc) .* sind(right_acc_dir(kacc));
        right_acc_true(kacc, k, :) = [ax ay 0];
        curr_acc = right_accs{kacc, 1};
        [acc_readings, ~] = curr_acc( ...
            reshape(right_acc_true(kacc, k, :), [1 3]), ...
            [0 0 0]);
        right_acc_data(kacc, k, :) = acc_readings;
        right_acc_true(kacc, k, 3) = -g;
        
        all_acc_data(kacc2, k, :) = right_acc_data(kacc, k, :);
        all_acc_true(kacc2, k, :) = right_acc_true(kacc, k, :);
        
        kacc2 = kacc2 + 1;
    end
    
    % ALGORITHM EVALUATION
    
    left_cent_accel_guess = ...
        -reshape(left_acc_data(:, k, 1), [numel(left_acc_dir) 1]) ...
            .* cosd(left_acc_dir) ...
        + -reshape(left_acc_data(:, k, 2), [numel(left_acc_dir) 1]) ...
            .* sind(left_acc_dir);
    left_cent_accel_guess_hist(k, :) = left_cent_accel_guess;
    
    right_cent_accel_guess = ...
        -reshape(right_acc_data(:, k, 1), [numel(right_acc_dir) 1]) ...
            .* cosd(right_acc_dir) ...
        + -reshape(right_acc_data(:, k, 2), [numel(right_acc_dir) 1]) ...
            .* sind(right_acc_dir);
    right_cent_accel_guess_hist(k, :) = right_cent_accel_guess;
    
    all_cent_accel_guess = [left_cent_accel_guess; right_cent_accel_guess];
    all_cent_accel_guess_hist(k, :) = all_cent_accel_guess;
    
    left_tang_accel_guess = ...
        -reshape(left_acc_data(:, k, 1), [numel(left_acc_dir) 1]) ...
            .* sind(left_acc_dir) ...
        + reshape(left_acc_data(:, k, 2), [numel(left_acc_dir) 1]) ...
            .* cosd(left_acc_dir);
    left_tang_accel_guess_hist(k, :) = left_tang_accel_guess;
    
    right_tang_accel_guess = ...
        -reshape(right_acc_data(:, k, 1), [numel(right_acc_dir) 1]) ...
            .* sind(right_acc_dir) ...
        + reshape(right_acc_data(:, k, 2), [numel(right_acc_dir) 1]) ...
            .* cosd(right_acc_dir);    
    right_tang_accel_guess_hist(k, :) = right_tang_accel_guess;

    all_tang_accel_guess = [left_tang_accel_guess; right_tang_accel_guess];
    all_tang_accel_guess_hist(k, :) = all_tang_accel_guess;
    
    ang_accel_guess = ( right_tang_accel_guess(acc_pairs(:, 2)) ...
        - left_tang_accel_guess(acc_pairs(:, 1)) ) ...
        ./ acc_dists;
    ang_accel_guess_hist(k, :) = ang_accel_guess;
    
    all_ang_accel_guess = ( all_tang_accel_guess(all_acc_pairs(:, 2)) ...
        - all_tang_accel_guess(all_acc_pairs(:, 1)) ) ...
        ./ all_acc_dists;
    all_ang_accel_guess_hist(k, :) = all_ang_accel_guess;
    
    cent_acc_diffs = abs( right_cent_accel_guess(acc_pairs(:, 2)) ...
        - left_cent_accel_guess(acc_pairs(:, 1)) );
    all_cent_acc_diffs = abs( all_cent_accel_guess(all_acc_pairs(:, 2)) ...
        - all_cent_accel_guess(all_acc_pairs(:, 1)) );
    
    meas = cent_acc_diffs;
%     meas = all_cent_acc_diffs;
    
    pred = HEKF.update(meas, uu(k-1, 1), k*Ts, 'acc');
    
%     if(EKF.mags > 0)
        %TODO: Add magnetometer readigns
%     end
    
    angle = wrapToPi(yy(k-1, 1))*180/pi;
    inRange = (angle<1 && angle>-1);
    if(use_ir_beacons && inRange)
        pred = HEKF.update(0, uu(k-1, 1), k*Ts, 'beacon');
    end
    
    pred_hist(k, :) = pred;
    pred = pred(1:2);
    
    % Algo
    if pred(2) > targ_angvel
        uu(k, :) = [uu(k-1, 1) - deltaVolt 0];
        if uu(k, 1) < minVolt
            uu(k, 1) = minVolt;
        end
    elseif pred(2) < targ_angvel
        uu(k, :) = [uu(k-1, 1) + deltaVolt 0];
        if uu(k, 1) > maxVolt
            uu(k, 1) = maxVolt;
        end
    else
        uu(k, :) = uu(k-1, :);
    end
    
end

%% DETERMINE ANGULAR ACCELERATION
dV_all = diff(yy_all(:, 2));    %Calculates the difference between each element of yy_all(:, 2)
a_all = dV_all ./ dt;           %a = dV / dt


%% PHYSICAL QUANTITY PLOTS

fig = figure('units','normalized','outerposition',[0 0 1 1]);

TTplot = [TT duration + Ts];
TTplot_all = linspace(0, TTplot(end), size(yy_all, 1));

subplot(2, 2, 1);
axis on; grid on; hold on;
plot(TTplot, wrapToPi(yy(:, 1)).*180./pi, ...
    TTplot_all, wrapToPi(yy_all(:, 1)).*180./pi, ...
    'LineWidth', 2); 
HEKF.plotPos(fig, 2, 2, 1);
ylabel("Angular Position (deg%360)");
xlabel("Time (s)");
xlim([0 2]);
legend("Discrete @ Ts", "Continuous @ dt", "EKF Output");

subplot(2, 2, 2);
axis on; grid on; hold on;
plot(TTplot, yy(:, 2).*180./pi, ...
    TTplot_all, yy_all(:, 2).*180/pi, ...
    'LineWidth', 2);
yline(18000, "LineWidth", 2);
HEKF.plotVel(fig, 2, 2, 2);
ylabel("Angular Velocity (deg/s)");
xlabel("Time (s)");
legend("Discrete @ Ts", "Continuous @ dt", "Target Steady-State", "EKF Output");

subplot(2, 2, 3);
axis on; grid on;
plot(TTplot, uu(:, 1), 'LineWidth', 2);
ylabel("Input Voltage (V)");
xlabel("Time (s)");

subplot(2, 2, 4);
axis on; grid on; hold on;
plot(TTplot_all(1 : end - 1), a_all .* 180 / pi, 'g-', 'LineWidth', 2);
yline(a_max .* 180 / pi, 'LineWidth', 2);
ylabel("Angular Acceleration (deg/s^2)");
xlabel("Time (s)");
legend("Angular Acceleration", "Maximum Angular Acceleration");

sgtitle(sprintf(['Robot Dynamic Modeling: Kv=%.2e, D=%.3f, R=%.3f, ', ...
    'BotR=%.1f, BotW=%.1f, WheelR=%.1f, WheelW=%.1f, SR=%.1e, ', ...
    'Ts=%.1e, dt=%.1e'], ...
    Kv, D, R, r_robot_im, w_robot_im, r_wheel_im, w_wheel_im, ...
    slew_rate, Ts, dt), ...
    "FontSize", 18);


%% LEFT ACCELEROMETER READING PLOTS

figure('units','normalized','outerposition',[0 0 1 1])
Nacc = numel(left_acc_pos);

for k=1:Nacc
    
    subplot(2*Nacc, 3, 6*k-5);
    axis on; grid on;
    plot(TTplot, -reshape(left_acc_true(k, :, 1), [numel(TTplot) 1]), ...
        '-', "LineWidth", 2);
    hold on
    plot(TTplot, reshape(left_acc_data(k, :, 1), [numel(TTplot) 1]), ...
        '--', "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc #%d +X (m/s^2)", k));
    legend("True", "Measured");
    
    subplot(2*Nacc, 3, 6*k-4);
    axis on; grid on;
    plot(TTplot, -reshape(left_acc_true(k, :, 2), [numel(TTplot) 1]), ...
        '-', "LineWidth", 2);
    hold on
    plot(TTplot, reshape(left_acc_data(k, :, 2), [numel(TTplot) 1]), ...
        '--', "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc #%d +Y (m/s^2)", k));
    legend("True", "Measured");
    
    subplot(2*Nacc, 3, 6*k-3);
    axis on; grid on;
    plot(TTplot, -reshape(left_acc_true(k, :, 3), [numel(TTplot) 1]), ...
        '-', "LineWidth", 2);
    hold on
    plot(TTplot, reshape(left_acc_data(k, :, 3), [numel(TTplot) 1]), ...
        '--', "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc #%d +Z (m/s^2)", k));
    legend("True", "Measured");
    
    subplot(2*Nacc, 3, 6*k-2);
    axis on; grid on;
    plot(TTplot, ...
        reshape(left_acc_data(k, :, 1), [numel(TTplot) 1]) + ...
        reshape(left_acc_true(k, :, 1), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(left_acc_data(k, :, 1), [numel(TTplot) 1]) + ...
        reshape(left_acc_true(k, :, 1), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +X (m/s^2)", k));
    legend("Directional", "Absolute");
    
    subplot(2*Nacc, 3, 6*k-1);
    axis on; grid on;
    plot(TTplot, ...
        reshape(left_acc_data(k, :, 2), [numel(TTplot) 1]) + ...
        reshape(left_acc_true(k, :, 2), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(left_acc_data(k, :, 2), [numel(TTplot) 1]) + ...
        reshape(left_acc_true(k, :, 2), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +Y (m/s^2)", k));
    legend("Directional", "Absolute");
    
    subplot(2*Nacc, 3, 6*k);
    axis on; grid on;
    plot(TTplot, ...
        reshape(left_acc_data(k, :, 3), [numel(TTplot) 1]) + ...
        reshape(left_acc_true(k, :, 3), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(left_acc_data(k, :, 3), [numel(TTplot) 1]) + ...
        reshape(left_acc_true(k, :, 3), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +Z (m/s^2)", k));
    legend("Directional", "Absolute");
   
end

sgtitle("Left-Side Accelerometers", "FontSize", 18);

%% LEFT ACCELEROMETER READING PLOTS

figure('units','normalized','outerposition',[0 0 1 1])
Nacc = numel(right_acc_pos);

for k=1:Nacc
    
    subplot(2*Nacc, 3, 6*k-5);
    axis on; grid on;
    plot(TTplot, -reshape(right_acc_true(k, :, 1), [numel(TTplot) 1]), ...
        '-', "LineWidth", 2);
    hold on
    plot(TTplot, reshape(right_acc_data(k, :, 1), [numel(TTplot) 1]), ...
        '--', "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc #%d +X (m/s^2)", k));
    legend("True", "Measured");
    
    subplot(2*Nacc, 3, 6*k-4);
    axis on; grid on;
    plot(TTplot, -reshape(right_acc_true(k, :, 2), [numel(TTplot) 1]), ...
        '-', "LineWidth", 2);
    hold on
    plot(TTplot, reshape(right_acc_data(k, :, 2), [numel(TTplot) 1]), ...
        '--', "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc #%d +Y (m/s^2)", k));
    legend("True", "Measured");
    
    subplot(2*Nacc, 3, 6*k-3);
    axis on; grid on;
    plot(TTplot, -reshape(right_acc_true(k, :, 3), [numel(TTplot) 1]), ...
        '-', "LineWidth", 2);
    hold on
    plot(TTplot, reshape(right_acc_data(k, :, 3), [numel(TTplot) 1]), ...
        '--', "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc #%d +Z (m/s^2)", k));
    legend("True", "Measured");
    
    subplot(2*Nacc, 3, 6*k-2);
    axis on; grid on;
    plot(TTplot, ...
        reshape(right_acc_data(k, :, 1), [numel(TTplot) 1]) + ...
        reshape(right_acc_true(k, :, 1), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(right_acc_data(k, :, 1), [numel(TTplot) 1]) + ...
        reshape(right_acc_true(k, :, 1), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +X (m/s^2)", k));
    legend("Directional", "Absolute");
    
    subplot(2*Nacc, 3, 6*k-1);
    axis on; grid on;
    plot(TTplot, ...
        reshape(right_acc_data(k, :, 2), [numel(TTplot) 1]) + ...
        reshape(right_acc_true(k, :, 2), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(right_acc_data(k, :, 2), [numel(TTplot) 1]) + ...
        reshape(right_acc_true(k, :, 2), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +Y (m/s^2)", k));
    legend("Directional", "Absolute");
    
    subplot(2*Nacc, 3, 6*k);
    axis on; grid on;
    plot(TTplot, ...
        reshape(right_acc_data(k, :, 3), [numel(TTplot) 1]) + ...
        reshape(right_acc_true(k, :, 3), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(right_acc_data(k, :, 3), [numel(TTplot) 1]) + ...
        reshape(right_acc_true(k, :, 3), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +Z (m/s^2)", k));
    legend("Directional", "Absolute");
   
end

sgtitle("Right-Side Accelerometers", "FontSize", 18);


%% EKF ERROR PLOTS
error = pred_hist(:, 1:2) - [wrapToPi(yy(:, 1)) yy(:, 2)];
error = [wrapToPi(error(:, 1)) error(:, 2)];
figure('units','normalized','outerposition',[0 0 1 1])

subplot(1, 2, 1);
axis on; grid on; hold on;
plot(TTplot, error(:, 1) .* 180 / pi, 'LineWidth', 2);
title('HEKF Position Error');
xlabel('Time (s)');
ylabel('Position Error (deg)');

subplot(1, 2, 2);
axis on; grid on; hold on;
plot(TTplot, lowpass(error(:, 2), .5) .* 180 / pi, 'LineWidth', 2);
title('HEKF Velocity Error');
xlabel('Time (s)');
ylabel('Velocity Error (deg/s)');

sgtitle("HEKF Error", "FontSize", 18);


