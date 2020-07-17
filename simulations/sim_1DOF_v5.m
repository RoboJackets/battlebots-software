clear;
close all;

%% PHYSICAL PARAMETERS AND CONSTANTS

% using: Scorpion HKIV-4020-860KV
Kv = 860; % Motor Constant (RPM / Volt)
gear_rat = 1; % gear ratio (RPM / RPM)
Kt = inv(2*pi*Kv*gear_rat/60); % Kt = 1/Kv if Kv is in (rad/s)/Volt
% Kt = 318; % Motor Constant (N * m / A)
D = 1e-5; % Frictional Loss Constant (N * m * s / rad)
R = 12e-3; % Motor Resistance (Ohms)

r_robot_im = 3; % robot radius (in)
w_robot_im = 11.8; % robot weight (lbs)
r_wheel_im = 1.5; % wheel radius (in)
w_wheel_im = 0.2; % wheel weight (lbs)
% the im suffix stands for imperial 

r_robot_outer_im = 5;
w_robot_tot_im = 12;

r_robot_outer = 0.0254 * r_robot_outer_im;
w_robot_tot = 0.453592 * w_robot_tot_im;
mir = r_robot_outer^2 * w_robot_tot * 1/2; % 3/4 is between a disk (1/2) and a ring (1)

mu_static = 0.8; %Coefficient of static friction between wheel and ground

slew_rate = 1e40; % V / s

[sys, A, B, ~, ~, alpha, beta] = create1DOFSVM(Kt, D, R, ...
    r_robot_im, w_robot_im, r_wheel_im, w_wheel_im, mir);


    
%% SIMULATION PARAMETERS

dt = 1e-7; % modeling physics at this rate (seconds)
Ts = 1/(1e3); % algorithms updating at this rate (seconds)
hekf_dt = Ts / 10; % 10 HEKF sim steps per update
duration = 6; % simulation duration (seconds)

initial_position = 0; % inital heading (radians)
initial_velocity = 0; % initial angular velocity (radians / second)
initial_voltage = 3; % initial motor voltage (volts fuck u think)
initial_extrestorque = 0; % initial external resistive torque (volts)

trans_accel = [0 0]; % trans accel in [+x, +y] direction
use_perfect_accs = false;
use_random_accs = false;
accScaling = 1.0;
lin_fact = 1.0095;
if(~use_perfect_accs)
    accScaling = 1;
end
use_ir_beacons = false;
use_mag = true;

%% SENSOR PARAMETERS

% General %
sys_temp = 25;


% IR %
ir_eps = 1e-1; % epsilon for detecting ir beacon


% MAG % 
mag_pos_im = [3 90];
mag_pos = [mag_pos_im(:, 1) .* 0.0254 mag_pos_im(:, 2)];
mag_dir = 45;
mag_params = getMagParams("HMC1052", 10, 5, false);
mag = imuSensor("accel-mag", "SampleRate", 1/Ts, ...
    "Temperature", sys_temp, ...
    "Magnetometer", mag_params); 

% ACCS %
% all angles are in degrees for defining parameters ok just deal
g = 9.81; % m / s^2
% modeling the ADXL375 accelerometer 
% www.analog.com/media/en/technical-documentation/data-sheets/ADXL375.pdf

% location in polar coordinates, each row is a 
% different sensor with radius and heading from center (distance in inches)
acc_pos_im = [0.35 0; 0.35 90; 0.35 180; 0.35 270];
acc_pos = [acc_pos_im(:, 1) .* 0.0254 acc_pos_im(:, 2)];

% angle deviation CW from +j_hat vector on acc
acc_dir = [45; 45; 45; 45]; 

acc_params = [];
accs = cell(numel(acc_params), 1);
for k=1:size(acc_pos, 1)
    if use_perfect_accs
        acc_params = [acc_params ; getAccParams("perfect", 1)];        
    else
        acc_params = [acc_params ; getAccParams("ADXL375", use_random_accs)];
    end

    accs{k, 1} = imuSensor("accel-mag", "SampleRate", 1/Ts, ...
        "Temperature", sys_temp, ...
        "Accelerometer", acc_params(k));
end

%% PAIR ACCELEROMETERS

[pairs1, pairs2] = meshgrid(1:numel(acc_dir), 1:numel(acc_dir));
pairs = [pairs1(:) pairs2(:)];
pairs = pairs(pairs(:, 2) > pairs(:, 1), :);
pairs_d = [acc_pos(pairs(:, 2), 1).*cosd(acc_pos(pairs(:, 2), 2)) ...
    - acc_pos(pairs(:, 1), 1).*cosd(acc_pos(pairs(:, 1), 2)) ...
    acc_pos(pairs(:, 2), 1).*sind(acc_pos(pairs(:, 2), 2)) ...
    - acc_pos(pairs(:, 1), 1).*sind(acc_pos(pairs(:, 1), 2))
];
pairs_d = [sqrt(pairs_d(:, 2).^2 + pairs_d(:, 1).^2) atan2(pairs_d(:, 2), pairs_d(:, 1))];


%% CALCULATE MAX ANGULAR ACCELERATION
%The maximum force of static friction on each wheel
%Given by half the weight of the robot (converted to N) * static friction
%coefficient
w_robot = w_robot_im * 0.453592;          %Convert to MKS units
r_robot = r_robot_im * 0.0254;
I_robot = 0.5 .* w_robot .* r_robot .^ 2; %Robot body inertia

f_max = mu_static * (w_robot / 2) * g; 
torque_max = 2 * f_max * r_robot;
a_max = torque_max / I_robot;             %Torque = I * a

%% SIMULATION SETUP

% sysd = c2d(sys, dt); % ??? discretize ??? maybe ??? faster ???
sysd = sys;

u0 = [initial_voltage initial_extrestorque]; % initial input state
x0 = [initial_position initial_velocity];    % initial state space state
TT = 0:Ts:duration;   % algorithm evaluation times (seconds)
tt = 0:dt:Ts;
steps = numel(TT)+1;  % number of times algorithm is run (#)
uu = zeros(steps, 2); % all inputs (steps -> motor voltage (volts), 
                      %              external resistive torque (volts))
yy = zeros(steps, 2); % all states (steps -> angular position in rad, 
                      %                         angular velocity in rad/s)
pred_hist = zeros(steps, 2);
ang_accel_pred_hist = zeros(steps, 1);
acc_true = zeros(size(acc_pos, 1), steps, 3);                      
acc_data = zeros(size(acc_pos, 1), steps, 3);
mag_data = zeros(steps, 3); % +x, +y mag field
mag_true = zeros(steps, 3);
yy_all = zeros(numel(tt).*(steps-1), 2); % yy but with like wayyy more
uu(1, :) = u0;
yy(1, :) = x0;
for k=1:size(acc_pos, 1)
    acc_true(k, 1, :) = [0 0 0];
    acc_data(k, 1, :) = [0 0 0];
end

%% EKF Setup

targ_angvel = 370; % target angular velocity (rad/s)
thresh_angvel = 0.04 * targ_angvel;

deltaVolt = 4e-2; % max change in volts every Ts
deltaVolt_SS = 4e-2; % max change in volts every Ts once steady-state hit
ss_hit = false;
maxVolt = 22; % max voltage we can input
minVolt = -22; % min voltage we can input

%                      dt       alpha  beta  accD,          wheelR                botR                  imus
HEKF = MeltyBrain_HEKF(hekf_dt, alpha, beta, pairs_d(:, 1), ...
    0.0254 .* r_wheel_im, 0.0254 .* r_robot_im, size(pairs, 1), ...
    deg2rad(mag_dir))
 
ang_accel_win = zeros(7, 1);


%% SIMULATION EXECUTION 

for k=2:steps
    
    fprintf("%d/%d\n", k, steps);
    
    % Update the physics 
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
    ang_vel = yy_all_temp(:, 2); % extract fine angular velocity
    ang_accel = diff(ang_vel) ./ dt; % numerical angular acceleration
    tang_accel = ang_accel * acc_pos(:, 1).'; % tangential acceleration at
    % each sensor in m/s^2
    tang_accel_read = tang_accel(end, :).'; % tangential acceleration read
    % by the accelerometer
    cent_accel = (ang_vel.^2) * acc_pos(:, 1).';
    cent_accel_read = cent_accel(end, :).';
    
    for kacc = 1:size(acc_pos, 1)
        % these formulas are correct, trust me, i would know, i made them
        acck_ang_cent = deg2rad(acc_pos(kacc, 2) + acc_dir(kacc));
        acck_ang_xy = deg2rad(acc_dir(kacc)) - yy(k, 1);
        ay = -cent_accel_read(kacc).*sin(acck_ang_cent) + ...
            tang_accel_read(kacc).*cos(acck_ang_cent) + ...
            trans_accel * [sin(acck_ang_xy); cos(acck_ang_xy)];
        ax = -cent_accel_read(kacc).*cos(acck_ang_cent) + ...
            -tang_accel_read(kacc).*sin(acck_ang_cent) + ...
            trans_accel * [cos(acck_ang_xy); -sin(acck_ang_xy)];
        acc_true(kacc, k, :) = [ax ay 0];
        curr_acc = accs{kacc, 1};
%       [acc_readings, ~] = curr_acc(squeeze(acc_true(kacc, 1:k, :)), ...
%           [zeros(k, 2) yy(1:k, 2)]);
%           acc_data(kacc, k, :) = acc_readings(end, :);
        [acc_readings, ~] = curr_acc( ...
            reshape(acc_true(kacc, k, :), [1 3]), ...
            [0 0 0]);
        acc_data(kacc, k, :) = -acc_readings;
        acc_true(kacc, k, 3) = acc_true(kacc, k, 3) - g; % for plotting
    end
    
    %Update the Kalman Filter with the accelerometer reading and commanded
    %voltage
    % precalculate the centripetal and tangential acceleration 
    % from the measured data first

    % a_c_pair_sensor1 = a_y1*sin(heading_1 + <pair) + a_x1*cos(heading_1 +
    % <pair)
    % a_c_pair_sensor2 = a_y2*sin(heading_2 + <pair) + a_x2*cos(heading_2 +
    % <pair)
    % a_c_pair = abs(a_c_pair_sensor2 - a_c_pair_sensor1)
    cent_accel_guess_sens2 = ...
        reshape(acc_data(pairs(:, 2), k, 2), [size(pairs, 1) 1]) .* ...
            sin(deg2rad(acc_dir(pairs(:, 2))) + pairs_d(:, 2)) ...
        + reshape(acc_data(pairs(:, 2), k, 1), [size(pairs, 1) 1]) .* ...
            cos(deg2rad(acc_dir(pairs(:, 2))) + pairs_d(:, 2));
    cent_accel_guess_sens1 = ...
        reshape(acc_data(pairs(:, 1), k, 2), [size(pairs, 1) 1]) .* ...
            sin(deg2rad(acc_dir(pairs(:, 1))) + pairs_d(:, 2)) ...
        + reshape(acc_data(pairs(:, 1), k, 1), [size(pairs, 1) 1]) .* ...
            cos(deg2rad(acc_dir(pairs(:, 1))) + pairs_d(:, 2));
    cent_accel_guess = cent_accel_guess_sens2 - cent_accel_guess_sens1;
    
    % a_t_pair_sensor1 = a_y1*cos(heading_1 + <pair) - a_x1*sin(heading_1 +
    % <pair)
    % a_t_pair_sensor2 = a_y2*cos(heading_2 + <pair) - a_x2*sin(heading_2 +
    % <pair)
    % a_t_pair = abs(a_c_pair_sensor2 - a_c_pair_sensor1)    
    tang_accel_guess_sens2 = ...
        reshape(acc_data(pairs(:, 2), k, 2), [size(pairs, 1) 1]) .* ...
            cos(deg2rad(acc_dir(pairs(:, 2))) + pairs_d(:, 2)) ...
        - reshape(acc_data(pairs(:, 2), k, 1), [size(pairs, 1) 1]) .* ...
            sin(deg2rad(acc_dir(pairs(:, 2))) + pairs_d(:, 2));
    tang_accel_guess_sens1 = ...
        reshape(acc_data(pairs(:, 1), k, 2), [size(pairs, 1) 1]) .* ...
            cos(deg2rad(acc_dir(pairs(:, 1))) + pairs_d(:, 2)) ...
        - reshape(acc_data(pairs(:, 1), k, 1), [size(pairs, 1) 1]) .* ...
            sin(deg2rad(acc_dir(pairs(:, 1))) + pairs_d(:, 2));
    tang_accel_guess = tang_accel_guess_sens2 - tang_accel_guess_sens1;
    
    % Assemble accelerometer measurements
    meas = abs(cent_accel_guess);
    meas = meas .* accScaling;
    % Predict state using EKF
    %                  meas  u             t       sensor
    pred = HEKF.update(meas, uu(k - 1, 1), k * Ts, 'acc');
    
    % Assemble beacon measurements
    if(k == 2)
        prevAngle = 0;      %Angle in the previous controller step
    end
    angle = wrapToPi(yy(k - 1, 1)) * 180 / pi;
    inRange = prevAngle < 0 && angle > 0;
    if(use_ir_beacons && inRange)
        pred = HEKF.update(0, uu(k - 1, 1), k * Ts, 'beacon');
    end
    reflectionChance = 0.0;
    inRange = (angle > 179 || angle < -179);
    if(use_ir_beacons && inRange && rand() < reflectionChance)
        pred = HEKF.update(0, uu(k - 1, 1), k * Ts, 'beacon');
    end
    prevAngle = angle;
    
    % Assemble magnetometer measurements
    
    if use_mag
        mag_ang_cent = deg2rad(mag_pos(2) + mag_dir);
        mag_ang_xy = deg2rad(mag_dir) - yy(k, 1);
        mag_cent_accel = (ang_vel.^2) .* mag_pos(1);
        mag_cent_accel_read = mag_cent_accel(end);
        mag_tang_accel = ang_accel .* mag_pos(1);
        mag_tang_accel_read = mag_tang_accel(end);
        mag_ay = -mag_cent_accel_read.*sin(mag_ang_cent) + ...
            mag_tang_accel_read.*cos(mag_ang_cent) + ...
            trans_accel * [sin(mag_ang_xy); cos(mag_ang_xy)];
        mag_ax = -mag_cent_accel_read.*cos(mag_ang_cent) + ...
            -mag_tang_accel_read.*sin(mag_ang_cent) + ...
            trans_accel * [cos(mag_ang_xy); -sin(mag_ang_xy)];
        mag_acc = [mag_ax mag_ay 0];
        
        mag_ang = yy(k, 1) - deg2rad(mag_dir);
        mag_orientation = quaternion(cos(mag_ang/2), 0, 0, sin(mag_ang/2));

        [~, mag_reading] = mag(mag_acc, [0 0 yy(k, 2)], mag_orientation);
                
        mag_data(k, :) = mag_reading;
        
        Bmag = norm(mag_reading(1:2), 2);
        
        mag_true(k, :) = [Bmag .* cos(mag_ang), -Bmag .* sin(mag_ang), 0];
        
        if pred(2) > targ_angvel / 3
            pred = HEKF.update(mag_reading(1), uu(k-1, 1), k*Ts, 'mag_x', Bmag);
            pred = HEKF.update(mag_reading(2), uu(k-1, 1), k*Ts, 'mag_y', Bmag);
        end
    
    end
    
    pred_hist(k, :) = pred;
    
    % Guess at the current angular acceleration
    if pred(2) < thresh_angvel
        ang_accel_guess = tang_accel_guess ./ pairs_d(:, 1);
        ang_accel_guess = mean(ang_accel_guess);
        ang_accel_win = [ang_accel_guess; ang_accel_win(1:end-1)];
        ang_accel_filt = mean(ang_accel_win);
        ang_accel_pred = ang_accel_filt;
    else
        ang_accel_pred = (pred(2) - pred_hist(k-1, 2)) ./ Ts;
    end
    ang_accel_pred_hist(k) = ang_accel_pred;
    
    % Check if SS hit according to HEKF
    if ~ss_hit && pred(2) >= targ_angvel
        ss_hit = true;
    end
    
    % Change voltages or something %
    if ang_accel_pred < 0.95*a_max % safe from slip land % 
        if pred(2) > targ_angvel % too fast too quick
            if ss_hit
                uu(k, :) = [uu(k-1, 1) - deltaVolt_SS 0];
            else
                uu(k, :) = [uu(k-1, 1) - deltaVolt 0];                
            end
            if uu(k, 1) < minVolt
                uu(k, 1) = minVolt;
            end
        else % too slow :( %
            if ss_hit
                uu(k, :) = [uu(k-1, 1) + deltaVolt_SS 0];
            else
                uu(k, :) = [uu(k-1, 1) + deltaVolt 0];                
            end
            if uu(k, 1) > maxVolt
                uu(k, 1) = maxVolt;
            end
        end
    else
        % big change in V to prevent slip %
        uu(k, :) = [uu(k-1, 1) - 2.5e0*deltaVolt 0];
        if uu(k, 1) < minVolt
            uu(k, 1) = minVolt;
        end
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
plot(TTplot, wrapToPi(yy(:, 1)) .*180./pi, ...
    TTplot_all, wrapToPi(yy_all(:, 1)).*180./pi, ...
    'LineWidth', 2); 
HEKF.plotPos(fig, 2, 2, 1);
ylabel("Angular Position (deg%360)");
xlabel("Time (s)");
xlim([0 1]);
legend("Discrete @ Ts", "Continuous @ dt", "EKF Output");

subplot(2, 2, 2);
axis on; grid on; hold on;
plot(TTplot, yy(:, 2).*180./pi, ...
    TTplot_all, yy_all(:, 2).*180/pi, ...
    'LineWidth', 2);
yline(targ_angvel * 180 / pi, "LineWidth", 2);
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
axis on; grid on;
plot(TTplot_all(1 : end - 1), a_all .* 180 / pi, 'LineWidth', 2);
hold on;
plot(TTplot, ang_accel_pred_hist .* 180 / pi, ':', 'LineWidth', 2);
yline(a_max .* 180 / pi, 'LineWidth', 2);
hold off;
legend("Actual Angular Acceleration",  "Estimated Angular Acceleration", ...
    "Maximum Allowable Angular Acceleration");
ylabel("Angular Acceleration (deg/s^2)");
xlabel("Time (s)");
h = get(gca, 'Children');
set(gca, 'Children', [h(3) h(2) h(1)]);

sgtitle(sprintf(['Robot Dynamic Modeling: Kv=%.2e, D=%.1e, R=%.3f, ', ...
    'BotR=%.1f, WheelR=%.1f, SR=%.1e, ', ...
    'Ts=%.1e, dt=%.1e'], ...
    Kv, D, R, r_robot_im, r_wheel_im, ...
    slew_rate, Ts, dt), ...
    "FontSize", 18);


%% ACCELEROMETER PLOTS

figure('units','normalized','outerposition',[0 0 1 1])
Nacc = size(acc_pos, 1);

for k=1:Nacc
    
    subplot(2*Nacc, 3, 6*k-5);
    axis on; grid on;
    plot(TTplot, reshape(acc_true(k, :, 1), [numel(TTplot) 1]), ...
        '-', "LineWidth", 2);
    hold on
    plot(TTplot, reshape(acc_data(k, :, 1), [numel(TTplot) 1]), ...
        '--', "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc #%d +X (m/s^2)", k));
    legend("True", "Measured");
    
    subplot(2*Nacc, 3, 6*k-4);
    axis on; grid on;
    plot(TTplot, reshape(acc_true(k, :, 2), [numel(TTplot) 1]), ...
        '-', "LineWidth", 2);
    hold on
    plot(TTplot, reshape(acc_data(k, :, 2), [numel(TTplot) 1]), ...
        '--', "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc #%d +Y (m/s^2)", k));
    legend("True", "Measured");
    
    subplot(2*Nacc, 3, 6*k-3);
    axis on; grid on;
    plot(TTplot, reshape(acc_true(k, :, 3), [numel(TTplot) 1]), ...
        '-', "LineWidth", 2);
    hold on
    plot(TTplot, reshape(acc_data(k, :, 3), [numel(TTplot) 1]), ...
        '--', "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc #%d +Z (m/s^2)", k));
    legend("True", "Measured");
    
    subplot(2*Nacc, 3, 6*k-2);
    axis on; grid on;
    plot(TTplot, ...
        reshape(acc_data(k, :, 1), [numel(TTplot) 1]) - ...
        reshape(acc_true(k, :, 1), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(acc_data(k, :, 1), [numel(TTplot) 1]) - ...
        reshape(acc_true(k, :, 1), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +X (m/s^2)", k));
    legend("Directional", "Absolute");
    
    subplot(2*Nacc, 3, 6*k-1);
    axis on; grid on;
    plot(TTplot, ...
        reshape(acc_data(k, :, 2), [numel(TTplot) 1]) - ...
        reshape(acc_true(k, :, 2), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(acc_data(k, :, 2), [numel(TTplot) 1]) - ...
        reshape(acc_true(k, :, 2), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +Y (m/s^2)", k));
    legend("Directional", "Absolute");
    
    subplot(2*Nacc, 3, 6*k);
    axis on; grid on;
    plot(TTplot, ...
        reshape(acc_data(k, :, 3), [numel(TTplot) 1]) - ...
        reshape(acc_true(k, :, 3), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(acc_data(k, :, 3), [numel(TTplot) 1]) - ...
        reshape(acc_true(k, :, 3), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +Z (m/s^2)", k));
    legend("Directional", "Absolute");
   
end

sgtitle("Accelerometers", "FontSize", 18);

%% MAGNETOMETER PLOTS

if use_mag

    figure;
    
    subplot(3, 2, 1);
    plot(TTplot, mag_data(:, 1), 'LineWidth', 2);
    hold on;
    plot(TTplot, mag_true(:, 1), 'LineWidth', 2);
    hold off;
    ylabel("Magnetic Field +x (uT)");
    xlabel("Time (s)");
    title("X-Axis Magnetometer");
    legend(["Measured", "True"]);
    
    subplot(3, 2, 2);
    plot(TTplot, mag_data(:, 2), 'LineWidth', 2);
    hold on;
    plot(TTplot, mag_true(:, 2), 'LineWidth', 2);
    hold off;
    ylabel("Magnetic Field +y (uT)");
    xlabel("Time (s)");
    title("Y-Axis Magnetometer");
    legend(["Measured", "True"]);
    
    subplot(3, 2, 3);
    plot(TTplot, mag_data(:, 1) - mag_true(:, 1), 'LineWidth', 2);
    ylabel("Magnetic Field +x Error (uT)");
    xlabel("Time (s)");
    
    subplot(3, 2, 4);
    plot(TTplot, mag_data(:, 2) - mag_true(:, 2), 'LineWidth', 2);
    ylabel("Magnetic Field +x Error (uT)");
    xlabel("Time (s)");
    
    subplot(3, 2, 5:6);
    plot(TTplot, atan2(mag_data(:, 1), mag_data(:, 2)), 'LineWidth', 2);
    hold on
    plot(TTplot, wrapToPi(yy(:, 1)), 'LineWidth', 2);
    hold off
    ylabel("Atan2");
    xlabel("Time (s)");
    legend("Measured", "True");
    
    sgtitle("Magnetometers", "FontSize", 18);
    
end

%% EKF ERROR PLOTS

% error = HEKF.x_all' - yy;
error = pred_hist - [wrapToPi(yy(:, 1)) yy(:, 2)];
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
plot(TTplot, lowpass(error(:, 2) .* 180 / pi, 0.5), 'LineWidth', 2);
title('HEKF Velocity Error');
xlabel('Time (s)');
ylabel('Velocity Error (deg/s)');

sgtitle("HEKF Error", "FontSize", 18);


