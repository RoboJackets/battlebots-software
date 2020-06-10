clear;
close all;

%% PHYSICAL PARAMETERS AND CONSTANTS

% using: Turnigy TrackStar 1/8th Sensored Brushless Motor 1900KV 
Kv = 1900; % Motor Constant (RPM / Volt)
Kt = inv(2*pi*Kv/60); % Kt = 1/Kv if Kv is in (rad/s)/Volt
% Kt = 318; % Motor Constant (N * m / A)
D = 6e-2; % Frictional Loss Constant (N * m * s / rad)
R = 12e-3; % Motor Resistance (Ohms)

r_robot_im = 5; % robot radius (in)
w_robot_im = 11.8; % robot weight (lbs)
r_wheel_im = 2.5; % wheel radius (in)
w_wheel_im = 0.2; % wheel weight (lbs)
% the im suffix stands for imperial 

slew_rate = 1e40; % V / s

sys = create1DOFSVM(Kt, D, R, ...
    r_robot_im, w_robot_im, r_wheel_im, w_wheel_im);


    
%% SIMULATION PARAMETERS

dt = 1e-7; % modeling physics at this rate (seconds)
Ts = 1/(3.2e3); % algorithms updating at this rate (seconds)
duration = 8; % simulation duration (seconds)

initial_position = 0; % inital heading (radians)
initial_velocity = 0; % initial angular velocity (radians / second)
initial_voltage = 15; % initial motor voltage (volts fuck u think)
initial_extrestorque = 0; % initial external resistive torque (volts)

%% SENSOR PARAMETERS

% all angles are in degrees for defining parameters ok just deal
g = 9.81; % m / s^2
sys_temp = 25; % temperature in celsius
% modeling the ADXL375 accelerometer 
% www.analog.com/media/en/technical-documentation/data-sheets/ADXL375.pdf
acc_pos_im = [2 0; 3 0]; % location in polar coordinates, each row is a 
% different sensor with radius and heading from center (distance in inches)
acc_pos = [acc_pos_im(:, 1) .* 0.0254 acc_pos_im(:, 2)];
acc_dir = [0 ; 0]; % angle deviation CW of +y on acc from direction of 
% tangential acceleration if robot rotating CCW
acc_params = [getAccParams("ADXL375", 1); getAccParams("perfect", 1)];
accs = cell(numel(acc_params), 1);
for k=1:numel(acc_params)
    accs{k, 1} = imuSensor("SampleRate", 1/Ts, ...
        "Temperature", sys_temp, ...
        "Accelerometer", acc_params(k));
end

%% ALGORITHM PARAMETERS

% shit goes here think of something smart varun i can't do all the heavy
%  lifting jeez

%% SIMULATION SETUP

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
acc_true = zeros(size(acc_pos, 1), steps, 3);                      
acc_data = zeros(size(acc_pos, 1), steps, 3);
yy_all = zeros(numel(tt).*(steps-1), 2); % yy but with like wayyy more                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  l = zeros(numel(tt).*(steps-1), 2); % yy but with like wayyy more
uu(1, :) = u0;
yy(1, :) = x0;
for k=1:size(acc_pos, 1)
    acc_true(k, 1, :) = [0 0 0];
    acc_data(k, 1, :) = [0 0 0];
end

%% SIMULATION EXECUTION 

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
    ang_vel = yy_all_temp(:, 2); % extract fine angular velocity
    ang_accel = diff(ang_vel) ./ dt; % numerical angular acceleration
    tang_accel = ang_accel * acc_pos(:, 1).'; % tangential acceleration at
    % each sensor in m/s^2
    tang_accel_read = tang_accel(end, :).'; % tangential acceleration read
    % by the accelerometer
    cent_accel = (ang_vel.^2) * acc_pos(:, 1).';
    cent_accel_read = cent_accel(end, :).';
    for kacc = 1:size(acc_pos, 1)
        ay = tang_accel_read(kacc) .* cosd(acc_dir(kacc)) ...
            - cent_accel_read(kacc) .* sind(acc_dir(kacc));
        ax = -cent_accel_read(kacc) .* cosd(acc_dir(kacc)) ...
            - tang_accel_read(kacc) .* sind(acc_dir(kacc));
        acc_true(kacc, k, :) = [ax ay 0];
        curr_acc = accs{kacc, 1};
%         [acc_readings, ~] = curr_acc(squeeze(acc_true(kacc, 1:k, :)), ...
%             [zeros(k, 2) yy(1:k, 2)]);
%         acc_data(kacc, k, :) = acc_readings(end, :);
        [acc_readings, ~] = curr_acc( ...
            reshape(acc_true(kacc, k, :), [1 3]), ...
            [zeros(1, 2) yy(k, 2)]);
        acc_data(kacc, k, :) = acc_readings;
        acc_true(kacc, k, 3) = -g; % for plotting
    end
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ALGO GOES HERE BUT HERE'S THE LOGIC FOR NOW %
    uu(k, :) = uu(k-1, :); % CLOSED-EYE KALMAN 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
end

%% PHYSICAL QUANTITY PLOTS 

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
ylabel("Angular Velocity (deg/s)");
xlabel("Time (s)");
legend("Discrete @ Ts", "Continuous @ dt");

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

%% SENSOR READING PLOTS

figure('units','normalized','outerposition',[0 0 1 1])
Nacc = size(acc_pos, 1);

for k=1:Nacc
    
    subplot(2*Nacc, 3, 6*k-5);
    axis on; grid on;
    plot(TTplot, -reshape(acc_true(k, :, 1), [numel(TTplot) 1]), ...
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
    plot(TTplot, -reshape(acc_true(k, :, 2), [numel(TTplot) 1]), ...
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
    plot(TTplot, -reshape(acc_true(k, :, 3), [numel(TTplot) 1]), ...
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
        reshape(acc_data(k, :, 1), [numel(TTplot) 1]) + ...
        reshape(acc_true(k, :, 1), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(acc_data(k, :, 1), [numel(TTplot) 1]) + ...
        reshape(acc_true(k, :, 1), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +X (m/s^2)", k));
    legend("Directional", "Absolute");
    
    subplot(2*Nacc, 3, 6*k-1);
    axis on; grid on;
    plot(TTplot, ...
        reshape(acc_data(k, :, 2), [numel(TTplot) 1]) + ...
        reshape(acc_true(k, :, 2), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(acc_data(k, :, 2), [numel(TTplot) 1]) + ...
        reshape(acc_true(k, :, 2), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +Y (m/s^2)", k));
    legend("Directional", "Absolute");
    
    subplot(2*Nacc, 3, 6*k);
    axis on; grid on;
    plot(TTplot, ...
        reshape(acc_data(k, :, 3), [numel(TTplot) 1]) + ...
        reshape(acc_true(k, :, 3), [numel(TTplot) 1]), '-', ...
        "LineWidth", 2);
    hold on
    plot(TTplot, ...
        abs(reshape(acc_data(k, :, 3), [numel(TTplot) 1]) + ...
        reshape(acc_true(k, :, 3), [numel(TTplot) 1])), '--', ...
        "LineWidth", 2);
    hold off
    xlabel("Time (s)");
    ylabel(sprintf("Acc Error #%d +Z (m/s^2)", k));
    legend("Directional", "Absolute");
   
end

sgtitle("Accelerometers", "FontSize", 18);
