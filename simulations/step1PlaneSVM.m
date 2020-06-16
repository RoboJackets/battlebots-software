function [next_state, state_history] = ...
    step1DOFSVM(sys, A, B, curr_state, input, dt, Ts, sr, prev_input)
%% STEP1DOFSVM  Steps planar SVM for a uniform disk with wheels 
%                   meltybrain battlebot. 
% next_state - next state of robot, 1-by-4 vector of wheels' angular speed
% sys - A state space object ("ss" object) representing a 1DOF meltybrain
% curr_state - current state of robot, 1-by-4 vector of wheels' angular
% speed
% input - input to the robot, 1-by-4 vector of motors input voltage and 
%          external resisitve torques applied to motors
% dt - continuous simulation timestep, NOT logic timestep
% Ts - discrete logic timestep, NOT continuous simulation timestep
% sr - slew rate of voltage  response (V / s)
% prev_input - previous input to the robot, 1-by-4 vector of motor input 
%               voltage and external resisitve torque applied to motor
% NOTE: Ts >> dt

ni = nargin;

if ni == 8
    prev_input = [0 0];
end

tt = 0:dt:Ts; % time vector
input_voltage = zeros(2, numel(tt));

%fprintf("sandbox is %d\n", prev_input(1) + sr .* tt);
% fprintf("size of sr .* tt is %d\n", size(sr .* tt));
% fprintf("size of prev_input is %d\n", size(prev_input(1)));
% fprintf("size of input_voltage(1,:) is %d\n", size(input_voltage(1,:)));
input_voltage(1,:) = prev_input(1) + sr .* tt;
input_voltage(2,:) = prev_input(2) + sr .* tt;

%if(input_voltage(1) > input(1))
%    input_voltage(1) = input(1);
%end
%if(input_voltage(2,:) > input(2))
%    input_voltage(2) = input(2);
%end
% fprintf("size of input_voltage is %d\n", size(input_voltage));
actual_input = zeros(2, numel(tt));
actual_input(1,:) = input_voltage(1,:) - input(3);
actual_input(2,:) = input_voltage(2,:) - input(4);

% fprintf("size of actual_input is %d\n", size(actual_input));

% [yy, ~, ~] = lsim(sys, actual_input, tt, curr_state);
yy = zeros(4, numel(tt)+1);
yy(:, 1) = curr_state.';
for k=2:numel(tt)+1
%     temp = size(yy(:, k-1));
%     %fprintf("yy(:, k-1) is %d\n", temp);
%     temp = size(A * yy(:, k-1));
%     fprintf("A*yy(:, k-1) is %d\n", temp);
%     temp = size(B * actual_input(k-1));
% %     fprintf("B*actual_input(k-1) is %d\n", temp);
%     
%     temp = size(B);
%     fprintf("B is %d\n", temp);
%     
%     fprintf("size of actual_input(k-1, :) is %d\n", size(actual_input(:, k-1)));
    yy(:, k) = yy(:, k-1) + dt*((A * yy(:, k-1)) + (B * actual_input(:, k-1))); 
                                            % column output vector
end
yy = yy(:, 2:end).';
next_state = yy(end, :);
state_history = yy;

end

