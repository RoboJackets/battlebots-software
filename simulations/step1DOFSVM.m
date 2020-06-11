function [next_state, state_history] = ...
    step1DOFSVM(sys, A, B, curr_state, input, dt, Ts, sr, prev_input)
%% STEP1DOFSVM  Steps 1DOF SVM for a uniform disk with wheels 
%                   meltybrain battlebot. 
% next_state - next state of robot, 1-by-2 vector of robot angular position
%               and angular velocity
% 
% sys - A state space object ("ss" object) representing a 1DOF meltybrain
% curr_state - current state of robot, 1-by-2 vector of robot angular
%               position and angular velocity
% input - input to the robot, 1-by-2 vector of motor input voltage and 
%          external resisitve torque applied to motor
% dt - continuous simulation timestep, NOT logic timestep
% Ts - discrete logic timestep, NOT continuous simulation timestep
% sr - slew rate of voltage  response (V / s)
% prev_input - previous input to the robot, 1-by-2 vector of motor input 
%               voltage and external resisitve torque applied to motor
% NOTE: Ts >> dt

ni = nargin;

if ni == 8
    prev_input = [0 0];
end

tt = 0:dt:Ts; % time vector
input_voltage = prev_input(1) + sr .* tt;
input_voltage(input_voltage > input(1)) = input(1); % input voltage 
%                                                    affected by slew rate
input_voltage(isnan(input_voltage)) = 0;
actual_input = input_voltage.' - input(2);
% [yy, ~, ~] = lsim(sys, actual_input, tt, curr_state);
yy = zeros(numel(tt)+1, 2);
yy(1, :) = curr_state;
for k=2:numel(tt)+1
    prevy = yy(k-1, :); % column state vector 
    prevy = prevy.';
    input = actual_input(k-1, :).'; % column input vector 
    nexty = prevy + dt*(A * prevy + B * input); 
                                            % column output vector
    nexty = nexty.';
    yy(k, :) = nexty;
end
yy = yy(2:end, :);
next_state = yy(end, :);
state_history = yy;

end

