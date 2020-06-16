function [next_state, state_history] = ...
    step1DOFSVM(sys, curr_state, input, dt, Ts, sr, prev_input)
%% STEP1DOFSVM  Steps planar SVM for a uniform disk with wheels 
%                   meltybrain battlebot. 
% next_state - next state of robot, 1-by-2 vector of wheels' angular speed
% sys - A state space object ("ss" object) representing a 1DOF meltybrain
% curr_state - current state of robot, 1-by-2 vector of wheels' angular
% speed
% input - input to the robot, 1-by-4 vector of motors input voltage and 
%          external resisitve torques applied to motors
% dt - continuous simulation timestep, NOT logic timestep
% Ts - discrete logic timestep, NOT continuous simulation timestep
% sr - slew rate of voltage  response (V / s)
% prev_input - previous input to the robot, 1-by-2 vector of motor input 
%               voltage and external resisitve torque applied to motor
% NOTE: Ts >> dt

ni = nargin;

if ni == 6
    prev_input = [0 0 0 0];
end

tt = 0:dt:Ts; % time vector
input_voltage(1) = prev_input(1) + sr .* tt;
input_voltage(2) = prev_input(3) + sr .* tt;
if(input_voltage > input(1))% check if slew rate allows for more change than is attempted
    input_voltage(1) = input(1); 
end
if(input_voltage(2) > input(3)) 
    input_voltage(2) = input(3);
end
  
input_voltage(isnan(input_voltage)) = 0;

actual_input(1) = input_voltage(1) - input(2);
actual_input(2) = input_voltage(2) - input(4);
[yy, ~, ~] = lsim(sys, actual_input, tt, curr_state);
next_state = yy(end, :);
state_history = yy;

end

