function next_state = step1DOFSVM(sys, curr_state, input, dt, Ts)
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
% NOTE: Ts >> dt

tt = 0:dt:Ts; % time vector
actual_input = input(1) - input(2);
[yy, ~, ~] = lsim(sys, repmat(actual_input, numel(tt), 1), tt, curr_state);
next_state = yy(end, :);

end

