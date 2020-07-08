function [sys, A, B, C, D, alpha, beta] = create1DOFSVM(Kt, D, R, r_robot_im, ...
    w_robot_im, r_wheel_im, w_wheel_im)
% CREATE1DOFSVM   Creates 1DOF SVM for a uniform disk with wheels 
%                   meltybrain battlebot. 
%
% sys - A state space object ("ss" object)
% A, B, C, D - Matrices to define sys using the "ss" MATLAB function
% 
% Kt - Motor Torque Constant (in N*m/A)
% D - Frictional Loss Constant (N*m*s/rad)
% R - Motor Resistance (Ohms)
% r_robot_im - Radius of Robot Body (in inches)
% w_robot_im - Weight of Robot Body (in lbs)
% r_wheel_im - Distance from Bot Center to Wheels (in inches)
% w_robot_im - Weight of Robot Wheels (in lbs)
% TODO: REVIEW EQNS FOR ACCURACY

r_robot = 0.0254 .* r_robot_im; % robot radius (m)
w_robot = 0.453592 .* w_robot_im; % robot mass (kg)
r_wheel = 0.0254 .* r_wheel_im; % wheel radius (m)
w_wheel = 0.453592 .* w_wheel_im; % wheel mass (kg)

% inertia in kg * m^2
I_robot = 0.5 .* w_robot .* r_robot .^ 2; % robot body inertia
I_wheel = 0.5 .* w_wheel .* r_wheel .^ 2; % wheel inertia
J = 2 .* I_wheel + I_robot .* r_wheel .^ 2 ./ (r_robot .^ 2); 
    % effective inertia 
alpha = (Kt.^2 + D .* R ./ 2) ./ (J .* R ./ 2); % in seconds / meter
% R / 2 as these are TWO MOTORS acting IDENTICALLY
beta = Kt ./ (J .* R); % in (V * m)^-1

A = [0 1 ; 0 -alpha];
B = [0 ; r_wheel .* beta ./ r_robot];
C = eye(2);
D = [0; 0];
sys = ss(A, B, C, D);

end