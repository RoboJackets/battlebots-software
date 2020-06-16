clear, close all
% eqn creds to Alex what a guy

%% CONSTANTS

Kt = 1; % Motor Constant (N * m / A)
D = 0.01; % Frictional Loss Constant (N * m * s / rad)
R = 1; % Motor Resistance ( Ohms )
g = 386.4; % acceleration due to gravity (in / s^2)


%% USER DEFINED PARAMETERS (IMPERIAL)

r_robot_im = 6; % robot radius (in)
w_robot_im = 12; % robot weight (lbs)
r_wheel_im = 1.5; % wheel radius (in)
w_wheel_im = 0.2; % wheel weight (lbs)

%% CALCULATED PARAMETERS

r_robot = 0.0254 .* r_robot_im; % robot radius (m)
w_robot = 0.453592 .* w_robot_im; % robot mass (kg)
r_wheel = 0.0254 .* r_wheel_im; % wheel radius (m)
w_wheel = 0.453592 .* w_wheel_im; % wheel mass (kg)

% inertia in kg * m^2
I_robot = 0.5 .* w_robot .* r_robot .^ 2; % robot body inertia
I_wheel = 0.5 .* w_wheel .* r_wheel .^ 2; % wheel inertia
J = 2 .* I_wheel + I_robot .* r_wheel .^ 2 ./ (r_robot .^ 2); 
    % effective inertia 
alpha = (Kt.^2 + D .* R) ./ (J .* R); % in seconds / meter
beta = Kt ./ (J .* R); % in (V * m)^-1

%% STATE SPACE SETUP

A = [0 1 ; 0 -alpha];
B = [0 ; r_wheel .* beta ./ r_robot];
C = [0 1];
D = [0 0];
sys = ss(A, B, C, D);

%% STATE SPACE PARAMETERS

duration = 8; % duration of simulation (seconds)
Ts = 1e-3; % data clock rate (seconds)








%% System Model:
%  


% System constants


% Simulation Variables
dt = 0.001;  %Time Step
T = 0.01;     %Controller Period of 100 Hz
t = 0:dt:8;   %4 seconds of simulation


x = zeros(2, length(t)); % vector to hold state (can change system order)
y = zeros(1, length(t)); % vector to hold measuremetns

% x(:, 1) = [0; 0.001]; % here you can set initial state

u = 0; % initial value of control input

% Control parameters

% Put tunable parameters here

for k=1:length(t)

    % DT Controller
    if(abs(t(k)-l*T) < dt/2 || t(k) == 0)  %avoids numerical error in t
        % Put discrete-time controller updates here
    else
        % As per zero order hold model, assign current value to previous
    end
    
    % CT Controller if there's nothing in DT
    
    % Continuous Time state update with forward euler method
    x(:,k+1) = x(:,k) + dt*f(x(:,k), u);
end

% time-invariant state space model has state derivative as function of
% current inputs and current state.  This function returns the derivative
function xdot = f(x, u)

end
