clear, close all
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
