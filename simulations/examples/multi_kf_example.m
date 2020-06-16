clear, close all
%% System Model:
% This is a spinning body with a gyroscope and accelerometer measuring its
% angular velocity and encoders measuring its angular position.  We wish to
% get accurate measurements of the body's angular velocity + position.  It
% is accelerating with constant acceleration
% x = [theta omega]'

% Simulation Variables
dt = 0.0001;  %Time Step
T = 0.01;     %Controller Period of 100 Hz
t = 0:dt:4;   %4 seconds of simulation

x = zeros(2, length(t)); % motor state
y = zeros(1, length(t)); % measured output (speed)

xhat = [0; 0]; % estimated state
P = [100 100; 100 100]; %Uncertainty

% Continuous model
A = [0 1; 0 0];
B = [0; 1];

% Discrete model
A_d = [1 T; 0 1];
B_d = [0.5*T^2; T];

C1 = [1 0]; % encoder lets you measure angular position
C2 = [0 1]; % gyroscope lets you measure angular velocity
C3 = [0 1]; % centripetal acceleration is proportional to squared velocity

C = [C1;C2;C3];
l = 1;

R = 0.000000001*[1 0 0; 0 1 0; 0 0 1];
Q = 0*[1 0; 0 1];
K = zeros(2);

P = zeros(2);

Rvar = 0;
Pvar = 0.1;

for k=1:length(t)
    
    
   
    % Kalman Filter running at 100Hz
    if(abs(t(k)-l*T) < dt/2 || t(k) == 0)  %avoids numerical error in t
        u = sin(pi*t(k));
        
        %Predict
        xhat = A_d*xhat + B_d * u;
        P = A*P*A' + Q;
        
        %Measure
        z = [x(1,k); x(2,k); x(2,k)] + normrnd(0, Pvar, 3, 1); 
        
        %Update
        K = P*C'*inv(C*P*C' + R);
        xhat = xhat + K*(z - C*xhat);
        P = (eye(2)-K*C)*P;
        
        %measure output
        y(k) = xhat(1);
        l=l+1;
    else
        y(k) = y(k-1);
    end
    
    % Continuous Time variable updates
    x(:,k+1) = x(:,k) + dt*(A*x(:,k) + B*u) + normrnd(0, Rvar, 2, 1);
    

end

figure(1)
hold on
box on
plot(t, x(1,1:end-1), 'k-');
plot(t, y, 'b-')
title("Position");
legend("x", "y");

function y = clip(x, xmin, xmax)
    y = min(max(x,xmin), xmax);
end