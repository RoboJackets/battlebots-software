%% Timed On-Off Simulation
% This simulation records the motion of a differential drive robot with the
% timed on-off behavior.  For half of the period of revolution, one motor
% runs at full speed and the other runs at a reduced speed.  After half a
% revolution is complete, the motor that was originally running at full
% speed runs at the reduced speed, and vice versa.  The net effect is the
% robot moving along the x-axis (as shown in this simulation. 

tw = 8/12; % track width in ft

I = 0.044; % kg m^2

dt = 0.000001; % simulation timestep in seconds
time = 2
t = 0:dt:time; % time vector in seconds

x = zeros(3, length(t)); % x, y, theta

spin_speed = 3000; % rpm
% ss_rad = spin_speed * 0.104719755; % rad/s
ss_rad = 370; % Overridden with the value found in robot parameters spreadsheet

u0 = [-ss_rad*tw/2 ss_rad*tw/2]; % two wheel velocities
uv = zeros(length(t), 2);
adjf = 0.8; % Adjustment factor to determine power of motor during the "off" phase


for k = 1:length(t)-1
    %% Control Algorithm
    if(wrapAngle(x(3,k)) > pi/2 && wrapAngle(x(3, k)) < 3*pi/2)
        u = u0.* [1 adjf]; % Right wheel slowed from the 9 o clock to 3 o clock headings
    else
        u = u0.* [adjf 1]; % Left wheel slowed otherwise
    end
    linvel = (u(1) + u(2))/2; % Differential Drive Kinematics
    angvel = (u(2) - u(1))/tw;
    uv(k, :) = [u(1), u(2)];
    x(:, k+1) = x(:, k) + dt * [linvel * cos(x(3, k)); linvel * sin(x(3, k)); angvel]; % State Update
end
omega_avg = x(3,end)/time;  % Average angular speed calculated from ending angle and time
energy = 0.5 * I * omega_avg^2 % Average stored energy
speed = x(1, end)/2 % Movement speed calculated from final position in x-direction

subplot(1,3,1)
plot(t, x(1,:))
subplot(1, 3, 2)
plot(t, x(2,:))
subplot(1,3,3)
plot(t, wrapAngle(x(3,:)))


% wraps an angle between 0 and 2pi
function t = wrapAngle(angle)
t = angle;
while(t > 2*pi)
    t = t - 2*pi;
end
while(t < 0)
    t = t + 2*pi;
end
end