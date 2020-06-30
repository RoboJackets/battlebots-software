tw = 8/12; % track width in ft

I = 0.044; % kg m^2

dt = 0.000001; % seconds
t = 0:dt:2; % time vector in seconds

x = zeros(3, length(t)); % x, y, theta

spin_speed = 3000; % rpm
% ss_rad = spin_speed * 0.104719755; % rad/s
ss_rad = 370;

u0 = [-ss_rad*tw/2 ss_rad*tw/2]; % two wheel velocities
uv = zeros(length(t), 2);
adjf = 0.8;


for k = 1:length(t)-1
    %% Control Algorithm
    if(wrapAngle(x(3,k)) > pi/2 && wrapAngle(x(3, k)) < 3*pi/2)
        u = u0.* [1 adjf];
    else
        u = u0.* [adjf 1];
    end
    linvel = (u(1) + u(2))/2; % Differential Drive Kinematics
    angvel = (u(2) - u(1))/tw;
    uv(k, :) = [u(1), u(2)];
    x(:, k+1) = x(:, k) + dt * [linvel * cos(x(3, k)); linvel * sin(x(3, k)); angvel]; 
end
omega_avg = x(3,end)/2;
energy = 0.5 * I * omega_avg^2
speed = x(1, end)/2

subplot(1,3,1)
plot(t, x(1,:))
subplot(1, 3, 2)
plot(t, x(2,:))
subplot(1,3,3)
plot(t, wrapAngle(x(3,:)))


function t = wrapAngle(angle)
t = angle;
while(t > 2*pi)
    t = t - 2*pi;
end
while(t < 0)
    t = t + 2*pi;
end
end