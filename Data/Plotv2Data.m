M = readmatrix("DriftCalibratedSpinup2.txt");
close all;

s = 12.58e-3;
a1 = M(:, 1:3) * 9.8/100;
a2 = M(:, 4:6) * 9.8/100;
a3 = M(:, 7:9) * 9.8/100;
a4 = M(:, 10:12) * 9.8/100;

wavg_log = M(:, 13)/100;
position_log = M(:, 14)/100;

w1 = sqrt(abs((a1(:, 2) - a2(:, 2)))/s);
w2 = sqrt(abs((a4(:, 2) - a3(:, 2)))/s);

w3 = sqrt(abs((a4(:, 1) - a1(:, 1)))/s);
w4 = sqrt(abs((a3(:, 1) - a2(:, 1)))/s);


dt = 0.001; % 1ms

wavg = 0.25*(w1 + w2 + w3 + w4);
position = cumsum(wavg * 0.001);
position = wrapTo2Pi(position);

t = 0:dt:(length(position)-dt)*dt;
subplot(1,2,1);
plot(t, w1/(2*pi), '--');
hold on
plot(t, w2/(2*pi), '--');
plot(t, w3/(2*pi), '--');
plot(t, w4/(2*pi), '--');
plot(t, wavg/(2*pi));
legend("\omega_1","\omega_2","\omega_3","\omega_4","\omega_{avg}")
title("\omega (1/sec)");
subplot(1,2,2);
plot(t, position);
title("\theta (rad)");

figure()
subplot(1,2,1);
% wavg_log = (wavg_log - mean(wavg_log));
plot(t, (wavg_log));
subplot(1,2,2);
plot(t, position_log);
%hold on
%plot(t,wrapTo2Pi( cumsum(wavg_log)*0.001));
% w1filt = filter(Hd, w1);
% w2filt = filter(Hd, w2);
% w3filt = filter(Hd, w3);
% w4filt = filter(Hd, w4);
% wavgfilt = 0.25*(w1filt + w2filt + w3filt + w4filt);
% positionfilt = cumsum(wavgfilt * 0.001);
% positionfilt = wrapTo2Pi(positionfilt);
% 
% figure()
% subplot(1,2,1)
% plot(t, wavgfilt/(2*pi));
% subplot(1,2,2)
% plot(t, positionfilt);
