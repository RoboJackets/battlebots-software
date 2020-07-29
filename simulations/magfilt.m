clear;
close all;

%% Params

% filter params %
fs = 1.6e3;
fc = 60;
fstop = 100;
fnorm = [0 fc/(fs/2) fstop/(fs/2) 1];
amps = [1 1 0 0];
order = 10;
taps = order + 1;

% signal params %
Lsig = 500;
fsig = 55; % sig we want
fbad = 100; % sig we don't want or something
phsig = 0;
phbadA = 45;
phbadB = 45;
sig_snr = 10;
bad_snr = 20;

%% Generate Signals "A"

tt = 0:1/fs:(Lsig-1)/fs;
nnA = randn(Lsig, 1);
sigA = sqrt((10^(sig_snr / 10) * var(nnA))) * sin(2*pi*fsig*tt + pi/180*phsig).';
badA = sqrt((10^(bad_snr / 10) * var(nnA))) * sin(2*pi*fbad*tt + pi/180*phbadA).';
xxA = sigA + badA + nnA;

nnB = randn(Lsig, 1);
sigB = sqrt((10^(sig_snr / 10) * var(nnB))) * sin(2*pi*fsig*tt + pi/180*phsig + pi/2).';
badB = sqrt((10^(bad_snr / 10) * var(nnB))) * sin(2*pi*fbad*tt + pi/180*phbadB).';
xxB = sigB + badB + nnB;

%% Design Filter
[b, a] = butter(order, fc ./ (fs/2), 'low');
% b = firpm(order, fnorm, amps);
% a = 1;
figure;
freqz(b, a);

%% Apply Filter in Various Ways

figure;

subplot(5, 1, 1);
plot(tt, xxA, 'LineWidth', 2);
hold on
plot(tt, xxB, 'LineWidth', 2);
hold off
legend("A", "B");
title("Input Signal");

subplot(5, 1, 2);
plot(tt, sigA, 'LineWidth', 2);
hold on
plot(tt, sigB, 'LineWidth', 2);
hold off
legend("A", "B");
title("Desired Output Signal");

subplot(5, 1, 3);
yy_filtA = filter(b, a, xxA);
yy_filtB = filter(b, a, xxB);
plot(tt, yy_filtA, 'LineWidth', 2);
hold on
plot(tt, yy_filtB, 'LineWidth', 2);
hold off
legend("A", "B");
title("Signal Obtained With 'Filter'");

subplot(5, 1, 4);
yy_filtfiltA = filtfilt(b, a, xxA);
yy_filtfiltB = filtfilt(b, a, xxB);
plot(tt, yy_filtfiltA, 'LineWidth', 2);
hold on
plot(tt, yy_filtfiltB, 'LineWidth', 2);
hold off
legend("A", "B");
title("Signal Obtained With 'FiltFilt'");

subplot(5, 1, 5);
yy_sketchyfiltfiltA = filter(b, a, flipud(filter(b, a, flipud(xxA))));
yy_sketchyfiltfiltB = filter(b, a, flipud(filter(b, a, flipud(xxB))));
plot(tt, yy_sketchyfiltfiltA, 'LineWidth', 2);
hold on
plot(tt, yy_sketchyfiltfiltB, 'LineWidth', 2);
hold off
legend("A", "B");
title("Signal Obtained With FiltFilt made with 'Filter'");

figure;
plot(atan2(sigB, sigA));
hold on
% plot(atan2(yy_sketchyfiltfiltB, yy_sketchyfiltfiltA), 'LineWidth', 2);
plot(atan2(yy_filtfiltB, yy_filtfiltA));
% plot(atan2(yy_filtB, yy_filtA));
hold off
