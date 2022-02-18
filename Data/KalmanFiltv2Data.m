M = readmatrix("Timer9(2_13_22).txt");

s = 12.58e-3;
a1 = M(:, 1:3) * 9.8/100;
a2 = M(:, 4:6) * 9.8/100;
a3 = M(:, 7:9) * 9.8/100;
a4 = M(:, 10:12) * 9.8/100;

w1 = sqrt(abs((a1(:, 2) - a2(:, 2)))/s);
w2 = sqrt(abs((a4(:, 2) - a3(:, 2)))/s);

w3 = sqrt(abs((a4(:, 1) - a1(:, 1)))/s);
w4 = sqrt(abs((a3(:, 1) - a2(:, 1)))/s);

