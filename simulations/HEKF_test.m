dt = 1e-6;
alpha = 2;
beta = 4;
accDists = [.1 ; .1 ; .1 ; .1 ; .1 ; .1];
wheelR = 1.5;
botR = 5.5;
imus = 6;
mag_dir = 0;

filter = MeltyBrain_HEKF(dt, alpha, beta, accDists, wheelR, botR, imus, mag_dir);

meas = [2 ; 2 ; 2 ; 2 ; 2 ; 2];
for i = 1 : 10
    filter.update(meas, 6, i / 10, 'acc', 0);
    disp(filter.x);
end