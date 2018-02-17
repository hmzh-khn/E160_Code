%% Analysis for tick rates at various tick rates (100, )
format long;

% T = 100
dists100 = [4.735, 4.770, 4.790, 4.765, 4.780]*100; % meters to centimeters
ticks100 = [30098, 30163, 30406, 30336, 30126];
cmpt100 = mean(dists100)/mean(ticks100)
