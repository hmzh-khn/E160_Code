%% Analysis for tick rates at powers of 20, 40, 60
format long;

% 20%
P20 = [468 468 467.7 470.9 468.6];
T20 = [30102 30143 30061 30188 30045];

tpcm20 = mean(T20)/mean(P20); % ticks per centimeter
cmpt20 = mean(P20)/mean(T20); % centimeters per tick


% 40%
P40 = [491 489 487 488 490]; % T1 not had ticks 490
T40 = [30263 30215 30237 30191 30440];

tpcm40 = mean(T40)/mean(P40); % ticks per centimeter
cmpt40 = mean(P40)/mean(T40); % centimeters per tick


% 60%
P60 = [505.5 516.5 502 505 506];
T60 = [30161 30675 30084 30300 30321];


err = [std(P20./T20) ,std(P40./T40), std(P60./T60)]
tpcm60 = mean(T60)/mean(P60); % ticks per centimeter
cmpt60 = mean(P60)/mean(T60); % centimeters per tick

powers = [20,40,60];
cmpt = [cmpt20 cmpt40 cmpt60];


errorbar(powers, cmpt, err)
axis([15,65,0.015,0.017])
title('Team Swana cm per encoder tick as a function of percent power')
xlabel('Power level (%)')
ylabel('Cm per Encoder Tick')