lines = dlmread('MatlabReadyData.txt',',')

straightPlot = figure
hold on
plot([0,0],[0,5])
axis([-0.5,0.5,0,6])
title('Team Swana Straight Line Testing 1400 ticks per second')
xlabel('y position (m)')
ylabel('x position (m)')
color = ['b','g','r','c','m'];
for group = 0:5:length(lines)-1
    for i = 1:5
        plot(lines(group+i,3)/100,lines(group+i,2),['x',color(i)])
    end
end

dists = ['5.0', '4.5' , '4.0', '3.5' ,'3.0' ,'2.5', '2.0', '1.5', '1.0', '0.5'];

endX = zeros(length(lines),1);
endY = zeros(length(lines),1);


%{
for dist =1:10
    for trial = 1:5
        trialData = dlmread(['T',str(trial),'_RunStraight_LeftWheel_100_Meters_']
    end
end
%}
hold off
figure
hold on
circleData = dlmread('../Log/circleTest.txt', ' ',1,0);

plot([-4*pi,4*pi],[-4*pi,4*pi])
axis([-4*pi,4*pi,-4*pi,4*pi])
for group = 0:5:length(circleData)-1
    for i = 1:5
        a = circleData(group+1,3);
        plot(a*circleData(group+i,2),a*circleData(group+i,7),['x',color(i)])
    end
end
title('Team Swana Rotational Characterization')
xlabel('Expected Theta value (radians)')
ylabel('Measured angle (radians)')
hold off