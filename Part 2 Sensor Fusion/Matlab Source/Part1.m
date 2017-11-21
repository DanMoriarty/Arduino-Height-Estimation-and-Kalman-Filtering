%%Part 1
clc
%clear all
%close all
%import accelerometer and altimeter data
%filename = 'WalkingSample.csv';
%filename = 'TestSample.csv';
%filename = 'WalkingSampleLong.csv';
%filename = 'WalkingSampleWithAccel.csv' %%%%%
filename = 'Static.csv'
data = csvread(filename,2);

%initial values
avgsample = 10;
accel0 = sum(data(1:avgsample,1))/avgsample; 
temp0 = sum(data(1:avgsample,2))/avgsample; 
pressure0  = sum(data(1:avgsample,3)/avgsample);
%...relative pressure 4
approxAlt0 = sum(data(1:avgsample,5)/avgsample);
%...Relative alt 6
calcHeight0 = sum(data(1:avgsample,7)/avgsample);
%...relative height 8


%data
start = 5;
accel = data(start:end,1);
Temp = data(start:end,2);
Pressure = data(start:end,3);
calcHeight = data(start:end,7); %this is relative in the formula though not on the input
ApproxAlt = data(start:end,5);

dt= 0.1;
time = zeros(size(data,1),1);
time(1) = dt;
for i=2:size(data,1)
    time(i) = i*dt;
end
%%

RelativePressure = zeros(size(Pressure,1),1);
p0 = pressure0;

for i=1:size(Pressure,1)
    RelativePressure(i) = Pressure(i)-p0;
end

%% Calculate Height using formula
height0 = 0;
HeightCalcM = zeros(size(Pressure,1),1);

for i=1:size(Pressure,1)
    HeightCalcM(i)=getHeight(Pressure(i),p0);
end

for i=1:avgsample
    height0 = height0 + getHeight(Pressure(i),p0);
end
height0 = height0/avgsample;

RelHeightCalcM = zeros(size(Pressure,1),1);
for i=1:size(Pressure,1)
    RelHeightCalcM(i) = HeightCalcM(i)-height0;
end

%%
subplot(2,2,1), plot(time(1:size(ApproxAlt,1)),ApproxAlt(1:end))
title('Altitude')
xlabel('seconds')
ylabel('m')
subplot(2,2,2), plot(time(1:size(Pressure,1)),Pressure(1:end))
title('Pressure')
xlabel('seconds')
ylabel('Pa')
subplot(2,2,3), plot(time(1:size(Temp,1)),Temp(1:end))
title('Temperature')
xlabel('seconds')
ylabel('*C')
subplot(2,2,4), plot(time(1:size(HeightCalcM,1)),HeightCalcM(1:end))
title('Calculated Height')
xlabel('seconds')
ylabel('m')

%subplot(2,2,4), plot(time(1:size(calcHeight,1)),calcHeight(1:end))
%title('Calculated Height')

%% Calibrated Acceleration
CalAccelM = zeros(size(accel,1),1);
for i=1:size(CalAccelM,1)
    %CalAccelM(i) = accel(i)-accel0-0.1;
    CalAccelM(i) = (1/70)*(accel(i) -284 - 0.5*(422-284))-1;
end

%% Discrete Integration of Accelerometer Data
velocity = zeros(size(CalAccelM,1),1);
for i = 2:size(CalAccelM,1)
    d = CalAccelM(i)-CalAccelM(i-1);
    velocity(i) = velocity(i-1) + CalAccelM(i)*dt+0.5*dt*d; %trapezoidal
end
velocity(1)=[];

displacement= zeros(size(velocity,1),1);
for i = 2:size(velocity,1)
    d = velocity(i)-velocity(i-1);
    displacement(i) = displacement(i-1) + velocity(i)*dt+0.5*dt*d; %backwards piecewise
end
displacement(1)=[];

figure,
subplot(1,3,1), plot(time(1:size(CalAccelM,1)),CalAccelM(1:end))
title('Acceleration')
xlabel('seconds')
ylabel('m/s^2')
subplot(1,3,2), plot(time(1:size(velocity,1)),velocity(1:end))
title('Velocity')
xlabel('seconds')
ylabel('m/s')
subplot(1,3,3), plot(time(1:size(displacement,1)),displacement(1:end))
title('Displacement')
xlabel('seconds')
ylabel('m')

%% Discrete Differentiation of Altimeter Data
velocitydif= zeros(size(HeightCalcM,1),1);
for i = 2:size(HeightCalcM,1)
    velocitydif(i) = (HeightCalcM(i)-HeightCalcM(i-1))/dt;
end
displacement(1)=[];

figure,
subplot(1,2,1), plot(time(1:size(HeightCalcM,1)),HeightCalcM(1:end))
title('Displacement')
xlabel('seconds')
ylabel('m/s')
subplot(1,2,2), plot(time(1:size(velocitydif,1)),velocitydif(1:end))
title('Velocity')
xlabel('seconds')
ylabel('m')

%% DATA

%%%%% From Altimeter
%RAW
Temp; %Temperature - output 2
Pressure; %Pressure - output 3
ApproxAlt; %Approximate Altitude - output 5

%CALCULATED ONLINE
%...relative pressure 4 - output 4
%...Relative alt - output 6
calcHeight; %Calculated Height - output 7
%...relative height - output 8

%CALCULATED OFFLINE
RelativePressure; %Relative Pressure
HeightCalcM; %Calculated Height
RelHeightCalcM; %Relative Calculated Height
velocitydif; %velocity -> differentiate RelHeightCalcM

%%%%% From Accelerometer
%RAW
accel; %Acceleration - output 1
%CALCULATED ONLINE
%CALCULATED OFFLINE
CalAccelM; %Calibrated (zero mean) acceleration
velocity; %Velocity -> integrate acceleration
displacement; %Displacement -> integrate velocity

