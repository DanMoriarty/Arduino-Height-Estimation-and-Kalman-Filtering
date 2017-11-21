%%Getting Covariance Estimates
clc
close all
clear all
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
sum = 0;
max = 0;
min = 10000;
for i=1:size(ApproxAlt,1)
    if ApproxAlt(i)>max
        max = ApproxAlt(i);
    end
    if ApproxAlt(i)<min
        min= ApproxAlt(i);
    end
    sum = sum+ApproxAlt(i);
end
avg = sum/size(ApproxAlt,1)
min
max

