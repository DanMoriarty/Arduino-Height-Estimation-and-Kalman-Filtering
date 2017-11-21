function [ R, v ] = getR(RelHeightCalcM,~,accel)
%Requires data from part 1, using file Static.csv, stored into the workspace

%R matrix captures measurement noise. This is almost always high frequency
%in nature.
%Therefore, we can analyse outputs to capture a rough estimate for the covariance relating to
%high-frequency changes: those which quickly-time varying.

%% METHOD
%Capture accelerometer and altimeter readings for an extended amount of time, but zero input
%(steady state) - Run Part1 with <filename =  'Static.csv'>
%Take a number of samples for very short time intervals
%For each sample: 
%-calculate the mean
%-Then, calculate the magnitude of average deviation
%Average this value across all samples, square it for the covariance
%Do this for every state to get every diagonal element of the R matrix

%% Time Sampling
%4 time samples arbitrarily chosen:
T = {};
SampleLength = 20;

%select start times arbitrarily - choose times where the curve is
%relatively flat, and so should have a fairly accurate mean
S = [1, 120, 260, 430];

%% Vertical Displacement - Sensor 1
for i =1:length(S)
    T{1,i} = RelHeightCalcM(S(i):S(i)+SampleLength-1,1);
end
T
sum = 0;
TotalCovarSum=0;
for i =1:size(T,2)
    %for each time sample:
    
    %get mean
    for j = 1:size(T{i})
        sum = sum+T{i}(j);
    end
    mean = sum/size(T{i},1);
    
    %Get average covariance
    covarSum = 0;
    for j = 1:size(T{i})
        covarSum=covarSum+abs(T{i}(j)-mean);
    end
    covarAvg = covarSum/size(T{i},1);
    
    %store in total averages for every sample
    TotalCovarSum = TotalCovarSum + covarAvg;
end
TotalAvgCovar = TotalCovarSum/i;

v1 = TotalAvgCovar
r1 = TotalAvgCovar^2

%% Vertical Acceleration - Sensor 2
for i =1:length(T)
    T{i} = accel(S(i):S(i)+SampleLength-1,1);
end

sum = 0;
TotalCovarSum=0;
for i =1:length(T)
    %for each time sample:
    
    %get mean
    for j = 1:size(T{i})
        sum = sum+T{i}(j);
    end
    mean = sum/length(T{i});
    
    %Get average covariance
    covarSum = 0;
    for j = 1:size(T{i})
        covarSum=covarSum+abs(T{i}(j)-mean);
    end
    covarAvg = covarSum/length(T{i});
    
    %store in total averages for every sample
    TotalCovarSum = TotalCovarSum + covarAvg;
end
TotalCovarSum
TotalAvgCovar = TotalCovarSum/i;

v3 = TotalAvgCovar
r3 = TotalAvgCovar^2
%%
v = [v1; v3];
%R = v*v'
R = [r1 0 ; 0 r3]

end

