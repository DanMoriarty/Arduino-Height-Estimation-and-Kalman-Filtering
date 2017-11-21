function [ Q ,w ] = getQ( RelHeightCalcM,~,accel,R)
%Requires data from part 1, using file Static.csv, stored into the workspace
%Also requires getR being called prior

%Q matrix captures state noise. This mostly manifests itself as low frequency
%in nature.
%Therefore, we can analyse outputs to capture a rough estimate for the covariance relating to
%low-frequency changes: those which are slowly-time varying such as drift or steady state offset.

%% METHOD
%Capture accelerometer and altimeter readings for an extended amount of time, but zero input
%(steady state) - Run Part1 with <filename =  'Static.csv'>
%Run getR to get the average high frequency covariance
%For the whole state dataset from start to finish, find the maximum and
%minimum recorded values (as well as the mean)
%Subtract the related R covariance from the max/min
%Get the absolute difference between the max/min and the mean
%square this value to get the covariance
%repeat this for every available state

%% Vertical Displacement
sum = 0;
max = 0;
min = 10000;
for i =1:length(RelHeightCalcM)
    
    %get mean
    sum = sum+RelHeightCalcM(i);
    
    %get max
    if RelHeightCalcM(i)>max
        max = RelHeightCalcM(i);
    end
    
    %get min
    if RelHeightCalcM(i)<min
        min = RelHeightCalcM(i);
    end
end
mean = sum/length(RelHeightCalcM);
max = max - R(1,1);
min = min + R(1,1);

dif = (abs(max)+abs(min))/2;
covar = abs(dif - mean);
w1 = covar;
w1 = 1;
q1 = covar^2;

%% Vertical Velocity
%can't reliably estimate velocity covariance from samples, also can't
%measure, so set arbitrarily
w2 = 0.001;
q2 = 0;

%% Vertical Acceleration
sum = 0;
for i =1:length(accel)
    
    %get mean
    sum = sum+accel(i);
    
    %get max
    if RelHeightCalcM(i)>max
        max = accel(i);
    end
    
    %get min
    if RelHeightCalcM(i)<min
        min = accel(i);
    end
end
mean = sum/length(accel);
max = max - R(1,1);
min = min + R(1,1);

dif = (abs(max)+abs(min))/2;
covar = abs(dif - mean);
w3 = covar;
q3 = covar^2;

%%
w = [w1; w2; w3];
Q = w*w'
%Q = [q1 0 0; q2 0 0; q3 0 0]


end

