%% Part 2
clc
close all
%%
Part1
%% Parameters
dt = 0.1; % sample time as collected from arduino delay
m = 37 + 1.3 + 1.27; % weight of board, not including cable


%% Establishing Continuous Simple State Space Representation
%x1 = displacement
%x2 = velocity
%x3 = acceleration

%u1 = controls
%u2 = input noise channel
%u3 = measurement noise channel

%Dynamics
A = [0 1 0; 0 0 1; 0 0 0]; %our acceleration model

%Inputs wrt dynamics
b1 = 0; %input noise on displacement state
b2 = 0; %input noise on velocity state
b3 = 0.3; %input noise on acceleration state
Bcontrol = [0 0 1/m]'; %control input
Binputnoise = [b3 b2 b3]'; %input noise
Boutputnoise = [0 0 0]'; % output noise effect on states

%Outputs
C1 = [1 0 0]; % displacement
%C2 = [0 1 0]; % velocity
C3 = [0 0 1]; % acceleration

%Inputs wrt outputs
d1 = 0.01; %measurement noise on displacement readings
%d2 =  %measurement noise on velocity readings
d3 = 0.1;%measurement noise on acceleration readings
Dcontrols = [0; 0];
Dinputnoise = [0; 0];
Doutputnoise = [d1;d3];


B = [Bcontrol Binputnoise Boutputnoise];
C = [C1;C3];
D = [Dcontrols Dinputnoise Doutputnoise];

% %continuous state space model
% accelss1 = ss(A,B,C,D)
% 
% %Naming the channels
% accelss1.InputGroup.controls = [1];
% accelss1.InputGroup.noise = [2];
% accelss1.OutputGroup.displacement = [1];
% accelss1.OutputGroup.acceleration = [2]

%% Discretized State Space
%Using euler approximation of derivatives, because Euler

A = dt^3/m*[0 1 0; 0 0 1; 1 -3 3];
Bcontroldd = dt^3/m*[0; 0; 1];

B = [Bcontroldd Binputnoise Boutputnoise];

%% Get Q and R matrices
% [R, v] = getR(RelHeightCalcM,'',accel);
% [Q, w] = getQ(RelHeightCalcM,'',accel,R);
[R, v] = getR(RelHeightCalcM,'',CalAccelM);
[Q, w] = getQ(RelHeightCalcM,'',CalAccelM,R);
%% Get Correlation
NN = w*v';
%% Initial Conditions
X0 = [0; 0; 0];
P0 = zeros(3);

%% Full State Space Construction

%Discrete state space model
accelss2 = ss(A,B,C,D,dt)

%Naming the channels
accelss2.InputGroup.controls = [1];
accelss2.InputGroup.noise = [2];
accelss2.OutputGroup.displacement = [1];
accelss2.OutputGroup.acceleration = [2]
%% Kalman Filtering
%Using Kalman Filter Function:

%[KEST,L,P] = kalman(accelss2,eye(3),eye(2),NN)
[KEST,L,P] = kalman(accelss2,Q,R,NN)
%[KEST,L,P] = kalman(accelss2,Q,R,NN,[1; 1; 1],[1;1])

%% Estimation using Kalman Filter
figure, hold on
plot(time(1:length(RelHeightCalcM)),RelHeightCalcM);
estHeight = zeros(length(RelHeightCalcM),1);
x = [0 0 0]';
u = 0;
for i=1:length(RelHeightCalcM)
    %x = x+KEST(1,:).A*x + KEST(1,:).B(1:end,1)*u
    %estHeight(i) = KEST(1,:).C*x
    x = x-L(1:end,1)*(accelss2.C(1,1:end)*x-RelHeightCalcM(i));
    estHeight(i) = accelss2.C(1,1:end)*x;
end
plot(time(1:length(estHeight),1),estHeight)
legend 'Height' 'Estimated Height'
title('Kalman Filtered Height')
xlabel('seconds')
ylabel('m')
%%
%% Estimation using Kalman Filter
figure, hold on
plot(time(1:length(CalAccelM)),CalAccelM);
estAccel = zeros(length(CalAccelM),1);
x = [0 0 0]';
u = 0;
for i=1:length(estAccel)
    %x = x+KEST(1,:).A*x + KEST(1,:).B(1:end,1)*u
    %estHeight(i) = KEST(1,:).C*x
    x = x-L(1:end,2)*(accelss2.C(2,1:end)*x-CalAccelM(i));
    estAccel(i) = accelss2.C(2,1:end)*x;
end
plot(time(1:length(estAccel),1),estAccel)
legend 'Acceleration' 'Estimated Acceleration'
title('Kalman Filtered Acceleration')
xlabel('seconds')
ylabel('m/s^2')
%%
% %% Kalman Filtering
% %Using Kalman Filter Function:
% Q = [w(1)^2 0 0; 0 w(2)^2 0; 0 0 w(3)^2];
% R = [v(1)^2 0; 0 v(2)^2];
% [KEST,L,P] = kalman(accelss2,Q,R)
% 
% %% Estimation using Kalman Filter
% figure, hold on
% plot(time(1:length(RelHeightCalcM)),RelHeightCalcM);
% estHeight = zeros(length(RelHeightCalcM),1);
% x = [0 0 0]';
% u = 0;
% for i=1:length(RelHeightCalcM)
%     %x = x+KEST(1,:).A*x + KEST(1,:).B(1:end,1)*u
%     %estHeight(i) = KEST(1,:).C*x
%     x = x-L(1:end,1)*(accelss2.C(1,1:end)*x-RelHeightCalcM(i));
%     estHeight(i) = accelss2.C(1,1:end)*x;
% end
% plot(time(1:length(estHeight),1),estHeight)
