%% Acceleration Measurements alone - Continuous Time
clear all 
clc
%Dynamics
A = [0 1 0; 0 0 1; 0 0 0] %our discrete acceleration model

%Sensors
C1 = [0 0 0]; % displacement
%C2 = [0 1 0]; % velocity
C3 = [0 0 1]; % acceleration

C = double(or(C1,C3))

O = [C' (C*A)' (C*A*A)']'

ContinuousModelRank = rank(O)

% %% Acceleration Measurements alone - Discrete Time
% 
% A = [0 1 0; 0 0 1; 1 -3 3];
% 
% O = [C' (C*A)' (C*A*A)']'
% 
% DiscreteModelRank = rank(O)

%% Acceleration and Height Measurements
% clear all 
% clc
%Dynamics
A = [0 1 0; 0 0 1; 0 0 0] %our discrete acceleration model

%Sensors
C1 = [1 0 0]; % displacement
%C2 = [0 1 0]; % velocity
C3 = [0 0 1]; % acceleration

C = double(or(C1,C3))

O = [C' (C*A)' (C*A*A)']'

ContinuousModelRank = rank(O)

% %% Acceleration and Height Measurements - Discrete Time
% 
% dt = 1;
% m = 37 + 1.3 + 1.27;
% A =[0 1 0; 0 0 1; 1 -3 3];
% 
% O = [C' (C*A)' (C*A*A)']'
% 
% DiscreteModelRank = rank(O)

