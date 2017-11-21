function [displacement_bel, xpred, K] = KF(accel,z)
persistent xbel
persistent Pbel
persistent xmodel

if isempty(xbel)
    xbel = [0; 0];
end
if isempty(Pbel)
    Pbel = [1 0; 0 1];
end
if isempty(xmodel)
    xmodel = [0; 0];
end

%%%%%Kalman Filter Parameters
dt = 0.01;
%continuous SS
% A = [0 1; 0 0];
% B = [0; 1];
% C = [1 0];
% D = 0;
%Discrete SS
A = [0 1; -1 2];
B = [0; dt^2];
C = [1 0];
D = 0;

Q=[1 0; 0 1];
R = 100*dt^4;

%model only
xmodel = A*xmodel + B*accel;

%kalman filtered
x = A*xbel + B*accel;
P = A*Pbel*A' + Q;

% xbel = x;
K = P*C'*inv(C*P*C'+R);
%   K=[1.618; 1];
xbel = x-K*(C*x-z);
Pbel=(eye(2)-K*C)*P;

xpred = xmodel(1);
displacement_bel = C*xbel;
