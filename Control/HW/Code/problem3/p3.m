%% p3.m
% reference: 
% https://www.mathworks.com/matlabcentral/answers/94722-how-can-i-solve-the-matrix-riccati-differential-equation-within-matlab

clear;
close all;
clc;

%% Solve Riccati equation
T = 10; % terminal time

A = [0 1; -1.6 -0.4];
B = [0;1];

Q = [2 0; 0 0.01];
R = 0.1;
P1 = [1; 0; 0; 0.01]; % 2x2 reshaped as 4x1 for ode45

% a couple tricks:
% The Riccati equation in optimal control has a terminal condition P(T) =
% P1. To use ode45, modify the Riccati equation by negating the signs of
% each term and treatng P1 as an initial condition instead of a terminal
% condition.
[tP, P] = ode45(@(t,P) p3_riccati(t, P, A, B, Q, R), [T 0], P1);

%% Simulate system with optimal control defined by solution to Riccati equation
x0 = [10; 0]; % initial conditions
[t, x] = ode45(@(t,x) p3_sys_ode(t, x, tP, P, A, B, R), [0 T], x0);
% re-compute the optimal control u(t):
for i = 1:length(t)
    P11 = interp1(tP, P(:,1), t(i));
    P12 = interp1(tP, P(:,2), t(i));
    P21 = interp1(tP, P(:,3), t(i));
    P22 = interp1(tP, P(:,4), t(i));
    Pt = [P11, P12; P21, P22];
    
    u(i) = -inv(R)*B'*Pt*(x(i,:)');
end

%% Plot simulation results:
figure
plot(tP, P)
title('Solution to Riccati equation')
xlabel('time')
ylabel('P(t)')

figure
plot(t,x)
title('State trajectory')
xlabel('time')
ylabel('x(t)')

figure
plot(t,u)
title('Control signal')
xlabel('time')
ylabel('u(t)')