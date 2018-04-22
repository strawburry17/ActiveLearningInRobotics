%% p2.m
clear;
close all;
clc;

global A B Q R P1;
global x0;
global T N t;

A = [0 1; -1.6 -0.4];
B = [0;1];

Q = [2 0; 0 0.1];
R = 0.1;
P1 = [1 0; 0 0.1];

T = 10;
N = 101;
t = linspace(0,T,N);

x0 = [10; 0]; % initial conditions

solinit = bvpinit(t,@p2_tpbvp_init); % initial guess at bvp solution
sol = bvp4c(@p2_tpbvp, @p2_tpbvp_bc, solinit);

% unpack the "sol" struct returned by bvp4c:
time = sol.x;
state = sol.y(1:2,:);
adjoint = sol.y(3:4,:);

% calculate the control signal from the adjoint:
u = -R\(B')*adjoint;

% make plots:
figure(1);
plot(time,state(1,:),'b-',time,state(2,:),'r-');
title('Optimal state trajectory');
xlabel('time');
ylabel('state');

figure(2);
plot(time,u);
title('Optimal control signal');
xlabel('time');
ylabel('control effort');