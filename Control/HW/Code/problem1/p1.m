%% p1.m
clear;
close all;
clc;

global N T t dt;
global x0 y0 th0;
global xd yd thd;

N = 101;
T = 2*pi;
t = linspace(0, T, N);
dt = t(2) - t(1);

% desired trajectory:
xd = (2/pi)*t;
yd = linspace(0,0,N);
thd = linspace(pi/2,pi/2,N);

% initial conditions:
x0 = 0;
y0 = 0;
th0 = pi/2;

x(1) = x0;
y(1) = y0;
th(1) = th0;

% initial control inputs:
u1 = ones(1,N);
u2 = -0.5*ones(1,N);

for i = 1:N-1
        x(i+1) = x(i) + cos(th(i))*u1(i)*dt;
        y(i+1) = y(i) + sin(th(i))*u1(i)*dt;
        th(i+1) = th(i) + u2(i)*dt;
end

state_and_input0 = [x; y; th; u1; u2];

figure(1);
plot(x,y);
title('Initial trajectory');
xlabel('x');
ylabel('y');

% no linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];

% no bounds:
lb = [];
ub = [];

% Options for fmincon
options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
    'MaxFunEvals',100000, 'Display','iter',...
    'DiffMinChange',0.001,'Algorithm','sqp');
% Solve for the best simulation time + control input
optimal = fmincon(@p1_obj_fun, state_and_input0, A, b, Aeq, Beq, lb, ub, ...
    @p1_constraints, options);

x_opt = optimal(1,:);
y_opt = optimal(2,:);
th_opt = optimal(3,:);
u1_opt = optimal(4,:);
u2_opt = optimal(5,:);

figure(2);
plot(x_opt,y_opt);
title('Optimal trajectory');
xlabel('x');
ylabel('y');

figure(3);
plot(t,u1_opt,'b-',t,u2_opt,'r-')
title('Optimal control inputs');
xlabel('time (s)');
ylabel('control effort');