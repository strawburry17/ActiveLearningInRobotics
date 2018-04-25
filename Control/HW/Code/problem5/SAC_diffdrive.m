%% SAC_diffdrive.m
% main program
% uses Sequential Action Control (SAC) to parallel-park a diff-drive
% kinematic car.
clear;
close all;
clc;

%% Initialization

% Time parameters
Tf = 2*pi;
Th = 2.0; % predictive time horizon is 1 second long
% Th = Tf; % comment in for debugging
N = 1001;

% System parameters
x0 = [0;0;pi/2];

% Desired trajectory
t_init = linspace(0,Tf,N);
global xd;
xd = [linspace(0,4,N); linspace(0,0,N); linspace(pi/2, pi/2,N)];
figure
plot(xd(1,:),xd(2,:))
title('Desired Trajectory')
xlabel('x')
ylabel('y')

% Control parameters
dt_init = 0.1; % default control duration (s)
u1t = [linspace(1,1,N); linspace(-0.5,-0.5,N)];
u1 = u1t(:,1);
figure
plot(t_init,u1t)
title('Nominal control')
xlabel('time (s)')
ylabel('u(t)')
legend('u1','u2','Location','Best')

% Optimization parameters
global Q R;
global beta;
Q = diag([1,1,1]);
R = diag([1,1]);
alpha_d_init = -1000; % desired sensitivity, value from SAC paper by Ansari & Murphey
gamma = -10; % gain for adjusting alpha_d online
beta = 1.0;

%% Main loop:
t_curr = 0;
% while (t_curr < Tf)
   t0 = t_curr;
   tf = t_curr + Th; % look 1 second into the future
   
%    x_init = x(t0);
   [tx, x] = ode45(@(t,x) p5_sys_ode(t, x, u1t, t_init), [t0 tf], x0);
   x = x';
   
   rho0 = zeros(3,1);
   [trho, rho] = ode45(@(t,rho) p5_adjoint_ode(t,rho,x,u1t,tx,t_init), flip(tx), rho0);
   trho = flip(trho);
   rho = flip(rho)';
   
   J1_init = p5_J1(tx,x);
   alpha_d = gamma*J1_init;
%    alpha_d = alpha_d_init; % comment in for debugging
   
   % compute u2star from (x,rho) using Thm 1:
   for i = 1:length(tx)
        u1i = [interp1(t_init,u1t(1,:),tx(i)); interp1(t_init,u1t(2,:),tx(i))];
        theta = x(3,i);
        h = [cos(theta),0;
             sin(theta),0;
             0,         1];
        Lambda =  h'*rho(:,i)*rho(:,i)'*h;
        u2star(:,i) = inv(Lambda + R')*(Lambda*u1i + h'*rho(:,i)*alpha_d);
   end
   
   % specify/search for time tau > t0 to apply u2star:
   % use fmincon to do unconstrained single-variable optimization of
   % another objective function Jtau:
   tau = fmincon(@(t) p5_Jtau(u1,u2star,t,t0,x,tx,rho,trho), t0,[],[],[],[]);
   
   % line search for control duration:
   k = 0;
   J1_new = Inf;
   kmax = 20;
%    while(() && (k < kmax))
%        lambda_i = (omega^k)*dt_init;
%        k = k+1;
%    end
% end