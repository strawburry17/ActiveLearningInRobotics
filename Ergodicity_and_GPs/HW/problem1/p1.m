%% p1.m
% based on iLQR code from HW 1 problem 4
clear;
close all;
clc;

%% Initialization:
N = 101; % number of timesteps
T = 10; % terminal time
t_init = linspace(0, T, N);

% distribution with respect to which the state trajectory should be ergodic
X = linspace(-5,5,101);
Y = linspace(-5,5,101);
for i=1:length(X)
    for j=1:length(Y)
        Z(i,j) = distro(X(i),Y(j));
        for k1 = 1:5
            for k2 = 1:5
                Zf(i,j) = Z(i,j)*fourierBasisPoint([X(i),Y(j)],[k1; k2],5,5);
            end
        end
    end
end
imagesc(X,Y,Zf);
set(gca,'YDir','normal')
hold on

% initial conditions and initial control:
x0 = [1; 1];
u0 = [-sin(t_init); cos(t_init)];

% generate initial trajectory using u0:
[t_init, x_init] = ode45(@(t,x) sys_ode(t, x, u0, t_init), t_init, x0);
t_init = t_init';
x_init = x_init';

% visualize desired trajectory and initial trajectory:
% figure
plot(x_init(1,:),x_init(2,:),'r+-')
plot(x0(1),x0(2),'ro','MarkerSize',10)
hold off
axis([-5 5 -5 5])
title('Initial trajectory under u0(t) = [1; -0.5]');
xlabel('x')
ylabel('y')

figure

% objective function parameters:
global q Q R K L1 L2;
L1 = 5;
L2 = 5;
K = 2; % number of terms in Fourier series
q = 10;
Q = diag([1,1]);
R = diag([1,1]);
J0 = obj_fun(t_init,x_init,u0,T);
fprintf('Initial cost: %5.3f.\n',J0);

% Armijo line search parameters:
alpha = 0.4;
beta = 0.7;
epsilon = 0.01;

i = 1;
imax = 20;
nmax = 100;

norm_zeta = 100*epsilon;

x_i = x_init;
u_i = u0;

%% Gradient descent:
figure
while ((abs(norm_zeta) > epsilon) && (i < imax))
    % compute descent direction
    [zeta, a_i, b_i] = descent_dir(t_init,x_i,u_i,T);
    z_i = zeta(1:2,:);
    v_i = zeta(3:4,:);
    
    plot(x_i(1,:),x_i(2,:));
    title('Optimal trajectory');
    xlabel('x')
    ylabel('y')
    drawnow;
    
    norm_zeta = DJ(t_init,x_i,u_i,z_i,v_i,a_i,b_i);
    fprintf('%d:\tDJ = %5.3f\n',i,norm_zeta);
    
    J_i = obj_fun(t_init,x_i,u_i,T);
    fprintf('%d:\tJ = %5.3f\n',i,J_i);
    
    % compute Armijo line search using computed descent direction:
    n = 0;
    gamma = beta^n;
    x_cand = x_i;
    u_cand = u_i;
    while ((obj_fun(t_init,x_cand,u_cand,T) > (J_i + alpha*gamma*norm_zeta)) && (n < nmax))
        u_cand = u_i + gamma*v_i;
        [t_init, x_cand] = ode45(@(t,x) sys_ode(t, x, u_cand, t_init), t_init, x0);
        x_cand = x_cand';
        n = n+1;
        gamma = beta^n;
        fprintf('%d:\t%5.3f\n',n,gamma);
    end
    fprintf('\tgamma = %5.3f\n',gamma);
    x_i = x_cand;
    u_i = u_cand;
    
    i = i+1; 
end

plot(x_i(1,:),x_i(2,:));
title('Optimal trajectory');
xlabel('x')
ylabel('y')
drawnow;

figure
plot(t_init,u_i)
title('Optimal control signals')
xlabel('time')
ylabel('u(t)')
legend('u1','u2','Location','Best')