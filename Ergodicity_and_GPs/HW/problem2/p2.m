clear all;
close all;
clc;

%% Generate training dataset
global n N
n = 25;
n_sims = 10;
% N = n*2; % number of timesteps
T = 100; % terminal time
t_init = linspace(0, T, n);


global xdata ydata
fprintf("generating training data...\n");
tic;
for i = 1:n_sims
    % initial conditions and initial control:
%     x0 = [5*(rand); 10*(rand-0.5); pi*(rand - 0.5)];
    IC = [0; 0; pi/2];
%     u0 = [0.1.*(rand([1,n])-0.5) + sin(t_init); 0.1.*(rand([1,n])-0.5) - cos(t_init)];
%     u0 = [linspace(1,1,N); linspace(-0.5, -0.5, N)];
    u0 = [2*(rand-0.5).*sin(t_init) + 2*(rand-0.5).*cos(t_init) ...
        + (rand-0.5).*sin(5*t_init) + (rand-0.5).*cos(5*t_init) ...
        + 0.5.*(-0.5 + rand([1,n])); ...
        2*(rand-0.5).*sin(t_init) + 2*(rand-0.5).*cos(t_init) ...
        + (rand-0.5).*sin(5*t_init) + (rand-0.5).*cos(5*t_init) ...
        + 0.5.*(-0.5 + rand([1,n]))];
%     u0 = [(1 - 0.1*(rand-0.5)).*sin(t_init) + (1 - 0.1*(rand-0.5)).*cos(t_init) ...
%         + (1 - 0.1*(rand-0.5)).*sin(5*t_init) + (1 - 0.1*(rand-0.5)).*cos(5*t_init); ...
%         -(1 - 0.1*(rand-0.5)).*sin(t_init) - (1 - 0.1*(rand-0.5)).*cos(t_init) ...
%         - (1 - 0.1*(rand-0.5)).*sin(5*t_init) - (1 - 0.1*(rand-0.5)).*cos(5*t_init)];
%     u0 = [2.*(-0.5 + rand([1,n])); 2.*(-0.5 + rand([1,n]))];

    % generate initial trajectory using u0:
    [t_init, x_init] = ode45(@(t,x) kin_car_ode(t, x, u0, t_init), t_init, IC);
    t_init = t_init';
    x_init = x_init';
    
%     figure
%     plot(x_init(1,:),x_init(2,:))
%     title(['Training trajectory #',num2str(i),'.']);
%     xlabel('x')
%     ylabel('y')
    
    xdata_sim_i = [x_init', u0'];
    
%     ydata_sim_i = kin_car_ode(t_init,x_init,u0,t_init)';

    for j = 1:n
        ydata_sim_i(j,:) = kin_car_ode(t_init(j),x_init(:,j),u0,t_init);
    end
    
    xdata = [xdata; xdata_sim_i];
    ydata = [ydata; ydata_sim_i];
end
toc;
fprintf("generating training data...done.\n");
N = n_sims*n;

%% Calculate covariance matrix for GP
global sigma
sigma = 0.5; % assumed Gaussian noise

global K
fprintf("computing K...\n");
tic;
for i = 1:N
    for j = 1:N
        K(i,j) = radBasFcn(xdata(i,:),xdata(j,:));
    end
end
toc;
fprintf("computing K...done.\n");

%% Trajectory optimization using the GP
T_test = 2*pi;
t_test = linspace(0,T_test,n);
global dt
dt = t_test(2) - t_test(1);

% desired trajectory:
u_nom = [1.25*sin(t_test); -1.25*cos(t_test)];
[t_test, x_ref] = ode45(@(t,x) kin_car_ode(t, x, u_nom, t_test), t_test, IC);
t_test = t_test';
x_ref = x_ref';

global xd yd thd;
xd = x_ref(1,:);
yd = x_ref(2,:);
thd = x_ref(3,:);

% xd = (2/pi)*t_test;
% yd = linspace(0,0,n);
% thd = linspace(pi/2,pi/2,n);

figure
plot(xd,yd,'k-')
title('Desired trajectory')

global x0 y0 th0;

% initial conditions:
x0 = IC(1); y0 = IC(2); th0 = IC(3);

xGP(1) = x0;
yGP(1) = y0;
thGP(1) = th0;

% initial control inputs:
u1 = ones(1,n);
u2 = -0.5*ones(1,n);

for i = 1:n-1
        xGP(i+1) = xGP(i) + cos(thGP(i))*u1(i)*dt;
        yGP(i+1) = yGP(i) + sin(thGP(i))*u1(i)*dt;
        thGP(i+1) = thGP(i) + u2(i)*dt;
end

state_and_input0 = [xGP; yGP; thGP; u1; u2];

figure(1);
plot(xGP,yGP);
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
options = optimoptions(@fmincon, 'TolFun', 0.001, 'MaxIter', 100, ...
    'MaxFunEvals',100000, 'Display','iter',...
    'DiffMinChange',0.001,'Algorithm','sqp');
tic;
% Solve for the best simulation time + control input
optimal = fmincon(@p2_obj_fun, state_and_input0, A, b, Aeq, Beq, lb, ub, ...
    @p2_constraints, options);

fprintf('Optimization completed in %d seconds.\n',toc);

x_opt = optimal(1,:);
y_opt = optimal(2,:);
th_opt = optimal(3,:);
u1_opt = optimal(4,:);
u2_opt = optimal(5,:);

figure;
plot(x_opt,y_opt);
title('Optimal trajectory based on GP');
xlabel('x');
ylabel('y');

figure;
plot(t_test,u1_opt,'b-',t_test,u2_opt,'r-')
title('Optimal control inputs');
xlabel('time (s)');
ylabel('control effort');

%% Test the GP control signal on the real dynamics
u_gp_test = [sin(t_test); -cos(t_test)];
% u_gp_test = [0.75.*sin(t_init); 0.75.*sin(t_init)];
fprintf("testing the GP...\n");
% tic;
% [t_gp_test, x_gp_test] = ode45(@(t,x) gp_ode(t, x, u_gp_test, t_test), t_test, IC);
% toc;
% fprintf("testing the GP...done.\n");
% t_gp_test = t_gp_test';
% x_gp_test = x_gp_test';

[t_gp_test_ref, x_gp_test_ref] = ode45(@(t,x) kin_car_ode(t, x, [u1_opt;u2_opt], t_test), t_test, IC);
t_gp_test_ref = t_gp_test_ref';
x_gp_test_ref = x_gp_test_ref';

% visualize desired trajectory and initial trajectory:
figure
plot(x_opt,y_opt,'r-');
hold on
plot(x_gp_test_ref(1,:),x_gp_test_ref(2,:),'b:')
hold off
legend('GP','actual','Location','Best');
title('Comparison: optimal trajectory based on GP vs actual performance');
xlabel('x')
ylabel('y')

%% visualize
% plot(xdata,ydata,'k-')
% hold on
% plot(xtest,f_pred,'r*')
% errorbar(xtest,f_pred,v_pred);
% plot(linspace(-5,5,101),sin(linspace(-5,5,101)),'r:')
% hold off
