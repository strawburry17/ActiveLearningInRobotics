%% p2.m
clear;
close all;
clc;

%% Find the optimizer

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

%% Explore around the optimizer
% Now that we might be at the optimizer, let's look around a bit. Calculate
% the directional derivative of the objective function for 10 arbitray
% directions:
global Av Bv Cv Dv; % parameters for the control perturbation v
v = zeros(10,length(time)); % preallocate array
DJ = zeros(10,1); % preallocate array
for i = 1:10
    Av = 2*(0.5 - rand); % random amplitude btw -1 and 1
    Bv = rand; % random frequency
    Cv = wrapToPi(rand*2*pi); % random phase angle
    Dv = rand; % random DC bias
    
    v(i,:) = Av*sin(Bv*time + Cv) + Dv; % the control perturbation v
    direction{i} = sprintf('%5.3f sin(%5.3ft + %5.3f) + %5.3f',Av,Bv,Cv,Dv); % save string for later
    
    [t, z{i}] = ode45(@p2_pert_ode, time, [0 0]);
    
    z{i} = z{i}';
    
    DJ(i) = p2_dir_der_J(time,state,u,z{i},v(i,:));
end
figure(3);
plot(time,v);
title('Perturbations v to control signal u');
xlabel('time');
ylabel('control effort')

figure(4);
hold on
for i = 1:10
    plot(time,z{i});
end
hold off
title('Perturbations z to state trajectory x');
xlabel('time');
ylabel('z')

% Make a table of the directions and the values of the directional
% derivatives of the objective function:
tAbLe = table(direction',DJ);
tAbLe.Properties.VariableNames{1} = 'Control_perturbation_v';
tAbLe
