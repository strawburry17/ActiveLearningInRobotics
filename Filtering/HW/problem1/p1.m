clear;
close all;
clc;

%% Particle filter for the vehicle model in Homework 1
dt = 0.1; % timestep size
T = 6; % time horizon
t = 0:dt:T; % time vector

proc_noise_var = 0.05; % process noise variance
meas_noise_var = 0.25; % measurement noise variance
Q = proc_noise_var*eye(3); % covariance of process noise (matrix)
R = meas_noise_var*eye(3); % covariance of measurement noise (matrix)

u1 = 1; % control input 1: forward velocity
u2 = -0.5; % control input 2: rotational velocity

dxdt = @(t,x) [cos(x(3))*u1; sin(x(3))*u1; u2];

x0 = [0; 0; pi/2];

[t, x] = ode45(dxdt,t,x0);
t = t';
x = x';

% generate particles:
M = 1000; % sure, why not
xP = zeros(length(x0),M);
for m = 1:M
   xP(:,m) =  x0 + [normrnd(0,meas_noise_var); ...
    normrnd(0,meas_noise_var); normrnd(0,meas_noise_var)]; % initial measurement
end

figure
plot(x(1,:),x(2,:),'k')
hold on
% scatter(xP(1,:),xP(2,:),'r.')
xlabel('x')
ylabel('y')
axis([-1 5 -1 3]);

xprev = x0;
% xP_prev = xP;

for i = 1:length(t)
    
    % update state and observed state (Euler integration):
    xi = x(:,i);
%     x = xprev + [cos(xprev(3))*u1;
%                  sin(xprev(3))*u1;
%                  u2].*dt + ...
%                  [normrnd(0,proc_noise_var);
%                   normrnd(0,proc_noise_var);
%                   normrnd(0,proc_noise_var)];     
    
    % predict:
    for m = 1:M
       xP_pred(:,m) = xP(:,m) + [cos(xP(3,m))*u1;
                                 sin(xP(3,m))*u1;
                                 u2].*dt + ...
                             [normrnd(0,proc_noise_var);
                              normrnd(0,proc_noise_var);
                              normrnd(0,proc_noise_var)];
    end
    
    % update:
    zi = xi + [normrnd(0,meas_noise_var);
             normrnd(0,meas_noise_var);
             normrnd(0,meas_noise_var)];
    for m = 1:M
        z_pred(:,m) = xP_pred(:,m) + [normrnd(0,meas_noise_var);
                                     normrnd(0,meas_noise_var);
                                     normrnd(0,meas_noise_var)];
        P(m) = mvnpdf(zi - z_pred(:,m),zeros(3,1),R); % this might not be the correct function
    end
    P = P./sum(P); % normalize
    
    % resampling:
    for m = 1:M
%         cumulative_sum = cumsum(P);
%         cumulative_sum(end) = 1;
%         indexes = find(cumulative_sum, rand);
        xP(:,m) = xP_pred(:,find(rand <= cumsum(P),1));
    end
    
    x_est = mean(xP,2); % estimate the state
    
    % conditionally plot the particles:
    if mod(t(i),1)==0
        fprintf("t = %f\n",t(i));
        scatter(xP(1,:),xP(2,:),'.')
        hold on
        plot(x_est(1),x_est(2),'k*')
    end
    
         
    % log:
    x_log(:,i) = xi;
    z_log(:,i) = zi;
    x_est_log(:,i) = x_est;
    
    % set up for next iteration:
    xprev = x;
%     xP_prev = xP_pred;
end
hold off
% legend('Actual','t = 1','t = 2','t = 3','t = 4','t = 5','t = 6','Location','Best')
title('Actual path and estimated path')

%% Visualizations
% figure
% plot(t,x)
% xlabel('time (s)')
% legend('x','y','\theta','Location','Best')
% title('Actual state vs time')

figure
plot(x_log(1,:),x_log(2,:))
hold on
scatter(xP(1,:),xP(2,:),'r.')
hold off
xlabel('x')
ylabel('y')
title('Actual path')
axis([-1 5 -1 3]);

figure
plot(x_log(1,:),x_log(2,:))
hold on
plot(z_log(1,:),z_log(2,:))
hold off
legend('state','observation')