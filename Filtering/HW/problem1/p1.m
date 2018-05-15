clear;
close all;
clc;

%% Particle filter for the vehicle model in Homework 1
% references:
% https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb
% http://studentdavestutorials.weebly.com/particle-filter-with-matlab-code.html

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
xlabel('x')
ylabel('y')
axis([-1 5 -1 3]);

xprev = x0;

for i = 1:length(t)
    
    % update state and observed state (Euler integration):
    xi = x(:,i);   
    
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
        xP(:,m) = xP_pred(:,find(rand <= cumsum(P),1));
    end
    
    x_est = mean(xP,2); % estimate the state
    
    % conditionally plot the particles:
    if mod(t(i),1)==0
        fprintf("t = %f\n",t(i));
        switch t(i)
            case 0
                s = 'b.';
            case 1
                s = 'g.';
            case 2
                s = 'r.';
            case 3
                s = 'c.';
            case 4
                s = 'm.';
            case 5
                s = 'y.';
            case 6
                s = 'b.';
        end
    end
    scatter(xP(1,:),xP(2,:),s)
    hold on
    
         
    % log:
    x_log(:,i) = xi;
    z_log(:,i) = zi;
    x_est_log(:,i) = x_est;
    
    % set up for next iteration:
    xprev = x;
end
plot(x(1,:),x(2,:),'k')
plot(x_est_log(1,:),x_est_log(2,:),'k*')
hold off
title('Actual path and estimated path')

%% Visualizations

figure
plot(x_log(1,:),x_log(2,:))
hold on
plot(z_log(1,:),z_log(2,:))
hold off
legend('state','observation')