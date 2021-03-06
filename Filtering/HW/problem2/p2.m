clear;
close all;
clc;

% A very helpful reference:
% http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies

% continuous-time system:
A = [0 1;
    -1 0];
B = zeros(2,1);
C = eye(2);

% noise parameters:
process_noise_variance = 0.1;
measurement_noise_variance = 0.1;

x0 = [1;1];

dxdt = @(t,x) A*x + sqrt(0.1)*randn(2,1);
dt = 0.1;
T = 1.0;

% discretize:
Ak = eye(2) + A.*dt;
Bk = B.*dt;
Ck = C;
Qk = process_noise_variance*eye(2); % covariance of process noise (matrix)
Rk = measurement_noise_variance*eye(2); % covariance of measurement noise (matrix)

t = 0:dt:T;

[t, x] = ode45(dxdt,t,x0);
t = t';
x = x';

xk_prev = x0;
uk = 0;
Pk_prev = eye(2); % initial guess of the error covariance
for i = 1:length(t)
    % time update (prediction):
    xk_pred = Ak*xk_prev + Bk*uk; % predict the new state
    Pk_pred = Ak*Pk_prev*(Ak') + Qk; % predict the new error covariance

    % measurement update (correction):
    Kk = Pk_pred*(Ck')*inv(Ck*Pk_prev*(Ck') + Rk); % compute the Kalman gain
    zk = Ck*x(:,i) + normrnd(0,measurement_noise_variance);
    xk = xk_pred + Kk*(zk - Ck*xk_pred); % update the estimate with the measurement
    Pk = (eye(2) - Kk*Ck)*Pk_pred; % update the error covariance
    
    % set up for next iteration:
    xk_prev = xk;
    Pk_prev = Pk;
    
    % log:
    xkpred_log(i,:) = xk_pred;
    xk_log(i,:) = xk;
    Pk_log(i,:) = [Pk(1,1); Pk(1,2); Pk(2,1); Pk(2,2)];
    Kk_log(i,:) = [Kk(1,1); Kk(1,2); Kk(2,1); Kk(2,2)];
    
end

figure
plot(t,x)
xlabel('time (s)')
legend('x','y','Location','Best')
title('Actual state vs time')

figure
plot(t,xk_log)
xlabel('time (s)')
legend('xk','yk','Location','Best')
title('Predicted state vs time')

figure
plot(t,Pk_log)
xlabel('time (s)')
legend('Pk(1,1)','Pk(1,2)','Pk(2,1)','Pk(2,2)','Location','Best')
title('Covariance of state prediction vs time')

figure
plot(t,Kk_log)
xlabel('time (s)')
legend('Kk(1,1)','Kk(1,2)','Kk(2,1)','Kk(2,2)','Location','Best')
title('Kalman gain vs time')

figure
plot(t,x,'--')
hold on
plot(t,xk_log,'-')
hold off
xlabel('time (s)')
legend('x','y','xk','yk','Location','Best')
title('Comparison of actual and estimated state vs time')

figure
plot(t,x,'--')
hold on
plot(t,xkpred_log,'-')
plot(t,xk_log,'-')
hold off
xlabel('time (s)')
legend('x','y','xk_pred','yk_pred','xk','yk','Location','Best')
title('Comparison of actual, predicted, and estimated state vs time')