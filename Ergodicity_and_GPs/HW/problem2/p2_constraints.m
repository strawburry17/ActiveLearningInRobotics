%% p2_constraints.m
function [c, ceq] = p2_constraints(state_and_input)
    global n;
    global dt;
    global x0 y0 th0;
    global xd yd thd;
    global N K sigma xdata ydata
    c = []; % no linear inequality constraints
    
    x = state_and_input(1,:);
    y = state_and_input(2,:);
    th = state_and_input(3,:);
    u1 = state_and_input(4,:);
    u2 = state_and_input(5,:);
    
    ceq = [x(1)-x0, y(1)-y0, th(1)-th0];
    
    for i = 1:n-1
        
        xtest = [x(i);y(i);th(i);u1(i);u2(i)];
    
        k_arrow = zeros(N,1);
        for j = 1:N
            k_arrow(j,:) = radBasFcn(xdata(j,:), xtest);
        end
        kss = radBasFcn(xtest,xtest);

        f_pred = k_arrow'*inv(K + (sigma^2).*eye(N))*ydata;
        
        x_constraint = x(i+1) - x(i) - f_pred(1)*dt;
        y_constraint = y(i+1) - y(i) - f_pred(2)*dt;
        th_constraint = th(i+1) - th(i) - f_pred(3)*dt;
        ceq = [ceq; x_constraint, y_constraint, th_constraint];
    end
    
    ceq = [ceq; x(n) - xd(n), y(n) - yd(n), th(n) - thd(n)];