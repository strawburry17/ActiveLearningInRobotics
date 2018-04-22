%% p1_constraints.m
function [c, ceq] = p1_constraints(state_and_input)
    global N;
    global dt;
    global x0 y0 th0;
    global xd yd thd;
    c = []; % no linear inequality constraints
    
    x = state_and_input(1,:);
    y = state_and_input(2,:);
    th = state_and_input(3,:);
    u1 = state_and_input(4,:);
    u2 = state_and_input(5,:);
    
    ceq = [x(1)-x0, y(1)-y0, th(1)-th0];
    
    for i = 1:N-1
        x_constraint = x(i+1) - x(i) - cos(th(i))*u1(i)*dt;
        y_constraint = y(i+1) - y(i) - sin(th(i))*u1(i)*dt;
        th_constraint = th(i+1) - th(i) - u2(i)*dt;
        ceq = [ceq; x_constraint, y_constraint, th_constraint];
    end
    
    ceq = [ceq; x(N) - xd(N), y(N) - yd(N), th(N) - thd(N)];