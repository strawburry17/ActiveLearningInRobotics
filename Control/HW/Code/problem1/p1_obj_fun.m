%% p1_obj_fun
function J = p1_obj_fun(state_and_input)
    global N t;
    global xd yd thd;
    
    x = state_and_input(1,:);
    y = state_and_input(2,:);
    th = state_and_input(3,:);
    u1 = state_and_input(4,:);
    u2 = state_and_input(5,:);
    
    x_weight = 1;
    y_weight = 1;
    th_weight = 1;
    
    Q = diag([x_weight, y_weight, th_weight]); % weights on state part of running cost
    
    u1_weight = 1;
    u2_weight = 1;
    
    R = diag([u1_weight, u2_weight]); % weights on control part of running cost
    
    P1 = diag([10, 10, 10]); % weights on terminal cost
    
    running_cost = 0;
    
    for i = 1:N-1
        state_cost = [xd(i)-x(i);yd(i)-y(i);thd(i)-th(i)]'*Q*[xd(i)-x(i);yd(i)-y(i);thd(i)-th(i)];
        control_cost = [u1(i); u2(i)]'*R*[u1(i); u2(i)];
        running_cost = running_cost + (state_cost + control_cost);
    end
    
    terminal_cost = [xd(N)-x(N); yd(N)-y(N);thd(N)-th(N)]'*P1*[xd(N)-x(N); yd(N)-y(N);thd(N)-th(N)];
    
    J = running_cost + terminal_cost;