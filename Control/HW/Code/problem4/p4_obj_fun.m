%% p4_obj_fun.m
function J = p4_obj_fun(time,x,u)
    global Q R P1;
    global xd;
    
    runningcost = zeros(length(time),1);
    for i = 1:length(time)
        runningcost(i) = (x(:,i) - xd(:,i))'*Q*(x(:,i) - xd(:,i)) + u(:,i)'*R*u(:,i);
    end
    
    termcost = (x(:,end) - xd(:,end))'*P1*(x(:,end) - xd(:,end));
    J = 0.5*trapz(time,runningcost) + 0.5*termcost;

end