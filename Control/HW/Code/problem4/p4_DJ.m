%% p4_DJ.m
function DJ = p4_DJ(time,x,u,z,v)
    global Q R P1;
    global xd;
    
    runningcost = zeros(length(time),1);
    for i = 1:length(time)
        runningcost(i) = ((x(:,i)-xd(:,i))'*(Q' + Q)*z(:,i) + (u(:,i)')*(R' + R)*v(:,i));
    end
    
    int_runningcost = 0.5*trapz(time,runningcost);
%     DJ = int_runningcost;
    
    termcost = 0.5*(x(:,end)-xd(:,end))'*(P1' + P1)*z(:,end);
        
    DJ = int_runningcost + termcost;
end