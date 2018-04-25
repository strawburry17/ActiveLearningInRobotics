%% p5_J1.m
function J = p5_J1(time,x)
    global Q;
    global xd;
    
    runningcost = zeros(length(time),1);
    for i = 1:length(time)
        runningcost(i) = (x(:,i) - xd(:,i))'*Q*(x(:,i) - xd(:,i));
    end
    
    J = 0.5*trapz(time,runningcost);

end