%% DJ.m
function dj = DJ(time,x,u,z,v,a,b)
    
    runningcost = zeros(length(time),1);
    for i = 1:length(time)
        runningcost(i) =  a(:,i)'*z(:,i)+ b(:,i)'*v(:,i);
    end
    
    dj = trapz(time,runningcost);
    
end