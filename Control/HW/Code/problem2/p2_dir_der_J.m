%% p2_dir_der_J
function DJ = p2_dir_der_J(time,x,u,z,v)
    global Q R P1;
    
    runningcost = zeros(length(time),1);
    for i = 1:length(time)
        runningcost(i) = ((x(:,i)')*(Q' + Q)*z(:,i) + (u(:,i)')*(R' + R)*v(:,i));
    end
    
    int_runningcost = 0.5*trapz(time,runningcost);
    
    termcost = 0.5*(x(:,end)')*(P1' + P1)*z(:,end);
        
    DJ = int_runningcost + termcost;;
end