%% obj_fun.m
function J = obj_fun(tvec,xvec,uvec)
    global q R;
    
    ergodicCost = q*ergodicMetric(xvec);
    
    runningcost = zeros(length(tvec),1);
    for i = 1:length(tvec)
        runningcost(i) = uvec(:,i)'*R*uvec(:,i);
    end
    
    J = ergodicCost + trapz(tvec,runningcost);

end