%% ergodicMetric.m
function eps = ergodicMetric(xvec,tvec,T)
global K L1 L2

n = 2; % state dimension

eps = 0; % initialize metric

for k1 = 1:K
    for k2 = 1:K
        k = [k1; k2];
        Lambda_k = (1 + norm(k)^2)^(-(n+1)/2);
        
        ck = fourierTraj(xvec,tvec,T,k,L1,L2);

        phik = fourierDistro(k,L1,L2);
        
        eps = eps + (Lambda_k*(ck-phik)^2);
        
    end
end
end