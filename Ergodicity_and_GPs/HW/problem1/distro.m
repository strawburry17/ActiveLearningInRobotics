%% distro.m
function phi = distro(x)
    Sigma = diag([2,2]);
    mu = [0; 0];
    
    phi = (1/det(2*pi.*Sigma))*exp(-0.5.*(x - mu)'*inv(Sigma)*(x - mu));
end