%% distro.m
function phi = distro(x,y)

    Sigma = 2*eye(2);
    mu = zeros(2,1);
    
    scale_factor = (1./det(2.*pi.*Sigma));
    diff = [x; y] - mu;
    exponent = -0.5.*(diff')*(inv(Sigma)*diff);
    phi = scale_factor*exp(exponent);
end