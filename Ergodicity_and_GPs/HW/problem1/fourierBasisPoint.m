%% fourierBasisPoint.m
function Fk = fourierBasisPoint(xPt,k,L1,L2)

n = 2; % state dimension
k1 = k(1);
k2 = k(2);

hk_integrand = @(x,y) (((cos(k1*pi.*x./L1)).^2).*((cos(k2*pi.*y./L2)).^2));

hk = sqrt(integral2(hk_integrand,0,L1,0,L2));

Fk = (1/hk)*cos(k1*pi*xPt(1)/L1)*cos(k2*pi*xPt(2)/L2);

end