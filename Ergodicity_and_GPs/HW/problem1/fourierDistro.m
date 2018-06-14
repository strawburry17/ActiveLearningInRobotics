%% fourierDistro.m
function phi_k = fourierDistro(k,L1,L2)

k1 = k(1);
k2 = k(2);

numL = exp(-(((k1^2)/(L1^2)) + ((k2^2)/(L2^2)))*(pi^2));
numR = (erf((L1/2)) + erf((L1/2))) * (erf((L2/2)) + erf((L2/2)));
num = numL*numR;

den = sqrt((L1*L2*(2*k1*pi + sin(2*k1*pi))*(2*k2*pi + sin(2*k2*pi)))/(k1*k2));

phi_k = (pi/4)*real(num/den);

end