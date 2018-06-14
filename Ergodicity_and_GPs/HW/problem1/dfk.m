%% dfk.m
function [DFk] = dfk(x,k,L1,L2)

x1 = x(1);
x2 = x(2);

k1 = k(1);
k2 = k(2);

sqrtInDen = sqrt((L1*L2*(2*k1*pi + sin(2*k1*pi))*(2*k2*pi + sin(2*k2*pi)))/(k1*k2));

num1 = -(4*k1*pi^2)*cos(k2*pi*x2/L2)*sin(k1*pi*x1/L1);
num2 = -(4*k2*pi^2)*cos(k1*pi*x1/L1)*sin(k2*pi*x2/L2);

DFk1 = num1/(L1*sqrtInDen);
DFk2 = num2/(L2*sqrtInDen);

DFk = [DFk1, DFk2]; % row vector

end