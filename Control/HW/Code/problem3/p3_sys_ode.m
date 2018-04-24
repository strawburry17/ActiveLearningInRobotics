%% p3_riccati.m
function dxdt = p3_sys_ode(t, x, tP, Pt, A, B, R)
    % linearly interpolate Pt (solution to Riccati equation)
    P11 = interp1(tP, Pt(:,1), t);
    P12 = interp1(tP, Pt(:,2), t);
    P21 = interp1(tP, Pt(:,3), t);
    P22 = interp1(tP, Pt(:,4), t);
    P = [P11, P12; P21, P22];

    dxdt = (A - B*inv(R)*B'*P)*x;
    
end