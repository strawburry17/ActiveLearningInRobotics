%% p4_sys_ode.m
function dxdt = p4_sys_ode(t,x,uvec,tvec)
    % ut is an array storing the control signals
    % tu is a time vector
    u1 = interp1(tvec, uvec(1,:), t,'spline');
    u2 = interp1(tvec, uvec(2,:), t,'spline');
    u = [u1; u2];
    
    dxdt = [cos(x(3))*u1; sin(x(3))*u1; u2];
end