%% p5_adjoint_ode.m
function drhodt = p5_adjoint_ode(t,rho,xvec,uvec,twindow_vec,tglobal_vec)
    global Q;
    global xd;
    
    % xvec is an array storing the state trajectory
    % tvec is a time vector
    x1 = interp1(twindow_vec, xvec(1,:), t,'spline');
    x2 = interp1(twindow_vec, xvec(2,:), t,'spline');
    x3 = interp1(twindow_vec, xvec(3,:), t,'spline');
    x = [x1; x2; x3];
    
    % Interpolate xd:
    xd1 = interp1(tglobal_vec, xd(1,:), t,'spline');
    xd2 = interp1(tglobal_vec, xd(2,:), t,'spline');
    xd3 = interp1(tglobal_vec, xd(3,:), t,'spline');
    xdt = [xd1; xd2; xd3];
    
    % uvec is an array storing the control signals
    % tvec is a time vector
    u1 = interp1(tglobal_vec, uvec(1,:), t,'spline');
    u2 = interp1(tglobal_vec, uvec(2,:), t,'spline');
    u = [u1; u2];
    
    A = [0,0,-sin(x3)*u1;
         0,0, cos(x3)*u1;
         0,0, 0];
     
    drhodt = -A'*rho + (Q + Q')*(x - xdt);
    
end