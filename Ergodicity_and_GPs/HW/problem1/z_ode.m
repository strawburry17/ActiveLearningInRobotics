%% z_ode.m
function dzdt = z_ode(t,z,tvec, xvec, uvec, Pvec, rvec)
    global Q R;
    
    x1t = interp1(tvec,xvec(1,:),t,'spline');
    x2t = interp1(tvec,xvec(2,:),t,'spline');
    
    
    u1 = interp1(tvec, uvec(1,:), t,'spline');
    u2 = interp1(tvec, uvec(2,:), t,'spline');
    
    Pt = zeros(2);
    Pt(1,1) = interp1(tvec, Pvec(1,:), t,'spline');
    Pt(1,2) = interp1(tvec, Pvec(2,:), t,'spline');
    Pt(2,1) = interp1(tvec, Pvec(3,:), t,'spline');
    Pt(2,2) = interp1(tvec, Pvec(4,:), t,'spline');
    
    rt = zeros(2,1);
    rt(1) = interp1(tvec, rvec(1,:), t,'spline');
    rt(2) = interp1(tvec, rvec(2,:), t,'spline');
    
    At = zeros(2);
      
    Bt = ones(2);
      
    aT = ([x1t; x2t; thetat] - [xdt; ydt; thetadt])'*(Q' + Q); % TODO: CHANGE FOR ERGODIC CONTROL
    a = aT';
    
    bT = ([u1; u2])'*(R' + R); % TODO: CHANGE FOR ERGODIC CONTROL
    b = bT';
    
    v = -inv(R)*(Bt')*Pt*z - inv(R)*(Bt')*rt - inv(R)*b;
    
    dzdt = At*z + Bt*v;
end