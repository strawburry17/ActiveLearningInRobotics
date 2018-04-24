%% p4_z_ode.m
function dzdt = p4_z_ode(t,z,tvec, xvec, uvec, Pvec, rvec)
    global Q R;
    global xd;
    
    xt = interp1(tvec,xvec(1,:),t,'spline');
    yt = interp1(tvec,xvec(2,:),t,'spline');
    thetat = interp1(tvec,xvec(3,:),t,'spline');
    
    xdt = interp1(tvec,xd(1,:),t,'spline');
    ydt = interp1(tvec,xd(2,:),t,'spline');
    thetadt = interp1(tvec,xd(3,:),t,'spline');
    
    u1 = interp1(tvec, uvec(1,:), t,'spline');
    u2 = interp1(tvec, uvec(2,:), t,'spline');
    
    Pt = zeros(3);
    Pt(1,1) = interp1(tvec, Pvec(1,:), t,'spline');
    Pt(1,2) = interp1(tvec, Pvec(2,:), t,'spline');
    Pt(1,3) = interp1(tvec, Pvec(3,:), t,'spline');
    Pt(2,1) = interp1(tvec, Pvec(4,:), t,'spline');
    Pt(2,2) = interp1(tvec, Pvec(5,:), t,'spline');
    Pt(2,3) = interp1(tvec, Pvec(6,:), t,'spline');
    Pt(3,1) = interp1(tvec, Pvec(7,:), t,'spline');
    Pt(3,2) = interp1(tvec, Pvec(8,:), t,'spline');
    Pt(3,3) = interp1(tvec, Pvec(9,:), t,'spline');
    
    rt = zeros(3,1);
    rt(1) = interp1(tvec, rvec(1,:), t,'spline');
    rt(2) = interp1(tvec, rvec(2,:), t,'spline');
    rt(3) = interp1(tvec, rvec(3,:), t,'spline');
    
    At = [0,    0,  -sin(thetat)*u1;
          0,    0,  cos(thetat)*u1;
          0,    0,  0];
      
    Bt = [cos(thetat),   0;
          sin(thetat),   0;
          0,            1];
      
    aT = ([xt; yt; thetat] - [xdt; ydt; thetadt])'*(Q' + Q);
    a = aT';
    
    bT = ([u1; u2])'*(R' + R);
    b = bT';
    
    v = -inv(R)*(Bt')*Pt*z - inv(R)*(Bt')*rt - inv(R)*b;
    
    dzdt = At*z + Bt*v;
end