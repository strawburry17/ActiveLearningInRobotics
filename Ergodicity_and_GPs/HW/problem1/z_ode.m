%% z_ode.m
function dzdt = z_ode(t,z,tvec, xvec, uvec, Pvec, rvec, avec, bvec)
    global R;
    
%     x1t = interp1(tvec,xvec(1,:),t,'spline');
%     x2t = interp1(tvec,xvec(2,:),t,'spline');
%     
%     
%     u1 = interp1(tvec, uvec(1,:), t,'spline');
%     u2 = interp1(tvec, uvec(2,:), t,'spline');
    
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
      
%     a1 = interp1(tvec, avec(1,:), t,'spline');
%     a2 = interp1(tvec, avec(2,:), t,'spline');
%     a = [a1; a2];
    
    b1 = interp1(tvec, bvec(1,:), t,'spline');
    b2 = interp1(tvec, bvec(2,:), t,'spline');
    b = [b1; b2];
    
    v = -inv(R)*(Bt')*Pt*z - inv(R)*(Bt')*rt - inv(R)*b;
    
    dzdt = At*z + Bt*v;
end