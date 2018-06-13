%% remainder.m
% reference:
% https://www.mathworks.com/matlabcentral/answers/94722-how-can-i-solve-the-matrix-riccati-differential-equation-within-matlab
function drdt = remainder(t, r, tvec, xvec, uvec, Pvec)
    global Q R;

    x1t = interp1(tvec,xvec(1,:),t,'spline');
    x2t = interp1(tvec,xvec(2,:),t,'spline');

    x1dt = interp1(tvec,xd(1,:),t,'spline');
    x2dt = interp1(tvec,xd(2,:),t,'spline');

    u1 = interp1(tvec, uvec(1,:), t,'spline');
    u2 = interp1(tvec, uvec(2,:), t,'spline');

    Pt = zeros(2);
    Pt(1,1) = interp1(tvec, Pvec(1,:), t,'spline');
    Pt(1,2) = interp1(tvec, Pvec(2,:), t,'spline');
    Pt(2,1) = interp1(tvec, Pvec(3,:), t,'spline');
    Pt(2,2) = interp1(tvec, Pvec(4,:), t,'spline');

    At = zeros(2);
      
    Bt = ones(2);

    aT = ([x1t; x2t; thetat] - [xdt; ydt; thetadt])'*(Q' + Q); % TODO: CHANGE FOR ERGODIC CONTROL
    a = aT';
    
    bT = ([u1; u2])'*(R' + R); % TODO: CHANGE FOR ERGODIC CONTROL
    b = bT';


    drdt = -(At - Bt*inv(R)*Bt'*Pt)'*r - a + Pt*Bt*inv(R)*b;

end
