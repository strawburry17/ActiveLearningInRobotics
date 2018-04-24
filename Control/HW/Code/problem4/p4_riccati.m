%% p4_riccati.m
% reference:
% https://www.mathworks.com/matlabcentral/answers/94722-how-can-i-solve-the-matrix-riccati-differential-equation-within-matlab
function dPdt = p4_riccati(t, P, tvec, xvec, uvec)
    global Q R;
    
%     xt = interp1(tvec,x(1,:),t,'spline');
%     yt = interp1(tvec,x(2,:),t,'spline');
    theta = interp1(tvec,xvec(3,:),t,'spline');
    u1 = interp1(tvec, uvec(1,:), t,'spline');
%     u2 = interp1(tvec, uvec(2,:), t,'spline');
    
    At = [0,    0,  -sin(theta)*u1;
          0,    0,  cos(theta)*u1;
          0,    0,  0];
      
    Bt = [cos(theta),   0;
          sin(theta),   0;
          0,            1];
    
    P = reshape(P, size(At)); %Convert from "n^2"-by-1 to "n"-by-"n"
    
    dPdt =  -P*At - At.'*P + P*Bt*inv(R)*Bt.'*P - Q;
    
    dPdt = dPdt(:); %Convert from "n"-by-"n" to "n^2"-by-1
    
end