%% riccati.m
% reference:
% https://www.mathworks.com/matlabcentral/answers/94722-how-can-i-solve-the-matrix-riccati-differential-equation-within-matlab
function dPdt = riccati(t, P, tvec, xvec, uvec)
    global Q R;
    
%     x1t = interp1(tvec,x(1,:),t,'spline');
%     x2t = interp1(tvec,x(2,:),t,'spline');
%     u1 = interp1(tvec, uvec(1,:), t,'spline');
%     u2 = interp1(tvec, uvec(2,:), t,'spline');
    
    At = zeros(2);
      
    Bt = ones(2);
    
    P = reshape(P, size(At)); %Convert from "n^2"-by-1 to "n"-by-"n"
    
    dPdt =  -P*At - At.'*P + P*Bt*inv(R)*Bt.'*P - Q;
    
    dPdt = dPdt(:); %Convert from "n"-by-"n" to "n^2"-by-1
    
end