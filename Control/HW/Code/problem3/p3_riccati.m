%% p3_riccati.m
% reference:
% https://www.mathworks.com/matlabcentral/answers/94722-how-can-i-solve-the-matrix-riccati-differential-equation-within-matlab
function dPdt = p3_riccati(t, P, A, B, Q, R)
    P = reshape(P, size(A)); %Convert from "n^2"-by-1 to "n"-by-"n"
    
    dPdt =  -P*A - A.'*P + P*B*inv(R)*B.'*P - Q;
    
    dPdt = dPdt(:); %Convert from "n"-by-"n" to "n^2"-by-1
    
end