function dxdt = gp_ode(t,x,uvec,tvec)
    global N K sigma xdata ydata

    % ut is an array storing the control signals
    % tu is a time vector
    u1 = interp1(tvec, uvec(1,:), t,'spline');
    u2 = interp1(tvec, uvec(2,:), t,'spline');
    u = [u1; u2];
    
    xtest = [x; u];
    
    k_arrow = zeros(N,1);
    for i = 1:N
        k_arrow(i,:) = radBasFcn(xdata(i,:), xtest);
    end
    kss = radBasFcn(xtest,xtest);
    
    f_pred = k_arrow'*inv(K + (sigma^2).*eye(N))*ydata;
%     v_pred = kss - k_arrow'*inv(K + (sigma^2).*eye(N))*k_arrow;
    
    dxdt = f_pred';
end