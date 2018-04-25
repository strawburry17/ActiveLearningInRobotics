%% p5_mode_ins_grad.m
function m = p5_mode_ins_grad(t,x,tx,rho,trho,u1,u2)
    xt = [interp1(tx,x(1,:),t,'spline'); ...
        interp1(tx,x(2,:),t,'spline'); ...
        interp1(tx,x(3,:),t,'spline')];

    f1 = [cos(xt(3))*u1(1); sin(xt(3))*u1(1); u1(2)];
    f2 = [cos(xt(3))*u2(1); sin(xt(3))*u2(1); u2(2)];
    
    rhot = [interp1(trho,rho(1,:),t,'spline'); ...
        interp1(trho,rho(2,:),t,'spline'); ...
        interp1(trho,rho(3,:),t,'spline')];
    
    m = rhot'*(f2 - f1);
end