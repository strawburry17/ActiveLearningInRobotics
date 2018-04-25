%% p5_Jtau.m
function Jtau = p5_Jtau(u1,u2star,t,t0,x,tx,rho,trho)
    global beta;
    u2start = [interp1(tx,u2star(1,:),'spline');...
               interp1(tx,u2star(2,:),'spline')];
    
    m = p5_mode_ins_grad(t,x,tx,rho,trho,u1,u2start);
    
    Jtau = norm(u2start) + m + (t - t0)^beta;
end