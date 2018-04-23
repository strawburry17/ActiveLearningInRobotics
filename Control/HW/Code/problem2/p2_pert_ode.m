%% p2_pert_ode.m
function dzdt = p2_pert_ode(t, z)
    global A B;
    global Av Bv Cv Dv; % parameterize the control perturbation v
    
    v = Av*sin(Bv*t + Cv) + Dv;
    
    dzdt = A*z + B*v;
end