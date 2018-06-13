%% descent_dir.m
function zeta = descent_dir(time, x, u)
    global Q R;
    %% solve Riccati + remainder equations:
    PT = zeros(2);
   rT = zeros(2,1);
   [time, P] = ode45(@(t,P) p4_riccati(t, P, time, x, u), flip(time), PT);
   P = flip(P)';
   time = flip(time)';
   [time, r] = ode45(@(t,r) p4_remainder(t, r, time, x, u, P), flip(time), rT);
   r = flip(r)';
   time = flip(time)';

   %% simulate state perturbation z forward in time:
   [time, z] = ode45(@(t,z) p4_z_ode(t,z,time, x, u, P, r), time, zeros(1,3));
   z = z';

   %% compute control perturbation v from z:
   for i = 1:length(time)
        x1t = x(1,i);
        x2t = x(2,i);

        x1dt = xd(1,i);
        x2dt = xd(2,i);

        u1 = u(1,i);
        u2 = u(2,i);

        Pt = zeros(2);
        Pt(1,1) = P(1,i);
        Pt(1,2) = P(2,i);
        Pt(2,1) = P(3,i);
        Pt(2,2) = P(4,i);

        rt = zeros(2,1);
        rt(1) = r(1,i);
        rt(2) = r(2,i);

        At = zeros(2);
      
        Bt = ones(2);

        aT = ([x1t; x2t; thetat] - [x1dt; x2dt; thetadt])'*(Q' + Q); % TODO: CHANGE FOR ERGODIC CONTROL
        a = aT';

        bT = ([u1; u2])'*(R' + R); % TODO: CHANGE FOR ERGODIC CONTROL
        b = bT';

        v(:,i) = -inv(R)*(Bt')*Pt*z(:,i) - inv(R)*(Bt')*rt - inv(R)*b;
   end
   
   %% return descent direction zeta = (z, v):
   zeta = [z; v];
end