%% p4_descent_dir.m
function zeta = p4_descent_dir(time, x, u)
    global Q R P1;
    global xd;
    %% solve Riccati + remainder equations:
   r0 = P1*(x(:,end) - xd(:,end));
   [time, P] = ode45(@(t,P) p4_riccati(t, P, time, x, u), flip(time), P1);
   P = flip(P)';
   time = flip(time)';
   [time, r] = ode45(@(t,r) p4_remainder(t, r, time, x, u, P), flip(time), r0);
   r = flip(r)';
   time = flip(time)';

   %% simulate state perturbation z forward in time:
   [time, z] = ode45(@(t,z) p4_z_ode(t,z,time, x, u, P, r), time, zeros(1,3));
   z = z';

   %% compute control perturbation v from z:
   for i = 1:length(time)
        xt = x(1,i);
        yt = x(2,i);
        thetat = x(3,i);

        xdt = xd(1,i);
        ydt = xd(2,i);
        thetadt = xd(3,i);

        u1 = u(1,i);
        u2 = u(2,i);

        Pt = zeros(3);
        Pt(1,1) = P(1,i);
        Pt(1,2) = P(2,i);
        Pt(1,3) = P(3,i);
        Pt(2,1) = P(4,i);
        Pt(2,2) = P(5,i);
        Pt(2,3) = P(6,i);
        Pt(3,1) = P(7,i);
        Pt(3,2) = P(8,i);
        Pt(3,3) = P(9,i);

        rt = zeros(3,1);
        rt(1) = r(1,i);
        rt(2) = r(2,i);
        rt(3) = r(3,i);

        At = [0,    0,  -sin(thetat)*u1;
              0,    0,  cos(thetat)*u1;
              0,    0,  0];

        Bt = [cos(thetat),   0;
              sin(thetat),   0;
              0,            1];

        aT = ([xt; yt; thetat] - [xdt; ydt; thetadt])'*(Q' + Q);
        a = aT';

        bT = ([u1; u2])'*(R' + R);
        b = bT';

        v(:,i) = -inv(R)*(Bt')*Pt*z(:,i) - inv(R)*(Bt')*rt - inv(R)*b;
   end
   
   %% return descent direction zeta = (z, v):
   zeta = [z; v];
end