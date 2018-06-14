%% descent_dir.m
function [zeta, avec, bvec] = descent_dir(time, x, u, T)
    global q R K L1 L2
    %% compute time-varying signals a(t) and b(t)
    n = 2; % state dimension
    avecT = zeros(length(time),2);
    bvecT = zeros(length(time),2);
    for i = 1:length(time)
%         fprintf("time index = %f\n",i);
        for k1 = 1:K
            for k2 = 1:K
                k = [k1; k2];
                Lambda_k = (1 + norm(k)^2)^(-(n+1)/2);
                ck = fourierTraj(x,time,T,k,L1,L2);
                phik = fourierDistro(k,L1,L2);
                
                avecT(i,:) = avecT(i,:) + Lambda_k*(2*(ck-phik).*((1/T).*dfk(x(:,i),k,L1,L2)));
            end
        end
        
        bvecT(i,:) = u(:,i)'*R;
    end
    avec = q.*avecT';
    bvec = bvecT';

    %% solve Riccati + remainder equations:
   PT = zeros(2);
   rT = zeros(2,1);
   
   [time, P] = ode45(@(t,P) riccati(t, P, time, x, u), flip(time), PT);
   P = flip(P)';
   time = flip(time)';
   
   [time, r] = ode45(@(t,r) remainder(t, r, time, x, u, P, avec, bvec), flip(time), rT);
   r = flip(r)';
   time = flip(time)';

   %% simulate state perturbation z forward in time:
   [time, z] = ode45(@(t,z) z_ode(t,z,time, x, u, P, r, avec, bvec), time, zeros(1,2));
   z = z';

   %% compute control perturbation v from z:
   for i = 1:length(time)

        Pt = zeros(2);
        Pt(1,1) = P(1,i);
        Pt(1,2) = P(2,i);
        Pt(2,1) = P(3,i);
        Pt(2,2) = P(4,i);

        rt = zeros(2,1);
        rt(1) = r(1,i);
        rt(2) = r(2,i);
      
        Bt = ones(2);

        at = avec(:,i);
        bt = bvec(:,i);

        v(:,i) = -inv(R)*(Bt')*Pt*z(:,i) - inv(R)*(Bt')*rt - inv(R)*bt;
   end
   
   %% return descent direction zeta = (z, v):
   zeta = [z; v];
end