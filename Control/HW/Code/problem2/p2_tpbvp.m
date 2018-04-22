%% p2_tpbvp.m
function dydt = p2_tpbvp(t,y)
    global A B Q R;
    
    M = [A -(B/R)*(B');
         -Q -A'];
     
    dydt = M*y;    
end