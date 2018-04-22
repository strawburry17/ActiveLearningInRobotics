%% p2_tpbvp_bc.m
function bc = p2_tpbvp_bc(y0,yf)
    global P1 x0;
    bc = [y0(1) - x0(1);
          y0(2) - x0(2);
          yf(3:4) - P1*yf(1:2)];
end