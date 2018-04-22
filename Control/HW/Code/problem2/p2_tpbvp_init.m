%% p2_tpbvp_init.m
function v = p2_tpbvp_init(t)
    global x0;
    v = [x0; 1; 0];
end