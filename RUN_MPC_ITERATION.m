function RUN_MPC_ITERATION()
    
    global DATA
    global CONTROL
    global TP
    
    x0 = DATA.X_curr(7:9);
    u0 = CONTROL.U_hori;
    options = optimoptions('fmincon','MaxFunctionEvaluations',500);
    CONTROL.U_hori = fmincon(@(u)nonl_cf(x0,u, TP.waypoint), u0, [], [],...
        [], [], [], [], @(u)nonl_bound_func(x0,u, CONTROL.ulim,false),...
        options);
    CONTROL.U_MPC = DATA.U_hori(1:2);

end