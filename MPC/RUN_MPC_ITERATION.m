function RUN_MPC_ITERATION()
    
    global DATA
    global CONTROL
    global TP
    global SIM

    x0 = DATA.X_curr;
    u0 = CONTROL.U_hori;
    options = optimoptions('fmincon','MaxFunctionEvaluations',200);
    CONTROL.U_hori = fmincon(@(u)nonl_cf(x0,u, TP.waypoint), u0, [], [],...
        [], [], [], [], @(u)nonl_bound_func(x0, u, SIM.Ulim) ,...
        options);
    CONTROL.U = CONTROL.U_hori(1:2);
    if(SIM.limit_input)    
        CONTROL.U(1) = abs(max(min(CONTROL.U(1),SIM.lims.Fx),-SIM.lims.Fx));  
        CONTROL.U(2) = max(min(CONTROL.U(2),SIM.lims.delta),-SIM.lims.delta);  
    end
end