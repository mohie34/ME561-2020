function SIMULATE_TRIKE(t)
%Performs an ode45 integration of the Trike's dynamics with the Control input provided and updates the necessary data and trajectory vars
    
    global CONTROL
    global DATA
    global TP
    global SIM
    
    dynamics = @(t,X)(vehicle_dynamics_mpc(X, CONTROL.U));
    [~, Y] = ode45(dynamics, [t t+SIM.T_step], DATA.X_curr);
    DATA.X_curr = Y(end,:);
    
    DATA.T_sim = [DATA.T_sim, t];
    DATA.X_sim = [DATA.X_sim; DATA.X_curr];
    DATA.U_sim = [DATA.U_sim; CONTROL.U];
    TP.trike_pos = DATA.X_curr(7:9);
end

