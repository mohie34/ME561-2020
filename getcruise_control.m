function getcruise_control(P,I,D)
%Function to update the CONTROL struct for the Fx input to maintain
%a reference velocity
global CONTROL
global DATA
global SIM

if(~CONTROL.init_cruise)
    CONTROL.init_cruise = true;
    CONTROL.derr_vx = 0;
    CONTROL.interr_vx = 0;
    CONTROL.err_vx = 0;
end

% Update control error
vx = DATA.X_curr(1);
prev_err_vx = CONTROL.err_vx;
CONTROL.err_vx = CONTROL.ref_vx - vx;
CONTROL.derr_vx = (CONTROL.err_vx - prev_err_vx)/SIM.T_step;
CONTROL.interr_vx = CONTROL.interr_vx + (CONTROL.err_vx + prev_err_vx)*SIM.T_step/2;

if(SIM.debug)
    disp("vx");
    disp(vx);
end

% Apply control law
CONTROL.Fx = P*CONTROL.err_vx + I*CONTROL.interr_vx + D*CONTROL.derr_vx;

if(SIM.debug)    
    disp("CONTROL.Fx");
    disp(CONTROL.Fx);
end

end

