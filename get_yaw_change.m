function get_yaw_change(P,I,D)
%Function to update the CONTROL struct for the Fx input to maintain
%a reference velocity
global CONTROL
global DATA
global SIM

if(~CONTROL.init_lanechange)
    CONTROL.init_cruise = true;
    CONTROL.err_psi = 0;
    CONTROL.derr_psi = 0;
    CONTROL.interr_psi = 0;
end

% Update control error delta
psi = DATA.X_curr(10);
prev_err_psi = CONTROL.err_psi;
CONTROL.err_psi = CONTROL.ref_psi - psi;
CONTROL.derr_psi = (CONTROL.err_psi - prev_err_psi)/SIM.T_step;
CONTROL.interr_psi = CONTROL.interr_psi + (CONTROL.err_psi + prev_err_psi)*SIM.T_step/2;

%Update position error
CONTROL.delta = P*CONTROL.err_psi+I*CONTROL.interr_psi+D*CONTROL.derr_psi;
disp("CONTROL.delta");
disp(CONTROL.delta);
end

