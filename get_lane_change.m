function get_lane_change(P,I,D, P2, I2, D2)
%Function to update the CONTROL struct for the Fx input to maintain
%a reference velocity
global CONTROL
global DATA
global SIM


if(~CONTROL.init_lanechange)
    CONTROL.init_cruise = true;
    CONTROL.derr_y = 0;
    CONTROL.interr_y = 0;
    CONTROL.err_y = 0;
    CONTROL.err_psi = 0;
    CONTROL.derr_psi = 0;
    CONTROL.interr_psi = 0;
end

ref_psi = 0;

% Update control error delta
y = DATA.X_curr(8);
prev_err_y = CONTROL.err_y;
CONTROL.err_y = CONTROL.ref_y - y;
CONTROL.derr_y = (CONTROL.err_y - prev_err_y)/SIM.T_step;
CONTROL.interr_y = CONTROL.interr_y + (CONTROL.err_y + prev_err_y)*SIM.T_step/2;

% Update control error delta
psi = DATA.X_curr(10);
prev_err_psi = CONTROL.err_psi;
CONTROL.err_psi = ref_psi - psi;
CONTROL.derr_psi = (CONTROL.err_psi - prev_err_psi)/SIM.T_step;
CONTROL.interr_psi = CONTROL.interr_psi + (CONTROL.err_psi + prev_err_psi)*SIM.T_step/2;

%Update position error
CONTROL.delta = P*CONTROL.err_y+I*CONTROL.interr_y+D*CONTROL.derr_y;
CONTROL.delta = CONTROL.delta + P2*CONTROL.err_psi+I2*CONTROL.interr_psi+D2*CONTROL.derr_psi;
disp("CONTROL.delta");
disp(CONTROL.delta);
end

