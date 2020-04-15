function getcruise_control()
%Function to update the CONTROL struct for the Fx input to maintain
%a reference velocity
global CONTROL
global DATA
global SIM

% Update control error
vx = DATA.X_curr(1);
err_vx = CONTROL.ref_vx - vx;

% Apply control law
CONTROL.Fx = 50*err_vx;

end

