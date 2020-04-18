function get_lane_keeping(goal_point)
%Function to update the CONTROL struct for the Fx input to maintain
%a reference velocity
global CONTROL
global DATA

% Update control error delta
psi = DATA.X_curr(10);
vx = DATA.X_curr(1);

%Update position error
err_x = goal_point(1)- DATA.X_curr(7);
err_y = goal_point(2)- DATA.X_curr(8);
err_alpha = atan2(err_y, err_x)-psi;
disp("err_alpha");
disp(err_alpha);

k_delta = 0.35;
if abs(vx)<1e-3 || abs(err_alpha) < 1e-3
    CONTROL.delta = 0;
else
    CONTROL.delta = atan(2*1.8*sin(err_alpha)/(k_delta*vx));
end
disp("CONTROL.delta");
disp(CONTROL.delta);
end

