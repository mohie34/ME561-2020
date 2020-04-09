function [vx_new, vy_new, vz_new, vxd_new, vyd_new, vzd_new,...
          phi_new, theta_new, psi_new, phid_new, thetad_new, psid_new] ...
        = vehicle_dynamics(vx, vy, vz, vxd, vyd, vzd,...
                           phi, theta, psi, phid, thetad, psid,...
                           torque, delta)

% % states:           vx, vy, vz, vxd, vyd, vzd, phi, theta, psi, phid, thetad, psid
% %                   vx, velocity in x direction
% %                   vxd, derivative of vx
% %                   phi, theta, psi are rotation angle in x, y, z direction respectively              
% % input:            torque, rear wheel torque
% %                   delta, front wheel steering angle

% Hard coded parameters
g = 9.81;           % m/s^2
h = 0.4;            % height of COG Affects PHI for roll
Tf = 0.6;           % distance between front wheel
Ms = 30; Mt = 40;   % vehicle sprung mass / total mass 
lf = 0.3; lr = 0.4; % distance in x direction: front/rear wheel axis to COG
Isxx = 0.1; Isyy = 0.2; Iszz = 0.3;  % Inertia to be estimated

%STEP1: Calculate Slip Angle (eq.25)
alpha_fl = atan((vy+lf*psid)/(vx+Tf*psi/2))-delta; %front left
alpha_fr = atan((vy+lf*psid)/(vx+Tf*psi/2))-delta; %front right
alpha_r = atan((vy-lr*psid)/vx); %rear

%STEP2: Calculate Fzf and Fzr (reference: Rob535 HW2)
Fzf = lf*Ms*g/(lf+lr);
Fzr = lr*Ms*g/(lf+lr);

%STEP3: Use magic tire to calculate Fx and Fy for each tire
Fxfl, Fyfl = magic_tire(alpha_fl, Fzf);
Fxfr, Fyfr = magic_tire(alpha_fr, Fzf);
Fxr, Fyr = magic_tire(alpha_r, Fzr);

%STEP4: Calculate XF and XY for each tire
XFfl = Fxfl * cos(delta) - Fyfl * sin(delta);
YFfl = Fyfl * cos(delta) + Fxfl * sin(delta);
XFfr = Fxfr * cos(delta) - Fyfr * sin(delta);
YFfr = Fyfr * cos(delta) + Fxfr * sin(delta);
XFr = Fxr * cos(0) - Fyr * sin(0);
YFr = Fyr * cos(0) + Fxr * sin(0);

%STEP5: Estimate states tbd

% Mt*(vx_dot + theta_dot*v_z - vy*psi_dot) = X*F_fl + X*F_fr + X*F_r;
% Mt*(vy_dot + psi_dot*v_x - phi_dot*v_z) = Y*F_fl + Y_fr + Y*F_r;
% Ms*(vz_dot + psi_dot*v_y - theta_do*v_x) = F*z_fl + F*z_fr + F*z_r;


end

