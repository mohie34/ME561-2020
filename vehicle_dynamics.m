function [states] = vehicle_dynamics(states_prev, inpt)
%Write vehicle dynamics from the proposal's paper here. Pages 4-5-6

% Mx = ...
% My = ...
% Mz = ...

% For our trike, we follow fig 1-c p.4, Tr = 0.
% Follow eq 19 - 27

% F_i = tire force. 
% Mt = total mass
% Ms = Wheel moment of inertia

% use eq 7 to get X*F_fl, X*F_fr, and X*F_r

% Things we have at each iteration
% Constants = Mt, and Ms
%             Isxx, Isyy, Iszz
%             Tf, and Tr
%             [IMPORTANT] h = height of COG Affects PHI for roll
%             l_r and l_f = COG location on x-axis to rear and front
% Control input = delta and T
% States = vx, vy, vz and their dot
%          psi, theta, phi and their dots
%
% Need to calculate accelaration (double dot) for these states at each 
% iteration

Mt*(vx_dot + theta_dot*v_z - vy*psi_dot) = X*F_fl + X*F_fr + X*F_r;
Mt*(vy_dot + psi_dot*v_x - phi_dot*v_z) = Y*F_fl + Y_fr + Y*F_r;
Ms*(vz_dot + psi_dot*v_y - theta_do*v_x) = F*z_fl + F*z_fr + F*z_r;


end

