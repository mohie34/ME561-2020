function [U_global] = rotation(u_vehicle, rotation_angles)

% % Input:  u_vehicle, three element vector describing motion in vehicle's frame along x,y,z direction
% %         rotation_angles, three element vector describing row, pitch, yaw
% % Output: U_global, three element vector describing motion in global frame along x,y,z direction

%rotation angles
phi = rotation_angles(1);
theta = rotation_angles(2);
psi = rotation_angles(3);

%yaw, pitch, row
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];

U_global = Rz*Ry*Rx*u_vehicle;
end

