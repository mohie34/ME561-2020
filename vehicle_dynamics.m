
function [Xdot] ...
        = vehicle_dynamics(vx, vy, vz,...
                           phid, thetad, psid,...
                           psi, Fx_r, delta)


global SIM

% function [vxd, vyd, vzd,...
%           phidd, thetadd, psidd] ...
%         = vehicle_dynamics(vx, vy, vz,...
%                            phid, thetad, psid,...
%                            Fx_r, delta)

% % states:           vx, vy, vz, vxd, vyd, vzd, phi, theta, psi, phid, thetad, psid
% %                   vx, velocity in x direction
% %                   vxd, derivative of vx
% %                   phi, theta, psi are rotation angle in x, y, z direction respectively              
% % input:            torque, rear wheel torque
% %                   delta, front wheel steering angle

% Hard coded parameters
g = 9.81;           % m/s^2 [FINAL]
h = 0.4;            % height of COG Affects PHI for roll [EXPERMINTAL]
Tf = 0.86;          % distance between front wheel [m] [FINAL]
Ms = 80; Mt = 89;   % vehicle sprung mass / total mass 
r = 0.40; % [   | ]  % Ratio of front wheel to COG from total vehicle length
l_tot = 1.8;        % Need to be final
lf = r*l_tot; lr = (1-r)*l_tot; % distance in x direction: front/rear wheel axis to COG
Isxx = 800; Isyy = 1000; Iszz = 2000;  % Inertia to be estimated [EXPERIMENTAL]
                                     % TREAT IT AS A CUBE AND GET ROUGHT Inertai's

%STEP1: Calculate Slip Angle (eq.25)
alpha_fl = atan2((vy+lf*psid),(vx+Tf*psid/2))-delta; %front left
alpha_fr = atan2((vy+lf*psid),(vx+Tf*psid/2))-delta; %front right
alpha_r = atan2((vy-lr*psid),vx); %rear


alpha_fl = -alpha_fl;
alpha_fr = -alpha_fr;
alpha_r = -alpha_r;


%STEP2: Calculate Fzf and Fzr (reference: Rob535 HW2)
Fzf = lf*Ms*g/(lf+lr)/2;
Fzr = lr*Ms*g/(lf+lr);

%STEP3: Use magic tire to calculate Fx and Fy for each tire
Fxfl = 0;
Fyfl = magic_tire(alpha_fl, Fzf);
Fxfr = 0;
Fyfr = magic_tire(alpha_fr, Fzf);
Fyr = magic_tire(alpha_r, Fzr);

%STEP4: Calculate XF and XY for each tire (eq.7, 8)
XFfl = Fxfl * cos(delta) - Fyfl * sin(delta);
YFfl = Fyfl * cos(delta) + Fxfl * sin(delta);
XFfr = Fxfr * cos(delta) - Fyfr * sin(delta);
YFfr = Fyfr * cos(delta) + Fxfr * sin(delta);
XFr = Fx_r;
YFr = Fyr;

%STEP5a: Calculate double dot of phi, theta, psi (eq.22, 23, 24)
phidd = (1/Isxx)*((Isyy-Iszz)*thetad*psid-(YFfl+YFfr+YFr)*h);
thetadd = (1/Isyy)*((Iszz-Isxx)*thetad*psid+Fzr*lr-Fzf*lf+(XFfl+XFfr+XFr)*h);
psidd = (1/Iszz)*((Isxx-Isyy)*phid*thetad+(XFfl-XFfr)*Tf/2+(YFfl+YFfr)*lf-YFr*lr);  % to add self-aligning torque Mzi
thetadd = 0;
phidd = 0;
%STEP5b: Calculate dot of vx, vy, vz (eq.19, 20, 21)
vxd = (XFfl+XFfr+XFr)/Mt-thetad*vz+psid*vy;
vyd = (YFfl+YFfr+YFr)/Mt-psid*vx+phid*vz;
vzd = (Fzf+Fzr)/Ms-phid*vy+thetad*vx;
vzd = 0;

if(SIM.t > 99 && ~SIM.done && SIM.debug)
    SIM.done = true;
    disp("===============")
    disp("delta:")
    disp(delta)
    disp("vx:")
    disp(vx)
    disp("vy:")
    disp(vy)
    disp("psid:")
    disp(psid)
    disp("phid:")
    disp(phid)
    disp("Fyfl:")
    disp(Fyfl)
    disp("Fyfr:")
    disp(Fyfr)
    disp("alpha_r:")
    disp(alpha_r)
    disp("alpha_fl:")
    disp(alpha_fl)
    disp("t:")
    disp(SIM.t)
    pause(0.5)
    disp("(YFfl+YFfr)*lf")
    disp((YFfl+YFfr)*lf)
    disp("YFr*lr")
    disp(YFr*lr)
    disp("Fzr")
    disp(Fzr)
    disp("Fzf")
    disp(Fzf)
    disp("=====END======")
end

Xdot = zeros(6,1);
Xdot(1) = vxd;
Xdot(2) = vyd;
Xdot(3) = vzd;
Xdot(4) = phidd;
Xdot(5) = thetadd;
Xdot(6) = psidd;
Xdot(7) = vx*cos(psi) - vy*sin(psi);
Xdot(8) = vx*sin(psi) + vy*cos(psi);
Xdot(9) = psid;
end

