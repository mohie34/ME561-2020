
function [Xdot] = vehicle_dynamics_mpc(X,U)

    global SIM

    % % states:           vx, vy, vz, vxd, vyd, vzd, phi, theta, psi, phid, thetad, psid
    % %                   vx, velocity in x direction
    % %                   vxd, derivative of vx
    % %                   phi, theta, psi are rotation angle in x, y, z direction respectively              
    vx = X(1);
    vy = X(2);
    vz = X(3);
    phid = X(4);
    thetad = X(5);
    psid = X(6);
    phi = X(9);
    psi = X(10);

    % % input:            torque, rear wheel torque
    % %                   delta, front wheels steering angle
    Fx_r = U(1);
    delta = U(2);

    % Hard coded parameters
    g = 9.81;           % m/s^2 [FINAL]
    h = 1.1;            % height of COG Affects PHI for roll [EXPERMINTAL]
    Tf = 0.86;          % distance between front wheel [m] [FINAL]
    Ms = 80; Mt = 89;   % vehicle sprung mass / total mass 
    r = 0.40; % [   | ]  % Ratio of front wheel to COG from total vehicle length
    l_tot = 1.8;        % Need to be final
    lf = r*l_tot; lr = (1-r)*l_tot; % distance in x direction: front/rear wheel axis to COG
    Isxx = 50; Isyy = 400; Iszz = 400;  % Inertia to be estimated [EXPERIMENTAL]
                                         % TREAT IT AS A CUBE AND GET ROUGHT Inertai's

    %STEP1: Calculate Slip Angle (eq.25)
    alpha_fl = atan2((vy+lf*psid),(vx+Tf*psid/2))-delta; %front left
    alpha_fr = atan2((vy+lf*psid),(vx+Tf*psid/2))-delta; %front right
    alpha_r = atan2((vy-lr*psid),vx); %rear


    alpha_fl = -alpha_fl;
    alpha_fr = -alpha_fr;
    alpha_r = -alpha_r;


    %STEP2: Calculate Fzf and Fzr (reference: Rob535 HW2)
    Fzr = lr*Ms*g/(lf+lr);
    Fzf = lf*Ms*g/(lf+lr);
    Fzfl = lf*Ms*g/(lf+lr)/2;
    Fzfr = lf*Ms*g/(lf+lr)/2;
    if abs(phi)<0.01
        Fz_balance_rate = phi/0.01;
        Fzfr = Fzfr*(1+Fz_balance_rate);
        Fzfl = Fzfl*(1-Fz_balance_rate);
    elseif phi<0
        disp("right wheel ungrounded")
        Fzfl = Fzf*cos(phi);
        Fzfr = 0;
    elseif phi > 0
        disp("left wheel ungrounded")
        Fzfl = 0;
        Fzfr = Fzf*cos(phi);
    end

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
    phidd = (1/Isxx)*((Isyy-Iszz)*thetad*psid+(Fzfl-Fzfr)*Tf/2-(YFfl+YFfr+YFr)*h);
    thetadd = (1/Isyy)*((Iszz-Isxx)*thetad*psid+Fzr*lr-Fzf*lf+(XFfl+XFfr+XFr)*h);
    psidd = (1/Iszz)*((Isxx-Isyy)*phid*thetad+(XFfl-XFfr)*Tf/2+(YFfl+YFfr)*lf-YFr*lr);  % to add self-aligning torque Mzi

    if ~SIM.enable_pitch
        thetadd = 0;
    end
    if ~SIM.enable_roll
        phidd = 0;
    end

    %STEP5b: Calculate dot of vx, vy, vz (eq.19, 20, 21)
    vxd = (XFfl+XFfr+XFr)/Mt-thetad*vz+psid*vy;
    vyd = (YFfl+YFfr+YFr)/Mt-psid*vx+phid*vz;
    vzd = (Fzf+Fzr)/Ms-phid*vy+thetad*vx;
    vzd = 0;

    Xdot = zeros(6,1);
    Xdot(1) = vxd;                       % vx
    Xdot(2) = vyd;                       % vy
    Xdot(3) = vzd;                       % vz
    Xdot(4) = phidd;                     % phid
    Xdot(5) = thetadd;                   % thetad
    Xdot(6) = psidd;                     % psid
    Xdot(7) = vx*cos(psi) - vy*sin(psi); % X
    Xdot(8) = vx*sin(psi) + vy*cos(psi); % Y
    Xdot(9) = phid;                      % phi
    Xdot(10) = psid;                     % psi

end

