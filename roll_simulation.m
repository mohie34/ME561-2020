
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% MOHIE AMINE 65201212
% TIANYI ZHANG <ID>
%
% CODE FOR SIMULATING CRUISE CONTROL MANEUVERS ON A TRIKE VEHICLE
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all
clc

%================{SIM INITS}==================%
global SIM
SIM.debug = false;
SIM.steered = false;
SIM.enable_roll = true;
SIM.enable_pitch = false;
SIM.T_step = 0.1;
SIM.Tsim = 70;  %Sim time in seconds
SIM.x0 = [0;0;0;0;0;0;0;0;0;0]';
SIM.lims.delta = 0.4;
SIM.lims.Fx = 500;
%==============================================%

%==============={DATA INITS}===================%
global DATA
DATA.X_sim = [];
DATA.U_sim = [];
DATA.X_curr = SIM.x0 ;
DATA.T_vec = 0:SIM.T_step:SIM.Tsim-SIM.T_step;
%==============================================%

%=============={CONTROL INITS}=================%
global CONTROL
CONTROL.input_vec = [0, 0];
%==============================================%

for t = DATA.T_vec
    SIM.t = t;
    SIM.done = false;
    if t>50
        if ~SIM.steered
            CONTROL.input_vec = CONTROL.input_vec + [0.01, 0.01];
%             SIM.steered = true;
        else
            CONTROL.input_vec(2) = 0;
            CONTROL.input_vec = CONTROL.input_vec + [0.1, 0.0];
        end
    else
        CONTROL.input_vec = CONTROL.input_vec + [0.035, 0.0];
    end
    CONTROL.input_vec(2) = max(-SIM.lims.delta ,min(CONTROL.input_vec(2),SIM.lims.delta));
    if CONTROL.input_vec(1) > 20
        stop = true;
    end
    dynamics = @(t,X)(vehicle_dynamics(X(1),X(2),X(3),X(4),X(5),X(6),X(9),X(10),CONTROL.input_vec(1),CONTROL.input_vec(2)));
    [T, Y] = ode45(dynamics, [t t+SIM.T_step], DATA.X_curr);
    DATA.X_curr = Y(end,:);
    DATA.X_sim = [DATA.X_sim; DATA.X_curr];
    DATA.U_sim = [DATA.U_sim; CONTROL.input_vec];
end

plotdata()
