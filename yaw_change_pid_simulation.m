
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% MOHIE AMINE 65201212
% TIANYI ZHANG 83716362
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
SIM.enable_roll = false;
SIM.enable_pitch = false;
SIM.limit_input = true;
SIM.T_step = 0.1;
SIM.Tsim = 100;  %Sim time in seconds
SIM.x0 = [0;0;0;0;0;0;0;0;0;0]';
SIM.lims.delta = 0.8;
SIM.lims.Fx = 500;
%==============================================%

%==============={DATA INITS}===================%
global DATA
DATA.X_sim = [];
DATA.U_sim = [];
DATA.T_sim = [];
DATA.ref_psi = [];
DATA.X_curr = SIM.x0 ;
DATA.T_vec = 0:SIM.T_step:SIM.Tsim-SIM.T_step;
%==============================================%

%=============={CONTROL INITS}=================%
global CONTROL
CONTROL.ref_vx = 2;
CONTROL.ref_psi = 0;
CONTROL.Fx = 0;
CONTROL.delta = 0;
CONTROL.init_cruise = false;
CONTROL.init_lanechange = false;
%==============================================%

for t = DATA.T_vec
    % Involved with debug prints to be accessed in vehicle_dynamics
    SIM.t = t;
    DATA.T_sim = [DATA.T_sim , t];
    SIM.done = false;
    %======================================================================
    % Get Control Inputs
    getcruise_control(50,1,1)
    get_yaw_change(0.45,0.004,0.0002)
    
    if(t>50)
        CONTROL.ref_psi = pi/2;
    end
    if(SIM.limit_input)
        CONTROL.delta = max(min(CONTROL.delta,SIM.lims.delta),-SIM.lims.delta);      
        CONTROL.Fx = max(min(CONTROL.Fx,SIM.lims.Fx),-SIM.lims.Fx);      
    end
    CONTROL.input_vec = [CONTROL.Fx, CONTROL.delta];
    %======================================================================
    
    % Simulate dynamics and build data vector
    dynamics = @(t,X)(vehicle_dynamics(X(1),X(2),X(3),X(4),X(5),X(6),X(9),X(10),CONTROL.input_vec(1),CONTROL.input_vec(2)));
    [T, Y] = ode45(dynamics, [t t+SIM.T_step], DATA.X_curr);
    DATA.X_curr = Y(end,:);
    DATA.X_sim = [DATA.X_sim; DATA.X_curr];
    DATA.U_sim = [DATA.U_sim; CONTROL.input_vec];
    DATA.ref_psi = [DATA.ref_psi, CONTROL.ref_psi];
    %======================================================================
end

plotdata() % Check plotdata.m
