
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% MOHIE AMINE 65201212
% TIANYI ZHANG <ID>
%
% CODE FOR PATH FOLLOWING MPC CONTROL ON A RECUMBENT TRIKE VEHICLE
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all
clc

%================{TRAJECTORY INITS}=================%
global TRAJECTORY
GEN_TRAJECTORY()
%===================================================%

%==================={SIM INITS}=====================%
global SIM
SIM.debug = false;
SIM.steered = false;
SIM.enable_roll = false;
SIM.enable_pitch = false;
SIM.limit_input = true;
SIM.T_step = 0.1;
SIM.Tsim = 150;  %Sim time in seconds
SIM.x0 = [0;0;0;0;0;0;TRAJECTORY.start';0;...
            TRAJECTORY.init_heading]';
SIM.lims.delta = 0.8; %45 degrees
SIM.lims.Fx = 50;
SIM.Ulim = [SIM.lims.Fx; SIM.lims.delta];
SIM.plot_realtime = true;
%===================================================%

%==================={DATA INITS}====================%
global DATA
DATA.X_sim = [];
DATA.U_sim = [];
DATA.T_sim = [];
DATA.ref_sim = [];
DATA.X_curr = SIM.x0 ;
DATA.T_vec = 0:SIM.T_step:SIM.Tsim-SIM.T_step;
%===================================================%

%================={CONTROL INITS}===================%
global CONTROL
CONTROL.ref_vx = 4.5;
CONTROL.Fx = 25;
CONTROL.delta = 0.01;
CONTROL.N = 20;
CONTROL.Ts = SIM.T_step;
CONTROL.R = eye(2);
CONTROL.R(1) = 0.005;
CONTROL.nx_costly = 2;
CONTROL.Qnl = 20*eye(CONTROL.nx_costly);
CONTROL.nu = 2;
CONTROL.U = [CONTROL.Fx; CONTROL.delta];
CONTROL.U_hori = repmat(CONTROL.U,CONTROL.N,1);
%===================================================%

%================{TRAJ_PLAN INITS}==================%
global TP
TP.waypoint = TRAJECTORY.start;
TP.waypoint_thresh = 0.5; % L2 Distance from trike to waypoint to update
TP.update_thresh = 1.5;      % L2 Distance between two separate waypoints
TP.trike_pos = DATA.X_curr(7:9);
TP.last_pos = false;    % Boolen to indicate we going to last wp
TP.pos_indx = 1;
TP.waypoints_completed = [];
%===================================================%

for t = DATA.T_vec
    
    UPDATE_WAYPOINT();
    RUN_MPC_ITERATION();
    SIMULATE_TRIKE(t);
    
    if(SIM.plot_realtime)
        plot_only_trajectory()
    end
%     disp(CONTROL.U)
end

plot_only_trajectory()
plotdata_mpc() % Check plotdata.m
