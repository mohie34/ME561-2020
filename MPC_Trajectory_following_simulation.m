
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


%================{TRAJECTORY INITS}=================%
global TRAJECTORY
generate_trajectory()
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
SIM.lims.delta = 0.4;
SIM.lims.Fx = 500;
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
CONTROL.Fx = 0;
CONTROL.delta = 0;
CONTROL.N = 20;
CONTROL.Ts = SIM.T_step;
CONTROL.R = 2*eye(2);
CONTROL.Qnl = eye(10);
CONTROL.nx = 10;
CONTROL.nu = 2;
CONTROL.U_MPC = [CONTROL.Fx, CONTROL.delta];
CONTROL.U_hori = repmat(CONTROL.U_MPC,CONTROL.N,1);
%===================================================%

%================{TRAJ_PLAN INITS}==================%
global TP
TP.waypoint = TRAJECTORY.start;
TP.waypoint_thresh = 0.75; % L2 Distance from trike to waypoint to update
TP.update_thresh = 3;      % L2 Distance between two separate waypoints
TP.trike_pos = DATA.X_curr(7:9);
TP.last_pos = false;    % Boolen to indicate we going to last wp
TP.pos_indx = 1;
TP.waypoints_completed = [];
%===================================================%

for t = DATA.T_vec
    
    % Involved with debug prints to be accessed in vehicle_dynamics
    SIM.t = t;
    DATA.T_sim = [DATA.T_sim, t];
    SIM.done=false;       % Boolean needed for debug prints
    update_waypoint();
    %======================================================================
    
    % Get Control Inputs    
    if t>50 && t<100 % From 50 - 100 sec, do right turn and see if speed is maintained
        CONTROL.delta = CONTROL.delta + 0.000;
    else
        CONTROL.delta = 0;
    end  
    if(SIM.limit_input)
        CONTROL.delta = max(min(CONTROL.delta,SIM.lims.delta),-SIM.lims.delta);      
        CONTROL.Fx = max(min(CONTROL.Fx,SIM.lims.Fx),-SIM.lims.Fx);      
    end
    CONTROL.Fx = 1.0;
    CONTROL.input_vec = [CONTROL.Fx, CONTROL.delta];
    %======================================================================
    
    % Simulate dynamics and build data vector
    dynamics = @(t,X)(vehicle_dynamics(X(1),X(2),X(3),X(4),X(5),X(6),X(9),X(10)...
                              ,CONTROL.input_vec(1),CONTROL.input_vec(2)));
    [T, Y] = ode45(dynamics, [t t+SIM.T_step], DATA.X_curr);
    DATA.X_curr = Y(end,:);
    DATA.X_sim = [DATA.X_sim; DATA.X_curr];
    DATA.U_sim = [DATA.U_sim; CONTROL.input_vec];
    TP.trike_pos = DATA.X_curr(7:9);
    
    if(SIM.plot_realtime)
%         plotdata_mpc()
        plot_only_trajectory()
    end
    %======================================================================
end

plot_only_trajectory()
plotdata_mpc() % Check plotdata.m
