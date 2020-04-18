
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
DATA.ref_sim = [];
DATA.X_curr = SIM.x0 ;
DATA.T_vec = 0:SIM.T_step:SIM.Tsim-SIM.T_step;
%==============================================%

%=============={CONTROL INITS}=================%
global CONTROL
CONTROL.ref_vx = 4;
CONTROL.Fx = 0;
CONTROL.delta = 0;
%==============================================%
global TRAJECTORY
traj_gen_pid_control();

for t = DATA.T_vec
    % Involved with debug prints to be accessed in vehicle_dynamics
    SIM.t = t;
    SIM.done = false;
    %======================================================================
    
    %find goal point
    goal_point = find_nearest_goal_point(TRAJECTORY.change_lane);
    disp("goal_point");
    disp(goal_point);
    % Get Control Inputs
    get_lane_keeping(goal_point);
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
    %======================================================================
end

plotdata() % Check plotdata.m
