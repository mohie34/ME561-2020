

close all
clear all
clc


global SIM
SIM.debug = false;
SIM.steered = false;

x0 = [0.01;0.0;0.0;0.0;0.0;0.0]';
T_step = 0.1;
Tsim = 200;  %Sim time in seconds
T_vec = 0:T_step:Tsim-T_step;
input_vec = [0, 0];
stop = false;

X_sim = [];
U_sim = [];
X_curr = x0;

for t = T_vec
    SIM.t = t;
    SIM.done = false;
    if t>100
        if ~SIM.steered
            input_vec = input_vec + [0.1, 0.1];
            SIM.steered = true;
        else
            input_vec(2) = 0;
            input_vec = input_vec + [0.1, 0.0];
        end
    else
        input_vec = input_vec + [0.1, 0.0];
    end
    if input_vec(1) > 20
        stop = true;
    end
    if stop
        input_vec(1) = 0;
    end
    dynamics = @(t,X)(vehicle_dynamics(X(1),X(2),X(3),X(4),X(5),X(6),input_vec(1),input_vec(2)));
    [Tsim, Ysim] = ode45(dynamics, [t t+T_step], X_curr);
    X_curr = Ysim(end,:);
    X_sim = [X_sim; X_curr];
    U_sim = [U_sim; input_vec];
end


subplot(2,4,1)
plot(T_vec,X_sim(:,1))
title("Vx")
subplot(2,4,2)
plot(T_vec,X_sim(:,2))
title("Vy")
subplot(2,4,3)
plot(T_vec,X_sim(:,3))
title("Vz")
subplot(2,4,4)
plot(T_vec,X_sim(:,4))
title("Phid")
subplot(2,4,5)
plot(T_vec,X_sim(:,5))
title("Thetad")
subplot(2,4,6)
plot(T_vec,X_sim(:,6))
title("Psid")
subplot(2,4,7)
plot(T_vec,U_sim(:,1),'r')
title("Fxr")
subplot(2,4,8)
plot(T_vec,U_sim(:,2),'r')
title("delta")
% [a,b,c,d,e,f] = vehicle_dynamics(0.01,0.01,0.01,0.01,0.01,0.01,100,0)