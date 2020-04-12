

close all
clear all
clc


v0 = [0.01;0.01;0.0;0.0;0.0;0.0];
T_step = 0.1;
input_vec = [100,1.57];


dynamics = @(t,X)(vehicle_dynamics(X(1),X(2),X(3),X(4),X(5),X(6),input_vec(1),input_vec(2)));
[Tsim, Ysim] = ode45(dynamics, 0:T_step:1000, v0);

subplot(3,3,1)
plot(Tsim,Ysim(:,1))
title("Vx")
subplot(3,3,2)
plot(Tsim,Ysim(:,2))
title("Vy")
subplot(3,3,3)
plot(Tsim,Ysim(:,3))
title("Vz")
subplot(3,3,4)
plot(Tsim,Ysim(:,4))
title("Phid")
subplot(3,3,5)
plot(Tsim,Ysim(:,5))
title("Thetad")
subplot(3,3,6)
plot(Tsim,Ysim(:,6))
title("Psid")

% [a,b,c,d,e,f] = vehicle_dynamics(0.01,0.01,0.01,0.01,0.01,0.01,100,0)