function plotdata_mpc()
%Plots the states contained within the DATA global struct

global DATA
global TRAJECTORY

figure(1)
subplot(3,4,1)
plot(DATA.T_sim,DATA.X_sim(:,1)*2.23694,'b')
title("Vx [mph]")
subplot(3,4,2)
plot(DATA.T_sim,DATA.X_sim(:,2),'b')
title("Vy [m/s]")
subplot(3,4,3)
plot(DATA.T_sim,DATA.X_sim(:,3),'b')
title("Vz [m/s]")
subplot(3,4,4)
plot(DATA.T_sim,DATA.X_sim(:,4)*180/pi,'b')
title("Phid [deg/s]")
subplot(3,4,5)
plot(DATA.T_sim,DATA.X_sim(:,5)*180/pi,'b')
title("Thetad [deg/s]")
subplot(3,4,6)
plot(DATA.T_sim,DATA.X_sim(:,6)*180/pi,'b')
title("Psid [deg/s]")
subplot(3,4,7)
hold on
plot(DATA.X_sim(:,7),DATA.X_sim(:,8),'k','linewidth',1)
hold on
plot(TRAJECTORY.center_line(:,1),TRAJECTORY.center_line(:,2),'b')
hold on
plot(TRAJECTORY.bounds_left_lower(:,1),TRAJECTORY.bounds_left_lower(:,2), 'r')
hold on
plot(TRAJECTORY.bounds_left_upper(:,1),TRAJECTORY.bounds_left_upper(:,2), 'r')
hold on
plot(TRAJECTORY.bounds_right_lower(:,1),TRAJECTORY.bounds_right_lower(:,2), 'r')
hold on
plot(TRAJECTORY.bounds_right_upper(:,1),TRAJECTORY.bounds_right_upper(:,2), 'r')
title("Trajectory X(t) vs Y(t)")
axis equal
subplot(3,4,8)
plot(DATA.T_sim,DATA.X_sim(:,9)*180/pi,'k')
title("Trike Roll Phi [deg]")
subplot(3,4,9)
plot(DATA.T_sim,DATA.X_sim(:,10)*180/pi,'k')
title("Trike Yaw Psi [deg]")
subplot(3,4,10)
plot(DATA.T_sim,DATA.U_sim(1:2:end-1),'r')
title("Fxr")
subplot(3,4,11)
plot(DATA.T_sim,DATA.U_sim(2:2:end)*180/pi,'r')
title("delta [deg]")
subplot(3,4,12)
plot(DATA.T_sim,DATA.T_comp*1000,'b')
title("Computation Time [ms]")

end

