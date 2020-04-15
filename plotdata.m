function plotdata()
%Plots the states contained within the DATA global struct

global DATA
global CONTROL

subplot(3,4,1)
plot(DATA.T_vec,DATA.X_sim(:,1)*2.23694)
title("Vx [mph]")
subplot(3,4,2)
plot(DATA.T_vec,DATA.X_sim(:,2))
title("Vy [m/s]")
subplot(3,4,3)
plot(DATA.T_vec,DATA.X_sim(:,3))
title("Vz [m/s]")
subplot(3,4,4)
plot(DATA.T_vec,DATA.X_sim(:,4)*180/pi)
title("Phid [deg/s]")
subplot(3,4,5)
plot(DATA.T_vec,DATA.X_sim(:,5)*180/pi)
title("Thetad [deg/s]")
subplot(3,4,6)
plot(DATA.T_vec,DATA.X_sim(:,6)*180/pi)
title("Psid [deg/s]")
subplot(3,4,7)
plot(DATA.X_sim(:,7),DATA.X_sim(:,8),'k')
title("Trajectory X(t) vs Y(t)")
axis equal
subplot(3,4,8)
plot(DATA.T_vec,DATA.X_sim(:,9)*180/pi,'k')
title("Trike Roll Phi [deg]")
subplot(3,4,9)
plot(DATA.T_vec,DATA.X_sim(:,10)*180/pi,'k')
title("Trike Yaw Psi [deg]")
subplot(3,4,10)
plot(DATA.T_vec,DATA.U_sim(:,1),'r')
title("Fxr")
subplot(3,4,11)
plot(DATA.T_vec,DATA.U_sim(:,2)*180/pi,'r')
title("delta [deg]")

end

