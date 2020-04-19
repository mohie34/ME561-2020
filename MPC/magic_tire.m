function [Fy] = magic_tire(alpha, Fz)
%%This function calculates lateral forces on the tires
By = 0.27; Cy = 1.2; Dy = 0.7; Ey = -1.6;

Fy = Fz*Dy*sin(Cy*atan(By*(1-Ey)*alpha+Ey*atan(By*alpha)));

end

