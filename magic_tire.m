function [Fy] = magic_tire(alpha, Fz)
%%This function calculates longitudinal and lateral force

%Hard coded coefficients
By = 10; Cy = 1.3; Dy = 1; Ey = 0.97;

%Magic tire formula
Fy = Fz*Dy*sin(Cy*atan(By*(1-Ey)*alpha+Ey*atan(By*alpha)));

end

