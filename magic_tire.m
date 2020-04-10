function [Fx, Fy] = magic_tire(alpha, Fz)
%%This function calculates longitudinal and lateral force

%Hard coded coefficients
Bx = 10; Cx = 1.3; Dx = 1; Ex = 0.97;
By = 1; Cy = 2; Dy = 3; Ey = 4;

%Magic tire formula
Fx = Fz*Dx*sin(Cx*atan(Bx*(1-Ex)*alpha+Ex*atan(Bx*alpha)));
Fy = Fz*Dy*sin(Cy*atan(By*(1-Ey)*alpha+Ey*atan(By*alpha)));

end

