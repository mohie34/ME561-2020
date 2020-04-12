function [Fy] = magic_tire(alpha, Fz)
%%This function calculates longitudinal and lateral force
global SIM
%Hard coded coefficients
By = 10; Cy = 1.9; Dy = 1; Ey = 0.97;

%Magic tire formula
if(SIM.t > 99 && ~SIM.done && SIM.debug)
    disp("Magic inner Atan entry:")
    disp(By*alpha)
    disp("Magic outer Atan entry:")
    disp(By*(1-Ey)*alpha+Ey*atan(By*alpha))
    disp("Magic sin output")
    disp(sin(Cy*atan(By*(1-Ey)*alpha+Ey*atan(By*alpha))))
end

Fy = Fz*Dy*sin(Cy*atan(By*(1-Ey)*alpha+Ey*atan(By*alpha)));

end

