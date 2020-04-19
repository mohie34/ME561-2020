function xout = nonl_euler_dynamics(x0,u,ref)

    global CONTROL

    xfwd = x0;
    xout = zeros(CONTROL.N*CONTROL.nx_costly,1);
    for i = 1:CONTROL.N
        uin = u(i*CONTROL.nu-1:i*CONTROL.nu);
        dx = vehicle_dynamics_mpc(xfwd, uin);
        xfwd = xfwd+dx'*CONTROL.Ts;
        xerr = xfwd(7:8) - ref;   
        xout(CONTROL.nx_costly*i-1:CONTROL.nx_costly*i) = xerr;
    end
    
end


