function cost = nonl_cf(x0,u,ref)


    global CONTROL
    
    x = nonl_euler_dynamics(x0,u,ref);
    Qbar =   kron(CONTROL.Qnl,eye(CONTROL.N));
    Rbar =   kron(CONTROL.R,eye(CONTROL.N));
    cost = x'*Qbar*x + u'*Rbar * u ; %May want to add cost on u later.
end