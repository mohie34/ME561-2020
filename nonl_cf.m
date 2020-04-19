function cost = nonl_cf(x0,u,ref)


    global CONTROL

    x = nonl_euler_dynamics(x0,u,ref);
    x = x(CONTROL.nx+1:end);
% 
%     for i=1:MPC.nx:MPC.nx*(MPC.N-2)+1
%             Qbar(i:i+MPC.nx-1,i:i+MPC.nx-1) = MPC.Qnl;
%     end
%         Qbar(MPC.nx*(MPC.N-1)+1:MPC.nx*MPC.N,MPC.nx*(MPC.N-1)+1:MPC.nx*MPC.N) = MPC.Qnl;  %Was MPC.P

    Qbar =   kron(CONTROL.Qnl,eye(CONTROL.N));
    cost = x'*Qbar*x; %May want to add cost on u later.

end