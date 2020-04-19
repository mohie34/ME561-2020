function [h, heq] = nonl_bound_func(x0, u, ulim)
        
        global CONTROL
        h = [u-repmat(ulim,CONTROL.N,1);-u-repmat(ulim,CONTROL.N,1);...
            ]; %(u(2:end)-u(1:end-1))/MPC.Ts
        heq = [];
end