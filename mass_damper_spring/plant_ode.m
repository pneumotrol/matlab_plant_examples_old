function dxdt = plant_ode(~,x,u,param)
    m = param.m;
    d = param.d;
    k = param.k;

    dxdt = [
        x(2);
        (-k*x(1)-d*x(2)+u(1))/m;
        ];
end
