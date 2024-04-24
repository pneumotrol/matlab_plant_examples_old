function dxdt = plant_ode(~,x,u,param)
    m = param.m;
    d = param.d;

    dxdt = [
        x(2);
        (-d*x(2)+u(1))/m;
        ];
end
