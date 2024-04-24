function dxdt = plant_ode(~,x,u,param)
    g = param.g;
    m = param.m;
    l = param.l;

    dxdt = [
        x(2);
        -(g/l)*sin(x(1))+(1/(m*l^2))*u(1);
        ];
end
