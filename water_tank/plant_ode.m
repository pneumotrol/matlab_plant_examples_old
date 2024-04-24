function dxdt = plant_ode(~,x,u,param)
    g = param.g;
    A = param.A;
    a = param.a;

    if x(1) < 0
        x(1) = 0;
    end

    dxdt = (-a*sqrt(2*g*x(1))+u(1))/A;
end
