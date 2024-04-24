function dxdt = plant_ode(~,x,u,param)
    g = param.g;
    A1 = param.A1;
    a1 = param.a1;
    A2 = param.A2;
    a2 = param.a2;

    if x(1) < 0
        x(1) = 0;
    end

    if x(2) < 0
        x(2) = 0;
    end

    dxdt = [
        (-a1*sqrt(2*g*x(1))+u(1))/A1;
        (a1*sqrt(2*g*x(1))-a2*sqrt(2*g*x(2)))/A2;
        ];
end
