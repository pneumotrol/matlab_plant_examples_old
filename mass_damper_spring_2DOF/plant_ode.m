function dxdt = plant_ode(~,x,u,param)
    m1 = param.m1;
    d1 = param.d1;
    k1 = param.k1;
    m2 = param.m2;
    d2 = param.d2;
    k2 = param.k2;

    dxdt = [
        x(3);
        x(4);
        (-k1*x(1)-d1*x(3)-k2*(x(1)-x(2))-d2*(x(3)-x(4))+u(1))/m1;
        (k2*x(1)+d2*x(3)-k2*x(2)-d2*x(4)+u(2))/m2;
        ];
end
