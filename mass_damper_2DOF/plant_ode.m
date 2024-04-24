function dxdt = plant_ode(~,x,u,param)
    m1 = param.m1;
    d1 = param.d1;
    m2 = param.m2;
    d2 = param.d2;

    dxdt = [
        x(3);
        x(4);
        (-d1*x(3)-d2*(x(3)-x(4))+u(1))/m1;
        (d2*x(3)-d2*x(4)+u(2))/m2;
        ];
end
