function dxdt = plant_ode(~,x,u,param)
    g = param.g;
    m = param.m;
    J = param.J;
    r = param.r;

    dxdt = [
        x(3);
        x(4);
        m*x(1)*x(4)^2/(m+J/(r^2))-m*g*sin(x(2))/(m+J/(r^2));
        -2*x(3)*x(4)/x(1)-g*cos(x(2))/x(1)+u(1)/(m*x(1)^2);
        ];
end
