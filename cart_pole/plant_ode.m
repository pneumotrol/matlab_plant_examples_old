function dxdt = plant_ode(~,x,u,param)
    g = param.g;
    M = param.M;
    D = param.D;
    m = param.m;
    J = param.J;
    lg = param.lg;
    d = param.d;

    % q1= x(1);
    q2 = x(2);
    dq1 = x(3);
    dq2 = x(4);
    u1 = u(1);

    dxdt = [
        dq1;
        dq2;
        (J*u1+lg^2*m*u1-J*D*dq1-D*dq1*lg^2*m+dq2^2*lg^3*m^2*sin(q2)+g*lg^2*m^2*cos(q2)*sin(q2)+J*dq2^2*lg*m*sin(q2)+d*dq2*lg*m*cos(q2))/(J*M+J*m+lg^2*m^2+lg^2*M*m-lg^2*m^2*cos(q2)^2);
        -(d*dq2*M+d*dq2*m+lg*m*u1*cos(q2)+g*lg*m^2*sin(q2)+dq2^2*lg^2*m^2*cos(q2)*sin(q2)-D*dq1*lg*m*cos(q2)+g*lg*M*m*sin(q2))/(J*M+J*m+lg^2*m^2+lg^2*M*m-lg^2*m^2*cos(q2)^2);
        ];
end
