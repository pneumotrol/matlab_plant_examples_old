function sysc = plant_sysc(param,option)
    g = param.g;
    M = param.M;
    D = param.D;
    m = param.m;
    J = param.J;
    lg = param.lg;
    d = param.d;

    switch option.mode
        case {"bottom"}
            % equilibrium point
            sysc.xe = [0;0;0;0];
            sysc.ue = 0;

            % coefficients of state equation
            sysc.A = [
                0,0,1,0;
                0,0,0,1;
                0,(g*lg^2*m^2)/(J*M+J*m+lg^2*M*m),-(D*(J+lg^2*m))/(J*M+J*m+lg^2*M*m),(d*lg*m)/(J*M+J*m+lg^2*M*m);
                0,-(g*lg*m*(M+m))/(J*M+J*m+lg^2*M*m),(D*lg*m)/(J*M+J*m+lg^2*M*m),-(d*(M+m))/(J*M+J*m+lg^2*M*m);
                ];

            sysc.B = [
                0;
                0;
                (J+lg^2*m)/(J*M+J*m+lg^2*M*m);
                -(lg*m)/(J*M+J*m+lg^2*M*m);
                ];

            sysc.C = eye(4);

            sysc.D = zeros(4,1);
        case {"top"}
            % equilibrium point
            sysc.xe = [0;pi;0;0];
            sysc.ue = 0;

            % coefficients of state equation
            sysc.A = [
                0,0,1,0;
                0,0,0,1;
                0,(g*lg^2*m^2)/(J*M+J*m+lg^2*M*m),-(D*(J+lg^2*m))/(J*M+J*m+lg^2*M*m),-(d*lg*m)/(J*M+J*m+lg^2*M*m);
                0,(g*lg*m*(M+m))/(J*M+J*m+lg^2*M*m),-(D*lg*m)/(J*M+J*m+lg^2*M*m),-(d*(M+m))/(J*M+J*m+lg^2*M*m);
                ];

            sysc.B = [
                0;
                0;
                (J+lg^2*m)/(J*M+J*m+lg^2*M*m);
                (lg*m)/(J*M+J*m+lg^2*M*m);
                ];

            sysc.C = eye(4);

            sysc.D = zeros(4,1);
    end
end
