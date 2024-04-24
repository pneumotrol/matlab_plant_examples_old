function sysc = plant_sysc(param,~)
    m1 = param.m1;
    d1 = param.d1;
    k1 = param.k1;
    m2 = param.m2;
    d2 = param.d2;
    k2 = param.k2;

    % equilibrium point
    sysc.xe = [0;0;0;0];
    sysc.ue = [0;0];

    % coefficients of state equation
    sysc.A = [
        0,0,1,0;
        0,0,0,1;
        -(k1+k2)/m1,k2/m1,-(d1+d2)/m1,d2/m1;
        k2/m2,-k2/m2,d2/m2,-d2/m2;
        ];

    sysc.B = [
        0,0;
        0,0;
        (1/m1),0;
        0,(1/m2);
        ];

    sysc.C = eye(4);

    sysc.D = zeros(4,2);
end
