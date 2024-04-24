function param = plant_param()
    % dimensions
    param.xdim = 4;
    param.udim = 2;

    % parameters
    param.m1 = 1; % mass (kg)
    param.d1 = 1; % damping coefficient (N.s/m)
    param.k1 = 1; % spring constant (N/m)
    param.m2 = 1; % mass (kg)
    param.d2 = 1; % damping coefficient (N.s/m)
    param.k2 = 1; % spring constant (N/m)
end
