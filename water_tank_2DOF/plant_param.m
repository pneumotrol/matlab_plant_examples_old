function param = plant_param()
    % dimensions
    param.xdim = 2;
    param.udim = 1;

    % parameters
    param.g = 9.81; % gravitational acceleration (m/s^2)
    param.A1 = 1; % cross-sectional area of tank (m^2)
    param.a1 = 0.1; % cross-sectional area of outflow (m^2)
    param.A2 = 1; % cross-sectional area of tank (m^2)
    param.a2 = 0.1; % cross-sectional area of outflow (m^2)
end
