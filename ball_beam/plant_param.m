function param = plant_param()
    % dimensions
    param.xdim = 4;
    param.udim = 1;

    % parameters
    % Control Tutorials for MATLAB and Simulink
    % https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
    param.g = 9.8; % gravitational acceleration (m/s^2)
    param.m = 0.11; % mass of ball (kg)
    param.J = 9.99e-6; % moments of inertia of ball (kg.m^2)
    param.r = 0.015; % radius of ball (m)
end
