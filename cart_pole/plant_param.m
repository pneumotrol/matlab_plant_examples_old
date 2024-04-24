function param = plant_param()
    % dimensions
    param.xdim = 4;
    param.udim = 1;

    % parameters
    % Control Tutorials for MATLAB and Simulink
    % https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
    param.g = 9.8; % gravitational acceleration (m/s^2)
    param.M = 0.5; % mass of cart (kg)
    param.D = 0.1; % damping coefficent of cart (N.s/m)
    param.m = 0.2; % mass of pole (kg)
    param.J = 0.006; % moments of inertia of pole (kg.m^2)
    param.lg = 0.3; % length from joint to center of mass (m)
    param.d = 0.0; % damping coefficent of pole (N.s/m)
end
