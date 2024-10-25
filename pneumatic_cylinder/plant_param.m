function param = plant_param()
    % dimensions
    param.xdim = 4;
    param.udim = 4;

    % parameters
    param.R = ANR().R; % specific gas constant (J/Kg.K)
    param.k = ANR().k; % heat capacity ratio (-)
    param.Ta = ANR().T; % atmospheric temperature (K)
    param.Pa = ANR().P; % atmospheric pressure (Pa)
    param.Ps = 500e3 + ANR().P; % supply pressure (Pa)
    param.Av = 1e-6; % cross-sectional area of valve (m^2)
    param.alpha_in = 1.0; % polytropic coefficient of inflow (-)
    param.alpha_out = 1.0; % polytropic coefficient of outflow (-)
    param.alpha = 1.0; % polytropic coefficient for piston move (-)
    % T-DA25X75; KOGANEI
    d_tube = 25e-3; % cylinder tube diameter (m)
    d_rod = 10e-3; % piston rod diameter (m)
    param.M = 82.5e-3; % piston mass (kg)
    param.D = 50.0; % viscous damping coefficient (N.s/m)
    param.L = 75e-3; % piston length (m)
    param.Ac = pi*d_tube^2/4; % cross-sectional area of cap side (m^2)
    param.Ah = pi*(d_tube^2 - d_rod^2)/4; % cross-sectional area of head side (m^2)
    param.Ar = pi*d_rod^2/4; % cross-sectional area of piston rod (m^2)
    param.Vc0 = 0.5e-6; % clearance volume of head side (m^3)
    param.Vh0 = 0.5e-6; % clearance volume of cap side (m^3)
    param.Me = 0; % external mass (kg)
    param.Fe = 0; % external force (N)
    param.Vctube = pi*(2.5e-3/2)^2*10e-3; % tube volume of cap side (m^3)
    param.Vhtube = pi*(2.5e-3/2)^2*10e-3; % tube volume of head side (m^3)
end

% constants of ANR (Atmosphere Normale de Reference)
function constants = ANR()
    constants.R = 287;          % specific gas constant (J/Kg.K)
    constants.k = 1.4;          % heat capacity ratio (-)
    constants.T = 273.15 + 20;  % temperature (K)
    constants.P = 101.3e3;      % pressure (Pa)
end
