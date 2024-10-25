function param = plant_param()
    % dimensions
    param.xdim = 1;
    param.udim = 2;

    % parameters
    param.R = ANR().R; % specific gas constant (J/Kg.K)
    param.k = ANR().k; % heat capacity ratio (-)
    param.Ta = ANR().T; % atmospheric temperature (K)
    param.Pa = ANR().P; % atmospheric pressure (Pa)
    param.Ps = 500e3 + ANR().P; % supply pressure (Pa)
    param.Av = 1e-6; % cross-sectional area of valve (m^2)
    param.alpha_in = 1.0; % polytropic coefficient of inflow (-)
    param.alpha_out = 1.0; % polytropic coefficient of outflow (-)
    param.V = 50e-6; % chamber volume (m^3)
    param.Vtube = pi*(2.5e-3/2)^2*10e-3; % tube volume (m^3)
end

% constants of ANR (Atmosphere Normale de Reference)
function constants = ANR()
    constants.R = 287;          % specific gas constant (J/Kg.K)
    constants.k = 1.4;          % heat capacity ratio (-)
    constants.T = 273.15 + 20;  % temperature (K)
    constants.P = 101.3e3;      % pressure (Pa)
end
