% constants of ANR (Atmosphere Normale de Reference)
function constants = ANR()
    constants.R = 287;          % specific gas constant (J/Kg.K)
    constants.k = 1.4;          % heat capacity ratio (-)
    constants.T = 273.15 + 20;  % temperature (K)
    constants.P = 101.3e3;      % pressure (Pa)
end