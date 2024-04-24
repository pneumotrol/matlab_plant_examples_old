function sysc = plant_sysc(param,option)
    g = param.g;
    m = param.m;
    J = param.J;
    r = param.r;

    % equilibrium point
    if ~isfield(option,"qe")
        error("option must have the field of equilibrium point qe.");
    end

    qe = option.qe;
    taue = m*g*qe;
    sysc.xe = [qe;0;0;0];
    sysc.ue = taue;

    % coefficients of state equation
    sysc.A = [
        0,0,1,0;
        0,0,0,1;
        0,-m*g/(m+J/(r^2)),0,0;
        -g/(qe^2),0,0,0;
        ];

    sysc.B = [
        0;
        0;
        0;
        1/(m*qe^2);
        ];

    sysc.C = eye(4);

    sysc.D = zeros(4,1);
end
