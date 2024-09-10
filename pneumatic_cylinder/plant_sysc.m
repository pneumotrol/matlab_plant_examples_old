function sysc = plant_sysc(param,option)
    R = param.R;
    Ta = param.Ta;
    % V = param.V;

    % equilibrium point
    if ~isfield(option,"Pe")
        error("option must have the field of equilibrium point Pe.");
    end

    Pce = option.Pe;
    Phe = option.Pe;
    sysc.xe = [0;0;Pce;Phe];
    sysc.ue = [0;0];

    % coefficients of state equation
    % sysc.A = 0;

    % sysc.B = (R*Ta/V);

    % sysc.C = eye(1);

    % sysc.D = 0;
end
