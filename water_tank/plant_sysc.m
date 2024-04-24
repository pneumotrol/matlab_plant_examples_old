function sysc = plant_sysc(param,option)
    g = param.g;
    A = param.A;
    a = param.a;

    % equilibrium point
    has_ze = isfield(option,"ze");
    has_qe = isfield(option,"qe");
    if ~xor(has_ze,has_qe)
        error("option must have the field of equilibrium point either ze or qe.");
    end

    if has_ze
        ze = option.ze;
    end

    if has_qe
        ze = (option.qe/a)^2/(2*g);
    end

    sysc.xe = ze;
    sysc.ue = a*sqrt(2*g*ze);

    % coefficients of state equation
    sysc.A = -(a*sqrt(2*g))/(2*A*sqrt(ze));

    sysc.B = (1/A);

    sysc.C = eye(1);

    sysc.D = zeros(1,1);
end
