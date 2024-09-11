function sysc = plant_sysc(param,option)
    R = param.R;
    Ta = param.Ta;
    Pa = param.Pa;
    alpha_in = param.alpha_in;
    alpha_out = param.alpha_out;
    alpha = param.alpha;
    M = param.M;
    D = param.D;
    L = param.L;
    Ac = param.Ac;
    Ah = param.Ah;
    Ar = param.Ar;
    Vc0 = param.Vc0;
    Vh0 = param.Vh0;
    Me = param.Me;
    Fe = param.Fe;

    % equilibrium point
    if ~isfield(option,"qe")
        error("option must have the field of equilibrium point qe.");
    end

    has_Pce = isfield(option,"Pce");
    has_Phe = isfield(option,"Phe");
    if ~xor(has_Pce,has_Phe)
        error("option must have the field of equilibrium point either Pce or Phe.");
    end

    qe = option.qe;

    if has_Pce
        Pce = option.Pce;
        Phe = (Ac*Pce-Ar*Pa-Fe)/Ah;
    end

    if has_Phe
        Phe = option.Phe;
        Pce = (Ah*Phe+Ar*Pa+Fe)/Ac;
    end

    sysc.xe = [qe;0;Pce;Phe];
    sysc.ue = [0;0];

    % coefficients of state equation
    sysc.A = [
        0,1,0,0;
        0,-D/(M+Me),Ac/(M+Me),-Ah/(M+Me);
        0,-alpha*Ac*Pce/(Vc0+Ac*(L/2+qe)),0,0;
        0,alpha*Ah*Phe/(Vh0+Ah*(L/2-qe)),0,0;
        ];

    sysc.B = [
        0,0;
        0,0;
        R*Ta*0.5*(alpha_in+alpha_out)/(Vc0+Ac*(L/2+qe)),0;
        0,R*Ta*0.5*(alpha_in+alpha_out)/(Vh0+Ah*(L/2-qe));
        ];

    sysc.C = eye(4);

    sysc.D = zeros(4,2);
end
