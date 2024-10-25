function dxdt = plant_ode(~,x,u,param)
    R = param.R;
    Ta = param.Ta;
    Pa = param.Pa;
    Ps = param.Ps;
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

    q = x(1);
    dqdt = x(2);
    Pc = x(3);
    Ph = x(4);
    Avc_in = u(1);
    Avc_out = u(2);
    Avh_in = u(3);
    Avh_out = u(4);

    % penalty force
    Fp = -0.1*q/((q + L/2)*(q - L/2));

    dxdt = [
        dqdt;
        (-D*dqdt + Ac*Pc - Ah*Ph - Ar*Pa - Fe - Fp)/(M + Me);
        (R*Ta*(alpha_in*Avc_in*phi(Ps,Pc,param) - alpha_out*Avc_out*phi(Pc,Pa,param)) - alpha*Ac*Pc*dqdt)/(Vc0 + Ac*(0.5*L + q));
        (R*Ta*(alpha_in*Avh_in*phi(Ps,Ph,param) - alpha_out*Avh_out*phi(Ph,Pa,param)) + alpha*Ah*Ph*dqdt)/(Vh0 + Ah*(0.5*L - q));
        ];
end

function ret = phi(P1,P2,param)
    R = param.R;
    Ta = param.Ta;
    k = param.k;
    Pcr = (2/(k + 1))^(k/(k - 1)); % critical pressure ratio (-)
    C1 = sqrt(k*(2/(k + 1))^((k + 1)/(k - 1)));
    C2 = sqrt(2*k/(k - 1));

    if (P2/P1) < Pcr
        ret = C1*(P1/sqrt(R*Ta));
    elseif (P2/P1) < 1
        ret = C2*(P1/sqrt(R*Ta))*(P2/P1)^(1/k)*sqrt(1 - (P2/P1)^((k - 1)/k));
    elseif (P2/P1) < (1/Pcr)
        ret = -C2*(P2/sqrt(R*Ta))*(P1/P2)^(1/k)*sqrt(1 - (P1/P2)^((k - 1)/k));
    else
        ret = -C1*(P2/sqrt(R*Ta));
    end
end
