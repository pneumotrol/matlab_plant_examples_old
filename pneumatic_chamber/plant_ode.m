function dxdt = plant_ode(~,x,u,param)
    R = param.R;
    Ta = param.Ta;
    Pa = param.Pa;
    Ps = param.Ps;
    alpha_in = param.alpha_in;
    alpha_out = param.alpha_out;
    V = param.V;

    P = x(1);
    Av_in = u(1);
    Av_out = u(2);

    if (P < Pa)
        dxdt = (R*Ta/V)*(alpha_in*Av_in*phi(Ps,P,param) - alpha_in*Av_out*phi(P,Pa,param));
    elseif (P < Ps)
        dxdt = (R*Ta/V)*(alpha_in*Av_in*phi(Ps,P,param) - alpha_out*Av_out*phi(P,Pa,param));
    else
        dxdt = (R*Ta/V)*(alpha_out*Av_in*phi(Ps,P,param) - alpha_out*Av_out*phi(P,Pa,param));
    end
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
