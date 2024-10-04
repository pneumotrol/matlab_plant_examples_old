% (P2/P1) < Pcr
function ub = input_constraint_sub1(P1,P2,param) %#ok<INUSD>
    R = param.R;
    Ta = param.Ta;
    k = param.k;
    Av = param.Av;
    C1 = sqrt(k*(2/(k + 1))^((k + 1)/(k - 1)));

    ub = C1*Av*(P1/sqrt(R*Ta));
end
