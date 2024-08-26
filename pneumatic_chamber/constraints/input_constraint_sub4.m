% (1/Pcr) <= (P2/P1)
function ub = input_constraint_sub4(P1,P2,params) %#ok<INUSD>
    R = params.R;
    Ta = params.Ta;
    k = params.k;
    Av = params.Av;
    C1 = sqrt(k*(2/(k + 1))^((k + 1)/(k - 1)));

    ub = -C1*Av*(P2/sqrt(R*Ta));
end
