% 1 <= (P2/P1) < (1/Pcr)
function ub = input_constraint_sub3(P1,P2,param)
    R = param.R;
    Ta = param.Ta;
    k = param.k;
    Av = param.Av;
    C2 = sqrt(2*k/(k - 1));

    ub = -C2*Av*(P2/sqrt(R*Ta))*(P1/P2)^(1/k)*sqrt(1 - (P1/P2)^((k - 1)/k));
end
