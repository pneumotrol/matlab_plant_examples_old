% Pcr <= (P2/P1) < 1
function ub = input_constraint_sub2(P1,P2,param)
    R = param.R;
    Ta = param.Ta;
    k = param.k;
    Av = param.Av;
    C2 = sqrt(2*k/(k - 1));

    ub = C2*Av*(P1/sqrt(R*Ta))*(P2/P1)^(1/k)*sqrt(1 - (P2/P1)^((k - 1)/k));
end
