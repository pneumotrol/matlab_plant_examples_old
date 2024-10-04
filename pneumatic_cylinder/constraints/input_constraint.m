function ret = input_constraint(P1,P2,param)
    k = param.k;
    Pcr = (2/(k + 1))^(k/(k - 1)); % critical pressure ratio (-)

    if (P2/P1) < Pcr
        ret = input_constraint_sub1(P1,P2,param);
    elseif (P2/P1) < 1
        ret = input_constraint_sub2(P1,P2,param);
    elseif (P2/P1) < (1/Pcr)
        ret = input_constraint_sub3(P1,P2,param);
    else
        ret = input_constraint_sub4(P1,P2,param);
    end
end
