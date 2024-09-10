% Pa <= P < Ps
function dxdt = plant_ode_sub2(~,x,u,param)
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

    q = x(1);
    dqdt = x(2);
    Pc = x(3);
    Ph = x(4);
    dmc_in = u(1);
    dmc_out = u(2);
    dmh_in = u(3);
    dmh_out = u(4);

    dxdt = [
        dqdt;
        (-D*dqdt + Ac*Pc - Ah*Ph - Ar*Pa - Fe)/(M + Me);
        (R*Ta*(alpha_in*dmc_in - alpha_out*dmc_out) - alpha*Ac*Pc*dqdt)/(Vc0 + Ac*(0.5*L + q));
        (R*Ta*(alpha_in*dmh_in - alpha_out*dmh_out) + alpha*Ah*Ph*dqdt)/(Vh0 + Ah*(0.5*L - q));
        ];
end
