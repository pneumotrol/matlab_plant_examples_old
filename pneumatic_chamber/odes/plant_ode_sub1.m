% P < Pa
function dxdt = plant_ode_sub1(~,x,u,param) %#ok<INUSD>
    R = param.R;
    Ta = param.Ta;
    V = param.V;
    alpha_in = param.alpha_in;

    dm_in = u(1);
    dm_out = u(2);

    dxdt = (R*Ta/V)*(alpha_in*dm_in - alpha_in*dm_out);
end
