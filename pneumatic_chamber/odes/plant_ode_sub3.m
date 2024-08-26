% Ps <= P
function dxdt = plant_ode_sub3(~,x,u,params) %#ok<INUSD>
    dm_in = u(1);
    dm_out = u(2);

    R = params.R;
    Ta = params.Ta;
    V = params.V;
    alpha_out = params.alpha_out;

    dxdt = (R*Ta/V)*(alpha_out*dm_in - alpha_out*dm_out);
end
