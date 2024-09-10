function dxdt = plant_ode(~,x,u,param)
    dxdt = plant_ode_sub2(0,x,u,param);
    % Pa = param.Pa;
    % Ps = param.Ps;
    % 
    % P = x(1);
    % 
    % if (P < Pa)
    %     dxdt = plant_ode_sub1(0,x,u,param);
    % elseif (P < Ps)
    %     dxdt = plant_ode_sub2(0,x,u,param);
    % else
    %     dxdt = plant_ode_sub3(0,x,u,param);
    % end
end
