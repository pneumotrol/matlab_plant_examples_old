% plot impulse response
function fig = plot_impulse()
    param = plant_param();
    option = struct("Pe",(param.Pa+param.Ps)/2);
    sysc = plant_sysc(param,option);

    % initial response of linear model
    dt = 0.01;
    t = (0:dt:1)';
    u = zeros(length(t),1); u(1) = 1/dt; % impulse input at dmdt_in

    % saturation
    ub = input_constraint(param.Ps,param.Pa,param);
    if u(1) > max(ub,0)
        u(1) = max(ub,0);
    elseif u(1) < min(ub,0)
        u(1) = min(ub,0);
    end
    [~,~,x_sysc] = lsim(ss(sysc.A,sysc.B,sysc.C,sysc.D),u,t);

    % initial response of simscape and ode model
    simIn = Simulink.SimulationInput("simulation_impulse");
    simIn = simIn.setVariable("x0",sysc.xe).setVariable("t_end",t(end));
    simIn = simIn.setVariable("ue",sysc.ue).setVariable("xe",sysc.xe);
    simOut = sim(simIn);

    fig = figure("Name","pneumatic_chamber impulse response (from dmdt_in to all states)"); hold on;
    p1 = plot(simOut.logsout.getElement("x_simscape").Values/1e3,"-r");
    p2 = plot(simOut.logsout.getElement("x_ode").Values/1e3,"--b");
    p3 = plot(t,x_sysc(:,:,1)/1e3,"-.k");

    ax = gca; ax.FontSize = 12;
    xlabel("time (s)");
    ylabel("state");
    legend([p1(1),p2(1),p3(1)],["simscape","ode","sysc"]);
end
