% plot initial response
function fig = plot_initial()
    param = plant_param();
    option = struct("Pe",(param.Pa+param.Ps)/2);
    sysc = plant_sysc(param,option);
    x0 = [0;0;50e3;50e3];

    % initial response of linear model
    % [~,t,x_sysc] = initial(ss(sysc.A,sysc.B,sysc.C,sysc.D),x0,1);
    t = 0.1;

    % initial response of simscape and ode model
    simIn = Simulink.SimulationInput("simulation_initial");
    simIn = simIn.setVariable("x0",sysc.xe+x0).setVariable("t_end",t(end));
    simIn = simIn.setVariable("ue",sysc.ue).setVariable("xe",sysc.xe);
    simOut = sim(simIn);

    fig = figure("Name","pneumatic_cylinder initial response"); hold on;
    p1 = plot(simOut.logsout.getElement("x_simscape").Values.*[1e3,1e3,1e-3,1e-3],"-r");
    p2 = plot(simOut.logsout.getElement("x_ode").Values.*[1e3,1e3,1e-3,1e-3],"--b");
    % p3 = plot(t,x_sysc(:,:,1)/1e3,"-.k");

    ax = gca; ax.FontSize = 12;
    xlabel("time (s)");
    ylabel("state");
    legend([p1(1),p2(1),p3(1)],["simscape","ode","sysc"]);
    ylim(([param.Pa;param.Ps]-param.Pa)/1e3);
end
