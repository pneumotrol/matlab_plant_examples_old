% plot initial response
function fig = plot_initial()
    param = plant_param();
    option = struct("qe",1);
    sysc = plant_sysc(param,option);
    x0 = [0;pi/12;0;0];

    % initial response of linear model
    [~,t,x_sysc] = initial(ss(sysc.A,sysc.B,sysc.C,sysc.D),x0,1);

    % initial response of simscape and ode model
    simIn = Simulink.SimulationInput("simulation_initial");
    simIn = simIn.setVariable("x0",sysc.xe+x0).setVariable("t_end",t(end));
    simIn = simIn.setVariable("ue",sysc.ue).setVariable("xe",sysc.xe);
    simOut = sim(simIn);

    fig = figure("Name","ball_beam initial response"); hold on;
    p1 = plot(simOut.logsout.getElement("x_simscape").Values,"-r");
    p2 = plot(simOut.logsout.getElement("x_ode").Values,"--b");
    p3 = plot(t,x_sysc(:,:,1),"-.k");

    ax = gca; ax.FontSize = 12;
    xlabel("time (s)");
    ylabel("state");
    legend([p1(1),p2(1),p3(1)],["simscape","ode","sysc"]);
    ylim([-1.5,1.5]);
end
