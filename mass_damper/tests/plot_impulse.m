% plot impulse response from force f to all states
function fig = plot_impulse()
    param = plant_param();
    sysc = plant_sysc(param);

    % impulse response of linear model
    [~,t,x_sysc] = impulse(ss(sysc.A,sysc.B,sysc.C,sysc.D),10);

    % impulse response of simscape and ode model
    simIn = Simulink.SimulationInput("simulation_impulse");
    simIn = simIn.setVariable("x0",sysc.xe).setVariable("t_end",t(end));
    simIn = simIn.setVariable("ue",sysc.ue).setVariable("xe",sysc.xe);
    simOut = sim(simIn);

    fig = figure("Name","mass_damper impulse response (from f to all states)"); hold on;
    p1 = plot(simOut.logsout.getElement("x_simscape").Values,"-r");
    p2 = plot(simOut.logsout.getElement("x_ode").Values,"--b");
    p3 = plot(t,x_sysc(:,:,1),"-.k");

    ax = gca; ax.FontSize = 12;
    xlabel("time (s)");
    ylabel("state");
    legend([p1(1),p2(1),p3(1)],["simscape","ode","sysc"]);
end
