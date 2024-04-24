% plot impulse response from volume flow rate q to all states
function fig = plot_impulse()
    param = plant_param();
    option = struct("ze",1);
    sysc = plant_sysc(param,option);

    % impulse response of linear model
    [~,t,x_sysc] = impulse(ss(sysc.A,sysc.B,sysc.C,sysc.D),10);

    % impulse response of simscape and ode model
    simIn = Simulink.SimulationInput("simulation_impulse");
    simIn = simIn.setVariable("x0",sysc.xe).setVariable("t_end",t(end));
    simIn = simIn.setVariable("ue",sysc.ue).setVariable("xe",sysc.xe);
    simOut = sim(simIn);

    fig = figure("Name","water_tank impulse response (from q to all states)"); hold on;
    p1 = plot(simOut.logsout.getElement("x_simscape").Values,"-r");
    p2 = plot(simOut.logsout.getElement("x_ode").Values,"--b");
    p3 = plot(t,x_sysc(:,:,1),"-.k");

    ax = gca; ax.FontSize = 12;
    xlabel("time (s)");
    ylabel("state");
    legend([p1(1),p2(1),p3(1)],["simscape","ode","sysc"]);
end
