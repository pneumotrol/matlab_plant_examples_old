% plot input constraint
function fig = plot_input_constraint()
    param = plant_param();

    simIn = Simulink.SimulationInput("simulation_input_constraint");
    simIn = simIn.setVariable("t_end",10);
    simOut = sim(simIn);

    fig = figure("Name","pneumatic_chamber input constraint"); hold on;
    pressure = simOut.logsout.getElement("P").Values.Data / 1e3;
    ub_simscape = simOut.logsout.getElement("ub_simscape").Values.Data * 1e3;
    ub_ode = simOut.logsout.getElement("ub_ode").Values.Data * 1e3;
    p1 = plot(pressure,ub_simscape,"-r");
    p2 = plot(pressure,ub_ode,"--b");

    ax = gca; ax.FontSize = 12;
    xlabel("pressure (kPa)");
    ylabel("upper bound of mass flow rate (g/s)");
    legend([p1(1),p2(1)],["simscape","ode"]);
end
