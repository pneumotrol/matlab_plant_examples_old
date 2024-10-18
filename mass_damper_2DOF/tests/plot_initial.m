% plot initial response
function fig = plot_initial()
    param = plant_param();
    sysc = plant_sysc(param);
    x0 = [0;0;1;0];
    dt = 1e-3;
    t_end = 10;

    % set parameters
    simIn = Simulink.SimulationInput("plant_test");
    simIn = simIn.setVariable("input_signal_type","zero");
    simIn = simIn.setVariable("param",param).setVariable("sysc",sysc);
    simIn = simIn.setVariable("x0",x0 + sysc.xe);
    simIn = simIn.setVariable("ue",sysc.ue).setVariable("xe",sysc.xe);
    simIn = simIn.setVariable("dt",dt).setVariable("t_end",t_end);

    % initial response of simscape model
    simIn = simIn.setVariable("plant_model_type","simscape");
    simOut_simscape = sim(simIn);

    % initial response of ode model
    simIn = simIn.setVariable("plant_model_type","ode");
    simOut_ode = sim(simIn);

    % initial response of linear model
    [~,t_sysc,x_sysc] = initial(ss(sysc.A,sysc.B,sysc.C,sysc.D),x0,t_end);

    % plotting
    fig = figure("Name","mass_damper_2DOF initial response"); hold on;
    p1 = plot(simOut_simscape.logsout.getElement("x").Values,"-r","LineWidth",1);
    p2 = plot(simOut_ode.logsout.getElement("x").Values,"--b","LineWidth",1);
    p3 = plot(t_sysc,x_sysc(:,:,1),"-.k","LineWidth",1);

    ax = gca; ax.FontSize = 12;
    xlabel("time (s)");
    ylabel("state");
    legend([p1(1),p2(1),p3(1)],["simscape","ode","sysc"]);
end
