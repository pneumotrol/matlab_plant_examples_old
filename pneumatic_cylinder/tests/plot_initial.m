% plot initial response
function fig = plot_initial()
    param = plant_param();
    option = struct("qe",0,"Phe",(param.Pa+param.Ps)/2);
    sysc = plant_sysc(param,option);
    x0 = [0;0;10e3;10e3];
    Ts = 1e-3;

    % initial response of linear model
    [~,t,x_sysc] = initial(ss(sysc.A,sysc.B,sysc.C,sysc.D),x0,0.1);

    % set parameters
    simIn = Simulink.SimulationInput("plant_test");
    simIn = simIn.setVariable("input_signal_type","zero");
    simIn = simIn.setVariable("param",param);
    simIn = simIn.setVariable("x0",x0+sysc.xe);
    simIn = simIn.setVariable("ue",sysc.ue).setVariable("xe",sysc.xe);
    simIn = simIn.setVariable("Ts",Ts).setVariable("t_end",t(end));

    % initial response of simscape model
    simIn = simIn.setVariable("plant_model_type","simscape");
    simOut_simscape = sim(simIn);

    % initial response of ode model
    simIn = simIn.setVariable("plant_model_type","ode");
    simOut_ode = sim(simIn);

    fig = figure("Name","pneumatic_cylinder initial response"); hold on;
    p1 = plot(simOut_simscape.logsout.getElement("x").Values.*[1e3,1e3,1e-3,1e-3],"-r");
    p2 = plot(simOut_ode.logsout.getElement("x").Values.*[1e3,1e3,1e-3,1e-3],"--b");
    p3 = plot(t,x_sysc(:,:,1).*[1e3,1e3,1e-3,1e-3],"-.k");

    ax = gca; ax.FontSize = 12;
    xlabel("time (s)");
    ylabel("state");
    legend([p1(1),p2(1),p3(1)],["simscape","ode","sysc"]);
end
