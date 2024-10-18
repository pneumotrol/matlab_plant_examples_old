% plot impulse response
function fig = plot_impulse()
    param = plant_param();
    option = struct("qe",0,"Phe",(param.Pa+param.Ps)/2);
    sysc = plant_sysc(param,option);
    Ts = 1e-3;

    % initial response of linear model
    t = (0:1e-2:0.1)';
    u = zeros(length(t),2);
    u(1) = input_constraint(param.Ps,param.Pa,param);
    [~,~,x_sysc] = lsim(ss(sysc.A,sysc.B,sysc.C,sysc.D),u,t);

    % set parameters
    simIn = Simulink.SimulationInput("plant_test");
    simIn = simIn.setVariable("input_signal_type","impulse");
    simIn = simIn.setVariable("param",param);
    simIn = simIn.setVariable("x0",sysc.xe);
    simIn = simIn.setVariable("ue",sysc.ue).setVariable("xe",sysc.xe);
    simIn = simIn.setVariable("Ts",Ts).setVariable("t_end",t(end));

    % initial response of simscape model
    simIn = simIn.setVariable("plant_model_type","simscape");
    simOut_simscape = sim(simIn);

    % initial response of ode model
    simIn = simIn.setVariable("plant_model_type","ode");
    simOut_ode = sim(simIn);

    fig = figure("Name","pneumatic_chamber impulse response (from dmdt_in to all states)"); hold on;
    p1 = plot(simOut_simscape.logsout.getElement("x").Values.*[1e3,1e3,1e-3,1e-3],"-r");
    p2 = plot(simOut_ode.logsout.getElement("x").Values.*[1e3,1e3,1e-3,1e-3],"--b");
    p3 = plot(t,x_sysc(:,:,1).*[1e3,1e3,1e-3,1e-3],"-.k");

    ax = gca; ax.FontSize = 12;
    xlabel("time (s)");
    ylabel("state");
    legend([p1(1),p2(1),p3(1)],["simscape","ode","sysc"]);
end
