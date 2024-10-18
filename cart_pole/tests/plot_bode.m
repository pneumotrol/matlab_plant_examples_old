% plot bode diagram from force f to position q, angle theta
function figs = plot_bode()
    param = plant_param();
    option = struct("mode","bottom");
    sysc = plant_sysc(param,option);
    x0 = [0;0;0;0];

    % range of bode diagram
    w_vec = logspace(-2,2,51);

    % frequency response
    H_simscape = zeros(length(w_vec),2);
    H_ode = zeros(length(w_vec),2);

    simIn = Simulink.SimulationInput("plant_test");
    simIn = simIn.setVariable("input_signal_type","sine");
    simIn = simIn.setVariable("param",param).setVariable("sysc",sysc);
    simIn = simIn.setVariable("x0",x0 + sysc.xe);
    simIn = simIn.setVariable("ue",sysc.ue).setVariable("xe",sysc.xe);
    for i = 1:length(w_vec)
        w = w_vec(i);

        % FFT settings
        N = 2048; % number of points (-)
        dt = (2*pi)/(100*w); % sampling period (s)
        t_end = (N-1)*dt; % simulation time (s)
        w_fft = (2*pi*((1:N/2)-1)/(N*dt))'; % angular frequency vector (rad/s)

        simIn = simIn.setVariable("dt",dt).setVariable("t_end",t_end).setVariable("w",w);

        % frequency response of simscape
        simIn = simIn.setVariable("plant_model_type","simscape");
        simOut_simscape = sim(simIn);
        for j = 1:2
            H_simscape(i,j) = freqresp_at_w( ...
                simOut_simscape.logsout.getElement("u").Values.Data, ...
                simOut_simscape.logsout.getElement("x").Values.Data(:,j), ...
                w,w_fft);
        end

        % frequency response of ode
        simIn = simIn.setVariable("plant_model_type","ode");
        simOut_ode = sim(simIn);
        for j = 1:2
            H_ode(i,j) = freqresp_at_w( ...
                simOut_ode.logsout.getElement("u").Values.Data, ...
                simOut_ode.logsout.getElement("x").Values.Data(:,j), ...
                w,w_fft);
        end
    end

    % frequency response of linear model
    H_sysc = freqresp(ss(sysc.A,sysc.B,sysc.C,sysc.D),w_vec);
    H_sysc = [squeeze(H_sysc(1,1,:)),squeeze(H_sysc(2,1,:))];

    % from force f to position q
    figs(1) = figure("Name","cart_pole bode plot (from f to q)");
    plot_bode_sub(w_vec,H_simscape(:,1),H_ode(:,1),H_sysc(:,1));

    % from force f to angle theta
    figs(2) = figure("Name","cart_pole bode plot (from f to theta)");
    plot_bode_sub(w_vec,H_simscape(:,2),H_ode(:,2),H_sysc(:,2));
end

function plot_bode_sub(w_vec,H_simscape,H_ode,H_sysc)
    % magnitude
    subplot(2,1,1); hold on;
    plot(w_vec,20*log10(abs(H_simscape)),"-r","LineWidth",1);
    plot(w_vec,20*log10(abs(H_ode)),"--b","LineWidth",1);
    plot(w_vec,20*log10(abs(H_sysc)),"-.k","LineWidth",1);

    ax = gca; ax.FontSize = 12; ax.XScale = "log";
    xlabel("frequency (rad/s)");
    ylabel("magnitude (dB)");
    legend(["simscape","ode","sysc"]);

    % phase
    subplot(2,1,2); hold on;
    plot(w_vec,unwrap(angle(H_simscape))*180/pi,"-r","LineWidth",1);
    plot(w_vec,unwrap(angle(H_ode))*180/pi,"--b","LineWidth",1);
    plot(w_vec,unwrap(angle(H_sysc))*180/pi,"-.k","LineWidth",1);

    ax = gca; ax.FontSize = 12; ax.XScale = "log";
    xlabel("frequency (rad/s)");
    ylabel("phase (deg)");
    legend(["simscape","ode","sysc"]);
end

function H = freqresp_at_w(u,y,w,w_fft)
    Puu = fft(u);
    Pyy = fft(y);
    Puy = (Pyy.*conj(Puu))./(Puu.*conj(Puu));

    H = interp1(w_fft,Puy(1:length(w_fft)),w);
end
