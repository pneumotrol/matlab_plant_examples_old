% plot bode diagram from force f to position q, angle theta
function figs = plot_bode()
    param = plant_param();
    option = struct("mode","bottom");
    sysc = plant_sysc(param,option);

    % range of bode diagram
    w_vec = logspace(-2,2,51);

    % frequency response of linear model
    H_sysc = freqresp(ss(sysc.A,sysc.B,sysc.C,sysc.D),w_vec);
    H_sysc = [squeeze(H_sysc(1,1,:)),squeeze(H_sysc(2,1,:))];

    % frequency response of simscape and ode model
    H_simscape = zeros(length(w_vec),2);
    H_ode = zeros(length(w_vec),2);
    for i = 1:length(w_vec)
        w = w_vec(i);

        % FFT settings
        N = 2048; % number of points (-)
        Ts = (2*pi)/(100*w); % sampling period (s)
        t_end = (N-1)*Ts; % simulation time (s)
        w_fft = (2*pi*((1:N/2)-1)/(N*Ts))'; % angular frequency vector (rad/s)

        simIn = Simulink.SimulationInput("simulation_sine");
        simIn = simIn.setVariable("x0",sysc.xe).setVariable("t_end",t_end).setVariable("Ts",Ts).setVariable("w",w);
        simIn = simIn.setVariable("ue",sysc.ue).setVariable("xe",sysc.xe);
        simOut = sim(simIn);

        for j = 1:2
            % frequency response of simscape
            H_simscape(i,j) = freqresp_at_w( ...
                simOut.logsout.getElement("u").Values.Data, ...
                simOut.logsout.getElement("x_simscape").Values.Data(:,j), ...
                w,w_fft);

            % frequency response of ode
            H_ode(i,j) = freqresp_at_w( ...
                simOut.logsout.getElement("u").Values.Data, ...
                simOut.logsout.getElement("x_ode").Values.Data(:,j), ...
                w,w_fft);
        end
    end

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
    plot(w_vec,20*log10(abs(H_simscape)),"-r");
    plot(w_vec,20*log10(abs(H_ode)),"--b");
    plot(w_vec,20*log10(abs(H_sysc)),"-.k");

    ax = gca; ax.FontSize = 12; ax.XScale = "log";
    xlabel("frequency (rad/s)");
    ylabel("magnitude (dB)");
    legend(["simscape","ode","sysc"]);

    % phase
    subplot(2,1,2); hold on;
    plot(w_vec,unwrap(angle(H_simscape))*180/pi,"-r");
    plot(w_vec,unwrap(angle(H_ode))*180/pi,"--b");
    plot(w_vec,unwrap(angle(H_sysc))*180/pi,"-.k");

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
