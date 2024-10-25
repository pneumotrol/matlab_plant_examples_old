close_system(find_system);
close all;
clear;
clc;

% list of examples
examples = [
    "mass_damper_spring";
    "mass_damper";
    "pendulum";
    "mass_damper_spring_2DOF";
    "mass_damper_2DOF";
    "cart_pole";
    "water_tank";
    "water_tank_2DOF";
    "ball_beam";
    "pneumatic_chamber";
    "pneumatic_cylinder";
    ];

for i = 1:length(examples)
    example = examples(i);
    addpath(genpath(example));

    ts = tic();
    disp("start example "+example);

    % initial state response
    fig = plot_initial();
    % exportgraphics(fig,fullfile(example,"docs/initial.png"),"Resolution",150);

    % impulse response
    fig = plot_impulse();
    % exportgraphics(fig,fullfile(example,"docs/impulse.png"),"Resolution",150);

    % bode diagram
    figs = plot_bode();
    % if length(figs) == 1
    %     exportgraphics(figs,fullfile(example,"docs/bode.png"),"Resolution",150);
    % else
    %     for j = 1:length(figs)
    %         exportgraphics(figs(j),fullfile(example,"docs/bode"+j+".png"),"Resolution",150);
    %     end
    % end

    disp("finish example "+example+" in "+toc(ts)+" seconds");

    rmpath(genpath(example));
    close_system(find_system);
end
