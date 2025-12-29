%% Initialize settings
% set path
clear all
cf = pwd;

if contains(mfilename('fullpath'), "mainGUI")
    cd(fileparts(mfilename('fullpath')));
else
    tmp = matlab.desktop.editor.getActive;
    cd(fileparts(tmp.Filename));
end

[~, tmp] = regexp(genpath('.'), '\.\\\.git.*?;', 'match', 'split');
cellfun(@(xx) addpath(xx), tmp, 'UniformOutput', false);
close all hidden; clear; clc;
userpath('clear');
%%
clear;
filepathes = {... % TODO: ファイルのパスを記述
    "C:\Users\hiyou\Github\common_matlab\Data\Exp_data\2025.12.11_405Exp_various_trajectories\HLLQR_lemniscate_Log(11-Dec-2025_18_42_48).mat";
    "C:\Users\hiyou\Github\common_matlab\Data\Exp_data\2025.12.11_405Exp_various_trajectories\NN21MEC_lemniscate_Log(11-Dec-2025_18_44_08).mat";

    };
filepathes = {... % TODO: ファイルのパスを記述
    "C:\Users\hiyou\Github\common_matlab\Data\Sim_data\For IFAC2026\lemniscate_HLLQR_R=0.05_Log(30-Nov-2025_21_24_44).mat";
    "C:\Users\hiyou\Github\common_matlab\Data\Sim_data\For IFAC2026\lemniscate_NNMEC_R=0.05_Log(30-Nov-2025_21_31_32).mat";
    };

plt = PLOTTER(filepathes);
lgd = {"HL-LQR"; "HLLQR+NN-MEC"};
alpha = [0.95,0.9];
plt.set_plot_settings("LegendNames",lgd, "alpha",alpha);
plt.extract_time;
plt.extract_data;

plt.subfig_plot
