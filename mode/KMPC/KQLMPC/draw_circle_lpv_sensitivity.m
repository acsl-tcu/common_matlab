%% Circle trajectory figure for LPV model sensitivity
clear; close all hidden; clc;

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(fileparts(script_dir)));
addpath(genpath(repo_root));
cd(script_dir);

%% Data: circle tracking without/with model error
run_files = { ...
    'circlenoerr52s_Log(06-Apr-2026_17_37_10).mat', ...
    'circleerr52s_Log(06-Apr-2026_17_35_48).mat'};

run_labels = { ...
    'KQ-LMPC', ...
    'KQ-LMPC + Model Error'};

run_colors = [
    0.10, 0.35, 0.90;
    0.86, 0.24, 0.12
];

run_markers = {'o', 's'};
marker_spacing = [45, 45];

font_name = 'Times New Roman';
font_size = 14;
line_width = 1.8;
flight_phase_only = true;

output_dir = fullfile(script_dir, 'trajectory_figures');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% Load logs
runs = cell(1, numel(run_files));
for i = 1:numel(run_files)
    runs{i} = load_run(run_files{i}, run_labels{i});
end

time_range = intersect_time_range(runs);
phase_option = [];
if flight_phase_only
    phase_option = 'f';
end

%% Figures: reference vs nominal, reference vs nominal with model error
ref_phase = get_series(runs{1}.logger, "p", "r", time_range, phase_option);
plot_single_comparison(311, ref_phase, runs{1}, run_colors(1, :), run_markers{1}, ...
    marker_spacing(1), run_labels{1}, 'Circle Tracking without Model Error', ...
    fullfile(output_dir, 'circle_reference_vs_kqlmpc.png'), time_range, phase_option, ...
    font_name, font_size, line_width);

plot_single_comparison(312, ref_phase, runs{2}, run_colors(2, :), run_markers{2}, ...
    marker_spacing(2), run_labels{2}, 'Circle Tracking with Model Error', ...
    fullfile(output_dir, 'circle_reference_vs_kqlmpc_model_error.png'), time_range, phase_option, ...
    font_name, font_size, line_width);

%% Local functions
function run = load_run(file_name, label)
    if ~isfile(file_name)
        error('File not found: %s', file_name);
    end
    run = struct();
    run.label = label;
    run.logger = LOGGER(file_name);
end

function time_range = intersect_time_range(runs)
    t_starts = zeros(1, numel(runs));
    t_ends = zeros(1, numel(runs));
    for idx = 1:numel(runs)
        t = runs{idx}.logger.Data.t;
        t_starts(idx) = min(t);
        t_ends(idx) = max(t);
    end
    time_range = [max(t_starts), min(t_ends)];
end

function data = get_series(logger_obj, variable, attribute, time_range, phase_option)
    if nargin < 5 || isempty(phase_option)
        raw = logger_obj.data(1, variable, attribute, "ranget", time_range);
    else
        raw = logger_obj.data(1, variable, attribute, "ranget", time_range, "phase", phase_option);
    end
    data = normalize_series(raw);
end

function data = normalize_series(raw)
    data = double(squeeze(raw));
    if isvector(data)
        data = data(:);
    end
    if size(data, 1) < size(data, 2)
        data = data.';
    end
end

function plot_single_comparison(fig_id, ref_phase, run, run_color, run_marker, ...
    marker_spacing, run_label, fig_title, output_path, time_range, phase_option, ...
    font_name, font_size, line_width)
    fig = figure(fig_id); clf(fig);
    fig.Color = 'w';
    fig.Position(3:4) = [900, 720];
    ax = axes(fig);
    hold(ax, 'on');

    h_ref = plot3(ax, ref_phase(:, 1), ref_phase(:, 2), ref_phase(:, 3), ...
        'Color', [0.35, 0.35, 0.35], ...
        'LineStyle', '--', ...
        'LineWidth', 2.0, ...
        'DisplayName', 'Reference');

    p_phase = get_series(run.logger, "p", "e", time_range, phase_option);
    h_run = plot3(ax, p_phase(:, 1), p_phase(:, 2), p_phase(:, 3), ...
        'Color', run_color, ...
        'LineStyle', '-', ...
        'LineWidth', line_width, ...
        'Marker', run_marker, ...
        'MarkerIndices', 1:marker_spacing:size(p_phase, 1), ...
        'MarkerSize', 4.8, ...
        'MarkerFaceColor', 'none', ...
        'DisplayName', run_label);

    plot3(ax, p_phase(1, 1), p_phase(1, 2), p_phase(1, 3), ...
        'o', ...
        'MarkerSize', 7.0, ...
        'MarkerFaceColor', run_color, ...
        'MarkerEdgeColor', run_color, ...
        'HandleVisibility', 'off');

    grid(ax, 'on');
    box(ax, 'on');
    axis(ax, 'equal');
    view(ax, [-38, 24]);
    xlabel(ax, 'x [m]', 'FontName', font_name, 'FontSize', font_size);
    ylabel(ax, 'y [m]', 'FontName', font_name, 'FontSize', font_size);
    zlabel(ax, 'z [m]', 'FontName', font_name, 'FontSize', font_size);
    title(ax, fig_title, 'FontName', font_name, 'FontSize', font_size + 3);
    set(ax, 'FontName', font_name, 'FontSize', font_size, 'LineWidth', 1.0);
    legend(ax, [h_ref, h_run], {'Reference', run_label}, ...
        'Location', 'northeastoutside', ...
        'FontName', font_name, ...
        'FontSize', font_size - 1);

    exportgraphics(fig, output_path, 'Resolution', 300);
end
