%% Single publication-style trajectory comparison figure
clear; close all hidden; clc;

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(fileparts(script_dir)));
addpath(genpath(repo_root));
cd(script_dir);

%% User settings
file_no_error = 'circlenoerr52s_Log(06-Apr-2026_17_37_10).mat';
file_error    = 'circleerr52s_Log(06-Apr-2026_17_35_48).mat';
file_comp     = 'circleerrcom52smode5_Log(06-Apr-2026_17_34_20).mat';

run_files  = {file_no_error, file_error, file_comp};
run_labels = {'Nominal', 'Perturbed', 'Compensated'};
run_styles = {'-', '-', '-'};
run_colors = [
    0.10, 0.35, 0.90;
    0.86, 0.24, 0.12;
    0.00, 0.62, 0.32
];

font_name = 'Times New Roman';
font_size = 13;
line_width = 1.2;
ref_line_width = 3.0;
flight_phase_only = true;
run_markers = {'o', 's', '^'};
marker_spacing_each = [45, 45, 20];
marker_spacing_inset = [10, 10, 5];

%% Load logs
n_runs = numel(run_files);
runs = cell(1, n_runs);
for i = 1:n_runs
    runs{i} = load_run(run_files{i}, run_labels{i});
end

time_range = intersect_time_range(runs);
phase_option = [];
if flight_phase_only
    phase_option = 'f';
end

%% Figure
fig = figure(200); clf(fig);
fig.Color = 'w';
fig.Position(3:4) = [980, 760];
ax = axes(fig);
hold(ax, 'on');

ref_phase = get_series(runs{1}.logger, "p", "r", time_range, phase_option);
traj_data = cell(1, n_runs);
plot3(ax, ref_phase(:, 1), ref_phase(:, 2), ref_phase(:, 3), ...
    'Color', [0.35, 0.35, 0.35], ...
    'LineStyle', '--', ...
    'LineWidth', 2.0, ...
    'DisplayName', 'Reference');

plot_order = [2, 3, 1];
for idx = plot_order
    p_phase = get_series(runs{idx}.logger, "p", "e", time_range, phase_option);
    traj_data{idx} = p_phase;
    plot3(ax, p_phase(:, 1), p_phase(:, 2), p_phase(:, 3), ...
        'Color', run_colors(idx, :), ...
        'LineStyle', run_styles{idx}, ...
        'LineWidth', line_width + (idx == 1) * 0.4, ...
        'Marker', run_markers{idx}, ...
        'MarkerIndices', 1:marker_spacing_each(idx):size(p_phase, 1), ...
        'MarkerSize', 4.5, ...
        'MarkerFaceColor', 'none', ...
        'DisplayName', run_labels{idx});
    plot3(ax, p_phase(1, 1), p_phase(1, 2), p_phase(1, 3), ...
        'o', ...
        'MarkerSize', 6.5, ...
        'MarkerFaceColor', run_colors(idx, :), ...
        'MarkerEdgeColor', run_colors(idx, :), ...
        'HandleVisibility', 'off');
end

grid(ax, 'on');
box(ax, 'on');
axis(ax, 'equal');
view(ax, [-38, 24]);
xlabel(ax, 'x [m]', 'FontName', font_name, 'FontSize', font_size);
ylabel(ax, 'y [m]', 'FontName', font_name, 'FontSize', font_size);
zlabel(ax, 'z [m]', 'FontName', font_name, 'FontSize', font_size);
title(ax, 'Trajectory Tracking Performance in Simulation', ...
    'FontName', font_name, 'FontSize', font_size + 3);
set(ax, 'FontName', font_name, 'FontSize', font_size, 'LineWidth', 1.0);
legend(ax, 'Location', 'northeastoutside', 'FontName', font_name, 'FontSize', font_size - 1);

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
