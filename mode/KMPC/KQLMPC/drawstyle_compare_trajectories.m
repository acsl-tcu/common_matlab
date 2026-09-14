%% Draw-style trajectory comparison for prepared logs
% Edit the "User settings" block only.
clearvars -except cases state_source agent_id phase_option time_range output_dir reference_run_index;
close all hidden; clc;

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(fileparts(script_dir)));
addpath(genpath(repo_root));
cd(script_dir);

%% User settings
if ~exist('cases', 'var')
    cases = struct( ...
        'name', {'ExampleCase'}, ...
        'files', {{ ...
            'C:\path\to\run_nominal.mat', ...
            'C:\path\to\run_error.mat', ...
            'C:\path\to\run_comp.mat'}}, ...
        'labels', {{ ...
            'HLC', ...
            'HLC + error', ...
            'HLC + error + EDMD'}});
end

if ~exist('state_source', 'var')
    state_source = 'plant';  % 'plant' | 'estimator' | 'logger_e'
end
if ~exist('agent_id', 'var')
    agent_id = 1;
end
if ~exist('phase_option', 'var')
    phase_option = 'f';      % '' | 'f' | 'tfl'
end
if ~exist('time_range', 'var')
    time_range = [-inf, inf];
end
if ~exist('output_dir', 'var')
    output_dir = fullfile(script_dir, 'drawstyle_compare_figures');
end
if ~exist('reference_run_index', 'var')
    reference_run_index = 1;
end

run_colors = [
    0.10, 0.35, 0.90;
    0.86, 0.24, 0.12;
    0.00, 0.62, 0.32;
    0.85, 0.60, 0.10;
    0.45, 0.30, 0.75;
    0.20, 0.65, 0.70
];
run_styles = {'-', '-', '-', '-', '-', '-'};
run_markers = {'o', 's', '^', 'd', 'v', '>'};
marker_spacing = [45, 45, 20, 20, 20, 20];

font_name = 'Times New Roman';
font_size = 13;
line_width = 1.2;
ref_line_width = 2.0;
ref_color = [0.35, 0.35, 0.35];

if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% Plot each case
for case_idx = 1:numel(cases)
    files = cases(case_idx).files;
    labels = cases(case_idx).labels;
    assert(numel(files) == numel(labels), 'files and labels must have the same length.');
    assert(numel(files) <= size(run_colors, 1), 'Increase run_colors for more runs.');
    assert(reference_run_index >= 1 && reference_run_index <= numel(files), ...
        'reference_run_index is out of range.');

    runs = cell(1, numel(files));
    for i = 1:numel(files)
        runs{i} = load_run_data(files{i}, state_source, agent_id, time_range, phase_option, script_dir);
        runs{i}.label = labels{i};
    end

    common_range = intersect_time_range(runs);
    for i = 1:numel(runs)
        runs{i} = clip_run_to_time(runs{i}, common_range);
    end

    fig = figure(400 + case_idx); clf(fig);
    fig.Color = 'w';
    fig.Position(3:4) = [980, 760];
    ax = axes(fig);
    hold(ax, 'on');

    h_ref = plot3(ax, runs{reference_run_index}.ref(:, 1), runs{reference_run_index}.ref(:, 2), runs{reference_run_index}.ref(:, 3), ...
        'Color', ref_color, ...
        'LineStyle', '--', ...
        'LineWidth', ref_line_width, ...
        'DisplayName', 'Reference');

    h_runs = gobjects(1, numel(runs));
    plot_order = 1:numel(runs);
    if numel(runs) >= 3
        plot_order = [2, 3, 1, 4:numel(runs)];
    end

    for idx = plot_order
        p = runs{idx}.p;
        h_runs(idx) = plot3(ax, p(:, 1), p(:, 2), p(:, 3), ...
            'Color', run_colors(idx, :), ...
            'LineStyle', run_styles{idx}, ...
            'LineWidth', line_width + (idx == 1) * 0.4, ...
            'Marker', run_markers{idx}, ...
            'MarkerIndices', 1:marker_spacing(idx):size(p, 1), ...
            'MarkerSize', 4.5, ...
            'MarkerFaceColor', 'none', ...
            'DisplayName', runs{idx}.label);

        plot3(ax, p(1, 1), p(1, 2), p(1, 3), ...
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
    title(ax, sprintf('%s Trajectory Tracking Performance', cases(case_idx).name), ...
        'FontName', font_name, 'FontSize', font_size + 3);
    set(ax, 'FontName', font_name, 'FontSize', font_size, 'LineWidth', 1.0);
    legend(ax, [h_ref, h_runs], [{'Reference'}, labels], ...
        'Location', 'northeastoutside', ...
        'FontName', font_name, ...
        'FontSize', font_size - 1);

    safe_name = lower(regexprep(cases(case_idx).name, '[^a-zA-Z0-9]+', '_'));
    output_path = fullfile(output_dir, sprintf('%s_trajectory.png', safe_name));
    exportgraphics(fig, output_path, 'Resolution', 300);
    fprintf('Saved figure: %s\n', output_path);
end

%% Local functions
function run = load_run_data(file_path, state_source, agent_id, time_range, phase_option, script_dir)
    file_path = resolve_file(file_path, script_dir);

    switch lower(string(state_source))
        case "logger_e"
            logger = LOGGER(file_path);
            run = struct();
            run.t = get_series_logger(logger, 0, "t", "", time_range, phase_option);
            run.p = get_series_logger(logger, agent_id, "p", "e", time_range, phase_option);
            run.v = get_series_logger(logger, agent_id, "v", "e", time_range, phase_option);
            run.ref = get_series_logger(logger, agent_id, "p", "r", time_range, phase_option);
            run.ref_v = get_series_logger(logger, agent_id, "v", "r", time_range, phase_option);

        otherwise
            loaded = load(file_path);
            assert(isfield(loaded, 'log'), 'MAT file must contain a variable named log: %s', file_path);
            run = extract_run_from_log_struct(loaded.log, agent_id, char(state_source), time_range, phase_option);
    end
end

function run = extract_run_from_log_struct(log_struct, agent_id, state_source, time_range, phase_option)
    t_all = double(log_struct.Data.t(:));
    phase_all = char(log_struct.Data.phase(:).');
    last_idx = find(log_struct.Data.phase, 1, 'last');
    if isempty(last_idx)
        last_idx = numel(t_all);
    end

    t_all = t_all(1:last_idx);
    phase_all = lower(phase_all(1:last_idx)).';
    mask_time = t_all >= time_range(1) & t_all <= time_range(2);

    if isempty(phase_option)
        mask_phase = true(size(mask_time));
    else
        mask_phase = ismember(phase_all, lower(phase_option));
    end

    ids = find(mask_time & mask_phase);
    assert(~isempty(ids), 'No samples found for the selected phase/time range.');

    agent = log_struct.Data.agent(agent_id);
    n = numel(ids);

    run = struct();
    run.t = t_all(ids);
    run.p = nan(n, 3);
    run.v = nan(n, 3);
    run.ref = nan(n, 3);
    run.ref_v = nan(n, 3);

    for i = 1:n
        k = ids(i);
        st = get_state_entry(agent, state_source, k);
        ref = agent.reference.result{1, k}.state;

        run.p(i, :) = double(st.p(:))';
        run.v(i, :) = double(st.v(:))';
        run.ref(i, :) = extract_ref_p(ref)';
        run.ref_v(i, :) = extract_ref_v(ref)';
    end
end

function st = get_state_entry(agent, state_source, k)
    switch lower(state_source)
        case 'plant'
            st = agent.plant.result{1, k}.state;
        case 'estimator'
            st = agent.estimator.result{1, k}.state;
        otherwise
            error('Unsupported state_source: %s', state_source);
    end
end

function p = extract_ref_p(ref)
    if (isstruct(ref) && isfield(ref, 'p')) || isprop(ref, 'p')
        p = double(ref.p(:));
    elseif ((isstruct(ref) && isfield(ref, 'xd')) || isprop(ref, 'xd')) && numel(ref.xd) >= 3
        p = double(ref.xd(1:3));
    else
        p = zeros(3, 1);
    end
end

function v = extract_ref_v(ref)
    if (isstruct(ref) && isfield(ref, 'v')) || isprop(ref, 'v')
        v = double(ref.v(:));
    elseif ((isstruct(ref) && isfield(ref, 'xd')) || isprop(ref, 'xd')) && numel(ref.xd) >= 7
        v = double(ref.xd(5:7));
    else
        v = zeros(3, 1);
    end
end

function data = get_series_logger(logger_obj, agent_id, variable, attribute, time_range, phase_option)
    if variable == "t"
        if nargin < 6 || isempty(phase_option)
            raw = logger_obj.data(0, "t", "", "ranget", time_range);
        else
            raw = logger_obj.data(0, "t", "", "ranget", time_range, "phase", phase_option);
        end
        data = normalize_series(raw);
        return;
    end

    if nargin < 6 || isempty(phase_option)
        raw = logger_obj.data(agent_id, variable, attribute, "ranget", time_range);
    else
        raw = logger_obj.data(agent_id, variable, attribute, "ranget", time_range, "phase", phase_option);
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

function file_path = resolve_file(file_path, script_dir)
    if isfile(file_path)
        return;
    end
    candidate = fullfile(script_dir, file_path);
    assert(isfile(candidate), 'File not found: %s', file_path);
    file_path = candidate;
end

function time_range = intersect_time_range(runs)
    t_starts = zeros(1, numel(runs));
    t_ends = zeros(1, numel(runs));
    for idx = 1:numel(runs)
        t_starts(idx) = min(runs{idx}.t);
        t_ends(idx) = max(runs{idx}.t);
    end
    time_range = [max(t_starts), min(t_ends)];
end

function run = clip_run_to_time(run, time_range)
    ids = run.t >= time_range(1) & run.t <= time_range(2);
    run.t = run.t(ids);
    run.p = run.p(ids, :);
    run.v = run.v(ids, :);
    run.ref = run.ref(ids, :);
    run.ref_v = run.ref_v(ids, :);
end
