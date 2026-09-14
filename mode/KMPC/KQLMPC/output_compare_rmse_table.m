%% RMSE output for prepared logs
% Edit the "User settings" block only.
clearvars -except cases state_source agent_id phase_option time_range output_csv improvement_base_index;
clc;

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
if ~exist('output_csv', 'var')
    output_csv = fullfile(script_dir, 'compare_rmse_output.csv');
end
if ~exist('improvement_base_index', 'var')
    improvement_base_index = 1;
end

%% Compute RMSE
rows = {};
row_id = 0;

for case_idx = 1:numel(cases)
    files = cases(case_idx).files;
    labels = cases(case_idx).labels;
    assert(numel(files) == numel(labels), 'files and labels must have the same length.');
    assert(improvement_base_index >= 1 && improvement_base_index <= numel(files), ...
        'improvement_base_index is out of range.');

    runs = cell(1, numel(files));
    for i = 1:numel(files)
        runs{i} = load_run_data(files{i}, state_source, agent_id, time_range, phase_option, script_dir);
        runs{i}.label = labels{i};
    end

    common_range = intersect_time_range(runs);
    for i = 1:numel(runs)
        runs{i} = clip_run_to_time(runs{i}, common_range);
    end

    base_metrics = compute_rmse_metrics(runs{improvement_base_index});
    base_label = string(runs{improvement_base_index}.label);
    for i = 1:numel(runs)
        mt = compute_rmse_metrics(runs{i});
        row_id = row_id + 1;
        rows(row_id, :) = { ...
            string(cases(case_idx).name), ...
            string(runs{i}.label), ...
            base_label, ...
            mt.rmse_p1, ...
            mt.rmse_p2, ...
            mt.rmse_p3, ...
            mt.rmse_pnorm, ...
            mt.rmse_v1, ...
            mt.rmse_v2, ...
            mt.rmse_v3, ...
            mt.rmse_vnorm, ...
            safe_improvement(base_metrics.rmse_p1, mt.rmse_p1), ...
            safe_improvement(base_metrics.rmse_p2, mt.rmse_p2), ...
            safe_improvement(base_metrics.rmse_p3, mt.rmse_p3), ...
            safe_improvement(base_metrics.rmse_pnorm, mt.rmse_pnorm), ...
            safe_improvement(base_metrics.rmse_v1, mt.rmse_v1), ...
            safe_improvement(base_metrics.rmse_v2, mt.rmse_v2), ...
            safe_improvement(base_metrics.rmse_v3, mt.rmse_v3), ...
            safe_improvement(base_metrics.rmse_vnorm, mt.rmse_vnorm)};
    end
end

summary_table = cell2table(rows, 'VariableNames', { ...
    'Case', 'RunLabel', 'ImproveBaseLabel', ...
    'RMSE_p1', 'RMSE_p2', 'RMSE_p3', 'RMSE_pnorm', ...
    'RMSE_v1', 'RMSE_v2', 'RMSE_v3', 'RMSE_vnorm', ...
    'ImprovePct_p1_vs_base', 'ImprovePct_p2_vs_base', ...
    'ImprovePct_p3_vs_base', 'ImprovePct_pnorm_vs_base', ...
    'ImprovePct_v1_vs_base', 'ImprovePct_v2_vs_base', ...
    'ImprovePct_v3_vs_base', 'ImprovePct_vnorm_vs_base'});

summary_numeric = table2array(summary_table(:, 4:end));

disp(summary_table);
fprintf('Numeric RMSE matrix:\n');
disp(summary_numeric);
writetable(summary_table, output_csv);
fprintf('Saved RMSE table: %s\n', output_csv);

assignin('base', 'compare_rmse_summary_table', summary_table);
assignin('base', 'compare_rmse_summary_numeric', summary_numeric);

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

function mt = compute_rmse_metrics(run)
    ep = run.p - run.ref;
    ev = run.v - run.ref_v;

    mt = struct();
    mt.rmse_p1 = sqrt(mean(ep(:, 1).^2, 'omitnan'));
    mt.rmse_p2 = sqrt(mean(ep(:, 2).^2, 'omitnan'));
    mt.rmse_p3 = sqrt(mean(ep(:, 3).^2, 'omitnan'));
    mt.rmse_pnorm = sqrt(mean(sum(ep.^2, 2), 'omitnan'));
    mt.rmse_v1 = sqrt(mean(ev(:, 1).^2, 'omitnan'));
    mt.rmse_v2 = sqrt(mean(ev(:, 2).^2, 'omitnan'));
    mt.rmse_v3 = sqrt(mean(ev(:, 3).^2, 'omitnan'));
    mt.rmse_vnorm = sqrt(mean(sum(ev.^2, 2), 'omitnan'));
end

function val = safe_improvement(base_val, comp_val)
    if ~isfinite(base_val) || abs(base_val) < 1e-12 || ~isfinite(comp_val)
        val = NaN;
    else
        val = (base_val - comp_val) / base_val * 100;
    end
end
