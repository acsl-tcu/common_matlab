%% Nominal / model-error / EDMD-MEC trajectory comparison template
% Fill DATA_DIR and each case file name before running.
% Black line is read from log.Data.agent.reference.result{k}.state.p.
% It is labeled as Nominal here for presentation consistency.

clear; close all hidden; clc;

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(fileparts(script_dir)));
addpath(genpath(repo_root));

%% ====================== User Settings ======================
DATA_DIR = '';  % Example: 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data'

cases = struct( ...
    'name', { ...
        'star' ...
    }, ...
    'nominal_with_error_file', { ...
        'exphlmpcstarnomec_Log(18-Jun-2026_17_02_48).mat' ...
    }, ...
    'nominal_edmd_mec_file', { ...
        'exphlmpcstarmec2_Log(18-Jun-2026_17_05_11).mat' ...
    });

PHASE_VAL = 102;          % 102 = flight phase in many experimental logs
USE_PHASE_CROP = true;
VIEW_MODE = 2;            % 2: top view, 3: 3-D view
SAVE_FIGURE = false;      % true if you want automatic png export
OUTPUT_DIR = fullfile(script_dir, 'nominal_error_edmd_mec_figures');
CSV_OUT = fullfile(script_dir, 'nominal_error_edmd_mec_rmse_summary.csv');

font_name = 'Times New Roman';
font_size = 13;
line_width = 1.35;

col_ref  = [0.00 0.00 0.00];
col_pert = [0.86 0.24 0.12];
col_comp = [0.00 0.62 0.32];
label_nominal = 'Nominal';
label_error = 'Nominal with error';
label_mec = 'Nominal + EDMD-MEC';

if SAVE_FIGURE && ~exist(OUTPUT_DIR, 'dir')
    mkdir(OUTPUT_DIR);
end

%% ====================== Main ======================
summary = table('Size', [0 5], ...
    'VariableTypes', {'string','double','double','double','double'}, ...
    'VariableNames', {'Trajectory','Nominal_RMSE_m','NominalWithError_RMSE_m','Nominal_EDMD_MEC_RMSE_m','Improvement_pct'});

for case_idx = 1:numel(cases)
    case_name = string(cases(case_idx).name);
    pert_file = get_case_file(cases(case_idx), 'nominal_with_error_file', 'pert_file');
    comp_file = get_case_file(cases(case_idx), 'nominal_edmd_mec_file', 'comp_file');
    pert_path = fullfile(DATA_DIR, pert_file);
    comp_path = fullfile(DATA_DIR, comp_file);

    if ~isfile(pert_path)
        error('Perturbed file not found: %s', pert_path);
    end
    if ~isfile(comp_path)
        error('Compensated file not found: %s', comp_path);
    end

    P = load_run(pert_path, PHASE_VAL, USE_PHASE_CROP);
    C = load_run(comp_path, PHASE_VAL, USE_PHASE_CROP);

    [P, C] = align_by_length(P, C);

    rmse_nominal = 0.0;
    rmse_error = rmse_pos(P.p, P.pref);
    rmse_mec = rmse_pos(C.p, C.pref);
    improvement = 100 * (rmse_error - rmse_mec) / max(rmse_error, eps);

    fprintf('\n==== %s ====\n', case_name);
    fprintf('Nominal RMSE = %.6f m\n', rmse_nominal);
    fprintf('Nominal with error RMSE = %.6f m\n', rmse_error);
    fprintf('Nominal + EDMD-MEC RMSE = %.6f m\n', rmse_mec);
    fprintf('Improvement = %.2f %%\n', improvement);

    summary = [summary; {case_name, rmse_nominal, rmse_error, rmse_mec, improvement}]; %#ok<AGROW>

    fig = figure(300 + case_idx); clf(fig);
    fig.Color = 'w';
    fig.Position(3:4) = [900, 720];
    ax = axes(fig);
    hold(ax, 'on');

    h_ref = plot3(ax, P.pref(:,1), P.pref(:,2), P.pref(:,3), ...
        'Color', col_ref, ...
        'LineStyle', '-', ...
        'LineWidth', 1.8, ...
        'DisplayName', label_nominal);

    h_pert = plot3(ax, P.p(:,1), P.p(:,2), P.p(:,3), ...
        'Color', col_pert, ...
        'LineStyle', '-', ...
        'LineWidth', line_width, ...
        'DisplayName', label_error);

    h_comp = plot3(ax, C.p(:,1), C.p(:,2), C.p(:,3), ...
        'Color', col_comp, ...
        'LineStyle', '-', ...
        'LineWidth', line_width, ...
        'DisplayName', label_mec);

    plot3(ax, P.p(1,1), P.p(1,2), P.p(1,3), 'o', ...
        'MarkerFaceColor', col_pert, 'MarkerEdgeColor', col_pert, ...
        'HandleVisibility', 'off');
    plot3(ax, C.p(1,1), C.p(1,2), C.p(1,3), 'o', ...
        'MarkerFaceColor', col_comp, 'MarkerEdgeColor', col_comp, ...
        'HandleVisibility', 'off');

    grid(ax, 'on');
    box(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'x [m]', 'FontName', font_name, 'FontSize', font_size);
    ylabel(ax, 'y [m]', 'FontName', font_name, 'FontSize', font_size);
    zlabel(ax, 'z [m]', 'FontName', font_name, 'FontSize', font_size);
    title(ax, sprintf('%s: Nominal / Error / EDMD-MEC  RMSE %.3f -> %.3f m (%.1f%%)', ...
        case_name, rmse_error, rmse_mec, improvement), ...
        'FontName', font_name, 'FontSize', font_size + 1);
    set(ax, 'FontName', font_name, 'FontSize', font_size, 'LineWidth', 1.0);
    legend(ax, [h_ref, h_pert, h_comp], ...
        {label_nominal, label_error, label_mec}, ...
        'Location', 'best', 'FontName', font_name, 'FontSize', font_size - 1);

    if VIEW_MODE == 2
        view(ax, 2);
    else
        view(ax, [-38, 24]);
    end

    if SAVE_FIGURE
        safe_name = lower(regexprep(case_name, '[^a-zA-Z0-9]+', '_'));
        exportgraphics(fig, fullfile(OUTPUT_DIR, safe_name + "_nominal_error_edmd_mec.png"), 'Resolution', 300);
    end

    fig2 = figure(500 + case_idx); clf(fig2);
    fig2.Color = 'w';
    ax2 = axes(fig2);
    eP = vecnorm(P.p - P.pref, 2, 2);
    eC = vecnorm(C.p - C.pref, 2, 2);
    plot(ax2, P.t - P.t(1), eP, 'Color', col_pert, 'LineWidth', 1.2); hold(ax2, 'on');
    plot(ax2, C.t - C.t(1), eC, 'Color', col_comp, 'LineWidth', 1.2);
    grid(ax2, 'on');
    xlabel(ax2, 't [s]', 'FontName', font_name, 'FontSize', font_size);
    ylabel(ax2, '|p - p_{ref}| [m]', 'FontName', font_name, 'FontSize', font_size);
    legend(ax2, {label_error, label_mec}, 'Location', 'best');
    title(ax2, case_name + " model-error and EDMD-MEC position error", ...
        'FontName', font_name, 'FontSize', font_size + 1);
end

writetable(summary, CSV_OUT);
disp(summary);
fprintf('\nSaved RMSE summary: %s\n', CSV_OUT);

%% ====================== Local Functions ======================
function file_name = get_case_file(case_cfg, preferred_field, fallback_field)
    if isfield(case_cfg, preferred_field) && ~isempty(case_cfg.(preferred_field))
        file_name = case_cfg.(preferred_field);
    elseif isfield(case_cfg, fallback_field) && ~isempty(case_cfg.(fallback_field))
        file_name = case_cfg.(fallback_field);
    else
        error('Missing file field: %s', preferred_field);
    end
end

function r = load_run(file_path, phase_val, use_phase_crop)
    log = LOGGER(file_path);
    D = log.Data;

    if use_phase_crop
        ph = D.phase(:);
        idx = find(ph == phase_val);
        if numel(idx) < 2
            error('%s: phase == %g has fewer than two samples.', file_path, phase_val);
        end
        sel = idx(2):idx(end);
    else
        sel = 1:double(log.k);
    end

    r.t = D.t(sel);
    r.t = r.t(:);
    n = numel(sel);
    r.p = zeros(n, 3);
    r.pref = zeros(n, 3);

    est_res = D.agent.estimator.result;
    ref_res = D.agent.reference.result;

    for ii = 1:n
        k = sel(ii);
        r.p(ii,:) = get_state_position(est_res{1,k}.state).';
        r.pref(ii,:) = get_state_position(ref_res{1,k}.state).';
    end
end

function p = get_state_position(st)
    if isobject(st)
        if isprop(st, 'p')
            p = st.p(:);
            return;
        end
        if isprop(st, 'xd')
            xd = st.xd(:);
            p = xd(1:3);
            return;
        end
    elseif isstruct(st)
        if isfield(st, 'p')
            p = st.p(:);
            return;
        end
        if isfield(st, 'xd')
            xd = st.xd(:);
            p = xd(1:3);
            return;
        end
    end
    error('State has no readable p or xd field.');
end

function [A, B] = align_by_length(A, B)
    n = min([numel(A.t), numel(B.t), size(A.p,1), size(B.p,1)]);
    A.t = A.t(1:n); A.p = A.p(1:n,:); A.pref = A.pref(1:n,:);
    B.t = B.t(1:n); B.p = B.p(1:n,:); B.pref = B.pref(1:n,:);
end

function e = rmse_pos(p, pref)
    d = vecnorm(p - pref, 2, 2);
    e = sqrt(mean(d.^2));
end
