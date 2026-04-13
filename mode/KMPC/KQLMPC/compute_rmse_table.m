clear; clc;

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(fileparts(script_dir)));
addpath(genpath(repo_root));

root = script_dir;

pairs = {
    'Figure8',  'figure8err52s_Log(05-Apr-2026_18_25_40).mat',      'figure8errcom52smode5_Log(06-Apr-2026_15_17_59).mat',       'Perturbed',  'Compensated';
    'Heart',    'hearterr36s_Log(05-Apr-2026_15_10_12).mat',        'hearterrcom36smode5_Log(06-Apr-2026_15_15_45).mat',         'Perturbed',  'Compensated';
    'Square',   'square36err_Log(05-Apr-2026_17_40_58).mat',        'square36errcommode5_Log(05-Apr-2026_17_42_02).mat',    'Perturbed',  'Compensated';
    'Star',     'starerr48_Log(05-Apr-2026_17_15_22).mat',          'starerrcom48smode5_Log(06-Apr-2026_15_19_36).mat',           'Perturbed',  'Compensated';
    'Circle',   'circleerr52s_Log(06-Apr-2026_17_35_48).mat',          'circleerrcom52smode5_Log(06-Apr-2026_17_34_20).mat',  'Perturbed',  'Compensated';
};

rows = cell(size(pairs, 1), 9);

for i = 1:size(pairs, 1)
    case_name  = pairs{i, 1};
    base_file  = fullfile(root, pairs{i, 2});
    comp_file  = fullfile(root, pairs{i, 3});
    base_label = pairs{i, 4};
    comp_label = pairs{i, 5};

    base = extract_run(base_file);
    comp = extract_run(comp_file);

    rows(i, :) = { ...
        case_name, ...
        base_label, ...
        comp_label, ...
        base.pos_rmse, ...
        comp.pos_rmse, ...
        safe_improvement(base.pos_rmse, comp.pos_rmse), ...
        base.vel_rmse, ...
        comp.vel_rmse, ...
        safe_improvement(base.vel_rmse, comp.vel_rmse) ...
    };
end

T = cell2table(rows, 'VariableNames', { ...
    'Case', 'BaseLabel', 'CompLabel', ...
    'BasePosRMSE_m', 'CompPosRMSE_m', 'PosImprove_pct', ...
    'BaseVelRMSE_mps', 'CompVelRMSE_mps', 'VelImprove_pct'});

disp(T);
writetable(T, fullfile(root, 'rmse_table.csv'));

function metrics = extract_run(mat_path)
    logger = LOGGER(mat_path);

    p = normalize_series(logger.data(1, "p", "e"));
    v = normalize_series(logger.data(1, "v", "e"));
    p_ref = normalize_series(logger.data(1, "p", "r"));
    v_ref = normalize_series(logger.data(1, "v", "r"));

    n_pos = min(size(p, 1), size(p_ref, 1));
    n_vel = min(size(v, 1), size(v_ref, 1));

    pos_err = p(1:n_pos, :) - p_ref(1:n_pos, :);
    vel_err = v(1:n_vel, :) - v_ref(1:n_vel, :);

    pos_err_norm = vecnorm(pos_err, 2, 2);
    vel_err_norm = vecnorm(vel_err, 2, 2);

    metrics = struct();
    metrics.pos_rmse = sqrt(mean(pos_err_norm.^2, 'omitnan'));
    metrics.vel_rmse = sqrt(mean(vel_err_norm.^2, 'omitnan'));
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

function val = safe_improvement(base_val, comp_val)
    if ~isfinite(base_val) || base_val == 0 || ~isfinite(comp_val)
        val = NaN;
    else
        val = (base_val - comp_val) / base_val * 100;
    end
end
