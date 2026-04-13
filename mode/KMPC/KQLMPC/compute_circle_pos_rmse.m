clear; clc;

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(fileparts(script_dir)));
addpath(genpath(repo_root));

files = {
    'Perturbed',  fullfile(script_dir, 'circleerr_Log(05-Apr-2026_15_49_39).mat');
    'Unsmoothed', fullfile(script_dir, 'circleerror36comerror_Log(05-Apr-2026_17_26_22).mat');
    'Smoothed',   fullfile(script_dir, 'circleerror36commode5_Log(05-Apr-2026_17_27_44).mat');
};

rows = cell(size(files, 1), 2);

entry_info = detect_circle_entry(files{1, 2});
fprintf('Circle-entry analysis starts at t = %.3f s (index %d).\n', ...
    entry_info.t_start, entry_info.idx_start);

for i = 1:size(files, 1)
    label = files{i, 1};
    mat_path = files{i, 2};
    pos_rmse = extract_pos_rmse(mat_path, entry_info.idx_start);
    rows(i, :) = {label, pos_rmse};
end

T = cell2table(rows, 'VariableNames', {'Case', 'PosRMSE_m'});
disp(T);
writetable(T, fullfile(script_dir, 'circle_pos_rmse.csv'));

function pos_rmse = extract_pos_rmse(mat_path, idx_start)
    logger = LOGGER(mat_path);
    p = normalize_series(logger.data(1, "p", "e"));
    p_ref = normalize_series(logger.data(1, "p", "r"));

    n = min(size(p, 1), size(p_ref, 1));
    idx_start = min(max(1, idx_start), n);
    pos_err = p(idx_start:n, :) - p_ref(idx_start:n, :);
    pos_err_norm = vecnorm(pos_err, 2, 2);
    pos_rmse = sqrt(mean(pos_err_norm.^2, 'omitnan'));
end

function info = detect_circle_entry(mat_path)
    logger = LOGGER(mat_path);
    t = normalize_series(logger.data(0, "t", ""));
    p_ref = normalize_series(logger.data(1, "p", "r"));

    n = min(numel(t), size(p_ref, 1));
    t = t(1:n);
    p_ref = p_ref(1:n, :);

    xy = p_ref(:, 1:2);
    tail_idx = max(1, round(0.6 * n)):n;
    center_xy = mean(xy(tail_idx, :), 1, 'omitnan');
    radius = vecnorm(xy - center_xy, 2, 2);
    radius_final = median(radius(tail_idx), 'omitnan');

    tol = max(0.03, 0.08 * radius_final);
    window = max(10, round(0.03 * n));

    idx_start = 1;
    for k = 1:(n - window + 1)
        segment = radius(k:k + window - 1);
        if all(abs(segment - radius_final) <= tol)
            idx_start = k;
            break;
        end
    end

    info = struct();
    info.idx_start = idx_start;
    info.t_start = t(idx_start);
    info.radius_final = radius_final;
    info.center_xy = center_xy;
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
