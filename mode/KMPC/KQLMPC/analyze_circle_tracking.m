clear; clc;

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(fileparts(script_dir)));
addpath(genpath(repo_root));

files = {
    'Perturbed',  fullfile(script_dir, 'circleerr52s_Log(06-Apr-2026_17_35_48).mat');
    'Unsmoothed', fullfile(script_dir, 'circleerror36comerror_Log(05-Apr-2026_17_26_22).mat');
    'Smoothed',   fullfile(script_dir, 'circleerrcom52smode5_Log(06-Apr-2026_17_34_20).mat');
};

ref_info = analyze_reference(files{1,2});
fprintf('Estimated circle center: [%.4f, %.4f]\n', ref_info.center_xy(1), ref_info.center_xy(2));
fprintf('Estimated steady radius: %.4f m\n', ref_info.radius_ref);
fprintf('Steady-circle start index: %d (t = %.3f s)\n', ref_info.steady_idx(1), ref_info.t_steady_start);
fprintf('Last-lap index range: %d:%d\n', ref_info.last_lap_idx(1), ref_info.last_lap_idx(end));

rows = cell(size(files,1), 5);
for i = 1:size(files,1)
    label = files{i,1};
    metrics = analyze_run(files{i,2}, ref_info);
    rows(i,:) = {label, metrics.pos_rmse_all, metrics.pos_rmse_steady, ...
        metrics.pos_rmse_lastlap, metrics.center_offset};
end

T = cell2table(rows, 'VariableNames', ...
    {'Case','PosRMSE_All_m','PosRMSE_Steady_m','PosRMSE_LastLap_m','CenterOffset_m'});
disp(T);
writetable(T, fullfile(script_dir, 'circle_tracking_metrics.csv'));

function ref_info = analyze_reference(mat_path)
    logger = LOGGER(mat_path);
    t = normalize_series(logger.data(0, "t", ""));
    p_ref = normalize_series(logger.data(1, "p", "r"));
    n = min(numel(t), size(p_ref, 1));
    t = t(1:n);
    p_ref = p_ref(1:n, :);

    tail_idx = max(1, round(0.6*n)):n;
    xy_tail = p_ref(tail_idx,1:2);
    center_xy = mean(xy_tail, 1, 'omitnan');
    theta = unwrap(atan2(p_ref(:,2)-center_xy(2), p_ref(:,1)-center_xy(1)));

    theta_end = theta(end);
    idx_start = find(theta >= theta_end - 2*pi, 1, 'first');
    if isempty(idx_start)
        idx_start = max(1, n - round(0.2*n));
    end

    radius_all = vecnorm(p_ref(:,1:2) - center_xy, 2, 2);
    ref_radius_guess = radius_ref_candidate(radius_all(idx_start:n));
    tol = max(0.02, 0.05 * ref_radius_guess);
    window = max(20, round(0.04 * n));
    steady_start = idx_start;
    for k = 1:max(1, idx_start - window + 1)
        seg_ids = k:min(n, k + window - 1);
        seg = radius_all(seg_ids);
        if all(abs(seg - median(radius_all(idx_start:n), 'omitnan')) <= tol)
            steady_start = k;
            break;
        end
    end

    last_lap_idx = idx_start:n;
    radius_ref = mean(vecnorm(p_ref(last_lap_idx,1:2) - center_xy, 2, 2), 'omitnan');

    ref_info = struct();
    ref_info.center_xy = center_xy;
    ref_info.radius_ref = radius_ref;
    ref_info.steady_idx = steady_start:n;
    ref_info.t_steady_start = t(steady_start);
    ref_info.last_lap_idx = last_lap_idx;
end

function metrics = analyze_run(mat_path, ref_info)
    logger = LOGGER(mat_path);
    p = normalize_series(logger.data(1, "p", "e"));
    p_ref = normalize_series(logger.data(1, "p", "r"));

    n = min(size(p,1), size(p_ref,1));
    p = p(1:n,:);
    p_ref = p_ref(1:n,:);

    pos_err = p - p_ref;
    pos_err_norm = vecnorm(pos_err, 2, 2);

    steady_idx = ref_info.steady_idx;
    steady_idx = steady_idx(steady_idx <= n);
    if isempty(steady_idx)
        steady_idx = 1:n;
    end

    last_idx = ref_info.last_lap_idx;
    last_idx = last_idx(last_idx <= n);
    if isempty(last_idx)
        last_idx = max(1, n - 20):n;
    end

    p_last = p(last_idx,1:2);
    center_est = mean(p_last, 1, 'omitnan');
    center_offset = norm(center_est - ref_info.center_xy);

    metrics = struct();
    metrics.pos_rmse_all = sqrt(mean(pos_err_norm.^2, 'omitnan'));
    metrics.pos_rmse_steady = sqrt(mean(pos_err_norm(steady_idx).^2, 'omitnan'));
    metrics.pos_rmse_lastlap = sqrt(mean(pos_err_norm(last_idx).^2, 'omitnan'));
    metrics.center_offset = center_offset;
end

function val = radius_ref_candidate(radius_segment)
    val = median(radius_segment, 'omitnan');
    if ~isfinite(val) || val <= 0
        val = 1.0;
    end
end

function data = normalize_series(raw)
    data = double(squeeze(raw));
    if isvector(data)
        data = data(:);
    end
    if size(data,1) < size(data,2)
        data = data.';
    end
end
