function results_edmd = train_edmd_residual_paired_from_logs(opts)
% nominalログとdegradedログを分けて学習する関数
% nominalデータで公称モデルを作り，degradedデータで残差モデルを学習する
% TRAIN_EDMD_RESIDUAL_PAIRED_FROM_LOGS
% Train nominal EDMD from nominal logs and residual EDMD from degraded logs.

    if nargin < 1 || ~isstruct(opts)
        error('opts must be a struct.');
    end

    % 学習パラメータのデフォルト値を設定する
    opts = set_default(opts, 'agent_idx', 1);
    opts = set_default(opts, 'state_source', 'plant');
    opts = set_default(opts, 'lambda_nom', 1e-4);
    opts = set_default(opts, 'lambda_err', 1e-1);
    opts = set_default(opts, 'pos_err_thresh', 2.0);
    opts = set_default(opts, 'vel_thresh', 6.0);
    opts = set_default(opts, 'ang_thresh', pi / 3);
    opts = set_default(opts, 'obs_norm_thresh', 200.0);
    opts = set_default(opts, 'res_norm_thresh', 20.0);
    opts = set_default(opts, 'cos_guard', 1e-3);
    opts = set_default(opts, 'du_ratio_thresh', 0.35);
    opts = set_default(opts, 'du_abs_thresh', 3.0);
    opts = set_default(opts, 'jump_norm_hard_thresh', 80.0);
    opts = set_default(opts, 'mad_scale', 4.0);
    opts = set_default(opts, 'use_gpu', gpuDeviceCount > 0);
    opts = set_default(opts, 'run_indices', []);

    required_fields = {'data_dir', 'nominal_file_pattern', 'degraded_file_pattern', 'save_path'};
    for i = 1:numel(required_fields)
        if ~isfield(opts, required_fields{i}) || isempty(opts.(required_fields{i}))
            error('opts.%s is required.', required_fields{i});
        end
    end

    state_source = lower(char(opts.state_source));
    if ~ismember(state_source, {'estimator', 'plant'})
        error('opts.state_source must be ''estimator'' or ''plant''.');
    end

    if opts.use_gpu
        fprintf('[GPU] %s\n', gpuDevice().Name);
    else
        fprintf('[CPU] GPU not found, running on CPU.\n');
    end

    % nominalデータを読み込み，公称モデル用サンプルを作成する
    fprintf('\\n=== Load nominal dataset ===\\n');
    [Z_nom, ZN_nom, U_nom, DU_nom, stats_nom] = load_edmd_samples(opts, opts.nominal_file_pattern, state_source);
    [Z_nom, ZN_nom, U_nom, DU_nom, nom_keep] = prune_samples(Z_nom, ZN_nom, U_nom, DU_nom, opts);
    stats_nom.n_clean_after_prune = size(Z_nom, 2);
    fprintf('nominal kept after pruning: %d / %d\n', nnz(nom_keep), numel(nom_keep));

    % nominalデータからA_nom, B_nomを学習する
    fprintf('\\n=== Train nominal EDMD from nominal plant ===\\n');
    [A_nom, B_nom] = solve_ridge(Z_nom, U_nom, ZN_nom, opts.lambda_nom, opts.use_gpu);
    ZN_nom_hat = A_nom * Z_nom + B_nom * U_nom;
    rmse_nom = sqrt(mean((ZN_nom - ZN_nom_hat).^2, 'all'));
    fprintf('nominal samples : %d\n', size(Z_nom, 2));
    fprintf('nominal RMSE    : %.6e\n', rmse_nom);

    % degradedデータを読み込み，残差モデル用サンプルを作成する
    fprintf('\\n=== Load degraded dataset ===\\n');
    [Z_deg, ZN_deg, U_deg, DU_deg, stats_deg] = load_edmd_samples(opts, opts.degraded_file_pattern, state_source);
    [Z_deg, ZN_deg, U_deg, DU_deg, deg_keep] = prune_samples(Z_deg, ZN_deg, U_deg, DU_deg, opts);
    stats_deg.n_clean_after_prune = size(Z_deg, 2);
    fprintf('degraded kept after pruning: %d / %d\n', nnz(deg_keep), numel(deg_keep));

    % degradedデータと公称モデルの差からA_err, B_errを学習する
    fprintf('\\n=== Train residual EDMD from degraded plant against nominal model ===\\n');
    ZN_deg_nom = A_nom * Z_deg + B_nom * U_deg;
    ZBAR = ZN_deg - ZN_deg_nom;
    keep_idx = vecnorm(ZBAR, 2, 1) <= opts.res_norm_thresh;
    Z2 = Z_deg(:, keep_idx);
    DU2 = DU_deg(:, keep_idx);
    ZBAR2 = ZBAR(:, keep_idx);
    fprintf('residual samples kept: %d / %d\n', size(Z2, 2), size(Z_deg, 2));
    if isempty(Z2)
        error('No residual samples remained after residual filtering.');
    end

    [A_err, B_err] = solve_ridge(Z2, DU2, ZBAR2, opts.lambda_err, opts.use_gpu);
    ZBAR_hat = A_err * Z2 + B_err * DU2;
    rmse_err = sqrt(mean((ZBAR2 - ZBAR_hat).^2, 'all'));
    rho_err = max(abs(eig(A_err)));
    ZN_total = A_nom * Z_deg + B_nom * U_deg + A_err * Z_deg + B_err * DU_deg;
    rmse_total = sqrt(mean((ZN_deg - ZN_total).^2, 'all'));

    du_norm_all = vecnorm(DU_deg, 2, 1);
    du2_norm = vecnorm(DU2, 2, 1);
    zbar2_norm = vecnorm(ZBAR2, 2, 1);
    prune_delete_pct = 100 * (1 - nnz(deg_keep) / max(numel(deg_keep), 1));
    residual_keep_pct = 100 * size(Z2, 2) / max(size(Z_deg, 2), 1);
    b_col_norm = vecnorm(B_err, 2, 1);
    lambda_sweep = [1e-3, 1e-2, 1e-1, opts.lambda_err];
    lambda_sweep = unique(lambda_sweep, 'stable');
    lambda_b_norm = zeros(size(lambda_sweep));
    for li = 1:numel(lambda_sweep)
        [~, B_tmp] = solve_ridge(Z2, DU2, ZBAR2, lambda_sweep(li), false);
        lambda_b_norm(li) = norm(B_tmp, 'fro');
    end

    stats = struct();
    stats.state_source = state_source;
    stats.nominal = stats_nom;
    stats.degraded = stats_deg;
    stats.du_nonzero_count = nnz(du_norm_all > 1e-8);
    stats.du_nonzero_ratio = stats.du_nonzero_count / numel(du_norm_all);
    stats.du_mean_norm = mean(du_norm_all);
    stats.du_max_norm = max(du_norm_all);

    fprintf('residual RMSE         : %.6e\n', rmse_err);
    fprintf('combined RMSE         : %.6e\n', rmse_total);
    fprintf('rho(A_err)            : %.6f\n', rho_err);
    fprintf('delta_u nonzero ratio : %.4f\n', stats.du_nonzero_ratio);
    fprintf('rank(B_err)           : %d / %d\n', rank(B_err), size(B_err, 2));
    fprintf('||B_err||_F           : %.6e\n', norm(B_err, 'fro'));
    fprintf('||A_err||_F           : %.6e\n', norm(A_err, 'fro'));

    % 補償が弱い場合の原因を確認する診断値を表示する
    fprintf('\\n===== Weak compensation diagnostic =====\\n');
    fprintf('1. DU2 mean norm       : %.6f\n', mean(du2_norm));
    fprintf('   DU2 max norm        : %.6f\n', max(du2_norm));
    fprintf('2. rank(B_err)         : %d / %d\n', rank(B_err), size(B_err, 2));
    fprintf('3. ||B_err||_F         : %.6e\n', norm(B_err, 'fro'));
    fprintf('4. B_err column norms  : [%.4e %.4e %.4e %.4e]\n', ...
        b_col_norm(1), b_col_norm(2), b_col_norm(3), b_col_norm(4));
    fprintf('5. prune delete ratio  : %.2f %%\n', prune_delete_pct);
    fprintf('   residual keep ratio : %.2f %%\n', residual_keep_pct);
    fprintf('6. ZBAR2 mean norm     : %.6f\n', mean(zbar2_norm));
    fprintf('   ZBAR2 max norm      : %.6f\n', max(zbar2_norm));
    for li = 1:numel(lambda_sweep)
        fprintf('7. lambda %.3g ||B_err||_F : %.6e\n', ...
            lambda_sweep(li), lambda_b_norm(li));
    end

    % 学習結果をHLC_EDMD_MEC.mで読み込める形式にまとめる
    results_edmd = struct();
    results_edmd.A_nom = A_nom;
    results_edmd.B_nom = B_nom;
    results_edmd.A_err = A_err;
    results_edmd.B_err = B_err;
    results_edmd.A_total = A_nom + A_err;
    results_edmd.B_delta = B_err;
    results_edmd.rmse_nom = rmse_nom;
    results_edmd.rmse_err = rmse_err;
    results_edmd.rmse_total = rmse_total;
    results_edmd.rho_A_err = rho_err;
    results_edmd.n_z = size(Z_nom, 1);
    results_edmd.N = size(Z_deg, 2);
    results_edmd.observable_name = 'common_z_plus_kyo_z';
    results_edmd.state_source = state_source;
    results_edmd.stats = stats;
    results_edmd.keep_idx_residual = keep_idx;
    results_edmd.du_nonzero_count = stats.du_nonzero_count;
    results_edmd.du_nonzero_ratio = stats.du_nonzero_ratio;
    results_edmd.du_mean_norm = stats.du_mean_norm;
    results_edmd.du_max_norm = stats.du_max_norm;
    results_edmd.diag = struct();
    results_edmd.diag.du2_mean_norm = mean(du2_norm);
    results_edmd.diag.du2_max_norm = max(du2_norm);
    results_edmd.diag.b_err_rank = rank(B_err);
    results_edmd.diag.b_err_norm = norm(B_err, 'fro');
    results_edmd.diag.b_err_col_norm = b_col_norm;
    results_edmd.diag.prune_delete_pct = prune_delete_pct;
    results_edmd.diag.residual_keep_pct = residual_keep_pct;
    results_edmd.diag.zbar2_mean_norm = mean(zbar2_norm);
    results_edmd.diag.zbar2_max_norm = max(zbar2_norm);
    results_edmd.diag.lambda_sweep = lambda_sweep;
    results_edmd.diag.lambda_b_norm = lambda_b_norm;

    lambda_nom = opts.lambda_nom; %#ok<NASGU>
    lambda_err = opts.lambda_err; %#ok<NASGU>
    pos_err_thresh = opts.pos_err_thresh; %#ok<NASGU>
    vel_thresh = opts.vel_thresh; %#ok<NASGU>
    ang_thresh = opts.ang_thresh; %#ok<NASGU>
    obs_norm_thresh = opts.obs_norm_thresh; %#ok<NASGU>
    res_norm_thresh = opts.res_norm_thresh; %#ok<NASGU>
    cos_guard = opts.cos_guard; %#ok<NASGU>
    du_ratio_thresh = opts.du_ratio_thresh; %#ok<NASGU>
    du_abs_thresh = opts.du_abs_thresh; %#ok<NASGU>
    jump_norm_hard_thresh = opts.jump_norm_hard_thresh; %#ok<NASGU>
    mad_scale = opts.mad_scale; %#ok<NASGU>
    nominal_file_pattern = opts.nominal_file_pattern; %#ok<NASGU>
    degraded_file_pattern = opts.degraded_file_pattern; %#ok<NASGU>

    save(opts.save_path, 'results_edmd', 'lambda_nom', 'lambda_err', ...
        'pos_err_thresh', 'vel_thresh', 'ang_thresh', ...
        'obs_norm_thresh', 'res_norm_thresh', 'cos_guard', ...
        'du_ratio_thresh', 'du_abs_thresh', 'jump_norm_hard_thresh', ...
        'mad_scale', 'nominal_file_pattern', 'degraded_file_pattern');

    % results_edmdをMATファイルとして保存する
    fprintf('\\nSaved paired model: %s\\n', opts.save_path);
end

function [Z, ZN, U_NOM, DU, stats] = load_edmd_samples(opts, file_pattern, state_source)
% 指定パターンのログからEDMD学習サンプルを抽出する
    files = dir(fullfile(opts.data_dir, file_pattern));
    files = filter_files_by_run_indices(files, opts.run_indices);
    if isempty(files)
        error('No files matched: %s', fullfile(opts.data_dir, file_pattern));
    end
    fprintf('pattern: %s\n', file_pattern);
    fprintf('files found: %d\n', numel(files));

    Z = [];
    ZN = [];
    U_NOM = [];
    DU = [];
    stats = struct();
    stats.file_pattern = file_pattern;
    stats.file_used = 0;
    stats.n_raw = 0;
    stats.n_clean = 0;
    stats.n_missing_nominal = 0;

    for fi = 1:numel(files)
        fpath = fullfile(opts.data_dir, files(fi).name);
        fprintf('[%3d/%d] %s ... ', fi, numel(files), files(fi).name);
        try
            tmp = load(fpath);
            log_data = tmp.log;
            agent_log = log_data.Data.agent(opts.agent_idx);
            results_state = get_state_log(agent_log, state_source);
            results_ref = agent_log.reference.result;
            results_ctl = agent_log.controller.result;
            results_inp = agent_log.input;
            N_steps = min([numel(results_state), numel(results_ref), numel(results_ctl), numel(results_inp)]);
        catch ME
            fprintf('invalid log: %s\n', ME.message);
            continue;
        end

        if N_steps < 2
            fprintf('too short\n');
            continue;
        end
        stats.file_used = stats.file_used + 1;

        traj = nan(N_steps, 12);
        ref_seq = nan(N_steps, 9);
        u_raw_seq = nan(N_steps, 4);
        u_nom_seq = nan(N_steps, 4);
        has_nom_seq = false(N_steps, 1);

        for k = 1:N_steps
            try
                s = results_state{1, k}.state;
                traj(k, :) = [double(s.p(:))', double(extract_euler_state_q(s))', ...
                              double(s.v(:))', double(s.w(:))'];
                sr = results_ref{1, k}.state;
                ref_seq(k, :) = [double(sr.p(:))', double(extract_reference_q(sr))', ...
                                 double(sr.v(:))'];
                u_raw_seq(k, :) = reshape(double(results_inp{1, k}(1:4)), 1, []);
                [u_nom_k, has_nom_k] = extract_nominal_input(results_ctl{1, k});
                if has_nom_k
                    u_nom_seq(k, :) = u_nom_k(:)';
                    has_nom_seq(k) = true;
                end
            catch
            end
        end

        n_clean_file = 0;
        for k = 1:(N_steps - 1)
            if any(isnan(traj(k, :))) || any(isnan(traj(k + 1, :))) || ...
               any(isnan(u_raw_seq(k, :))) || any(isnan(ref_seq(k, :)))
                continue;
            end
            stats.n_raw = stats.n_raw + 1;
            x12_t = traj(k, :)';
            x12_tp1 = traj(k + 1, :)';
            u_raw_t = u_raw_seq(k, :)';
            if has_nom_seq(k)
                u_nom_t = u_nom_seq(k, :)';
            else
                u_nom_t = u_raw_t;
                stats.n_missing_nominal = stats.n_missing_nominal + 1;
            end
            delta_u = u_raw_t - u_nom_t;
            x16_t = [x12_t; u_nom_t];
            x16_tp1 = [x12_tp1; u_nom_t];
            z_t = build_kyo_observable(x16_t, opts.cos_guard);
            z_tp1 = build_kyo_observable(x16_tp1, opts.cos_guard);

            if norm(x12_t(1:3) - ref_seq(k, 1:3)') > opts.pos_err_thresh
                continue;
            end
            if norm(x12_t(7:9)) > opts.vel_thresh
                continue;
            end
            if any(abs(x12_t(4:6)) > opts.ang_thresh)
                continue;
            end
            if norm(z_t) > opts.obs_norm_thresh || norm(z_tp1) > opts.obs_norm_thresh
                continue;
            end

            Z = [Z, z_t]; %#ok<AGROW>
            ZN = [ZN, z_tp1]; %#ok<AGROW>
            U_NOM = [U_NOM, u_nom_t]; %#ok<AGROW>
            DU = [DU, delta_u]; %#ok<AGROW>
            n_clean_file = n_clean_file + 1;
            stats.n_clean = stats.n_clean + 1;
        end
        fprintf('raw=%d clean=%d\n', N_steps - 1, n_clean_file);
    end
    if isempty(Z)
        error('No valid samples remained for pattern: %s', file_pattern);
    end
end

function [Z, ZN, U_NOM, DU, mask_keep] = prune_samples(Z, ZN, U_NOM, DU, opts)
% 入力や状態ジャンプの外れ値を除去する
    du_norm = vecnorm(DU, 2, 1);
    u_nom_norm = vecnorm(U_NOM, 2, 1);
    du_ratio = du_norm ./ max(u_nom_norm, 1e-6);
    z_jump = vecnorm(ZN - Z, 2, 1);
    mask_keep = du_norm <= opts.du_abs_thresh & ...
                du_ratio <= opts.du_ratio_thresh & ...
                z_jump <= opts.jump_norm_hard_thresh & ...
                robust_upper_mask(du_norm, opts.mad_scale) & ...
                robust_upper_mask(z_jump, opts.mad_scale);
    Z = Z(:, mask_keep);
    ZN = ZN(:, mask_keep);
    U_NOM = U_NOM(:, mask_keep);
    DU = DU(:, mask_keep);
    if isempty(Z)
        error('All samples were removed by robust pruning.');
    end
end

function results_state = get_state_log(agent_log, state_source)
% plantまたはestimatorの状態ログを選択する
    switch state_source
        case 'estimator'
            results_state = agent_log.estimator.result;
        case 'plant'
            results_state = agent_log.plant.result;
        otherwise
            error('Unknown state source: %s', state_source);
    end
end

function q = extract_euler_state_q(st)
    if isa(st, 'STATE_CLASS')
        q = st.getq('euler');
        return;
    end
    qval = st.q(:);
    if numel(qval) == 3
        q = qval;
    elseif numel(qval) == 4
        q = Quat2Eul(qval);
    else
        error('Unsupported attitude length: %d', numel(qval));
    end
end

function q = extract_reference_q(st)
    if isprop(st, 'q') || (isstruct(st) && isfield(st, 'q'))
        q = extract_euler_state_q(st);
    elseif isprop(st, 'xd') || (isstruct(st) && isfield(st, 'xd'))
        xd = double(st.xd(:));
        q = xd(4:6);
    else
        error('reference state has neither q nor xd.');
    end
end

function [u_nom, ok] = extract_nominal_input(ctrl_result)
% controllerログから公称入力を取得する
    u_nom = nan(4, 1);
    ok = false;
    if isempty(ctrl_result)
        return;
    end
    candidates = {'u_nom', 'unom', 'input_nom', 'nominal_input', 'u'};
    for i = 1:numel(candidates)
        name = candidates{i};
        if isstruct(ctrl_result) && isfield(ctrl_result, name)
            val = ctrl_result.(name);
            if isnumeric(val) && numel(val) >= 4
                u_nom = reshape(double(val(1:4)), [], 1);
                ok = true;
                return;
            end
        end
    end
    if isstruct(ctrl_result) && isfield(ctrl_result, 'input') && isfield(ctrl_result, 'deltau')
        val_input = ctrl_result.input;
        val_du = ctrl_result.deltau;
        if isnumeric(val_input) && isnumeric(val_du) && numel(val_input) >= 4 && numel(val_du) >= 4
            u_nom = reshape(double(val_input(1:4)), [], 1) - reshape(double(val_du(1:4)), [], 1);
            ok = true;
            return;
        end
    end
    if isstruct(ctrl_result) && isfield(ctrl_result, 'pre_u') && isfield(ctrl_result, 'deltau')
        val_pre = ctrl_result.pre_u;
        val_du = ctrl_result.deltau;
        if isnumeric(val_pre) && isnumeric(val_du) && numel(val_pre) >= 4 && numel(val_du) >= 4
            u_nom = reshape(double(val_pre(1:4)), [], 1) - reshape(double(val_du(1:4)), [], 1);
            ok = true;
            return;
        end
    end
end

function z = build_kyo_observable(x, cos_guard)
% EDMDで使用する26次元のlifted stateを構成する
    P1 = x(1); P2 = x(2); P3 = x(3);
    Q1 = x(4); Q2 = x(5); Q3 = x(6);
    V1 = x(7); V2 = x(8); V3 = x(9);
    W1 = x(10); W2 = x(11); W3 = x(12);
    c1 = cos(Q1); s1 = sin(Q1);
    c2 = cos(Q2); s2 = sin(Q2);
    c3 = cos(Q3); s3 = sin(Q3);
    c1_safe = guard_abs(c1, cos_guard);
    c2_safe = guard_abs(c2, cos_guard);
    R13 = c3 * s2 * c1 + s3 * s1;
    R23 = s3 * s2 * c1 - c3 * s1;
    R33 = c2 * c1;
    common_z = [P1; P2; P3; Q1; Q2; Q3; V1; V2; V3; W1; W2; W3; R13; R23; R33; 1];
    kyo_z = [W1 * W2; W2 * W3; W3 * W1; W2 * c1; W3 * s1; ...
             W1 * c2 / c1_safe; W2 * s1 / c2_safe; W3 * c1 / c2_safe; ...
             W2 * s1 * s2 / c2_safe; W3 * c1 * s2 / c2_safe];
    z = [common_z; kyo_z];
end

function y = guard_abs(x, eps_val)
    if abs(x) < eps_val
        y = sign_nonzero(x) * eps_val;
    else
        y = x;
    end
end

function s = sign_nonzero(x)
    if x >= 0
        s = 1;
    else
        s = -1;
    end
end

function files = filter_files_by_run_indices(files, run_indices)
    if isempty(run_indices)
        return;
    end
    run_indices = unique(double(run_indices(:)'));
    keep = false(size(files));
    for i = 1:numel(files)
        token = regexp(files(i).name, '_(\d+)_Log', 'tokens', 'once');
        if isempty(token)
            continue;
        end
        keep(i) = ismember(str2double(token{1}), run_indices);
    end
    files = files(keep);
end

function [A_mat, B_mat] = solve_ridge(Z_cur, U_cur, Z_next, lambda, use_gpu)
% ridge回帰によりEDMDのA行列とB行列を求める
    Phi = [Z_cur; U_cur];
    if use_gpu
        Phi_g = gpuArray(single(Phi));
        Z_g = gpuArray(single(Z_next));
        gram = Phi_g * Phi_g' + lambda * eye(size(Phi_g, 1), 'like', Phi_g);
        AB_g = Z_g * Phi_g' / gram;
        AB = double(gather(AB_g));
    else
        gram = Phi * Phi' + lambda * eye(size(Phi, 1));
        AB = Z_next * Phi' / gram;
    end
    nz = size(Z_cur, 1);
    A_mat = AB(:, 1:nz);
    B_mat = AB(:, nz + 1:end);
end

function mask = robust_upper_mask(x, mad_scale)
    med_x = median(x);
    mad_x = median(abs(x - med_x));
    if mad_x < 1e-12
        mask = true(size(x));
        return;
    end
    robust_sigma = 1.4826 * mad_x;
    mask = x <= med_x + mad_scale * robust_sigma;
end

function s = set_default(s, name, value)
    if ~isfield(s, name) || isempty(s.(name))
        s.(name) = value;
    end
end