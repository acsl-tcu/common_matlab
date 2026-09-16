function results_hl_edmd_error = train_hl_edmd_error_from_logs(opts)
% HLC_EDMD_ERROR.m 用の学習関数
% loggerデータから閉ループ誤差モデルを学習し，Az/Bz/Aa/Ba/Kz/Kaを保存する
% TRAIN_HL_EDMD_ERROR_FROM_LOGS
% Learn closed-loop error dynamics for HLC from logger data:
%   z_{k+1} = A z_k + B du_k
% where du_k = u_raw_k - u_nom_k.

    if nargin < 1 || ~isstruct(opts)
        error('opts must be a struct.');
    end

    % 学習パラメータのデフォルト値を設定する
    opts = set_default(opts, 'agent_idx', 1);
    opts = set_default(opts, 'state_source', 'estimator');
    opts = set_default(opts, 'lambda_z', 1e-5);
    opts = set_default(opts, 'lambda_att', 1e-5);
    opts = set_default(opts, 'max_abs_ez', 1.5);
    opts = set_default(opts, 'max_abs_evz', 3.0);
    opts = set_default(opts, 'max_abs_eang', pi / 4);
    opts = set_default(opts, 'max_abs_werr', 6.0);
    opts = set_default(opts, 'max_abs_du1', 3.0);
    opts = set_default(opts, 'max_abs_dutau', 0.35);
    opts = set_default(opts, 'max_norm_z', 20.0);
    opts = set_default(opts, 'max_norm_att', 30.0);
    opts = set_default(opts, 'use_gpu', false);
    opts = set_default(opts, 'qz_scale', 40.0);
    opts = set_default(opts, 'rz_scale', 0.2);
    opts = set_default(opts, 'qa_scale', 15.0);
    opts = set_default(opts, 'ra_scale', 0.5);

    required_fields = {'data_dir', 'file_pattern', 'save_path'};
    for i = 1:numel(required_fields)
        key = required_fields{i};
        if ~isfield(opts, key) || isempty(opts.(key))
            error('opts.%s is required.', key);
        end
    end

    state_source = lower(char(opts.state_source));
    if ~ismember(state_source, {'plant', 'estimator'})
        error('opts.state_source must be ''plant'' or ''estimator''.');
    end

    % 指定されたフォルダから学習対象ログを取得する
    files = dir(fullfile(opts.data_dir, opts.file_pattern));
    if isempty(files)
        error('No files matched: %s', fullfile(opts.data_dir, opts.file_pattern));
    end

    ZV = [];
    ZVN = [];
    DU1 = [];
    ZA = [];
    ZAN = [];
    DUT = [];

    stats = struct();
    stats.state_source = state_source;
    stats.file_pattern = opts.file_pattern;
    stats.file_used = 0;
    stats.raw_transitions = 0;
    stats.used_vertical = 0;
    stats.used_attitude = 0;
    stats.skipped_missing_input = 0;

    fprintf('\n=== HL error model dataset ===\n');
    fprintf('state source: %s\n', state_source);
    fprintf('files found : %d\n', numel(files));

    % 各ログファイルから状態，参照，入力を抽出する
    for fi = 1:numel(files)
        fpath = fullfile(opts.data_dir, files(fi).name);
        fprintf('[%3d/%d] %s ... ', fi, numel(files), files(fi).name);

        try
            tmp = load(fpath);
            log_data = tmp.log;
        catch ME
            fprintf('load failed: %s\n', ME.message);
            continue;
        end

        try
            agent_log = log_data.Data.agent(opts.agent_idx);
            state_log = get_state_log(agent_log, state_source);
            ref_log = agent_log.reference.result;
            ctl_log = agent_log.controller.result;
            if isfield(agent_log, 'input')
                inp_log = agent_log.input;
            else
                inp_log = cell(size(ctl_log));
            end
            N_steps = min([numel(state_log), numel(ref_log), numel(ctl_log), numel(inp_log)]);
        catch ME
            fprintf('invalid log structure: %s\n', ME.message);
            continue;
        end

        if N_steps < 2
            fprintf('too short\n');
            continue;
        end

        stats.file_used = stats.file_used + 1;

        traj = nan(N_steps, 12);
        refs = nan(N_steps, 12);
        u_nom_seq = nan(N_steps, 4);
        u_raw_seq = nan(N_steps, 4);
        ok_input = false(N_steps, 1);

        % 各時刻の12次元状態，参照状態，入力を配列に変換する
        for k = 1:N_steps
            try
                traj(k, :) = extract_state12(state_log{1, k}.state)';
                refs(k, :) = extract_reference12(ref_log{1, k}.state)';
                [u_nom_k, u_raw_k, ok_k] = extract_input_pair(ctl_log{1, k}, inp_log{1, k});
                if ok_k
                    u_nom_seq(k, :) = u_nom_k(:)';
                    u_raw_seq(k, :) = u_raw_k(:)';
                    ok_input(k) = true;
                end
            catch
            end
        end

        used_file = 0;
        % 隣接時刻の誤差遷移から学習サンプルを作成する
        for k = 1:(N_steps - 1)
            stats.raw_transitions = stats.raw_transitions + 1;

            if any(isnan(traj(k, :))) || any(isnan(traj(k + 1, :))) || ...
               any(isnan(refs(k, :))) || any(isnan(refs(k + 1, :))) || ...
               ~ok_input(k)
                stats.skipped_missing_input = stats.skipped_missing_input + 1;
                continue;
            end

            e_k = compute_error_state(traj(k, :)', refs(k, :)');
            e_kp1 = compute_error_state(traj(k + 1, :)', refs(k + 1, :)');
            du_k = u_raw_seq(k, :)' - u_nom_seq(k, :)';

            zv_k = lift_vertical_error(e_k);
            zv_kp1 = lift_vertical_error(e_kp1);
            za_k = lift_attitude_error(e_k);
            za_kp1 = lift_attitude_error(e_kp1);

            use_vertical = abs(e_k(3)) <= opts.max_abs_ez && ...
                           abs(e_k(9)) <= opts.max_abs_evz && ...
                           abs(du_k(1)) <= opts.max_abs_du1 && ...
                           norm(zv_k) <= opts.max_norm_z && ...
                           norm(zv_kp1) <= opts.max_norm_z;

            use_attitude = all(abs(e_k(4:6)) <= opts.max_abs_eang) && ...
                           all(abs(e_k(10:12)) <= opts.max_abs_werr) && ...
                           norm(du_k(2:4)) <= opts.max_abs_dutau && ...
                           norm(za_k) <= opts.max_norm_att && ...
                           norm(za_kp1) <= opts.max_norm_att;

            if use_vertical
                ZV = [ZV, zv_k]; %#ok<AGROW>
                ZVN = [ZVN, zv_kp1]; %#ok<AGROW>
                DU1 = [DU1, du_k(1)]; %#ok<AGROW>
                stats.used_vertical = stats.used_vertical + 1;
                used_file = used_file + 1;
            end

            if use_attitude
                ZA = [ZA, za_k]; %#ok<AGROW>
                ZAN = [ZAN, za_kp1]; %#ok<AGROW>
                DUT = [DUT, du_k(2:4)]; %#ok<AGROW>
                stats.used_attitude = stats.used_attitude + 1;
                used_file = used_file + 1;
            end
        end

        fprintf('usable blocks=%d\n', used_file);
    end

    if isempty(ZV) || isempty(ZA)
        disp(stats);
        fprintf('高度系の学習サンプル数: %d\n', size(ZV, 2));
        fprintf('姿勢系の学習サンプル数: %d\n', size(ZA, 2));
    
        error('Insufficient samples for vertical or attitude model.');
    end

    % 高度方向の誤差モデルを学習する
    fprintf('\\n=== Train vertical error model ===\\n');
    [Az, Bz] = solve_ridge(ZV, DU1, ZVN, opts.lambda_z, opts.use_gpu);
    ZVN_hat = Az * ZV + Bz * DU1;
    rmse_z = sqrt(mean((ZVN - ZVN_hat).^2, 'all'));
    fprintf('vertical samples : %d\n', size(ZV, 2));
    fprintf('vertical RMSE    : %.6e\n', rmse_z);
    fprintf('rho(Az)          : %.6f\n', max(abs(eig(Az))));

    % 姿勢方向の誤差モデルを学習する
    fprintf('\\n=== Train attitude error model ===\\n');
    [Aa, Ba] = solve_ridge(ZA, DUT, ZAN, opts.lambda_att, opts.use_gpu);
    ZAN_hat = Aa * ZA + Ba * DUT;
    rmse_a = sqrt(mean((ZAN - ZAN_hat).^2, 'all'));
    fprintf('attitude samples : %d\n', size(ZA, 2));
    fprintf('attitude RMSE    : %.6e\n', rmse_a);
    fprintf('rho(Aa)          : %.6f\n', max(abs(eig(Aa))));
    fprintf('rank(Ba)         : %d / %d\n', rank(Ba), size(Ba, 2));

    qz = eye(size(Az, 1)) * opts.qz_scale;
    rz = eye(size(Bz, 2)) * opts.rz_scale;
    qa = eye(size(Aa, 1)) * opts.qa_scale;
    ra = eye(size(Ba, 2)) * opts.ra_scale;

    [Kz, z_dlqr_ok, z_dlqr_msg] = safe_dlqr(Az, Bz, qz, rz);
    [Ka, a_dlqr_ok, a_dlqr_msg] = safe_dlqr(Aa, Ba, qa, ra);

    % 学習結果をHLC_EDMD_ERROR.mで読み込める形式にまとめる
    results_hl_edmd_error = struct();
    results_hl_edmd_error.Az = Az;
    results_hl_edmd_error.Bz = Bz;
    results_hl_edmd_error.Aa = Aa;
    results_hl_edmd_error.Ba = Ba;
    results_hl_edmd_error.Kz = Kz;
    results_hl_edmd_error.Ka = Ka;
    results_hl_edmd_error.rmse_z = rmse_z;
    results_hl_edmd_error.rmse_a = rmse_a;
    results_hl_edmd_error.state_source = state_source;
    results_hl_edmd_error.lift_vertical = 'ez_evz_ez_times_evz';
    results_hl_edmd_error.lift_attitude = 'eq_ew_sin_eq_eq_times_ew';
    results_hl_edmd_error.stats = stats;
    results_hl_edmd_error.vertical_dlqr_ok = z_dlqr_ok;
    results_hl_edmd_error.vertical_dlqr_message = z_dlqr_msg;
    results_hl_edmd_error.attitude_dlqr_ok = a_dlqr_ok;
    results_hl_edmd_error.attitude_dlqr_message = a_dlqr_msg;

    save(opts.save_path, 'results_hl_edmd_error');
    % results_hl_edmd_errorをMATファイルとして保存する
    fprintf('\nSaved: %s\n', opts.save_path);
end

function state_log = get_state_log(agent_log, state_source)
% plantまたはestimatorの状態ログを選択する
    switch state_source
        case 'plant'
            state_log = agent_log.plant.result;
        case 'estimator'
            state_log = agent_log.estimator.result;
        otherwise
            error('Unknown state source: %s', state_source);
    end
end

function x12 = extract_state12(st)
% 状態を[位置;姿勢角;速度;角速度]の12次元に変換する
    q = extract_euler_q(st);
    if isprop(st, 'w') || (isstruct(st) && isfield(st, 'w'))
        w = double(st.w(:));
    else
        w = zeros(3, 1);
    end
    x12 = [double(st.p(:)); q; double(st.v(:)); w];
end

function x12 = extract_reference12(st)
% 参照状態を12次元ベクトルに変換する
    if isprop(st, 'p') || (isstruct(st) && isfield(st, 'p'))
        p = double(st.p(:));
    elseif isprop(st, 'xd') || (isstruct(st) && isfield(st, 'xd'))
        xd = double(st.xd(:));
        p = xd(1:3);
    else
        p = zeros(3, 1);
    end

    if isprop(st, 'q') || (isstruct(st) && isfield(st, 'q'))
        q = extract_euler_q(st);
    elseif isprop(st, 'xd') || (isstruct(st) && isfield(st, 'xd'))
        xd = double(st.xd(:));
        if numel(xd) >= 6
            q = xd(4:6);
        else
            q = zeros(3, 1);
        end
    else
        q = zeros(3, 1);
    end

    if isprop(st, 'v') || (isstruct(st) && isfield(st, 'v'))
        v = double(st.v(:));
    elseif isprop(st, 'xd') || (isstruct(st) && isfield(st, 'xd'))
        xd = double(st.xd(:));
        if numel(xd) >= 9
            v = xd(7:9);
        else
            v = zeros(3, 1);
        end
    else
        v = zeros(3, 1);
    end

    if isprop(st, 'w') || (isstruct(st) && isfield(st, 'w'))
        w = double(st.w(:));
    elseif isprop(st, 'xd') || (isstruct(st) && isfield(st, 'xd'))
        xd = double(st.xd(:));
        if numel(xd) >= 12
            w = xd(10:12);
        else
            w = zeros(3, 1);
        end
    else
        w = zeros(3, 1);
    end

    x12 = [p; q; v; w];
end

function q = extract_euler_q(st)
    if isa(st, 'STATE_CLASS')
        q = double(st.getq('euler'));
        return;
    end

    qval = double(st.q(:));
    if numel(qval) == 3
        q = qval;
    elseif numel(qval) == 4
        q = Quat2Eul(qval);
    else
        error('Unsupported attitude length: %d', numel(qval));
    end
end

function [u_nom, u_raw, ok] = extract_input_pair(ctrl_result, raw_input_value)
% HLC入力と実際の入力を取り出し，補償・励振入力を求める準備をする
    u_nom = nan(4, 1);
    u_raw = nan(4, 1);
    ok = false;

    if isstruct(ctrl_result)
        nom_keys = {'u_nom', 'unom', 'nominal_input', 'input_nom'};
        raw_keys = {'u_raw', 'input'};

        for i = 1:numel(nom_keys)
            key = nom_keys{i};
            if isfield(ctrl_result, key) && isnumeric(ctrl_result.(key)) && numel(ctrl_result.(key)) >= 4
                u_nom = reshape(double(ctrl_result.(key)(1:4)), [], 1);
                break;
            end
        end

        for i = 1:numel(raw_keys)
            key = raw_keys{i};
            if isfield(ctrl_result, key) && isnumeric(ctrl_result.(key)) && numel(ctrl_result.(key)) >= 4
                u_raw = reshape(double(ctrl_result.(key)(1:4)), [], 1);
                break;
            end
        end
    end

    if any(isnan(u_raw)) && isnumeric(raw_input_value) && numel(raw_input_value) >= 4
        u_raw = reshape(double(raw_input_value(1:4)), [], 1);
    end

    ok = ~any(isnan(u_nom)) && ~any(isnan(u_raw));
end

function e = compute_error_state(x, xr)
% 現在状態と参照状態の差から12次元誤差を計算する
    e = zeros(12, 1);
    e(1:3) = x(1:3) - xr(1:3);
    e(4:6) = wrap_to_pi(x(4:6) - xr(4:6));
    e(7:9) = x(7:9) - xr(7:9);
    e(10:12) = x(10:12) - xr(10:12);
end

function z = lift_vertical_error(e)
% 高度誤差と鉛直速度誤差をリフトアップする
    ez = e(3);
    evz = e(9);
    z = [ez; evz; ez * evz];
end

function z = lift_attitude_error(e)
% 姿勢角誤差と角速度誤差をリフトアップする
    eq = e(4:6);
    ew = e(10:12);
    z = [eq; ew; sin(eq); eq .* ew];
end

function ang = wrap_to_pi(ang)
    ang = mod(ang + pi, 2 * pi) - pi;
end

function [A_mat, B_mat] = solve_ridge(Z_cur, U_cur, Z_next, lambda, use_gpu)
% ridge回帰によりEDMDのA行列とB行列を求める
    if size(U_cur, 1) == 1
        Phi = [Z_cur; reshape(U_cur, 1, [])];
    else
        Phi = [Z_cur; U_cur];
    end

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

function [K, ok, msg] = safe_dlqr(A, B, Q, R)
    try
        K = dlqr(A, B, Q, R);
        ok = true;
        msg = '';
    catch ME
        K = zeros(size(B, 2), size(A, 1));
        ok = false;
        msg = ME.message;
    end
end

function s = set_default(s, name, value)
    if ~isfield(s, name) || isempty(s.(name))
        s.(name) = value;
    end
end