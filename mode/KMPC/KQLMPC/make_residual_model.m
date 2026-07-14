function results_edmd = make_residual_model(nominal_files, payload_files, out_mat, params)
% =========================================================================
% make_residual_model : EDMD残差モデル (results_edmd) 生成 [v2: LOGGER対応版]
% -------------------------------------------------------------------------
% 生成物: out_mat に results_edmd (A_nom/B_nom/A_err/B_err) と meta を保存
%
% 使い方 (フレームワークのパスが通った状態で実行すること — LOGGERクラスが必要):
%   nom = {"lpvmpc_hover_60s_Log(...).mat", "lpvmpc_p2p_100s_Log(...).mat", ...
%          "lpvmpc_circle_75s_Log(...).mat", "lpvmpc_figure70s_Log(...).mat"};
%   pay = {"payload_lpvmpc_hover_70s_Log(...).mat", ...};
%   results_edmd = make_residual_model(nom, pay, "edmd_residual_model_v2.mat", m=0.75);
%
% データ抽出の約定 (2026-07-14 実験形式):
%  - logger = LOGGER(fullpath) で読み込み (直接loadするとロックされる)
%  - 飛行区間: phase==102('f') のブロックのうち, ソフトスタートの最初の
%    ブロックを除外し, 2番目のブロック先頭〜最後の102まで
%  - 入力: logger.Data.agent.input (4xN: [推力; roll; pitch; yaw])
%  - 状態: logger.Data.agent.estimator.result{1,k}.state.p/q/v/w
%  - 入力は偏差座標 u~ = u - [m*g;0;0;0] で回帰 (オンラインmode2/5と整合)
% =========================================================================
arguments
    nominal_files
    payload_files
    out_mat (1,1) string = "edmd_residual_model_LPVMPC.mat"
    params.m (1,1) double = 0.75      % 名義質量 [kg] (DRONE_PARAM("DIATONE")と一致, 配重は含めない)
    params.g (1,1) double = 9.81
    params.dt (1,1) double = 0.025
    params.lambda_nom (1,1) double = 1e-4
    params.lambda_err (1,1) double = 1e-3
    params.q_scale (1,1) double = 200.0
    params.r_scale (1,1) double = 0.02
    params.trim_head_sec (1,1) double = 2.0   % 駐留/切替直後の過渡を捨てる秒数
    params.v_dwell (1,1) double = 0.05        % 駐留判定の速度閾値 [m/s]
end

u_hover = [params.m * params.g; 0; 0; 0];

[Zn, Znp, Un, stat_n] = build_snapshots(nominal_files, u_hover, params);
[Zp, Zpp, Up, stat_p] = build_snapshots(payload_files, u_hover, params);
fprintf('[data] nominal: %d対 / payload: %d対\n', size(Zn,2), size(Zp,2));
fprintf('[data] ホバリング推力中央値: nominal=%.3f, payload=%.3f (名義m*g=%.3f)\n', ...
    stat_n.thrust_median, stat_p.thrust_median, params.m*params.g);
fprintf('       → payload側がm*gより明確に大きければ配重が入力に見えている(正常)\n');

%% ===== 名義モデル回帰: z+ ≈ A_nom z + B_nom u~ =====
[AB_nom, vaf_nom] = ridge_edmd(Zn, Un, Znp, params.lambda_nom);
A_nom = AB_nom(:, 1:26);
B_nom = AB_nom(:, 27:30);
fprintf('[nom] 一步予測VAF: min=%.1f%% / median=%.1f%%\n', min(vaf_nom), median(vaf_nom));
fprintf('[nom] max|eig(A_nom)| = %.4f (1を大きく超えるなら要注意)\n', max(abs(eig(A_nom))));

%% ===== 残差計算 =====
R = Zpp - (A_nom * Zp + B_nom * Up);
r_dc = mean(R, 2);
fprintf('[res] 残差DC成分 |mean| = %.4g\n', norm(r_dc));
fprintf('[res] 主要DC: v系(7:9)=%.4g, 定数項(16)=%.4g, p系(1:3)=%.4g\n', ...
    norm(r_dc(7:9)), abs(r_dc(16)), norm(r_dc(1:3)));

%% ===== 残差モデル回帰: r ≈ A_err z + B_err u~ =====
[AB_err, vaf_err] = ridge_edmd(Zp, Up, R, params.lambda_err);
A_err = AB_err(:, 1:26);
B_err = AB_err(:, 27:30);
fprintf('[err] 残差VAF: min=%.1f%% / median=%.1f%% (低くて正常, DC方向が取れていれば良い)\n', ...
    min(vaf_err), median(vaf_err));

%% ===== オンライン整合性チェック =====
fprintf('--- オンライン整合性チェック ---\n');
fprintf('rank(B_err) = %d (4が理想)\n', rank(B_err));
fprintf('rank(ctrb(A_err,B_err)) = %d / 26\n', rank(ctrb(A_err, B_err)));
try
    K = dlqr(A_err, B_err, eye(26)*params.q_scale, eye(4)*params.r_scale);
    fprintf('dlqr: OK (|K|_F = %.3g)\n', norm(K, 'fro'));
catch ME
    fprintf('dlqr: 失敗 (%s) → mode1不可, mode2/5は影響なし\n', ME.message);
end
Be = B_err; reg = 1e-2 * eye(4);
du_rehearsal = (Be'*Be + reg) \ (Be' * R);
fprintf('mode2リハーサル: |δu|中央値 = [%.3f %.3f %.3f %.3f]\n', median(abs(du_rehearsal), 2));

%% ===== 保存 =====
results_edmd = struct('A_nom', A_nom, 'B_nom', B_nom, 'A_err', A_err, 'B_err', B_err);
meta = struct('created', datestr(now), 'nominal_files', {nominal_files}, ...
    'payload_files', {payload_files}, 'm', params.m, 'dt', params.dt, ...
    'u_convention', 'u_tilde = u - [m*g;0;0;0]', 'phase_rule', '2nd 102-block start .. last 102', ...
    'vaf_nom', vaf_nom, 'vaf_err', vaf_err, 'residual_dc', r_dc, ...
    'stat_nominal', stat_n, 'stat_payload', stat_p);
save(out_mat, 'results_edmd', 'meta');
fprintf('[save] %s に保存完了\n', out_mat);
end

%% ========================================================================
function [Z, Zp, Ut, stat] = build_snapshots(files, u_hover, params)
Z = []; Zp = []; Ut = []; thrust_all = [];
n_trim = round(params.trim_head_sec / params.dt);   % 過渡として捨てるサンプル数

for i = 1:numel(files)
    lg = LOGGER(char(files{i}));                     % 直接loadせずLOGGER経由で読む
    ph = lg.Data.phase(:)';

    % --- 飛行区間: ソフトスタートの最初の102ブロックを除外 ---
    i102 = find(ph == 102);
    if isempty(i102)
        warning('%s: phase==102 が見つからない, スキップ', files{i}); continue;
    end
    blk_start = i102([true, diff(i102) > 1]);        % 各102ブロックの先頭index
    if numel(blk_start) >= 2
        rng_idx = blk_start(2) : i102(end);          % 2番目のブロック先頭〜最後の102
    else
        rng_idx = i102(1) : i102(end);               % ブロックが1つしかない場合の保険
    end
    idx = rng_idx(ph(rng_idx) == 102);               % 範囲内の非102を除外

    % --- 入力 (数値4xN / Nx4 / cell配列 いずれにも対応) ---
    U = lg.Data.agent.input;
    if iscell(U)
        U = cellfun(@(c) local_u4(c), U(:)', 'UniformOutput', false);
        U = [U{:}];                                  % 4xN に連結
    end
    if size(U, 1) ~= 4
        if size(U, 2) == 4
            U = U';
        else
            error('agent.input の形が想定外: size=[%s], class=%s', ...
                num2str(size(lg.Data.agent.input)), class(lg.Data.agent.input));
        end
    end

    % --- 状態: estimator result のcellから組み立て ---
    res = lg.Data.agent.estimator.result;
    n_avail = min([numel(ph), size(U, 2), size(res, 2)]);
    idx = idx(idx <= n_avail - 1);                   % k+1が必要なので最後は除外
    X = zeros(12, n_avail);
    need = false(1, n_avail);
    need(idx) = true; need(min(idx+1, n_avail)) = true;
    for k = find(need)
        st = res{1, k}.state;
        X(:, k) = [st.p(:); st.q(:); st.v(:); st.w(:)];
    end

    % --- 駐留セグメントの頭を捨てる (切替直後・到点直後の過渡除去) ---
    vmag = vecnorm(X(7:9, :));
    keep = true(1, n_avail);
    dwell = vmag < params.v_dwell;
    d_start = find(dwell & ~[false, dwell(1:end-1)]);    % 駐留区間の開始点
    for s = d_start
        keep(s : min(s + n_trim - 1, n_avail)) = false;
    end
    % 飛行ブロック先頭の過渡も一律捨てる
    keep(idx(1) : min(idx(1) + n_trim - 1, n_avail)) = false;

    % --- 入力飽和サンプル除外 + 連続対の構築 ---
    ok = keep & [true(1, n_avail)];
    ok = ok & all(abs(U(2:4, 1:n_avail)) < 1.45, 1) & U(1, 1:n_avail) > 0.2;
    cnt0 = size(Z, 2);
    for k = idx
        if ~ok(k) || ~ok(k+1), continue; end
        Z  = [Z,  lift_z(X(:, k))];   %#ok<AGROW>
        Zp = [Zp, lift_z(X(:, k+1))]; %#ok<AGROW>
        Ut = [Ut, U(:, k) - u_hover]; %#ok<AGROW>
        thrust_all(end+1) = U(1, k);  %#ok<AGROW>
    end
    fprintf('  %s: 102ブロック%d個, 採用%d対 (累計%d)\n', ...
        files{i}, numel(blk_start), size(Z,2)-cnt0, size(Z,2));
end
stat = struct('thrust_median', median(thrust_all), 'n_pairs', size(Z, 2));
end

function [AB, vaf] = ridge_edmd(Z, U, Y, lambda)
G = [Z; U];
AB = (Y * G') / (G * G' + lambda * eye(size(G, 1)));
E = Y - AB * G;
vaf = max(0, (1 - var(E, 0, 2) ./ max(var(Y, 0, 2), 1e-12)) * 100);
end

function z = lift_z(x)
% コントローラの klift_edmd_residual と逐項一致 (26次元)
Q1=x(4); Q2=x(5); Q3=x(6); W1=x(10); W2=x(11); W3=x(12);
c1=cos(Q1); s1=sin(Q1); c2=cos(Q2); s2=sin(Q2); c3=cos(Q3); s3=sin(Q3);
c1s = sign(c1)*max(abs(c1),1e-3); if c1==0, c1s=1e-3; end
c2s = sign(c2)*max(abs(c2),1e-3); if c2==0, c2s=1e-3; end
R13 = c3*s2*c1 + s3*s1;
R23 = s3*s2*c1 - c3*s1;
R33 = c2*c1;
z = [x(1:12); R13; R23; R33; 1; ...
     W1*W2; W2*W3; W3*W1; W2*c1; W3*s1; ...
     W1*c2/c1s; W2*s1/c2s; W3*c1/c2s; W2*s1*s2/c2s; W3*c1*s2/c2s];
end
function u = local_u4(c)
% cellの中身を4x1に正規化。空(未記録の拍)はNaNにして自動除外させる
if isempty(c)
    u = nan(4, 1);
else
    u = c(:);
    if numel(u) ~= 4, u = nan(4, 1); end
end
end