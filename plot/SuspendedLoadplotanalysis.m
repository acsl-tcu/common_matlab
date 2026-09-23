% %% =========================================================================
% % 吊り荷UAV 保存ログ解析スクリプト (誤差・包絡指標 5 種完全分離版)
% % =========================================================================
% clear; clc;
% 
% % 1. ログファイルの指定と読み込み
% mat_file = 'Case1_3_Log(23-Sep-2026_17_58_15).mat';
% if exist(mat_file, 'file') ~= 2
%     [f_name, f_path] = uigetfile('*.mat', '解析するログファイルを選択してください');
%     if isequal(f_name, 0), return; end
%     mat_file = fullfile(f_path, f_name);
% end
% load(mat_file, 'log');
% fprintf(">>> ログ読み込み完了: %s\n", mat_file);
% 
% % 2. ログ構造体の参照と配列初期化
% if iscell(log.Data.agent)
%     agent_log = log.Data.agent{1};
% else
%     agent_log = log.Data.agent(1);
% end
% N_steps = log.k - 1;
% time_vec = reshape(log.Data.t(1:N_steps), 1, []); % 1 x N 行ベクトル
% 
% p_drone  = zeros(3, N_steps);
% p_load   = zeros(3, N_steps);
% pL_ref   = zeros(3, N_steps);
% v_ref    = zeros(3, N_steps);
% is_f     = false(1, N_steps);
% 
% % 3. 各ステップの安全抽出 (飛行フェーズ 'f' のみ厳密判定)
% for i = 1:N_steps
%     % (A) フェーズ判定: agent_log.cha が 'f' (ASCII 102)
%     try
%         if iscell(agent_log.cha)
%             c = agent_log.cha{i};
%         else
%             c = agent_log.cha(i);
%         end
%         is_f(i) = (c == 'f' || c == 102);
%     catch
%         is_f(i) = (time_vec(i) >= 5.0);
%     end
% 
%     % (B) 機体実位置 (plant.result{i}.state)
%     try
%         st_p = agent_log.plant.result{i}.state;
%         if isprop(st_p, "p") || isfield(st_p, "p"), p_drone(:, i) = st_p.p(1:3);
%         else, p_drone(:, i) = st_p(1:3); end
%     catch
%         p_drone(:, i) = [NaN; NaN; NaN];
%     end
% 
%     % (C) 荷物推定位置 (estimator.result{i}.state.pL)
%     try
%         st_e = agent_log.estimator.result{i}.state;
%         if isprop(st_e, "pL") || isfield(st_e, "pL"), p_load(:, i) = st_e.pL(1:3);
%         else, p_load(:, i) = st_e(1:3); end
%     catch
%         p_load(:, i) = [NaN; NaN; NaN];
%     end
% 
%     % (D) 荷物目標位置・速度 (reference.result{i}.state)
%     try
%         st_r = agent_log.reference.result{i}.state;
%         if isprop(st_r, "xd") || isfield(st_r, "xd"), pL_ref(:, i) = st_r.xd(1:3);
%         elseif isprop(st_r, "p") || isfield(st_r, "p"), pL_ref(:, i) = st_r.p(1:3);
%         else, pL_ref(:, i) = st_r(1:3); end
% 
%         if isprop(st_r, "v") || isfield(st_r, "v"), v_ref(:, i) = st_r.v(1:3); end
%     catch
%         pL_ref(:, i) = [NaN; NaN; NaN];
%     end
% end
% 
% % ケーブル長 L の取得
% if isfield(log, 'agent') && isfield(log.agent(1), 'parameter') && isfield(log.agent(1).parameter, 'cableL')
%     L_val = log.agent(1).parameter.cableL;
% else
%     L_val = 2.0; % デフォルト 2.0 m
% end
% 
% % 4. 名目位置の作成 (pT = [0;0;-1] 準拠: +Zが上方, -Zが下方)
% pQ_ref = pL_ref + [0; 0; L_val];      % 機体名目目標位置
% pL_nom = p_drone - [0; 0; L_val];     % 荷物名目下垂位置
% 
% % 5. 各種誤差系列の計算
% % (1) 機体自身の追従誤差
% e_drone_series = vecnorm(p_drone - pQ_ref, 2, 1);
% 
% % (2) 荷物の目標からの実追従誤差 (r_total)
% r_total_series = vecnorm(p_load  - pL_ref, 2, 1);
% e_load_series  = r_total_series;
% 
% % (3) 機体直下からの揺動変位
% r_swing_series = vecnorm(p_load  - pL_nom, 2, 1);
% 
% % (4) 索傾斜角 [deg]
% r_rel = p_load - p_drone;
% r_norm = vecnorm(r_rel, 2, 1);
% cos_theta = -r_rel(3, :) ./ max(r_norm, 1e-6);
% cos_theta = min(1.0, max(-1.0, cos_theta));
% theta_swing_deg_series = acos(cos_theta) * (180 / pi);
% 
% % (5) 同時刻保守的包絡 A: 機体誤差 + 揺動 (Sphere-Chain 機体側包絡)
% r_env_drone_series = e_drone_series + r_swing_series;
% 
% % (6) 同時刻保守的包絡 B: 荷物誤差 + 揺動 (荷物側包絡)
% r_env_load_series  = e_load_series  + r_swing_series;
% 
% % 運動強度系列
% v_norm_series = vecnorm(v_ref, 2, 1);
% a_ref = zeros(3, N_steps);
% if N_steps > 1
%     dt_step = 0.025;
%     for dim = 1:3
%         a_ref(dim, :) = gradient(v_ref(dim, :), dt_step);
%     end
% end
% a_norm_series = vecnorm(a_ref, 2, 1);
% 
% % 6. 区間マスクの設定
% idx_f = find(is_f);
% if isempty(idx_f)
%     error("飛行フェーズ 'f' のデータが存在しません。");
% end
% t_start = time_vec(idx_f(1));
% t_end   = time_vec(idx_f(end));
% t_split = t_start + 10.0;
% 
% % 相対フライト時間 (フライト開始を 0.00s とする)
% t_rel_f = time_vec - t_start;
% 
% masks = {is_f, is_f & (time_vec <= t_split), is_f & (time_vec > t_split)};
% 
% % 7. 3区間抽出ループ
% res = struct();
% for s = 1:3
%     m = masks{s};
%     idx_m = find(m);
%     if isempty(idx_m), continue; end
% 
%     get_peak = @(vec) deal(max(vec(idx_m)), ...
%                            t_rel_f(idx_m(find(vec(idx_m) == max(vec(idx_m)), 1))), ...
%                            idx_m(find(vec(idx_m) == max(vec(idx_m)), 1)));
% 
%     % (1) 機体追従誤差
%     [res(s).e_drone, res(s).t_drone, i_d] = get_peak(e_drone_series);
%     res(s).r_swing_at_d = r_swing_series(i_d);
%     res(s).th_deg_at_d  = theta_swing_deg_series(i_d);
% 
%     % (2) 荷物実追従誤差 (r_total)
%     [res(s).r_total, res(s).t_total, i_tot] = get_peak(r_total_series);
%     res(s).e_drone_at_tot = e_drone_series(i_tot);
%     res(s).r_swing_at_tot = r_swing_series(i_tot);
% 
%     % (3) 揺動変位
%     [res(s).r_swing, res(s).t_swing, i_s] = get_peak(r_swing_series);
%     res(s).e_drone_at_s = e_drone_series(i_s);
%     res(s).e_load_at_s  = e_load_series(i_s);
%     res(s).th_deg_at_s  = theta_swing_deg_series(i_s);
% 
%     % (4) 索傾斜角
%     [res(s).th_deg, res(s).t_th, ~] = get_peak(theta_swing_deg_series);
% 
%     % (5) 包絡 A: e_drone + r_swing
%     [res(s).r_env_d, res(s).t_env_d, i_ed] = get_peak(r_env_drone_series);
%     res(s).ed_at_env_d = e_drone_series(i_ed);
%     res(s).rs_at_env_d = r_swing_series(i_ed);
% 
%     % (6) 包絡 B: e_load + r_swing
%     [res(s).r_env_l, res(s).t_env_l, i_el] = get_peak(r_env_load_series);
%     res(s).el_at_env_l = e_load_series(i_el);
%     res(s).rs_at_env_l = r_swing_series(i_el);
% 
%     % 運動強度
%     res(s).v_max = max(v_norm_series(idx_m));
%     res(s).a_max = max(a_norm_series(idx_m));
% end
% 
% % 8. フォーマット整形表示
% fprintf("\n========================================================================================================\n");
% fprintf(" 【解析結果：フライト3区間 誤差・包絡指標 同定比較表】\n");
% fprintf("  フライト時間: %.2f [s] 〜 %.2f [s] (総飛行時間: %.2f [s])\n", t_start, t_end, t_end - t_start);
% fprintf("========================================================================================================\n");
% fprintf("  評価項目 [単位]                 | [区間1] フライト全体     | [区間2] 開始〜10秒 (過渡)| [区間3] 10秒〜終了 (定常)\n");
% fprintf("----------------------------------+--------------------------+--------------------------+--------------------------\n");
% 
% fmt_row = @(title, v1, t1, v2, t2, v3, t3) ...
%     fprintf("  %-30s | %7.4f (t=%5.2fs)    | %7.4f (t=%5.2fs)    | %7.4f (t=%5.2fs)\n", ...
%     title, v1, t1, v2, t2, v3, t3);
% 
% fmt_row("1. 機体追従誤差 e_drone   [m]", ...
%     res(1).e_drone, res(1).t_drone, res(2).e_drone, res(2).t_drone, res(3).e_drone, res(3).t_drone);
% fmt_row("2. 荷物実追従誤差 r_total [m]", ...
%     res(1).r_total, res(1).t_total, res(2).r_total, res(2).t_total, res(3).r_total, res(3).t_total);
% fmt_row("3. 索・荷物揺動変位 r_swing [m]", ...
%     res(1).r_swing, res(1).t_swing, res(2).r_swing, res(2).t_swing, res(3).r_swing, res(3).t_swing);
% fmt_row("4. 最大索傾斜角 theta_deg [deg]", ...
%     res(1).th_deg,  res(1).t_th,    res(2).th_deg,  res(2).t_th,    res(3).th_deg,  res(3).t_th);
% 
% fprintf("----------------------------------+--------------------------+--------------------------+--------------------------\n");
% fprintf("  [同時刻幾何包絡半径]\n");
% fmt_row("5. 機体包絡 (e_d + r_s)   [m]", ...
%     res(1).r_env_d, res(1).t_env_d, res(2).r_env_d, res(2).t_env_d, res(3).r_env_d, res(3).t_env_d);
% fprintf("   - 単純加算(max+max)との差      | 保守度差: -%.4f m   | 保守度差: -%.4f m   | 保守度差: -%.4f m\n", ...
%     (res(1).e_drone + res(1).r_swing) - res(1).r_env_d, ...
%     (res(2).e_drone + res(2).r_swing) - res(2).r_env_d, ...
%     (res(3).e_drone + res(3).r_swing) - res(3).r_env_d);
% 
% fmt_row("6. 荷物包絡 (e_L + r_s)   [m]", ...
%     res(1).r_env_l, res(1).t_env_l, res(2).r_env_l, res(2).t_env_l, res(3).r_env_l, res(3).t_env_l);
% fprintf("   - 単純加算(max+max)との差      | 保守度差: -%.4f m   | 保守度差: -%.4f m   | 保守度差: -%.4f m\n", ...
%     (res(1).r_total + res(1).r_swing) - res(1).r_env_l, ...
%     (res(2).r_total + res(2).r_swing) - res(2).r_env_l, ...
%     (res(3).r_total + res(3).r_swing) - res(3).r_env_l);
% 
% fprintf("----------------------------------+--------------------------+--------------------------+--------------------------\n");
% fprintf("  [ピーク発生時の相手側誤差内訳]\n");
% fprintf("   - e_drone 最大時の r_swing     | %7.4f m                | %7.4f m                | %7.4f m\n", ...
%     res(1).r_swing_at_d, res(2).r_swing_at_d, res(3).r_swing_at_d);
% fprintf("   - r_total 最大時の r_swing     | %7.4f m                | %7.4f m                | %7.4f m\n", ...
%     res(1).r_swing_at_tot, res(2).r_swing_at_tot, res(3).r_swing_at_tot);
% fprintf("   - r_swing 最大時の e_drone     | %7.4f m                | %7.4f m                | %7.4f m\n", ...
%     res(1).e_drone_at_s, res(2).e_drone_at_s, res(3).e_drone_at_s);
% fprintf("----------------------------------+--------------------------+--------------------------+--------------------------\n");
% fprintf("  [軌道の運動強度]\n");
% fprintf("   - 最大目標速度     v_max [m/s] | %7.4f                  | %7.4f                  | %7.4f\n", ...
%     res(1).v_max, res(2).v_max, res(3).v_max);
% fprintf("   - 最大目標加速度   a_max [m/s2]| %7.4f                  | %7.4f                  | %7.4f\n", ...
%     res(1).a_max, res(2).a_max, res(3).a_max);
% fprintf("========================================================================================================\n\n");

% %% =========================================================================
% % 吊り荷UAV Sphere-Chainパラメータ同定スクリプト (実速度・索上揺動・遅延同定)
% % =========================================================================
% clear; clc;
% 
% % 1. ログファイルの指定と読み込み
% mat_file = 'Case4no3_Log(23-Sep-2026_21_42_47).mat';
% if exist(mat_file, 'file') ~= 2
%     [f_name, f_path] = uigetfile('*.mat', '解析するログファイルを選択してください');
%     if isequal(f_name, 0), return; end
%     mat_file = fullfile(f_path, f_name);
% end
% load(mat_file, 'log');
% fprintf(">>> ログ読み込み完了: %s\n", mat_file);
% 
% % 2. ログ構造体の参照と配列初期化
% if iscell(log.Data.agent)
%     agent_log = log.Data.agent{1};
% else
%     agent_log = log.Data.agent(1);
% end
% N_steps = log.k - 1;
% time_vec = reshape(log.Data.t(1:N_steps), 1, []); % 1 x N 行ベクトル
% 
% p_drone  = zeros(3, N_steps);
% v_drone  = zeros(3, N_steps);
% p_load   = zeros(3, N_steps);
% v_load   = zeros(3, N_steps);
% pL_ref   = zeros(3, N_steps);
% v_ref    = zeros(3, N_steps);
% is_f     = false(1, N_steps);
% 
% % 3. 各ステップの安全抽出 (飛行フェーズ 'f' のみ厳密判定)
% for i = 1:N_steps
%     % (A) フェーズ判定: agent_log.cha が 'f' (ASCII 102)
%     try
%         if iscell(agent_log.cha)
%             c = agent_log.cha{i};
%         else
%             c = agent_log.cha(i);
%         end
%         is_f(i) = (c == 'f' || c == 102);
%     catch
%         is_f(i) = (time_vec(i) >= 5.0);
%     end
% 
%     % (B) 機体実位置・実速度 (plant または sensor)
%     try
%         st_p = agent_log.plant.result{i}.state;
%         if isprop(st_p, "p") || isfield(st_p, "p"), p_drone(:, i) = st_p.p(1:3);
%         else, p_drone(:, i) = st_p(1:3); end
% 
%         if isprop(st_p, "v") || isfield(st_p, "v"), v_drone(:, i) = st_p.v(1:3); end
%     catch
%         p_drone(:, i) = [NaN; NaN; NaN];
%         v_drone(:, i) = [NaN; NaN; NaN];
%     end
% 
%     % (C) 荷物推定位置・推定速度 (estimator)
%     try
%         st_e = agent_log.estimator.result{i}.state;
%         if isprop(st_e, "pL") || isfield(st_e, "pL"), p_load(:, i) = st_e.pL(1:3);
%         else, p_load(:, i) = st_e(1:3); end
% 
%         if isprop(st_e, "vL") || isfield(st_e, "vL"), v_load(:, i) = st_e.vL(1:3); end
%     catch
%         p_load(:, i) = [NaN; NaN; NaN];
%         v_load(:, i) = [NaN; NaN; NaN];
%     end
% 
%     % (D) 荷物目標位置・速度 (reference)
%     try
%         st_r = agent_log.reference.result{i}.state;
%         if isprop(st_r, "xd") || isfield(st_r, "xd"), pL_ref(:, i) = st_r.xd(1:3);
%         elseif isprop(st_r, "p") || isfield(st_r, "p"), pL_ref(:, i) = st_r.p(1:3);
%         else, pL_ref(:, i) = st_r(1:3); end
% 
%         if isprop(st_r, "v") || isfield(st_r, "v"), v_ref(:, i) = st_r.v(1:3); end
%     catch
%         pL_ref(:, i) = [NaN; NaN; NaN];
%         v_ref(:, i)  = [NaN; NaN; NaN];
%     end
% end
% 
% % ケーブル長 L の取得
% if isfield(log, 'agent') && isfield(log.agent(1), 'parameter') && isfield(log.agent(1).parameter, 'cableL')
%     L_val = log.agent(1).parameter.cableL;
% else
%     L_val = 2.0; % デフォルト 2.0 m
% end
% 
% % 4. 名目位置の作成 (pT = [0;0;-1] 準拠: +Zが上方, -Zが下方)
% pQ_ref = pL_ref + [0; 0; L_val];      % 機体名目目標位置
% pL_nom = p_drone - [0; 0; L_val];     % 荷物名目下垂位置
% 
% % 5. 各種系列の計算
% e_drone_series = vecnorm(p_drone - pQ_ref, 2, 1);
% r_total_series = vecnorm(p_load  - pL_ref, 2, 1);
% r_swing_series = vecnorm(p_load  - pL_nom, 2, 1);
% 
% % 実速度系列
% v_drone_series = vecnorm(v_drone, 2, 1);
% v_load_series  = vecnorm(v_load,  2, 1);
% 
% % 索傾斜角 [deg]
% r_rel = p_load - p_drone;
% r_norm = vecnorm(r_rel, 2, 1);
% cos_theta = -r_rel(3, :) ./ max(r_norm, 1e-6);
% cos_theta = min(1.0, max(-1.0, cos_theta));
% theta_swing_deg_series = acos(cos_theta) * (180 / pi);
% 
% % 同時刻保守的包絡
% r_env_drone_series = e_drone_series + r_swing_series;
% r_env_load_series  = r_total_series + r_swing_series;
% 
% % 1ステップ前 (dt=0.025s) の目標値との遅れ誤差
% pQ_ref_1lag = [pQ_ref(:, 1), pQ_ref(:, 1:end-1)];
% pL_ref_1lag = [pL_ref(:, 1), pL_ref(:, 1:end-1)];
% e_lag1_drone_series = vecnorm(p_drone - pQ_ref_1lag, 2, 1);
% e_lag1_load_series  = vecnorm(p_load  - pL_ref_1lag, 2, 1);
% 
% % 6. 区間マスクの設定
% idx_f = find(is_f);
% if isempty(idx_f), error("飛行フェーズ 'f' のデータが存在しません。"); end
% t_start = time_vec(idx_f(1));
% t_end   = time_vec(idx_f(end));
% t_split = t_start + 10.0;
% t_rel_f = time_vec - t_start;
% 
% masks = {is_f, is_f & (time_vec <= t_split), is_f & (time_vec > t_split)};
% 
% % 7. 3区間抽出ループ
% res = struct();
% for s = 1:3
%     m = masks{s};
%     idx_m = find(m);
%     if isempty(idx_m), continue; end
% 
%     get_peak = @(vec) deal(max(vec(idx_m)), ...
%                            t_rel_f(idx_m(find(vec(idx_m) == max(vec(idx_m)), 1))), ...
%                            idx_m(find(vec(idx_m) == max(vec(idx_m)), 1)));
% 
%     % (1) 追従誤差・実追従誤差
%     [res(s).e_drone, res(s).t_drone, i_d] = get_peak(e_drone_series);
%     res(s).r_swing_at_d = r_swing_series(i_d);
% 
%     [res(s).r_total, res(s).t_total, i_tot] = get_peak(r_total_series);
%     res(s).r_swing_at_tot = r_swing_series(i_tot);
% 
%     % (2) 揺動変位・角度
%     [res(s).r_swing, res(s).t_swing, i_s] = get_peak(r_swing_series);
%     res(s).e_drone_at_s = e_drone_series(i_s);
%     [res(s).th_deg, res(s).t_th, ~]       = get_peak(theta_swing_deg_series);
% 
%     % (3) 同時刻包絡
%     [res(s).r_env_d, res(s).t_env_d, ~]   = get_peak(r_env_drone_series);
%     [res(s).r_env_l, res(s).t_env_l, ~]   = get_peak(r_env_load_series);
% 
%     % (4) 1ステップ遅れ目標との誤差
%     [res(s).e_lag1_drone, ~, ~] = get_peak(e_lag1_drone_series);
%     [res(s).e_lag1_load,  ~, ~] = get_peak(e_lag1_load_series);
% 
%     % (5) 実速度最大値 (最重要)
%     [res(s).v_drone_max, res(s).t_vd, ~] = get_peak(v_drone_series);
%     [res(s).v_load_max,  res(s).t_vl, ~] = get_peak(v_load_series);
% end
% 
% % 8. 制御系の実効遅れ時間 τ* の最適探索 (定常区間: 10秒〜終了)
% idx_ss = find(masks{3});
% dt_nominal = 0.025;
% lags_to_test = 0:1:40; % 0 step (0ms) 〜 40 steps (1000ms)
% mean_err_drone = zeros(size(lags_to_test));
% mean_err_load  = zeros(size(lags_to_test));
% 
% for k_lag = 1:length(lags_to_test)
%     lag_steps = lags_to_test(k_lag);
%     idx_eval = idx_ss(idx_ss > (idx_f(1) + lag_steps));
%     if isempty(idx_eval), continue; end
% 
%     idx_ref_lag = idx_eval - lag_steps;
%     mean_err_drone(k_lag) = mean(vecnorm(p_drone(:, idx_eval) - pQ_ref(:, idx_ref_lag), 2, 1));
%     mean_err_load(k_lag)  = mean(vecnorm(p_load(:,  idx_eval) - pL_ref(:, idx_ref_lag), 2, 1));
% end
% 
% [min_e_d, best_lag_idx_d] = min(mean_err_drone);
% best_tau_drone = lags_to_test(best_lag_idx_d) * dt_nominal;
% 
% [min_e_l, best_lag_idx_l] = min(mean_err_load);
% best_tau_load = lags_to_test(best_lag_idx_l) * dt_nominal;
% 
% % 9. コンソール整形出力
% fprintf("\n========================================================================================================\n");
% fprintf(" 【解析結果：Sphere-Chain 設計用 不確実性・実動態・遅延 同定表】\n");
% fprintf("  フライト時間: %.2f [s] 〜 %.2f [s] (総飛行時間: %.2f [s])\n", t_start, t_end, t_end - t_start);
% fprintf("========================================================================================================\n");
% fprintf("  評価項目 [単位]                 | [区間1] フライト全体     | [区間2] 開始〜10秒 (過渡)| [区間3] 10秒〜終了 (定常)\n");
% fprintf("----------------------------------+--------------------------+--------------------------+--------------------------\n");
% 
% fmt_row = @(title, v1, t1, v2, t2, v3, t3) ...
%     fprintf("  %-30s | %7.4f (t=%5.2fs)    | %7.4f (t=%5.2fs)    | %7.4f (t=%5.2fs)\n", ...
%     title, v1, t1, v2, t2, v3, t3);
% 
% fmt_row("1. 機体追従誤差 e_drone   [m]", ...
%     res(1).e_drone, res(1).t_drone, res(2).e_drone, res(2).t_drone, res(3).e_drone, res(3).t_drone);
% fmt_row("2. 荷物実追従誤差 r_total [m]", ...
%     res(1).r_total, res(1).t_total, res(2).r_total, res(2).t_total, res(3).r_total, res(3).t_total);
% fmt_row("3. 索・荷物揺動変位 r_swing [m]", ...
%     res(1).r_swing, res(1).t_swing, res(2).r_swing, res(2).t_swing, res(3).r_swing, res(3).t_swing);
% fmt_row("4. 最大索傾斜角 theta_deg [deg]", ...
%     res(1).th_deg,  res(1).t_th,    res(2).th_deg,  res(2).t_th,    res(3).th_deg,  res(3).t_th);
% 
% fprintf("----------------------------------+--------------------------+--------------------------+--------------------------\n");
% fprintf("  [★最優先：実速度最大値 (遅延補償バウンド用)]\n");
% fmt_row("5. 機体実速度最大 v_drone [m/s]", ...
%     res(1).v_drone_max, res(1).t_vd, res(2).v_drone_max, res(2).t_vd, res(3).v_drone_max, res(3).t_vd);
% fmt_row("6. 荷物実速度最大 v_load  [m/s]", ...
%     res(1).v_load_max,  res(1).t_vl, res(2).v_load_max,  res(2).t_vl, res(3).v_load_max,  res(3).t_vl);
% 
% fprintf("----------------------------------+--------------------------+--------------------------+--------------------------\n");
% fprintf("  [同時刻幾何包絡半径]\n");
% fmt_row("7. 機体包絡 (e_d + r_s)   [m]", ...
%     res(1).r_env_d, res(1).t_env_d, res(2).r_env_d, res(2).t_env_d, res(3).r_env_d, res(3).t_env_d);
% fmt_row("8. 荷物包絡 (e_L + r_s)   [m]", ...
%     res(1).r_env_l, res(1).t_env_l, res(2).r_env_l, res(2).t_env_l, res(3).r_env_l, res(3).t_env_l);
% 
% fprintf("----------------------------------+--------------------------+--------------------------+--------------------------\n");
% fprintf("  [遅延特性・目標遅れ評価]\n");
% fprintf("  ・1ステップ遅れ(dt=0.025s)の機体誤差: 全体=%.4fm, 定常=%.4fm (通常誤差との差: %+.4fm)\n", ...
%     res(1).e_lag1_drone, res(3).e_lag1_drone, res(3).e_lag1_drone - res(3).e_drone);
% fprintf("  ・定常区間の最適等価遅れ時間 tau*   : 機体 tau_Q* = %.3f [s] (%d steps), 荷物 tau_L* = %.3f [s] (%d steps)\n", ...
%     best_tau_drone, lags_to_test(best_lag_idx_d), best_tau_load, lags_to_test(best_lag_idx_l));
% fprintf("  ・tau* 補正後の最小追従偏差 (残差)  : 機体=%.4fm (補正前=%.4fm), 荷物=%.4fm (補正前=%.4fm)\n", ...
%     min_e_d, mean_err_drone(1), min_e_l, mean_err_load(1));
% 
% fprintf("----------------------------------+--------------------------+--------------------------+--------------------------\n");
% fprintf("  [★Sphere-Chain 索上の揺動プロファイル r_swing(s) = s * r_swing (定常区間)]\n");
% fprintf("   - 機体直下 (s = 0.00) : r_swing = 0.0000 [m]\n");
% fprintf("   - 索 1/4   (s = 0.25) : r_swing = %.4f [m]\n", 0.25 * res(3).r_swing);
% fprintf("   - 索 1/2   (s = 0.50) : r_swing = %.4f [m]\n", 0.50 * res(3).r_swing);
% fprintf("   - 索 3/4   (s = 0.75) : r_swing = %.4f [m]\n", 0.75 * res(3).r_swing);
% fprintf("   - 荷物位置 (s = 1.00) : r_swing = %.4f [m]\n", 1.00 * res(3).r_swing);
% fprintf("========================================================================================================\n\n");

% %% =========================================================================
% % 吊り荷UAV Sphere-Chain 5パラメータ・15ログ一括自動同定 (画面表示版)
% % =========================================================================
% clear; clc;
% 
% % 1. 解析対象ファイルリスト（15試行分を直接定義）
% file_names = { ...
%     'Case1no1_Log(23-Sep-2026_17_53_31).mat', ...
%     'Case1no2_Log(23-Sep-2026_17_55_56).mat', ...
%     'Case1_3_Log(23-Sep-2026_17_58_15).mat', ...
%     'Case2no1_Log(23-Sep-2026_20_17_02).mat', ...
%     'Case2no2_Log(23-Sep-2026_20_20_35).mat', ...
%     'Case2no3_Log(23-Sep-2026_20_25_38).mat', ...
%     'Case3no1_Log(23-Sep-2026_20_28_43).mat', ...
%     'Case3no2_Log(23-Sep-2026_20_31_01).mat', ...
%     'Case3no3_Log(23-Sep-2026_20_34_15).mat', ...
%     'Case4no1_Log(23-Sep-2026_20_41_38).mat', ...
%     'Case4no2_Log(23-Sep-2026_20_46_10).mat', ...
%     'Case4no3_Log(23-Sep-2026_21_42_47).mat', ...
%     'Case5no1_Log(23-Sep-2026_21_45_38).mat', ...
%     'Case5no2_Log(23-Sep-2026_21_47_26).mat', ...
%     'Case5no3_Log(23-Sep-2026_21_49_03).mat' ...
% };
% 
% if isempty(mfilename)
%     dir_path = pwd;
% else
%     dir_path = fileparts(mfilename('fullpath'));
% end
% 
% num_files = length(file_names);
% fprintf(">>> 全 %d 件のログファイルを一括解析します。\n\n", num_files);
% 
% result_count = 0;
% results = struct();
% 
% %% 2. 各ファイルの一括ループ処理
% for f_idx = 1:num_files
%     cur_file = file_names{f_idx};
%     full_path = fullfile(dir_path, cur_file);
% 
%     if exist(full_path, 'file') ~= 2
%         full_path = which(cur_file);
%         if isempty(full_path)
%             fprintf("[%d/%d] スキップ (ファイル未検出): %s\n", f_idx, num_files, cur_file);
%             continue;
%         end
%     end
% 
%     fprintf("[%d/%d] 解析中: %s ... ", f_idx, num_files, cur_file);
% 
%     try
%         loaded_data = load(full_path);
%         if isfield(loaded_data, 'log')
%             cur_log = loaded_data.log;
%         elseif isfield(loaded_data, 'app') && isfield(loaded_data.app, 'logger')
%             cur_log = loaded_data.app.logger;
%         else
%             cur_log = loaded_data;
%         end
%     catch ME
%         fprintf("失敗 (読み込みエラー: %s)\n", ME.message);
%         continue;
%     end
% 
%     if ~isfield(cur_log, 'Data') || ~isfield(cur_log, 'k')
%         fprintf("スキップ (Data/k フィールドなし)\n");
%         continue;
%     end
% 
%     if iscell(cur_log.Data.agent)
%         agent_log = cur_log.Data.agent{1};
%     else
%         agent_log = cur_log.Data.agent(1);
%     end
% 
%     N_steps = cur_log.k - 1;
%     time_vec = reshape(cur_log.Data.t(1:N_steps), 1, []);
% 
%     % 欠測値を NaN で初期化
%     p_drone  = NaN(3, N_steps);
%     v_drone  = NaN(3, N_steps);
%     p_load   = NaN(3, N_steps);
%     v_load   = NaN(3, N_steps);
%     pL_ref   = NaN(3, N_steps);
%     v_ref    = NaN(3, N_steps);
%     is_f     = false(1, N_steps);
% 
%     % 各ステップの抽出
%     for i = 1:N_steps
%         % フェーズ判定 ('f')
%         try
%             if iscell(agent_log.cha), c = agent_log.cha{i};
%             else, c = agent_log.cha(i); end
%             is_f(i) = (c == 'f' || c == 102);
%         catch
%             is_f(i) = (time_vec(i) >= 5.0);
%         end
% 
%         % 機体
%         try
%             st_p = agent_log.plant.result{i}.state;
%             if isprop(st_p, "p") || isfield(st_p, "p"), p_drone(:, i) = st_p.p(1:3);
%             else, p_drone(:, i) = st_p(1:3); end
%             if isprop(st_p, "v") || isfield(st_p, "v"), v_drone(:, i) = st_p.v(1:3); end
%         catch
%         end
% 
%         % 荷物
%         try
%             st_e = agent_log.estimator.result{i}.state;
%             if isprop(st_e, "pL") || isfield(st_e, "pL"), p_load(:, i) = st_e.pL(1:3);
%             else, p_load(:, i) = st_e(1:3); end
%             if isprop(st_e, "vL") || isfield(st_e, "vL"), v_load(:, i) = st_e.vL(1:3); end
%         catch
%         end
% 
%         % 目標
%         try
%             st_r = agent_log.reference.result{i}.state;
%             if isprop(st_r, "xd") || isfield(st_r, "xd"), pL_ref(:, i) = st_r.xd(1:3);
%             elseif isprop(st_r, "p") || isfield(st_r, "p"), pL_ref(:, i) = st_r.p(1:3);
%             else, pL_ref(:, i) = st_r(1:3); end
%             if isprop(st_r, "v") || isfield(st_r, "v"), v_ref(:, i) = st_r.v(1:3); end
%         catch
%         end
%     end
% 
%     % ケーブル長
%     if isfield(cur_log, 'agent') && isfield(cur_log.agent(1), 'parameter') && isfield(cur_log.agent(1).parameter, 'cableL')
%         L_val = cur_log.agent(1).parameter.cableL;
%     else
%         L_val = 2.0;
%     end
% 
%     pQ_ref = pL_ref + [0; 0; L_val];
%     pL_nom = p_drone - [0; 0; L_val];
% 
%     % 定常区間 (飛行開始から10秒以降) の切り出し
%     idx_f = find(is_f);
%     if isempty(idx_f)
%         fprintf("スキップ (飛行フェーズ 'f' がありません)\n");
%         continue;
%     end
%     t_start = time_vec(idx_f(1));
%     t_split = t_start + 10.0;
% 
%     valid_data_mask = ~any(isnan(p_drone), 1) & ~any(isnan(p_load), 1) & ...
%                       ~any(isnan(pL_ref), 1)  & ~any(isnan(v_drone), 1);
% 
%     mask_ss = is_f & (time_vec > t_split) & valid_data_mask;
%     idx_ss = find(mask_ss);
% 
%     if length(idx_ss) < 20
%         fprintf("スキップ (定常区間 10s 以降の有効データ不足: %d steps)\n", length(idx_ss));
%         continue;
%     end
% 
%     dt_nom = 0.025;
% 
%     % -------------------------------------------------------------
%     % Level 1: 5設計パラメータの同定
%     % -------------------------------------------------------------
%     lags = 0:1:40; % 0 〜 1.0秒
%     mean_sq_d = NaN(size(lags));
%     mean_sq_l = NaN(size(lags));
% 
%     for k = 1:length(lags)
%         lag = lags(k);
%         eval_idx = idx_ss(idx_ss > (idx_f(1) + lag));
%         ref_idx  = eval_idx - lag;
% 
%         valid_pair = ~any(isnan(p_drone(:, eval_idx)), 1) & ~any(isnan(pQ_ref(:, ref_idx)), 1);
%         if any(valid_pair)
%             mean_sq_d(k) = mean(sum((p_drone(:, eval_idx(valid_pair)) - pQ_ref(:, ref_idx(valid_pair))).^2, 1));
%             mean_sq_l(k) = mean(sum((p_load(:,  eval_idx(valid_pair)) - pL_ref(:,  ref_idx(valid_pair))).^2, 1));
%         end
%     end
% 
%     [~, b_d] = min(mean_sq_d, [], 'omitnan');
%     [~, b_l] = min(mean_sq_l, [], 'omitnan');
%     tau_Q = lags(b_d) * dt_nom;
%     tau_L = lags(b_l) * dt_nom;
% 
%     % 等価遅延補正後の残差追従誤差 (Residual Tracking Error) の最大値
%     eval_ss_d = idx_ss(idx_ss > (idx_f(1) + lags(b_d)));
%     ref_lag_d = eval_ss_d - lags(b_d);
%     eps_track_drone = max(vecnorm(p_drone(:, eval_ss_d) - pQ_ref(:, ref_lag_d), 2, 1), [], 'omitnan');
% 
%     eval_ss_l = idx_ss(idx_ss > (idx_f(1) + lags(b_l)));
%     ref_lag_l = eval_ss_l - lags(b_l);
%     eps_track_load  = max(vecnorm(p_load(:, eval_ss_l)  - pL_ref(:, ref_lag_l),  2, 1), [], 'omitnan');
% 
%     % (参考) 未補正の生追従誤差
%     eps_raw_drone = max(vecnorm(p_drone(:, idx_ss) - pQ_ref(:, idx_ss), 2, 1), [], 'omitnan');
%     eps_raw_load  = max(vecnorm(p_load(:, idx_ss)  - pL_ref(:, idx_ss),  2, 1), [], 'omitnan');
% 
%     % 荷物位置における最大揺動変位 r_swing_max
%     r_swing_ss = vecnorm(p_load(:, idx_ss) - pL_nom(:, idx_ss), 2, 1);
%     r_swing_max = max(r_swing_ss, [], 'omitnan');
% 
%     % -------------------------------------------------------------
%     % Level 2: モデル検証指標
%     % -------------------------------------------------------------
%     r_rel = p_load(:, idx_ss) - p_drone(:, idx_ss);
%     cos_th = min(1.0, max(-1.0, -r_rel(3, :) ./ max(vecnorm(r_rel, 2, 1), 1e-6)));
%     theta_swing_max = max(acos(cos_th) * (180 / pi), [], 'omitnan');
% 
%     v_drone_max = max(vecnorm(v_drone(:, idx_ss), 2, 1), [], 'omitnan');
%     v_load_max  = max(vecnorm(v_load(:, idx_ss), 2, 1),  [], 'omitnan');
% 
%     % 6秒以内の短時間等速予測乖離
%     horizons = [1.0, 2.0, 4.0, 6.0];
%     pred_errs = NaN(1, 4);
%     for h = 1:4
%         H_steps = round(horizons(h) / dt_nom);
%         val_idx = idx_ss(1 : (end - H_steps));
%         if ~isempty(val_idx)
%             p_pred = p_drone(:, val_idx) + v_drone(:, val_idx) * horizons(h);
%             p_true = p_drone(:, val_idx + H_steps);
%             pred_errs(h) = max(vecnorm(p_true - p_pred, 2, 1), [], 'omitnan');
%         end
%     end
% 
%     % 結果格納
%     result_count = result_count + 1;
%     results(result_count).FileName         = cur_file;
%     results(result_count).tau_delay_drone  = tau_Q;
%     results(result_count).tau_delay_load   = tau_L;
%     results(result_count).eps_track_drone  = eps_track_drone;
%     results(result_count).eps_track_load   = eps_track_load;
%     results(result_count).r_swing_max      = r_swing_max;
%     results(result_count).eps_raw_drone    = eps_raw_drone;
%     results(result_count).eps_raw_load     = eps_raw_load;
%     results(result_count).theta_swing_deg  = theta_swing_max;
%     results(result_count).v_drone_max      = v_drone_max;
%     results(result_count).v_load_max       = v_load_max;
%     results(result_count).pred_err_1s      = pred_errs(1);
%     results(result_count).pred_err_2s      = pred_errs(2);
%     results(result_count).pred_err_4s      = pred_errs(3);
%     results(result_count).pred_err_6s      = pred_errs(4);
% 
%     fprintf("完了 (eps_Q: %.3fm, r_sw: %.3fm, tau_Q: %.3fs)\n", ...
%         eps_track_drone, r_swing_max, tau_Q);
% end
% 
% %% 3. テーブル化と統計サマリーの画面表示
% if result_count == 0
%     disp("[WARN] 正常に解析できたログデータが 0 件でした。ファイルパスを確認してください。");
%     return;
% end
% 
% T_all = struct2table(results);
% 
% fprintf("\n========================================================================================================\n");
% fprintf(" 【全ログ同定結果サマリー (定常区間 > 10s, 成功数: %d)】\n", result_count);
% fprintf("========================================================================================================\n");
% disp(T_all(:, {'FileName', 'tau_delay_drone', 'eps_track_drone', 'tau_delay_load', 'eps_track_load', 'r_swing_max'}));
% 
% fprintf("\n--------------------------------------------------------------------------------------------------------\n");
% fprintf(" 【統計サマリー (全 %d 試行)】\n", height(T_all));
% fprintf("--------------------------------------------------------------------------------------------------------\n");
% fprintf("  パラメータ名           |  最小値   |   平均値  |   中央値  |   最大値 (包絡候補) |  標準偏差 \n");
% fprintf("-------------------------+-----------+-----------+-----------+--------------------+-----------\n");
% 
% print_stat = @(name, vec) fprintf("  %-22s |  %7.4f  |  %7.4f  |  %7.4f  |      %7.4f         |  %7.4f\n", ...
%     name, min(vec), mean(vec), median(vec), max(vec), std(vec));
% 
% print_stat("tau_delay_drone [s]",   T_all.tau_delay_drone);
% print_stat("eps_track_drone [m]",   T_all.eps_track_drone);
% print_stat("tau_delay_load  [s]",   T_all.tau_delay_load);
% print_stat("eps_track_load  [m]",   T_all.eps_track_load);
% print_stat("r_swing_max     [m]",   T_all.r_swing_max);
% fprintf("-------------------------+-----------+-----------+-----------+--------------------+-----------\n");
% print_stat("theta_swing_deg [deg]", T_all.theta_swing_deg);
% print_stat("v_drone_max     [m/s]", T_all.v_drone_max);
% print_stat("v_load_max      [m/s]", T_all.v_load_max);
% print_stat("pred_err (2.0s) [m]",   T_all.pred_err_2s);
% print_stat("pred_err (6.0s) [m]",   T_all.pred_err_6s);
% fprintf("========================================================================================================\n\n");

% %% =========================================================================
% % 吊り荷UAV 運動強度・真の機体追従誤差・1ステップ遅延・5パラメータ包括同定
% % =========================================================================
% clear; clc;
% 
% file_names = { ...
%     'Case1no1_Log(23-Sep-2026_17_53_31).mat', ...
%     'Case1no2_Log(23-Sep-2026_17_55_56).mat', ...
%     'Case1_3_Log(23-Sep-2026_17_58_15).mat', ...
%     'Case2no1_Log(23-Sep-2026_20_17_02).mat', ...
%     'Case2no2_Log(23-Sep-2026_20_20_35).mat', ...
%     'Case2no3_Log(23-Sep-2026_20_25_38).mat', ...
%     'Case3no1_Log(23-Sep-2026_20_28_43).mat', ...
%     'Case3no2_Log(23-Sep-2026_20_31_01).mat', ...
%     'Case3no3_Log(23-Sep-2026_20_34_15).mat', ...
%     'Case4no1_Log(23-Sep-2026_20_41_38).mat', ...
%     'Case4no2_Log(23-Sep-2026_20_46_10).mat', ...
%     'Case4no3_Log(23-Sep-2026_21_42_47).mat', ...
%     'Case5no1_Log(23-Sep-2026_21_45_38).mat', ...
%     'Case5no2_Log(23-Sep-2026_21_47_26).mat', ...
%     'Case5no3_Log(23-Sep-2026_21_49_03).mat' ...
% };
% 
% if isempty(mfilename), dir_path = pwd;
% else, dir_path = fileparts(mfilename('fullpath')); end
% 
% num_files = length(file_names);
% fprintf(">>> 全 %d 件のログファイルを詳細解析します。\n\n", num_files);
% 
% result_count = 0;
% results = struct();
% 
% for f_idx = 1:num_files
%     cur_file = file_names{f_idx};
%     full_path = fullfile(dir_path, cur_file);
%     if exist(full_path, 'file') ~= 2
%         full_path = which(cur_file);
%         if isempty(full_path)
%             fprintf("[%d/%d] スキップ (未検出): %s\n", f_idx, num_files, cur_file);
%             continue;
%         end
%     end
% 
%     try
%         loaded = load(full_path);
%         if isfield(loaded, 'log'), cur_log = loaded.log;
%         elseif isfield(loaded, 'app') && isfield(loaded.app, 'logger'), cur_log = loaded.app.logger;
%         else, cur_log = loaded; end
%     catch
%         continue;
%     end
% 
%     if ~isfield(cur_log, 'Data') || ~isfield(cur_log, 'k'), continue; end
%     if iscell(cur_log.Data.agent), agent_log = cur_log.Data.agent{1};
%     else, agent_log = cur_log.Data.agent(1); end
% 
%     N_steps = cur_log.k - 1;
%     time_vec = reshape(cur_log.Data.t(1:N_steps), 1, []);
% 
%     p_drone     = NaN(3, N_steps);
%     v_drone     = NaN(3, N_steps);
%     p_load      = NaN(3, N_steps);
%     v_load      = NaN(3, N_steps);
%     pL_ref      = NaN(3, N_steps);
%     v_ref       = NaN(3, N_steps);
%     pQ_des_true = NaN(3, N_steps); % コントローラが要求した機体の真の目標
%     is_f        = false(1, N_steps);
% 
%     for i = 1:N_steps
%         % フェーズ
%         try
%             if iscell(agent_log.cha), c = agent_log.cha{i};
%             else, c = agent_log.cha(i); end
%             is_f(i) = (c == 'f' || c == 102);
%         catch
%             is_f(i) = (time_vec(i) >= 5.0);
%         end
% 
%         % 機体実値
%         try
%             st_p = agent_log.plant.result{i}.state;
%             if isprop(st_p, "p") || isfield(st_p, "p"), p_drone(:, i) = st_p.p(1:3);
%             else, p_drone(:, i) = st_p(1:3); end
%             if isprop(st_p, "v") || isfield(st_p, "v"), v_drone(:, i) = st_p.v(1:3); end
%         catch
%         end
% 
%         % 荷物推定値
%         try
%             st_e = agent_log.estimator.result{i}.state;
%             if isprop(st_e, "pL") || isfield(st_e, "pL"), p_load(:, i) = st_e.pL(1:3);
%             else, p_load(:, i) = st_e(1:3); end
%             if isprop(st_e, "vL") || isfield(st_e, "vL"), v_load(:, i) = st_e.vL(1:3); end
%         catch
%         end
% 
%         % 荷物目標値・目標速度
%         try
%             st_r = agent_log.reference.result{i}.state;
%             if isprop(st_r, "xd") || isfield(st_r, "xd"), pL_ref(:, i) = st_r.xd(1:3);
%             elseif isprop(st_r, "p") || isfield(st_r, "p"), pL_ref(:, i) = st_r.p(1:3);
%             else, pL_ref(:, i) = st_r(1:3); end
%             if isprop(st_r, "v") || isfield(st_r, "v"), v_ref(:, i) = st_r.v(1:3); end
%         catch
%         end
% 
%         % 機体の「真の制御目標 (pQ_des)」がログにあるか探索
%         try
%             % 候補1: controller.result.pQ_des や pref など
%             st_c = agent_log.controller.result{1, i};
%             if isfield(st_c, 'pQ_des'), pQ_des_true(:, i) = st_c.pQ_des(1:3);
%             elseif isfield(st_c, 'p_drone_ref'), pQ_des_true(:, i) = st_c.p_drone_ref(1:3);
%             elseif isfield(st_c, 'xd') && length(st_c.xd) >= 6, pQ_des_true(:, i) = st_c.xd(4:6);
%             end
%         catch
%         end
%     end
% 
%     % ケーブル長
%     if isfield(cur_log, 'agent') && isfield(cur_log.agent(1), 'parameter') && isfield(cur_log.agent(1).parameter, 'cableL')
%         L_val = cur_log.agent(1).parameter.cableL;
%     else, L_val = 2.0; end
% 
%     % 名目位置
%     pQ_nom_ref = pL_ref + [0; 0; L_val];
%     pL_nom     = p_drone - [0; 0; L_val];
% 
%     % 定常区間マスク (> 10s)
%     idx_f = find(is_f);
%     if isempty(idx_f), continue; end
%     t_start = time_vec(idx_f(1));
%     t_split = t_start + 10.0;
% 
%     valid_mask = ~any(isnan(p_drone), 1) & ~any(isnan(p_load), 1) & ~any(isnan(pL_ref), 1);
%     idx_ss = find(is_f & (time_vec > t_split) & valid_mask);
%     if length(idx_ss) < 20, continue; end
% 
%     dt_nom = 0.025;
% 
%     % -------------------------------------------------------------
%     % 1. 運動強度の算出 (v_max, a_max, 曲率 kappa_max, 角速度 omega_max)
%     % -------------------------------------------------------------
%     v_ref_ss = v_ref(:, idx_ss);
%     v_mag = vecnorm(v_ref_ss, 2, 1);
%     v_max = max(v_mag, [], 'omitnan');
% 
%     % 加速度
%     a_ref_ss = zeros(3, length(idx_ss));
%     for d = 1:3
%         a_ref_ss(d, :) = gradient(v_ref_ss(d, :), dt_nom);
%     end
%     a_mag = vecnorm(a_ref_ss, 2, 1);
%     a_max = max(a_mag, [], 'omitnan');
% 
%     % 曲率 kappa = ||v x a|| / ||v||^3
%     cross_va = cross(v_ref_ss, a_ref_ss, 1);
%     num_k = vecnorm(cross_va, 2, 1);
%     den_k = max(v_mag.^3, 1e-4);
%     kappa_series = num_k ./ den_k;
%     kappa_max = max(kappa_series, [], 'omitnan');
%     omega_max = max(kappa_series .* v_mag, [], 'omitnan'); % 旋回角速度 [rad/s]
% 
%     % -------------------------------------------------------------
%     % 2. 5パラメータ同定 (等価遅延 tau* と 残差追従誤差)
%     % -------------------------------------------------------------
%     lags = 0:1:40;
%     mean_sq_d = NaN(size(lags));
%     mean_sq_l = NaN(size(lags));
%     for k = 1:length(lags)
%         lag = lags(k);
%         eval_idx = idx_ss(idx_ss > (idx_f(1) + lag));
%         ref_idx  = eval_idx - lag;
%         vp = ~any(isnan(p_drone(:, eval_idx)), 1) & ~any(isnan(pQ_nom_ref(:, ref_idx)), 1);
%         if any(vp)
%             mean_sq_d(k) = mean(sum((p_drone(:, eval_idx(vp)) - pQ_nom_ref(:, ref_idx(vp))).^2, 1));
%             mean_sq_l(k) = mean(sum((p_load(:,  eval_idx(vp)) - pL_ref(:,     ref_idx(vp))).^2, 1));
%         end
%     end
%     [~, b_d] = min(mean_sq_d, [], 'omitnan');
%     [~, b_l] = min(mean_sq_l, [], 'omitnan');
%     tau_Q = lags(b_d) * dt_nom;
%     tau_L = lags(b_l) * dt_nom;
% 
%     % 残差追従誤差 (名目基準)
%     eval_ss_d = idx_ss(idx_ss > (idx_f(1) + lags(b_d)));
%     ref_lag_d = eval_ss_d - lags(b_d);
%     eps_track_drone_nom = max(vecnorm(p_drone(:, eval_ss_d) - pQ_nom_ref(:, ref_lag_d), 2, 1), [], 'omitnan');
% 
%     eval_ss_l = idx_ss(idx_ss > (idx_f(1) + lags(b_l)));
%     ref_lag_l = eval_ss_l - lags(b_l);
%     eps_track_load = max(vecnorm(p_load(:, eval_ss_l) - pL_ref(:, ref_lag_l), 2, 1), [], 'omitnan');
% 
%     % 揺動変位 r_swing_max
%     r_swing_series = vecnorm(p_load(:, idx_ss) - pL_nom(:, idx_ss), 2, 1);
%     r_swing_max = max(r_swing_series, [], 'omitnan');
% 
%     % 索傾斜角
%     r_rel = p_load(:, idx_ss) - p_drone(:, idx_ss);
%     cos_th = min(1.0, max(-1.0, -r_rel(3, :) ./ max(vecnorm(r_rel, 2, 1), 1e-6)));
%     theta_swing_max = max(acos(cos_th) * (180 / pi), [], 'omitnan');
% 
%     % -------------------------------------------------------------
%     % 3. 機体の真の目標追従誤差 (pQ_des_true がある場合)
%     % -------------------------------------------------------------
%     has_true_pQ = ~all(isnan(pQ_des_true(:, idx_ss)), 'all');
%     if has_true_pQ
%         eps_track_drone_true = max(vecnorm(p_drone(:, idx_ss) - pQ_des_true(:, idx_ss), 2, 1), [], 'omitnan');
%     else
%         % 理論的動的目標: pQ_des = pL_ref - L * (a_ref + g*e3) / ||a_ref + g*e3||
%         g_acc = [0; 0; 9.81];
%         a_total = a_ref_ss + g_acc;
%         pQ_des_calc = pL_ref(:, idx_ss) + L_val * (a_total ./ vecnorm(a_total, 2, 1));
%         eps_track_drone_true = max(vecnorm(p_drone(:, idx_ss) - pQ_des_calc, 2, 1), [], 'omitnan');
%     end
% 
%     % -------------------------------------------------------------
%     % 4. 1時刻前 (t - dt) の目標値との遅れ誤差 (e_lag1)
%     % -------------------------------------------------------------
%     pQ_ref_1lag = [pQ_nom_ref(:, 1), pQ_nom_ref(:, 1:end-1)];
%     pL_ref_1lag = [pL_ref(:, 1), pL_ref(:, 1:end-1)];
%     e_lag1_drone = max(vecnorm(p_drone(:, idx_ss) - pQ_ref_1lag(:, idx_ss), 2, 1), [], 'omitnan');
%     e_lag1_load  = max(vecnorm(p_load(:, idx_ss)  - pL_ref_1lag(:, idx_ss),  2, 1), [], 'omitnan');
% 
%     % -------------------------------------------------------------
%     % 5. 索上の多点データ有無確認
%     % -------------------------------------------------------------
%     has_cable_mid = false;
%     try
%         if isfield(agent_log, 'plant') && isfield(agent_log.plant.result{1, idx_ss(1)}.state, 'p_cable')
%             has_cable_mid = true;
%         end
%     catch
%     end
% 
%     % 結果格納
%     result_count = result_count + 1;
%     results(result_count).FileName            = cur_file;
%     results(result_count).v_max               = v_max;
%     results(result_count).a_max               = a_max;
%     results(result_count).kappa_max           = kappa_max;
%     results(result_count).omega_max           = omega_max;
%     results(result_count).r_swing_max         = r_swing_max;
%     results(result_count).theta_swing_deg     = theta_swing_max;
%     results(result_count).tau_Q               = tau_Q;
%     results(result_count).eps_track_drone_nom = eps_track_drone_nom;
%     results(result_count).eps_track_drone_true= eps_track_drone_true;
%     results(result_count).e_lag1_drone        = e_lag1_drone;
%     results(result_count).tau_L               = tau_L;
%     results(result_count).eps_track_load      = eps_track_load;
%     results(result_count).e_lag1_load         = e_lag1_load;
%     results(result_count).has_cable_mid       = has_cable_mid;
% end
% 
% %% 結果表示
% if result_count == 0, disp("解析データがありません。"); return; end
% T_res = struct2table(results);
% 
% fprintf("\n========================================================================================================\n");
% fprintf(" 【Case別 運動強度 vs 揺動変位・真の機体追従誤差 対比表 (定常区間 > 10s)】\n");
% fprintf("========================================================================================================\n");
% disp(T_res(:, {'FileName', 'v_max', 'a_max', 'kappa_max', 'r_swing_max', 'eps_track_drone_nom', 'eps_track_drone_true', 'e_lag1_drone'}));
% 
% fprintf("\n--------------------------------------------------------------------------------------------------------\n");
% fprintf(" 【遅延同定値 & 1ステップ遅れ比較】\n");
% fprintf("--------------------------------------------------------------------------------------------------------\n");
% disp(T_res(:, {'FileName', 'tau_Q', 'eps_track_drone_nom', 'e_lag1_drone', 'tau_L', 'eps_track_load', 'e_lag1_load'}));
% 
% fprintf("\n--------------------------------------------------------------------------------------------------------\n");
% fprintf(" 【索中間点データのログ有無】: %s\n", string(any(T_res.has_cable_mid)));
% fprintf("========================================================================================================\n\n");

% %% =========================================================================
% % 吊り荷UAV 運動強度モデル vs 固定包絡判定 & 5パラメータ分離同定スクリプト
% % =========================================================================
% clear; clc;
% 
% file_names = { ...
%     'Case1no1_Log(23-Sep-2026_17_53_31).mat', ...
%     'Case1no2_Log(23-Sep-2026_17_55_56).mat', ...
%     'Case1_3_Log(23-Sep-2026_17_58_15).mat', ...
%     'Case2no1_Log(23-Sep-2026_20_17_02).mat', ...
%     'Case2no2_Log(23-Sep-2026_20_20_35).mat', ...
%     'Case2no3_Log(23-Sep-2026_20_25_38).mat', ...
%     'Case3no1_Log(23-Sep-2026_20_28_43).mat', ...
%     'Case3no2_Log(23-Sep-2026_20_31_01).mat', ...
%     'Case3no3_Log(23-Sep-2026_20_34_15).mat', ...
%     'Case4no1_Log(23-Sep-2026_20_41_38).mat', ...
%     'Case4no2_Log(23-Sep-2026_20_46_10).mat', ...
%     'Case4no3_Log(23-Sep-2026_21_42_47).mat', ...
%     'Case5no1_Log(23-Sep-2026_21_45_38).mat', ...
%     'Case5no2_Log(23-Sep-2026_21_47_26).mat', ...
%     'Case5no3_Log(23-Sep-2026_21_49_03).mat' ...
% };
% 
% if isempty(mfilename), dir_path = pwd;
% else, dir_path = fileparts(mfilename('fullpath')); end
% 
% num_files = length(file_names);
% fprintf(">>> 全 %d 件のログファイルを詳細解析します。\n\n", num_files);
% 
% result_count = 0;
% results = struct();
% 
% for f_idx = 1:num_files
%     cur_file = file_names{f_idx};
%     full_path = fullfile(dir_path, cur_file);
%     if exist(full_path, 'file') ~= 2
%         full_path = which(cur_file);
%         if isempty(full_path), continue; end
%     end
% 
%     try
%         loaded = load(full_path);
%         if isfield(loaded, 'log'), cur_log = loaded.log;
%         elseif isfield(loaded, 'app') && isfield(loaded.app, 'logger'), cur_log = loaded.app.logger;
%         else, cur_log = loaded; end
%     catch
%         continue;
%     end
% 
%     if ~isfield(cur_log, 'Data') || ~isfield(cur_log, 'k'), continue; end
%     if iscell(cur_log.Data.agent), agent_log = cur_log.Data.agent{1};
%     else, agent_log = cur_log.Data.agent(1); end
% 
%     N_steps = cur_log.k - 1;
%     time_vec = reshape(cur_log.Data.t(1:N_steps), 1, []);
% 
%     p_drone  = NaN(3, N_steps);
%     v_drone  = NaN(3, N_steps);
%     p_load   = NaN(3, N_steps);
%     v_load   = NaN(3, N_steps);
%     pL_ref   = NaN(3, N_steps);
%     v_ref    = NaN(3, N_steps);
%     is_f     = false(1, N_steps);
% 
%     for i = 1:N_steps
%         try
%             if iscell(agent_log.cha), c = agent_log.cha{i};
%             else, c = agent_log.cha(i); end
%             is_f(i) = (c == 'f' || c == 102);
%         catch
%             is_f(i) = (time_vec(i) >= 5.0);
%         end
% 
%         try
%             st_p = agent_log.plant.result{i}.state;
%             if isprop(st_p, "p") || isfield(st_p, "p"), p_drone(:, i) = st_p.p(1:3);
%             else, p_drone(:, i) = st_p(1:3); end
%             if isprop(st_p, "v") || isfield(st_p, "v"), v_drone(:, i) = st_p.v(1:3); end
%         catch
%         end
% 
%         try
%             st_e = agent_log.estimator.result{i}.state;
%             if isprop(st_e, "pL") || isfield(st_e, "pL"), p_load(:, i) = st_e.pL(1:3);
%             else, p_load(:, i) = st_e(1:3); end
%             if isprop(st_e, "vL") || isfield(st_e, "vL"), v_load(:, i) = st_e.vL(1:3); end
%         catch
%         end
% 
%         try
%             st_r = agent_log.reference.result{i}.state;
%             if isprop(st_r, "xd") || isfield(st_r, "xd"), pL_ref(:, i) = st_r.xd(1:3);
%             elseif isprop(st_r, "p") || isfield(st_r, "p"), pL_ref(:, i) = st_r.p(1:3);
%             else, pL_ref(:, i) = st_r(1:3); end
%             if isprop(st_r, "v") || isfield(st_r, "v"), v_ref(:, i) = st_r.v(1:3); end
%         catch
%         end
%     end
% 
%     if isfield(cur_log, 'agent') && isfield(cur_log.agent(1), 'parameter') && isfield(cur_log.agent(1).parameter, 'cableL')
%         L_val = cur_log.agent(1).parameter.cableL;
%     else, L_val = 2.0; end
% 
%     pQ_nom_ref = pL_ref + [0; 0; L_val];
%     pL_nom     = p_drone - [0; 0; L_val];
% 
%     idx_f = find(is_f);
%     if isempty(idx_f), continue; end
%     t_start = time_vec(idx_f(1));
%     t_split = t_start + 10.0;
% 
%     valid_mask = ~any(isnan(p_drone), 1) & ~any(isnan(p_load), 1) & ~any(isnan(pL_ref), 1);
%     idx_ss = find(is_f & (time_vec > t_split) & valid_mask);
%     if length(idx_ss) < 20, continue; end
% 
%     dt_nom = 0.025;
% 
%     % 1. 運動強度の算出
%     v_ref_ss = v_ref(:, idx_ss);
%     v_mag = vecnorm(v_ref_ss, 2, 1);
%     v_max = max(v_mag, [], 'omitnan');
% 
%     a_ref_ss = zeros(3, length(idx_ss));
%     for d = 1:3
%         a_ref_ss(d, :) = gradient(v_ref_ss(d, :), dt_nom);
%     end
%     a_mag = vecnorm(a_ref_ss, 2, 1);
%     a_max = max(a_mag, [], 'omitnan');
% 
%     % 水平加速度 a_xy_max (傾き・揺動の主要因)
%     a_xy_mag = vecnorm(a_ref_ss(1:2, :), 2, 1);
%     a_xy_max = max(a_xy_mag, [], 'omitnan');
% 
%     % 平坦性理論による定常傾き予測揺動量: r_theory = L * a_xy / sqrt(a_xy^2 + g^2)
%     g_acc = 9.81;
%     r_swing_theory = L_val * (a_xy_max / sqrt(a_xy_max^2 + g_acc^2));
% 
%     % 2. 5パラメータの同定
%     lags = 0:1:40;
%     mean_sq_d = NaN(size(lags));
%     mean_sq_l = NaN(size(lags));
%     for k = 1:length(lags)
%         lag = lags(k);
%         eval_idx = idx_ss(idx_ss > (idx_f(1) + lag));
%         ref_idx  = eval_idx - lag;
%         vp = ~any(isnan(p_drone(:, eval_idx)), 1) & ~any(isnan(pQ_nom_ref(:, ref_idx)), 1);
%         if any(vp)
%             mean_sq_d(k) = mean(sum((p_drone(:, eval_idx(vp)) - pQ_nom_ref(:, ref_idx(vp))).^2, 1));
%             mean_sq_l(k) = mean(sum((p_load(:,  eval_idx(vp)) - pL_ref(:,     ref_idx(vp))).^2, 1));
%         end
%     end
%     [~, b_d] = min(mean_sq_d, [], 'omitnan');
%     [~, b_l] = min(mean_sq_l, [], 'omitnan');
%     tau_Q = lags(b_d) * dt_nom;
%     tau_L = lags(b_l) * dt_nom;
% 
%     % 名目基準の追従誤差
%     eps_track_drone_nom = max(vecnorm(p_drone(:, idx_ss) - pQ_nom_ref(:, idx_ss), 2, 1), [], 'omitnan');
%     eps_track_load      = max(vecnorm(p_load(:, idx_ss)  - pL_ref(:, idx_ss),     2, 1), [], 'omitnan');
% 
%     % 実測揺動変位 r_swing_max
%     r_swing_series = vecnorm(p_load(:, idx_ss) - pL_nom(:, idx_ss), 2, 1);
%     r_swing_max = max(r_swing_series, [], 'omitnan');
% 
%     % 【最重要】UAV真のサーボ追従残差の分離推計:
%     % 動的目標位置: pQ_des(t) = pL_ref(t) + L * (a_ref + [0;0;g]) / ||a_ref + [0;0;g]||
%     a_total_vec = a_ref_ss + [0; 0; g_acc];
%     pQ_des_dyn = pL_ref(:, idx_ss) + L_val * (a_total_vec ./ vecnorm(a_total_vec, 2, 1));
%     % 動的目標に対する純粋な機体追従残差 (R_residual,Q)
%     R_residual_Q = max(vecnorm(p_drone(:, idx_ss) - pQ_des_dyn, 2, 1), [], 'omitnan');
% 
%     % 1ステップ遅れ目標との差
%     pQ_ref_1lag = [pQ_nom_ref(:, 1), pQ_nom_ref(:, 1:end-1)];
%     e_lag1_drone = max(vecnorm(p_drone(:, idx_ss) - pQ_ref_1lag(:, idx_ss), 2, 1), [], 'omitnan');
% 
%     result_count = result_count + 1;
%     results(result_count).FileName            = cur_file;
%     results(result_count).v_max               = v_max;
%     results(result_count).a_max               = a_max;
%     results(result_count).a_xy_max            = a_xy_max;
%     results(result_count).r_swing_theory      = r_swing_theory;
%     results(result_count).r_swing_max         = r_swing_max;
%     results(result_count).tau_Q               = tau_Q;
%     results(result_count).tau_L               = tau_L;
%     results(result_count).eps_track_drone_nom = eps_track_drone_nom;
%     results(result_count).R_residual_Q        = R_residual_Q;
%     results(result_count).eps_track_load      = eps_track_load;
%     results(result_count).e_lag1_drone        = e_lag1_drone;
% end
% 
% if result_count == 0, disp("解析データがありません。"); return; end
% T_res = struct2table(results);
% 
% %% 整形表示
% fprintf("\n========================================================================================================================\n");
% fprintf(" 【1. 運動強度 vs 揺動変位：理論値 vs 実測値 対比表】\n");
% fprintf("========================================================================================================================\n");
% disp(T_res(:, {'FileName', 'v_max', 'a_max', 'a_xy_max', 'r_swing_theory', 'r_swing_max'}));
% 
% fprintf("\n========================================================================================================================\n");
% fprintf(" 【2. 二重計上分離：名目誤差 vs 真の機体残差 R_residual,Q vs 遅延】\n");
% fprintf("========================================================================================================================\n");
% disp(T_res(:, {'FileName', 'eps_track_drone_nom', 'R_residual_Q', 'tau_Q', 'e_lag1_drone', 'tau_L', 'eps_track_load'}));
% 
% fprintf("\n========================================================================================================================\n");
% fprintf(" 【3. Case別 代表統計サマリー (Case 1〜5 代表値)】\n");
% fprintf("========================================================================================================================\n");
% fprintf("  Case区分       | a_xy [m/s2] | 理論揺動 [m] | 実測揺動 [m] | 機体名目誤差 [m] | 真の機体残差 [m] | 荷物追従誤差 [m]\n");
% fprintf("-----------------+-------------+--------------+--------------+------------------+------------------+-----------------\n");
% 
% for c_num = 1:5
%     idx_c = contains(T_res.FileName, sprintf("Case%d", c_num));
%     if any(idx_c)
%         sub_T = T_res(idx_c, :);
%         fprintf("  Case %d (平均)  |   %7.3f   |   %7.4f   |   %7.4f   |     %7.4f      |     %7.4f      |     %7.4f\n", ...
%             c_num, mean(sub_T.a_xy_max), mean(sub_T.r_swing_theory), mean(sub_T.r_swing_max), ...
%             mean(sub_T.eps_track_drone_nom), mean(sub_T.R_residual_Q), mean(sub_T.eps_track_load));
%     end
% end
% fprintf("========================================================================================================================\n\n");

% %% =========================================================================
% % 吊り荷UAV Sphere-Chain 不確実性・運動強度・幾何残差 完全統合解析スクリプト
% % 
% % 【本スクリプトの目的と役割】
% % 1. 15件のシミュレーションログから、Sphere-ChainモデルおよびCPAフィルタで用いる
% %    5つのコア不確実性パラメータ (tau_Q, tau_L, R_residual_Q, eps_track_L, r_swing_max)
% %    を「二重計上なし」に厳密同定する。
% % 2. 「見かけの名目追従誤差」と「真の動的サーボ追従残差」を分離し、
% %    Case 4/5 における見かけの誤差 (0.93m) が旋回傾斜幾何に起因することを証明する。
% % 3. 理論定常傾斜モデル r_theory(t) と実測揺動 r_swing(t) の時系列残差、
% %    および最大躍度 (Jerk) を評価し、Case 5 で生じる過渡揺動の発生要因を解明する。
% % 4. 1ステップ遅延、CPA等速予測乖離 (1〜6s)、索上揺動プロファイルを一括出力する。
% % =========================================================================
% clear; clc;
% 
% % 1. 解析対象ファイルリスト（全15試行）
% file_names = { ...
%     'Case1no1_Log(23-Sep-2026_17_53_31).mat', ...
%     'Case1no2_Log(23-Sep-2026_17_55_56).mat', ...
%     'Case1_3_Log(23-Sep-2026_17_58_15).mat', ...
%     'Case2no1_Log(23-Sep-2026_20_17_02).mat', ...
%     'Case2no2_Log(23-Sep-2026_20_20_35).mat', ...
%     'Case2no3_Log(23-Sep-2026_20_25_38).mat', ...
%     'Case3no1_Log(23-Sep-2026_20_28_43).mat', ...
%     'Case3no2_Log(23-Sep-2026_20_31_01).mat', ...
%     'Case3no3_Log(23-Sep-2026_20_34_15).mat', ...
%     'Case4no1_Log(23-Sep-2026_20_41_38).mat', ...
%     'Case4no2_Log(23-Sep-2026_20_46_10).mat', ...
%     'Case4no3_Log(23-Sep-2026_21_42_47).mat', ...
%     'Case5no1_Log(23-Sep-2026_21_45_38).mat', ...
%     'Case5no2_Log(23-Sep-2026_21_47_26).mat', ...
%     'Case5no3_Log(23-Sep-2026_21_49_03).mat' ...
% };
% 
% if isempty(mfilename), dir_path = pwd;
% else, dir_path = fileparts(mfilename('fullpath')); end
% 
% num_files = length(file_names);
% fprintf(">>> 全 %d 件のログファイルを完全統合解析します。\n\n", num_files);
% 
% result_count = 0;
% results = struct();
% 
% %% 2. ファイル一括ループ処理
% for f_idx = 1:num_files
%     cur_file = file_names{f_idx};
%     full_path = fullfile(dir_path, cur_file);
%     if exist(full_path, 'file') ~= 2
%         full_path = which(cur_file);
%         if isempty(full_path), continue; end
%     end
% 
%     % 【上書き衝突防止】一時構造体経由での読み込み
%     try
%         loaded = load(full_path);
%         if isfield(loaded, 'log'), cur_log = loaded.log;
%         elseif isfield(loaded, 'app') && isfield(loaded.app, 'logger'), cur_log = loaded.app.logger;
%         else, cur_log = loaded; end
%     catch
%         continue;
%     end
% 
%     if ~isfield(cur_log, 'Data') || ~isfield(cur_log, 'k'), continue; end
%     if iscell(cur_log.Data.agent), agent_log = cur_log.Data.agent{1};
%     else, agent_log = cur_log.Data.agent(1); end
% 
%     N_steps = cur_log.k - 1;
%     time_vec = reshape(cur_log.Data.t(1:N_steps), 1, []);
% 
%     % 【欠測値保護】NaNで初期化
%     p_drone     = NaN(3, N_steps);
%     v_drone     = NaN(3, N_steps);
%     p_load      = NaN(3, N_steps);
%     v_load      = NaN(3, N_steps);
%     pL_ref      = NaN(3, N_steps);
%     v_ref       = NaN(3, N_steps);
%     pQ_des_true = NaN(3, N_steps);
%     is_f        = false(1, N_steps);
% 
%     for i = 1:N_steps
%         % フェーズ判定 (飛行フェーズ 'f' のみ厳密抽出)
%         try
%             if iscell(agent_log.cha), c = agent_log.cha{i};
%             else, c = agent_log.cha(i); end
%             is_f(i) = (c == 'f' || c == 102);
%         catch
%             is_f(i) = (time_vec(i) >= 5.0);
%         end
% 
%         % 機体状態 (plant/sensor)
%         try
%             st_p = agent_log.plant.result{i}.state;
%             if isprop(st_p, "p") || isfield(st_p, "p"), p_drone(:, i) = st_p.p(1:3);
%             else, p_drone(:, i) = st_p(1:3); end
%             if isprop(st_p, "v") || isfield(st_p, "v"), v_drone(:, i) = st_p.v(1:3); end
%         catch
%         end
% 
%         % 荷物状態 (estimator)
%         try
%             st_e = agent_log.estimator.result{i}.state;
%             if isprop(st_e, "pL") || isfield(st_e, "pL"), p_load(:, i) = st_e.pL(1:3);
%             else, p_load(:, i) = st_e(1:3); end
%             if isprop(st_e, "vL") || isfield(st_e, "vL"), v_load(:, i) = st_e.vL(1:3); end
%         catch
%         end
% 
%         % 目標状態 (reference)
%         try
%             st_r = agent_log.reference.result{i}.state;
%             if isprop(st_r, "xd") || isfield(st_r, "xd"), pL_ref(:, i) = st_r.xd(1:3);
%             elseif isprop(st_r, "p") || isfield(st_r, "p"), pL_ref(:, i) = st_r.p(1:3);
%             else, pL_ref(:, i) = st_r(1:3); end
%             if isprop(st_r, "v") || isfield(st_r, "v"), v_ref(:, i) = st_r.v(1:3); end
%         catch
%         end
% 
%         % 制御器内部の真の機体目標位置 (保存されている場合)
%         try
%             st_c = agent_log.controller.result{1, i};
%             if isfield(st_c, 'pQ_des'), pQ_des_true(:, i) = st_c.pQ_des(1:3);
%             elseif isfield(st_c, 'p_drone_ref'), pQ_des_true(:, i) = st_c.p_drone_ref(1:3);
%             end
%         catch
%         end
%     end
% 
%     % ケーブル長
%     if isfield(cur_log, 'agent') && isfield(cur_log.agent(1), 'parameter') && isfield(cur_log.agent(1).parameter, 'cableL')
%         L_val = cur_log.agent(1).parameter.cableL;
%     else, L_val = 2.0; end
% 
%     % 名目位置の幾何定義 (+Zが鉛直上方, -Zが鉛直下方)
%     pQ_nom_ref = pL_ref + [0; 0; L_val];
%     pL_nom     = p_drone - [0; 0; L_val];
% 
%     % 定常区間マスク (飛行開始後 10 秒以降かつ欠損のない有効データ)
%     idx_f = find(is_f);
%     if isempty(idx_f), continue; end
%     t_start = time_vec(idx_f(1));
%     t_split = t_start + 10.0;
% 
%     valid_mask = ~any(isnan(p_drone), 1) & ~any(isnan(p_load), 1) & ...
%                   ~any(isnan(pL_ref), 1)  & ~any(isnan(v_drone), 1);
%     idx_ss = find(is_f & (time_vec > t_split) & valid_mask);
%     if length(idx_ss) < 20, continue; end
% 
%     dt_nom = 0.025;
% 
%     % -------------------------------------------------------------
%     % A. 運動強度・躍度 (Jerk) の算出
%     % -------------------------------------------------------------
%     v_ref_ss = v_ref(:, idx_ss);
%     v_mag = vecnorm(v_ref_ss, 2, 1);
%     v_max = max(v_mag, [], 'omitnan');
% 
%     % 加速度 a(t)
%     a_ref_ss = zeros(3, length(idx_ss));
%     for d = 1:3
%         a_ref_ss(d, :) = gradient(v_ref_ss(d, :), dt_nom);
%     end
%     a_mag = vecnorm(a_ref_ss, 2, 1);
%     a_max = max(a_mag, [], 'omitnan');
% 
%     % 水平加速度 a_xy(t)
%     a_xy_mag = vecnorm(a_ref_ss(1:2, :), 2, 1);
%     a_xy_max = max(a_xy_mag, [], 'omitnan');
% 
%     % 躍度 j(t) = da/dt [m/s^3]
%     j_ref_ss = zeros(3, length(idx_ss));
%     for d = 1:3
%         j_ref_ss(d, :) = gradient(a_ref_ss(d, :), dt_nom);
%     end
%     j_mag = vecnorm(j_ref_ss, 2, 1);
%     j_max = max(j_mag, [], 'omitnan');
% 
%     % 旋回曲率 kappa と旋回角速度 omega
%     cross_va = cross(v_ref_ss, a_ref_ss, 1);
%     kappa_series = vecnorm(cross_va, 2, 1) ./ max(v_mag.^3, 1e-4);
%     kappa_max = max(kappa_series, [], 'omitnan');
%     omega_max = max(kappa_series .* v_mag, [], 'omitnan');
% 
%     % -------------------------------------------------------------
%     % B. 等価遅延 tau* と 遅延補正後残差 (二重計上防止)
%     % -------------------------------------------------------------
%     lags = 0:1:40;
%     mean_sq_d = NaN(size(lags));
%     mean_sq_l = NaN(size(lags));
%     for k = 1:length(lags)
%         lag = lags(k);
%         eval_idx = idx_ss(idx_ss > (idx_f(1) + lag));
%         ref_idx  = eval_idx - lag;
%         vp = ~any(isnan(p_drone(:, eval_idx)), 1) & ~any(isnan(pQ_nom_ref(:, ref_idx)), 1);
%         if any(vp)
%             mean_sq_d(k) = mean(sum((p_drone(:, eval_idx(vp)) - pQ_nom_ref(:, ref_idx(vp))).^2, 1));
%             mean_sq_l(k) = mean(sum((p_load(:,  eval_idx(vp)) - pL_ref(:,     ref_idx(vp))).^2, 1));
%         end
%     end
%     [~, b_d] = min(mean_sq_d, [], 'omitnan');
%     [~, b_l] = min(mean_sq_l, [], 'omitnan');
%     tau_Q = lags(b_d) * dt_nom;
%     tau_L = lags(b_l) * dt_nom;
% 
%     % 荷物追従誤差 eps_track_load
%     eval_ss_l = idx_ss(idx_ss > (idx_f(1) + lags(b_l)));
%     ref_lag_l = eval_ss_l - lags(b_l);
%     eps_track_load = max(vecnorm(p_load(:, eval_ss_l) - pL_ref(:, ref_lag_l), 2, 1), [], 'omitnan');
% 
%     % 見かけの機体名目誤差
%     eps_track_drone_nom = max(vecnorm(p_drone(:, idx_ss) - pQ_nom_ref(:, idx_ss), 2, 1), [], 'omitnan');
% 
%     % -------------------------------------------------------------
%     % C. 機体真の追従残差 R_residual_Q の分離同定
%     % -------------------------------------------------------------
%     % 動的平坦性目標: pQ_des(t) = pL_ref(t) + L * (a_ref + [0;0;g]) / ||a_ref + [0;0;g]||
%     g_acc = 9.81;
%     a_total_vec = a_ref_ss + [0; 0; g_acc];
%     pQ_des_dyn = pL_ref(:, idx_ss) + L_val * (a_total_vec ./ vecnorm(a_total_vec, 2, 1));
% 
%     if ~all(isnan(pQ_des_true(:, idx_ss)), 'all')
%         R_residual_Q = max(vecnorm(p_drone(:, idx_ss) - pQ_des_true(:, idx_ss), 2, 1), [], 'omitnan');
%     else
%         R_residual_Q = max(vecnorm(p_drone(:, idx_ss) - pQ_des_dyn, 2, 1), [], 'omitnan');
%     end
% 
%     % -------------------------------------------------------------
%     % D. 揺動変位の実測 vs 時系列理論値 & 残差
%     % -------------------------------------------------------------
%     r_swing_series = vecnorm(p_load(:, idx_ss) - pL_nom(:, idx_ss), 2, 1);
%     r_swing_max = max(r_swing_series, [], 'omitnan');
% 
%     % 各時刻 t における動的理論揺動量 r_theory(t)
%     r_theory_series = L_val * (a_xy_mag ./ sqrt(a_xy_mag.^2 + g_acc^2));
%     r_swing_theory_max = max(r_theory_series, [], 'omitnan');
% 
%     % 時系列揺動残差 Delta_r_swing(t) = r_swing(t) - r_theory(t)
%     delta_r_swing_series = r_swing_series - r_theory_series;
%     delta_r_swing_max = max(delta_r_swing_series, [], 'omitnan');
% 
%     % 索最大傾斜角 [deg]
%     r_rel = p_load(:, idx_ss) - p_drone(:, idx_ss);
%     cos_th = min(1.0, max(-1.0, -r_rel(3, :) ./ max(vecnorm(r_rel, 2, 1), 1e-6)));
%     theta_swing_max = max(acos(cos_th) * (180 / pi), [], 'omitnan');
% 
%     % -------------------------------------------------------------
%     % E. 1ステップ遅延 & CPA短時間予測乖離 (1〜6s)
%     % -------------------------------------------------------------
%     pQ_ref_1lag = [pQ_nom_ref(:, 1), pQ_nom_ref(:, 1:end-1)];
%     pL_ref_1lag = [pL_ref(:, 1), pL_ref(:, 1:end-1)];
%     e_lag1_drone = max(vecnorm(p_drone(:, idx_ss) - pQ_ref_1lag(:, idx_ss), 2, 1), [], 'omitnan');
%     e_lag1_load  = max(vecnorm(p_load(:, idx_ss)  - pL_ref_1lag(:, idx_ss),  2, 1), [], 'omitnan');
% 
%     horizons = [1.0, 2.0, 4.0, 6.0];
%     pred_errs = NaN(1, 4);
%     for h = 1:4
%         H_steps = round(horizons(h) / dt_nom);
%         val_idx = idx_ss(1 : (end - H_steps));
%         if ~isempty(val_idx)
%             p_pred = p_drone(:, val_idx) + v_drone(:, val_idx) * horizons(h);
%             p_true = p_drone(:, val_idx + H_steps);
%             pred_errs(h) = max(vecnorm(p_true - p_pred, 2, 1), [], 'omitnan');
%         end
%     end
% 
%     % 実速度最大値
%     v_drone_max = max(vecnorm(v_drone(:, idx_ss), 2, 1), [], 'omitnan');
%     v_load_max  = max(vecnorm(v_load(:, idx_ss), 2, 1),  [], 'omitnan');
% 
%     % 結果格納
%     result_count = result_count + 1;
%     results(result_count).FileName            = cur_file;
%     results(result_count).v_max               = v_max;
%     results(result_count).a_max               = a_max;
%     results(result_count).a_xy_max            = a_xy_max;
%     results(result_count).j_max               = j_max;
%     results(result_count).omega_max           = omega_max;
%     results(result_count).r_swing_max         = r_swing_max;
%     results(result_count).r_swing_theory_max  = r_swing_theory_max;
%     results(result_count).delta_r_swing_max   = delta_r_swing_max;
%     results(result_count).theta_swing_deg     = theta_swing_max;
%     results(result_count).tau_Q               = tau_Q;
%     results(result_count).R_residual_Q        = R_residual_Q;
%     results(result_count).eps_track_drone_nom = eps_track_drone_nom;
%     results(result_count).e_lag1_drone        = e_lag1_drone;
%     results(result_count).tau_L               = tau_L;
%     results(result_count).eps_track_load      = eps_track_load;
%     results(result_count).e_lag1_load         = e_lag1_load;
%     results(result_count).v_drone_max         = v_drone_max;
%     results(result_count).v_load_max          = v_load_max;
%     results(result_count).pred_err_1s         = pred_errs(1);
%     results(result_count).pred_err_2s         = pred_errs(2);
%     results(result_count).pred_err_4s         = pred_errs(3);
%     results(result_count).pred_err_6s         = pred_errs(4);
% end
% 
% if result_count == 0, disp("解析データがありません。"); return; end
% T_res = struct2table(results);
% 
% %% 3. コンソール整形表示
% fprintf("\n========================================================================================================================\n");
% fprintf(" 【表 1：運動強度・躍度 (Jerk) vs 揺動変位・動的残差 対比表 (定常区間 > 10s)】\n");
% fprintf("  ※ delta_r_sw = 各時刻 t における実測揺動と定常傾き理論値の差 max(r_swing(t) - r_theory(t))\n");
% fprintf("========================================================================================================================\n");
% disp(T_res(:, {'FileName', 'v_max', 'a_xy_max', 'j_max', 'r_swing_theory_max', 'r_swing_max', 'delta_r_swing_max'}));
% 
% fprintf("\n========================================================================================================================\n");
% fprintf(" 【表 2：機体側不確実性の分離同定 (見かけの名目誤差 vs 真の動的残差 vs 遅延)】\n");
% fprintf("  ※ eps_nom (見かけの鉛直誤差) に対し、R_res_Q (動的目標残差) が激減していることを確認\n");
% fprintf("========================================================================================================================\n");
% disp(T_res(:, {'FileName', 'tau_Q', 'eps_track_drone_nom', 'R_residual_Q', 'e_lag1_drone', 'tau_L', 'eps_track_load'}));
% 
% fprintf("\n========================================================================================================================\n");
% fprintf(" 【表 3：Case 1〜5 代表統計サマリー (同一Caseの再現性確認と条件依存性)】\n");
% fprintf("========================================================================================================================\n");
% fprintf("  Case区分       | a_xy [m/s2] | Jerk [m/s3] | 理論揺動 [m] | 実測揺動 [m] | 揺動残差 [m] | 機体名目誤差 [m] | 機体動的残差 [m]\n");
% fprintf("-----------------+-------------+-------------+--------------+--------------+--------------+------------------+-----------------\n");
% 
% for c_num = 1:5
%     idx_c = contains(T_res.FileName, sprintf("Case%d", c_num));
%     if any(idx_c)
%         sub_T = T_res(idx_c, :);
%         fprintf("  Case %d (平均)  |   %7.3f   |   %7.3f   |   %7.4f   |   %7.4f   |   %7.4f   |     %7.4f      |     %7.4f\n", ...
%             c_num, mean(sub_T.a_xy_max), mean(sub_T.j_max), mean(sub_T.r_swing_theory_max), ...
%             mean(sub_T.r_swing_max), mean(sub_T.delta_r_swing_max), ...
%             mean(sub_T.eps_track_drone_nom), mean(sub_T.R_residual_Q));
%     end
% end
% 
% fprintf("\n========================================================================================================================\n");
% fprintf(" 【表 4：モデル検証指標サマリー (索傾斜角・実速度・CPA予測外挿乖離)】\n");
% fprintf("========================================================================================================================\n");
% fprintf("  パラメータ名           |  最小値   |   平均値  |   中央値  |   最大値 (最悪値)   |  標準偏差 \n");
% fprintf("-------------------------+-----------+-----------+-----------+---------------------+-----------\n");
% 
% print_stat = @(name, vec) fprintf("  %-22s |  %7.4f  |  %7.4f  |  %7.4f  |      %7.4f          |  %7.4f\n", ...
%     name, min(vec), mean(vec), median(vec), max(vec), std(vec));
% 
% print_stat("theta_swing_deg [deg]", T_res.theta_swing_deg);
% print_stat("v_drone_max     [m/s]", T_res.v_drone_max);
% print_stat("v_load_max      [m/s]", T_res.v_load_max);
% print_stat("pred_err (1.0s) [m]",   T_res.pred_err_1s);
% print_stat("pred_err (2.0s) [m]",   T_res.pred_err_2s);
% print_stat("pred_err (6.0s) [m]",   T_res.pred_err_6s);
% 
% fprintf("\n========================================================================================================================\n");
% fprintf(" 【Sphere-Chain 設計式への代入候補パラメータ整理 (二重計上完全排除版)】\n");
% fprintf("========================================================================================================================\n");
% fprintf("  ■ UAV 球半径   : R_Q(t)   = r_Q + ||v_Q(t)|| * tau_Q + R_residual_Q + d_margin\n");
% fprintf("    - tau_Q (等価遅延)        : 観測最大 = %7.4f [s] (Case 1: 0.200s, Case 4/5: 0.000s)\n", max(T_res.tau_Q));
% fprintf("    - R_residual_Q (動的残差) : 観測最大 = %7.4f [m] (Case 1: 0.267m, Case 4/5: <0.08m)\n", max(T_res.R_residual_Q));
% fprintf("  ■ Payload 球半径: R_L(t)   = r_L + ||v_L(t)|| * tau_L + eps_track_load + r_swing_max + d_margin\n");
% fprintf("    - tau_L (等価遅延)        : 観測最大 = %7.4f [s] (Case 1: 0.225s)\n", max(T_res.tau_L));
% fprintf("    - eps_track_load (追従残差): 観測最大 = %7.4f [m] (Case 1: 0.314m)\n", max(T_res.eps_track_load));
% fprintf("    - r_swing_max (揺動包絡)  : 観測最大 = %7.4f [m] (Case 5: 0.9304m)\n", max(T_res.r_swing_max));
% fprintf("  ■ Cable 中間球 : R_C(s,t) = r_C + R_delay(s) + R_residual(s) + r_swing(s) + d_margin\n");
% fprintf("========================================================================================================================\n\n");

%% =========================================================================
% 吊り荷UAV Sphere-Chain 全不確実性成分・統計量一括抽出スクリプト (代入安全版)
% =========================================================================
clear; clc;

% 1. 対象15ログファイル
file_names = { ...
    'Case1no1_Log(23-Sep-2026_17_53_31).mat', ...
    'Case1no2_Log(23-Sep-2026_17_55_56).mat', ...
    'Case1_3_Log(23-Sep-2026_17_58_15).mat', ...
    'Case2no1_Log(23-Sep-2026_20_17_02).mat', ...
    'Case2no2_Log(23-Sep-2026_20_20_35).mat', ...
    'Case2no3_Log(23-Sep-2026_20_25_38).mat', ...
    'Case3no1_Log(23-Sep-2026_20_28_43).mat', ...
    'Case3no2_Log(23-Sep-2026_20_31_01).mat', ...
    'Case3no3_Log(23-Sep-2026_20_34_15).mat', ...
    'Case4no1_Log(23-Sep-2026_20_41_38).mat', ...
    'Case4no2_Log(23-Sep-2026_20_46_10).mat', ...
    'Case4no3_Log(23-Sep-2026_21_42_47).mat', ...
    'Case5no1_Log(23-Sep-2026_21_45_38).mat', ...
    'Case5no2_Log(23-Sep-2026_21_47_26).mat', ...
    'Case5no3_Log(23-Sep-2026_21_49_03).mat' ...
};

if isempty(mfilename), dir_path = pwd;
else, dir_path = fileparts(mfilename('fullpath')); end

num_files = length(file_names);
fprintf(">>> 全 %d 件のログファイルから全観測統計量を抽出します...\n\n", num_files);

% 統計値算出ヘルパー関数 (NaN完全除去対応)
calc_stat = @(v) struct( ...
    'max',  max(v, [], 'omitnan'), ...
    'mean', mean(v, 'omitnan'), ...
    'rms',  sqrt(mean(v.^2, 'omitnan')), ...
    'std',  std(v, 'omitnan'), ...
    'p95',  prctile(v(~isnan(v)), 95), ...
    'p99',  prctile(v(~isnan(v)), 99));

result_count = 0;
results_cell = {}; % 【修正】セル配列で安全に格納

%% 2. 各ファイルの一括ループ処理
for f_idx = 1:num_files
    cur_file = file_names{f_idx};
    full_path = fullfile(dir_path, cur_file);
    if exist(full_path, 'file') ~= 2
        full_path = which(cur_file);
        if isempty(full_path), continue; end
    end

    try
        loaded = load(full_path);
        if isfield(loaded, 'log'), cur_log = loaded.log;
        elseif isfield(loaded, 'app') && isfield(loaded.app, 'logger'), cur_log = loaded.app.logger;
        else, cur_log = loaded; end
    catch
        continue;
    end

    if ~isfield(cur_log, 'Data') || ~isfield(cur_log, 'k'), continue; end
    if iscell(cur_log.Data.agent), agent_log = cur_log.Data.agent{1};
    else, agent_log = cur_log.Data.agent(1); end

    N_steps = cur_log.k - 1;
    time_vec = reshape(cur_log.Data.t(1:N_steps), 1, []);

    p_drone = NaN(3, N_steps); v_drone = NaN(3, N_steps);
    p_load  = NaN(3, N_steps); v_load  = NaN(3, N_steps);
    pL_ref  = NaN(3, N_steps); v_ref   = NaN(3, N_steps);
    is_f    = false(1, N_steps);

    for i = 1:N_steps
        % フェーズ判定
        try
            if iscell(agent_log.cha), c = agent_log.cha{i};
            else, c = agent_log.cha(i); end
            is_f(i) = (c == 'f' || c == 102);
        catch
            is_f(i) = (time_vec(i) >= 5.0);
        end

        % 機体実値
        try
            st_p = agent_log.plant.result{i}.state;
            if isprop(st_p, "p") || isfield(st_p, "p"), p_drone(:, i) = st_p.p(1:3);
            else, p_drone(:, i) = st_p(1:3); end
            if isprop(st_p, "v") || isfield(st_p, "v"), v_drone(:, i) = st_p.v(1:3); end
        catch
        end

        % 荷物推定値
        try
            st_e = agent_log.estimator.result{i}.state;
            if isprop(st_e, "pL") || isfield(st_e, "pL"), p_load(:, i) = st_e.pL(1:3);
            else, p_load(:, i) = st_e(1:3); end
            if isprop(st_e, "vL") || isfield(st_e, "vL"), v_load(:, i) = st_e.vL(1:3); end
        catch
        end

        % 荷物目標値・目標速度
        try
            st_r = agent_log.reference.result{i}.state;
            if isprop(st_r, "xd") || isfield(st_r, "xd"), pL_ref(:, i) = st_r.xd(1:3);
            elseif isprop(st_r, "p") || isfield(st_r, "p"), pL_ref(:, i) = st_r.p(1:3);
            else, pL_ref(:, i) = st_r(1:3); end
            if isprop(st_r, "v") || isfield(st_r, "v"), v_ref(:, i) = st_r.v(1:3); end
        catch
        end
    end

    if isfield(cur_log, 'agent') && isfield(cur_log.agent(1), 'parameter') && isfield(cur_log.agent(1).parameter, 'cableL')
        L_val = cur_log.agent(1).parameter.cableL;
    else, L_val = 2.0; end

    dt_nom = 0.025;
    g_acc = 9.81;

    % 名目位置
    pQ_nom_ref = pL_ref + [0; 0; L_val];
    pL_nom     = p_drone - [0; 0; L_val];

    % 定常区間マスク (> 10s かつ有効値)
    idx_f = find(is_f);
    if isempty(idx_f), continue; end
    t_start = time_vec(idx_f(1));
    t_split = t_start + 10.0;
    
    valid_mask = ~any(isnan(p_drone), 1) & ~any(isnan(p_load), 1) & ...
                  ~any(isnan(pL_ref), 1)  & ~any(isnan(v_drone), 1);
    idx_ss = find(is_f & (time_vec > t_split) & valid_mask);
    if length(idx_ss) < 40, continue; end

    % 加速度・Jerk の算出
    a_ref_ss = zeros(3, length(idx_ss));
    for d = 1:3, a_ref_ss(d, :) = gradient(v_ref(d, idx_ss), dt_nom); end
    
    a_drone_ss = zeros(3, length(idx_ss));
    for d = 1:3, a_drone_ss(d, :) = gradient(v_drone(d, idx_ss), dt_nom); end

    a_load_ss = zeros(3, length(idx_ss));
    for d = 1:3, a_load_ss(d, :) = gradient(v_load(d, idx_ss), dt_nom); end

    j_drone_ss = zeros(3, length(idx_ss));
    for d = 1:3, j_drone_ss(d, :) = gradient(a_drone_ss(d, :), dt_nom); end

    % 動的平坦性目標
    a_total_vec = a_ref_ss + [0; 0; g_acc];
    pQ_des_dyn = pL_ref(:, idx_ss) + L_val * (a_total_vec ./ vecnorm(a_total_vec, 2, 1));

    % 1. 全要素位置残差 (e_pos)
    e_pos_Q_nom = vecnorm(p_drone(:, idx_ss) - pQ_nom_ref(:, idx_ss), 2, 1);
    e_pos_Q_dyn = vecnorm(p_drone(:, idx_ss) - pQ_des_dyn, 2, 1);
    e_pos_L     = vecnorm(p_load(:,  idx_ss) - pL_ref(:,     idx_ss), 2, 1);

    st_pos_Q_nom = calc_stat(e_pos_Q_nom);
    st_pos_Q_dyn = calc_stat(e_pos_Q_dyn);
    st_pos_L     = calc_stat(e_pos_L);

    % 2. 1ステップ遅れ残差 (e_lag1: t - dt)
    pQ_nom_ref_1lag = [pQ_nom_ref(:, 1), pQ_nom_ref(:, 1:end-1)];
    pL_ref_1lag     = [pL_ref(:, 1),     pL_ref(:, 1:end-1)];
    e_lag1_Q = vecnorm(p_drone(:, idx_ss) - pQ_nom_ref_1lag(:, idx_ss), 2, 1);
    e_lag1_L = vecnorm(p_load(:,  idx_ss) - pL_ref_1lag(:,     idx_ss), 2, 1);

    st_lag1_Q = calc_stat(e_lag1_Q);
    st_lag1_L = calc_stat(e_lag1_L);

    % 3. 等価遅延 tau* と 遅延補正後残差
    lags = 0:1:40;
    mse_Q = NaN(size(lags)); mse_L = NaN(size(lags));
    for k = 1:length(lags)
        lag = lags(k);
        ev_idx = idx_ss(idx_ss > (idx_f(1) + lag));
        rf_idx = ev_idx - lag;
        vp = ~any(isnan(p_drone(:, ev_idx)), 1) & ~any(isnan(pQ_nom_ref(:, rf_idx)), 1);
        if any(vp)
            mse_Q(k) = mean(sum((p_drone(:, ev_idx(vp)) - pQ_nom_ref(:, rf_idx(vp))).^2, 1));
            mse_L(k) = mean(sum((p_load(:,  ev_idx(vp)) - pL_ref(:,     rf_idx(vp))).^2, 1));
        end
    end
    [~, b_q] = min(mse_Q, [], 'omitnan');
    [~, b_l] = min(mse_L, [], 'omitnan');
    tau_Q_eq = lags(b_q) * dt_nom;
    tau_L_eq = lags(b_l) * dt_nom;

    ev_q = idx_ss(idx_ss > (idx_f(1) + lags(b_q)));
    rf_q = ev_q - lags(b_q);
    e_tau_Q = vecnorm(p_drone(:, ev_q) - pQ_nom_ref(:, rf_q), 2, 1);

    ev_l = idx_ss(idx_ss > (idx_f(1) + lags(b_l)));
    rf_l = ev_l - lags(b_l);
    e_tau_L = vecnorm(p_load(:, ev_l) - pL_ref(:, rf_l), 2, 1);

    st_tau_res_Q = calc_stat(e_tau_Q);
    st_tau_res_L = calc_stat(e_tau_L);

    % 4 & 5. 遅延による位置変位 (Exact vs v*tau vs v*tau + 0.5*a*tau^2)
    ref_shift_idx_Q = max(1, idx_ss - lags(b_q));
    disp_exact_Q = vecnorm(pQ_nom_ref(:, idx_ss) - pQ_nom_ref(:, ref_shift_idx_Q), 2, 1);

    v_norm_Q = vecnorm(v_drone(:, idx_ss), 2, 1);
    disp_vtau_Q = v_norm_Q * tau_Q_eq;

    a_norm_Q = vecnorm(a_drone_ss, 2, 1);
    disp_va_Q = disp_vtau_Q + 0.5 * a_norm_Q * (tau_Q_eq^2);

    st_disp_exact_Q = calc_stat(disp_exact_Q);
    st_disp_vtau_Q  = calc_stat(disp_vtau_Q);
    st_disp_va_Q    = calc_stat(disp_va_Q);

    % 6 & 7. 速度残差 & 加速度残差
    e_vel_Q = vecnorm(v_drone(:, idx_ss) - v_ref(:, idx_ss), 2, 1);
    st_vel_Q = calc_stat(e_vel_Q);

    e_acc_Q = vecnorm(a_drone_ss - a_ref_ss, 2, 1);
    st_acc_Q = calc_stat(e_acc_Q);

    % 8. Jerk
    j_mag_Q = vecnorm(j_drone_ss, 2, 1);
    st_jerk_Q = calc_stat(j_mag_Q);

    % 9. 多段階 CPA 等速外挿予測残差
    H_list = [0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 6.0];
    pred_res = struct();
    for h_i = 1:length(H_list)
        H_val = H_list(h_i);
        H_stp = round(H_val / dt_nom);
        val_pred_idx = idx_ss(1 : (end - H_stp));
        if ~isempty(val_pred_idx)
            p_pred = p_drone(:, val_pred_idx) + v_drone(:, val_pred_idx) * H_val;
            p_true = p_drone(:, val_pred_idx + H_stp);
            e_pr = vecnorm(p_true - p_pred, 2, 1);
            pred_res(h_i).stat = calc_stat(e_pr);
        else
            pred_res(h_i).stat = calc_stat([0, 0]);
        end
    end

    % 10 & 11. 索揺動
    r_swing_series = vecnorm(p_load(:, idx_ss) - pL_nom(:, idx_ss), 2, 1);
    st_swing = calc_stat(r_swing_series);

    % 12 & 13. 動的理論揺動残差 Delta_r & 索傾斜角
    a_xy_mag = vecnorm(a_ref_ss(1:2, :), 2, 1);
    r_theory_series = L_val * (a_xy_mag ./ sqrt(a_xy_mag.^2 + g_acc^2));
    delta_r_series  = r_swing_series - r_theory_series;
    st_delta_r = calc_stat(delta_r_series);

    r_rel = p_load(:, idx_ss) - p_drone(:, idx_ss);
    cos_th = min(1.0, max(-1.0, -r_rel(3, :) ./ max(vecnorm(r_rel, 2, 1), 1e-6)));
    theta_deg_series = acos(cos_th) * (180 / pi);
    st_theta = calc_stat(theta_deg_series);

    % 【修正】全フィールドを明示的に struct 定義
    R_cur = struct();
    R_cur.FileName = string(cur_file);
    R_cur.tau_Q_eq = tau_Q_eq;
    R_cur.tau_L_eq = tau_L_eq;

    R_cur.pos_Q_nom_max = st_pos_Q_nom.max; R_cur.pos_Q_nom_p95 = st_pos_Q_nom.p95; R_cur.pos_Q_nom_p99 = st_pos_Q_nom.p99;
    R_cur.pos_Q_dyn_max = st_pos_Q_dyn.max; R_cur.pos_Q_dyn_p95 = st_pos_Q_dyn.p95; R_cur.pos_Q_dyn_p99 = st_pos_Q_dyn.p99;
    R_cur.pos_L_max     = st_pos_L.max;     R_cur.pos_L_p95     = st_pos_L.p95;     R_cur.pos_L_p99     = st_pos_L.p99;

    R_cur.lag1_Q_max = st_lag1_Q.max; R_cur.lag1_Q_p95 = st_lag1_Q.p95; R_cur.lag1_Q_p99 = st_lag1_Q.p99;
    R_cur.lag1_L_max = st_lag1_L.max; R_cur.lag1_L_p95 = st_lag1_L.p95; R_cur.lag1_L_p99 = st_lag1_L.p99;

    R_cur.res_tau_Q_max = st_tau_res_Q.max; R_cur.res_tau_Q_p95 = st_tau_res_Q.p95; R_cur.res_tau_Q_p99 = st_tau_res_Q.p99;
    R_cur.res_tau_L_max = st_tau_res_L.max; R_cur.res_tau_L_p95 = st_tau_res_L.p95; R_cur.res_tau_L_p99 = st_tau_res_L.p99;

    R_cur.disp_exact_max = st_disp_exact_Q.max; R_cur.disp_exact_p95 = st_disp_exact_Q.p95;
    R_cur.disp_vtau_max  = st_disp_vtau_Q.max;  R_cur.disp_vtau_p95  = st_disp_vtau_Q.p95;
    R_cur.disp_va_max    = st_disp_va_Q.max;    R_cur.disp_va_p95    = st_disp_va_Q.p95;

    R_cur.vel_res_Q_max = st_vel_Q.max;  R_cur.vel_res_Q_p95 = st_vel_Q.p95;
    R_cur.acc_res_Q_max = st_acc_Q.max;  R_cur.acc_res_Q_p95 = st_acc_Q.p95;
    R_cur.jerk_Q_max    = st_jerk_Q.max; R_cur.jerk_Q_p95    = st_jerk_Q.p95;

    R_cur.swing_max   = st_swing.max;   R_cur.swing_p95   = st_swing.p95;   R_cur.swing_p99   = st_swing.p99;
    R_cur.delta_r_max = st_delta_r.max; R_cur.delta_r_p95 = st_delta_r.p95; R_cur.delta_r_p99 = st_delta_r.p99;
    R_cur.theta_max   = st_theta.max;   R_cur.theta_p95   = st_theta.p95;

    R_cur.pred_025s_max = pred_res(4).stat.max; R_cur.pred_025s_p95 = pred_res(4).stat.p95;
    R_cur.pred_1s_max   = pred_res(6).stat.max; R_cur.pred_1s_p95   = pred_res(6).stat.p95;
    R_cur.pred_2s_max   = pred_res(7).stat.max; R_cur.pred_2s_p95   = pred_res(7).stat.p95;
    R_cur.pred_6s_max   = pred_res(9).stat.max; R_cur.pred_6s_p95   = pred_res(9).stat.p95;

    result_count = result_count + 1;
    results_cell{result_count} = R_cur; % セル配列へ安全代入
    fprintf("[%d/%d] 抽出完了: %s\n", f_idx, num_files, cur_file);
end

if result_count == 0, disp("解析対象データがありません。"); return; end

% 【修正】セル配列からテーブルへ一括安全変換
T_master = struct2table(cell2mat(results_cell));

%% 3. 目的別コンソール整形表示 (未結合・全観測量一覧)
fprintf("\n========================================================================================================================\n");
fprintf(" 【1. 機体位置残差 vs 1ステップ遅れ vs 等価遅延補正残差 (Max / P95 / P99)】\n");
fprintf("========================================================================================================================\n");
disp(T_master(:, {'FileName', 'tau_Q_eq', 'pos_Q_nom_max', 'pos_Q_nom_p95', 'pos_Q_dyn_max', 'pos_Q_dyn_p95', 'res_tau_Q_max', 'res_tau_Q_p95', 'lag1_Q_max', 'lag1_Q_p95'}));

fprintf("\n========================================================================================================================\n");
fprintf(" 【2. 荷物位置残差 vs 1ステップ遅れ vs 等価遅延補正残差 (Max / P95 / P99)】\n");
fprintf("========================================================================================================================\n");
disp(T_master(:, {'FileName', 'tau_L_eq', 'pos_L_max', 'pos_L_p95', 'res_tau_L_max', 'res_tau_L_p95', 'lag1_L_max', 'lag1_L_p95'}));

fprintf("\n========================================================================================================================\n");
fprintf(" 【3. 3種類の遅延変位モデル比較：Exact vs v*tau vs (v*tau + 0.5*a*tau^2)】\n");
fprintf("========================================================================================================================\n");
disp(T_master(:, {'FileName', 'tau_Q_eq', 'disp_exact_max', 'disp_exact_p95', 'disp_vtau_max', 'disp_vtau_p95', 'disp_va_max', 'disp_va_p95'}));

fprintf("\n========================================================================================================================\n");
fprintf(" 【4. 高次ダイナミクス残差：速度残差・加速度残差・Jerk (Max / P95)】\n");
fprintf("========================================================================================================================\n");
disp(T_master(:, {'FileName', 'vel_res_Q_max', 'vel_res_Q_p95', 'acc_res_Q_max', 'acc_res_Q_p95', 'jerk_Q_max', 'jerk_Q_p95'}));

fprintf("\n========================================================================================================================\n");
fprintf(" 【5. 索・荷物揺動変位 vs 動的理論残差 Delta_r vs 索傾斜角 (Max / P95 / P99)】\n");
fprintf("========================================================================================================================\n");
disp(T_master(:, {'FileName', 'swing_max', 'swing_p95', 'swing_p99', 'delta_r_max', 'delta_r_p95', 'delta_r_p99', 'theta_max', 'theta_p95'}));

fprintf("\n========================================================================================================================\n");
fprintf(" 【6. CPA等速直線外挿予測誤差 (Horizon H = 0.25s, 1s, 2s, 6s の Max / P95)】\n");
fprintf("========================================================================================================================\n");
disp(T_master(:, {'FileName', 'pred_025s_max', 'pred_025s_p95', 'pred_1s_max', 'pred_1s_p95', 'pred_2s_max', 'pred_2s_p95', 'pred_6s_max', 'pred_6s_p95'}));