%ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー比較用ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー

%% run_eval_sway_compare.m
clear -except app; clc;

% ---------- 入力 .mat ----------
file_no = "qqq_Log(17-Feb-2026_16_42_19).mat";   % SWAYなし
file_on = "rrr_Log(17-Feb-2026_15_43_04).mat";   % SWAYあり
% ---------- 許容角 ----------
theta_max = deg2rad(15);

% ---------- series 抽出（phase->cha 付き） ----------
S_no = extract_series_from_logmat_cellresult_with_phase(file_no);
S_on = extract_series_from_logmat_cellresult_with_phase(file_on);

% flight があるか確認
disp("unique(S_no.cha) = "); disp(unique(S_no.cha).');
disp("unique(S_on.cha) = "); disp(unique(S_on.cha).');
fprintf("N_flight(no) = %d\n", sum(S_no.cha=='f'));
fprintf("N_flight(on) = %d\n", sum(S_on.cha=='f'));

% ---------- option ----------
opt = struct();
opt.use_flight_phase_only = true;
opt.flight_cha = 'f';      % flight フェーズ文字
opt.align = true;          % flight開始を t=0 に揃える
opt.zero_time_at_window_start = true;
% ---- 角度＋速度のON/OFF閾値（SWAY側と合わせる）----
opt.use_theta_vr_switch = true;

% 例：SWAY側のparamに合わせる（あなたのparamに合わせて変更）
opt.theta_on  = deg2rad(5);   % 例: deg2rad(6)
opt.theta_off = deg2rad(3);   % 例: deg2rad(4)

opt.vr_on  = 0.2;               % 例: 0.25 [m/s]
opt.vr_off = 0.1;               % 例: 0.15 [m/s]
% --- 区間を完全に手で指定（aligned 後の相対時間 [s]） ---
opt.t_range_no = [0 30];   % OFF 側だけ
opt.t_range_on = [0 30];   % ON 側だけ

% 互換のため opt.t_range は空にしておく（両方同じ窓を当てないため）
opt.t_range = [];

% ---------- 評価 ----------
OUT = eval_sway_compare_from_series(S_no, S_on, theta_max, opt);

% 追加：区間長チェック（論文で突っ込まれがち）
fprintf("\n[CHECK] duration_off=%.2f s, duration_on=%.2f s\n", OUT.A0.t(end), OUT.A1.t(end));




%ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー１つだけ用ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
%% run_eval_sway_compare.m
% clear; clc;
% 
% % ---------- 入力 .mat ----------
% file_on = "log_on.mat";   % SWAYあり
% 
% % ---------- 許容角 ----------
% theta_max = deg2rad(15);
% 
% % ---------- series 抽出（phase->cha 付き） ----------
% S_on = extract_series_from_logmat_cellresult_with_phase(file_on);
% 
% % flight があるか確認
% disp("unique(S_on.cha) = "); disp(unique(S_on.cha).');
% fprintf("N_flight(on) = %d\n", sum(S_on.cha=='f'));
% 
% % ---------- option ----------
% opt = struct();
% opt.use_flight_phase_only = true;
% opt.flight_cha = 'f';      % flight フェーズ文字
% opt.align = true;          % flight開始を t=0 に揃える
% opt.t_range = [];          % フライト全体比較（部分評価したいときだけ [t0 t1]）
% 
% % 例：SWAY側のparamに合わせる（あなたのparamに合わせて変更）
% opt.theta_on  = deg2rad(7);   % 例: deg2rad(6)
% opt.theta_off = deg2rad(3);   % 例: deg2rad(4)
% 
% opt.vr_on  = 0.2;               % 例: 0.25 [m/s]
% opt.vr_off = 0.1;               % 例: 0.15 [m/s]
% 
% % ---------- 評価 ----------
% OUT = eval_sway_compare_from_series(S_on, [], theta_max, opt);
% fprintf("\n[CHECK] duration=%.2f s\n", OUT.A0.t(end));