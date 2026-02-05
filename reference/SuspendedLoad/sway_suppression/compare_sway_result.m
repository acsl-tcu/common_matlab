%ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー比較用ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー

%% run_eval_sway_compare.m
clear; clc;

% ---------- 入力 .mat ----------
file_no = "off_1_change_Log(04-Feb-2026_17_07_54).mat";   % SWAYなし
file_on = "on_1_change_Log(04-Feb-2026_17_04_26).mat";   % SWAYあり

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
opt.t_range = [];          % フライト全体比較（部分評価したいときだけ [t0 t1]）

% SWAYのS定義と合わせる（切替包絡の表示用）
opt.sr = 0.4;
opt.env_win_sec = 0.5;

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
% % SWAYのS定義と合わせる（切替包絡の表示用）
% opt.sr = 0.4;
% opt.env_win_sec = 0.5;
% 
% % ---------- 評価 ----------
% OUT = eval_sway_compare_from_series(S_on, [], theta_max, opt);
% fprintf("\n[CHECK] duration=%.2f s\n", OUT.A0.t(end));