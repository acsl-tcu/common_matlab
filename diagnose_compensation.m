%% diagnose_compensation.m
% 補償が弱い原因を、HL_MPC_EDMD_MEC が記録する診断フィールドから分析する。
% compensated 採集の1ファイルを読み込んで実行する。
%
% 使い方: FPATH を compensated のログに変えて実行。

clear; clc;
%% ===== 設定 =====
FPATH = 'C:\Users\student\Documents\GitHub\common_matlab\Data\Sim_data\HLMPC_oneclick_20260614_152032_com_903_Log(14-Jun-2026_15_24_24).mat';

raw = load(FPATH);
fn = fieldnames(raw); L = raw.(fn{1});
cr = L.Data.agent.controller.result;   % controller result列
K = numel(cr);

% 各ステップの診断量を集める
du_pred   = nan(4,K);   % 補償の生の予測
du_cmd    = nan(4,K);   % クリップ後（du_max適用後）
du_actual = nan(4,K);   % 実際にplantへ反映された差
sat_loss  = nan(4,K);   % 飽和で失われた量
u_nom     = nan(4,K);
mode_log  = nan(1,K);
loaded_log= nan(1,K);

for k=1:K
    c = cr{1,k};
    if isempty(c); continue; end
    try; du_pred(:,k)   = c.delta_u_edmd_pred(:); catch; end
    try; du_cmd(:,k)    = c.delta_u_edmd_cmd(:); catch; end
    try; du_actual(:,k) = c.delta_u_edmd_actual(:); catch; end
    try; sat_loss(:,k)  = c.delta_u_edmd_saturation_loss(:); catch; end
    try; u_nom(:,k)     = c.u_nom(:); catch; end
    try; mode_log(k)    = c.residual_mode; catch; end
    try; loaded_log(k)  = c.residual_loaded; catch; end
end

% flight区間のみ（u_nomが有効なところ）
valid = ~any(isnan(du_pred),1);
idx = find(valid);
if isempty(idx); error('診断データが空。compensatedのログか確認。'); end

fprintf('========== 補償診断 ==========\n');
fprintf('総ステップ=%d  有効=%d\n', K, numel(idx));
fprintf('residual.mode=%g  loaded=%g  (mode=0かloaded=0なら補償OFF)\n', ...
    mode_log(idx(1)), loaded_log(idx(1)));
fprintf('-----------------------------------\n');

labels = {'thrust','roll','pitch','yaw'};
for ch=1:4
    p = du_pred(ch,idx);   p = p(~isnan(p));
    cmd = du_cmd(ch,idx);  cmd = cmd(~isnan(cmd));
    act = du_actual(ch,idx); act = act(~isnan(act));
    sl = sat_loss(ch,idx); sl = sl(~isnan(sl));
    un = u_nom(ch,idx); un = un(~isnan(un));
    fprintf('[%s]\n', labels{ch});
    fprintf('  du_pred   平均|.|=%.5f 最大|.|=%.5f\n', mean(abs(p)), max(abs(p)));
    fprintf('  du_actual 平均|.|=%.5f 最大|.|=%.5f\n', mean(abs(act)), max(abs(act)));
    fprintf('  飽和損失  平均|.|=%.5f (大きいなら du_max が小さすぎ)\n', mean(abs(sl)));
    if ~isempty(un) && mean(abs(un))>1e-9
        fprintf('  補償/u_nom 比 = %.2f%% (小さいなら補償が弱い)\n', 100*mean(abs(act))/mean(abs(un)));
    end
end

%% ===== 総合判定 =====
fprintf('\n========== 判定 ==========\n');
pred_mag = mean(abs(du_pred(:,idx)),'all','omitnan');
act_mag  = mean(abs(du_actual(:,idx)),'all','omitnan');
sat_mag  = mean(abs(sat_loss(:,idx)),'all','omitnan');

if mode_log(idx(1))==0 || loaded_log(idx(1))==0
    fprintf('✗ 補償がOFF (mode=0またはloaded=0)。モデル未ロード。\n');
    fprintf('  → edmd_model_file パス・results_edmd構造を確認。\n');
elseif pred_mag < 1e-4
    fprintf('✗ du_pred自体がほぼ0。残差モデルが補償方向を出していない。\n');
    fprintf('  → B_err の該当列ノルムが小さい(質量/慣性方向が辨識できていない)\n');
    fprintf('  → 学習データの励起不足、または rhs(目標残差)が小さい。\n');
elseif sat_mag > 0.3*pred_mag
    fprintf('△ 飽和損失が大きい。du_max が小さく補償が切られている。\n');
    fprintf('  → Controller_HL_MPC_EDMD_MEC の du_max を上げる。\n');
elseif act_mag < 1e-3
    fprintf('△ 補償は計算されているが実反映が小さい。alpha/beta を確認。\n');
else
    fprintf('○ 補償は出ている(平均|du_actual|=%.4f)。\n', act_mag);
    fprintf('  効果が弱いなら: alpha上げ / du_max上げ / B_err辨識改善。\n');
end
fprintf('============================\n');