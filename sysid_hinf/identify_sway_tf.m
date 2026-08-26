function [Pid, data, fit] = identify_sway_tf(matfile, opt)
% identify_sway_tf  励振データから「τy→θ」の伝達関数を同定する.
%
%   [Pid, data, fit] = identify_sway_tf("sysid_data.mat")
%
%   System Identification Toolbox を使用（tfest / ssest）.
%   同定結果を解析モデル susp_load_linear_model.m と Bode で重ね描きして検証する.
%
%   入力:
%     matfile : run_sysid_experiment.m が保存した .mat（既定 "sysid_data.mat"）
%               フィールド t, tau_y, theta, pL, dt を持つこと.
%               （実機データも同じ形式に整えれば使用可能）
%     opt     : name-value
%               "np" : 極の数（既定 4）
%               "nz" : 零点の数（既定 1）
%               "ssorder" : ssest の次数（既定 4）
%               "val_frac" : 検証に回すデータ割合（既定 0.3）
%
%   出力:
%     Pid  : 同定した連続時間 tf（最良モデル）
%     data : 使用した iddata
%     fit  : 各手法の当てはまり率 [%] 等

arguments
  matfile = "sysid_data.mat"
  opt.np (1,1) double = 4
  opt.nppl (1,1) double = 6
  opt.nz (1,1) double = 1
  opt.ssorder (1,1) double = 4
  opt.val_frac (1,1) double = 0.3
end

S = load(matfile);
t = S.t(:);  u = S.tau_y(:);  y = S.theta(:);  dt = S.dt; pl = S.pL(:,1);

% --- 前処理: トレンド除去（振り子の平均ドリフト成分を除く） ---
z = iddata(y, u, dt);
z = detrend(z, 0);            % 直流除去
% z = detrend(z, 1);         % 直線トレンドも除きたい場合
data = z;                    % 出力引数 data（使用した iddata）

pl=iddata(pl,u,dt);
pl=detrend(pl,0);
datapl=pl;

% --- 推定用 / 検証用に分割 ---
n  = size(z,1);
ne = round((1-opt.val_frac)*n);
ze = z(1:ne);                 % estimation
zv = z(ne+1:end);             % validation

npl  = size(pl,1);
nepl = round((1-opt.val_frac)*npl);
ple = pl(1:nepl);               % estimation
plv = pl(nepl+1:end);           % validation

% --- 同定 1: 伝達関数 tfest ---
Opt = tfestOptions("Display","off","InitMethod","all");
sys_tf = tfest(ze, opt.np, opt.nz, Opt);

sys_tf_pl = tfest(ple, opt.nppl, opt.nz, Opt);

% --- 同定 2: 状態空間 ssest ---
sys_ss = ssest(ze, opt.ssorder, ssestOptions("Display","off"));
sys_ss_pl=ssest(ple, opt.ssorder, ssestOptions("Display","off"));

% --- 検証（validation データでの当てはまり率） ---
%  compare の 2番目の出力(fit)は、モデル1個だと数値配列、複数だと cell で返る.
%  どちらでもスカラーを取り出せるように get_fit で吸収する.
fit_tf = get_fit(zv, sys_tf);
fit_ss = get_fit(zv, sys_ss);

fit_tf_pl = get_fit(plv, sys_tf_pl);
fit_ss_pl = get_fit(plv, sys_ss_pl);

fit = struct("tfest", fit_tf, "ssest", fit_ss);
fprintf("Fit (validation):  tfest = %.1f %%,  ssest = %.1f %%\n", fit_tf, fit_ss);

fitpl= struct("tfest", fit_tf_pl, "ssest", fit_ss_pl);
fprintf("Fit (validation, pL):  tfest = %.1f %%,  ssest = %.1f %%\n", fit_tf_pl, fit_ss_pl);

% --- 最良モデルを採用（連続時間 tf に変換） ---
if fit_tf >= fit_ss
  Pid = tf(sys_tf);  best = "tfest";
else
  Pid = tf(ss(sys_ss)); best = "ssest";
end
fprintf("採用モデル: %s\n", best);
Pid

% --- 解析モデルと Bode 比較 ---
Pana = susp_load_linear_model([]);   % 既定パラメータの解析モデル

figure("Name","validation: measured vs models");
compare(zv, sys_tf, sys_ss); grid on;

figure("Name","Bode: identified vs analytical");
w = logspace(-1, 2.3, 800);
bode(Pana, "k--", sys_tf, "b", sys_ss, "r", w); grid on;
legend("解析モデル (物理)", "tfest", "ssest", "Location","southwest");
title("トルク \tau_y \rightarrow 揺れ角 \theta");

% --- 同定モデルから共振の抽出（極から直接計算：damp と pole の並び順に依存しない） ---
pc = pole(Pid);  pc = pc(imag(pc) > 1e-6);            % 振動極（上側）
if ~isempty(pc)
  wn_c   = abs(pc);
  zeta_c = -real(pc)./abs(pc);
  cand   = zeta_c > 0 & zeta_c < 0.9;                 % 揺れらしい軽減衰モード
  if ~any(cand), cand = true(size(zeta_c)); end
  [~, idx] = min(zeta_c(cand));
  ic = find(cand); ic = ic(idx);
  fprintf("同定された揺れ共振:  wn = %.3f rad/s (%.3f Hz),  zeta = %.3f\n", ...
          wn_c(ic), wn_c(ic)/2/pi, zeta_c(ic));
end

% ==================== pL に対しても同じ処理 ====================

% --- pL: 最良モデルを採用（連続時間 tf に変換） ---
if fit_tf_pl >= fit_ss_pl
  Pidpl = tf(sys_tf_pl);  bestpl = "tfest";
else
  Pidpl = tf(ss(sys_ss_pl)); bestpl = "ssest";
end
fprintf("採用モデル (pL): %s\n", bestpl);
Pidpl

% --- pL: 検証データでの実測 vs モデル比較 ---
figure("Name","validation (pL): measured vs models");
compare(plv, sys_tf_pl, sys_ss_pl); grid on;

% --- pL: Bode（tfest vs ssest。解析モデルは theta 用のため重ねていない） ---
figure("Name","Bode (pL): tfest vs ssest");
bode(sys_tf_pl, "b", sys_ss_pl, "r", w); grid on;
legend("tfest", "ssest", "Location","southwest");
title("トルク \tau_y \rightarrow pL");

% --- pL: 同定モデルから共振の抽出 ---
pc_pl = pole(Pidpl);  pc_pl = pc_pl(imag(pc_pl) > 1e-6);      % 振動極（上側）
if ~isempty(pc_pl)
  wn_c_pl   = abs(pc_pl);
  zeta_c_pl = -real(pc_pl)./abs(pc_pl);
  cand_pl   = zeta_c_pl > 0 & zeta_c_pl < 0.9;                % 軽減衰モード
  if ~any(cand_pl), cand_pl = true(size(zeta_c_pl)); end
  [~, idx_pl] = min(zeta_c_pl(cand_pl));
  ic_pl = find(cand_pl); ic_pl = ic_pl(idx_pl);
  fprintf("同定された共振 (pL):  wn = %.3f rad/s (%.3f Hz),  zeta = %.3f\n", ...
          wn_c_pl(ic_pl), wn_c_pl(ic_pl)/2/pi, zeta_c_pl(ic_pl));
end

save("identified_model.mat", "Pid", "sys_tf", "sys_ss", "fit", ...
     "Pidpl", "sys_tf_pl", "sys_ss_pl", "fitpl", "datapl");
end

% ---- 補助: compare の当てはまり率をスカラーで取り出す ----
function f = get_fit(z, sys)
[~, f] = compare(z, sys);
if iscell(f), f = f{1}; end        % モデル複数なら cell
f = f(1);                          % 出力チャネルの先頭
end