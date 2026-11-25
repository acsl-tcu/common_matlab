function [Pid, notch, Pid_notched, data, fit] = identify_sway_tf(matfile, opt)
% identify_sway_tf  励振データから「τy→θ」の伝達関数を同定する.
%
%   [Pid, notch, Pid_notched, data, fit] = identify_sway_tf("sysid_data.mat")
%
%   System Identification Toolbox を使用（tfest のみ）.
%   同定結果を解析モデル susp_load_linear_model.m と Bode で重ね描きして検証し,
%   自分で指定したパラメータ（wn, zeta_z, zeta_p）でノッチフィルタを設計する
%   （トルク指令 \tau_y に適用する想定）.
%
%   入力:
%     matfile : run_sysid_experiment.m が保存した .mat（既定 "sysid_data.mat"）
%               フィールド t, tau_y, theta, dt を持つこと.
%               （実機データも同じ形式に整えれば使用可能）
%     opt     : name-value
%               "np"      : 極の数（既定 4）
%               "nz"      : 零点の数（既定 1）
%               "val_frac": 検証に回すデータ割合（既定 0.3）
%               "wn"      : ノッチ中心周波数 [rad/s]（既定 10*2*pi、自分で設定）
%               "zeta_z"  : ノッチのゼロ側減衰比（既定 0.05, 小さいほど深い）
%               "zeta_p"  : ノッチのポール側減衰比（既定 0.5, 大きいほど広い）
%
%   出力:
%     Pid         : 同定した連続時間 tf（tfest モデル、\tau_y \rightarrow \theta）
%     notch       : 指定パラメータで設計したノッチフィルタ（連続時間 tf、\tau_y に適用する想定）
%     Pid_notched : \tau_y にノッチを通した後の \tau_y \rightarrow \theta 特性（= Pid*notch）
%     data        : 使用した iddata
%     fit         : tfest の当てはまり率 [%]

arguments
  matfile = "sysid_data.mat"
  opt.np (1,1) double = 4
  opt.nz (1,1) double = 1
  opt.val_frac (1,1) double = 0.3
  opt.wn (1,1) double = 1.85
  opt.zeta_z (1,1) double = 0.005
  opt.zeta_p (1,1) double = 2
end
close all
S = load(matfile);
t = S.t(:);  u = S.tau_y(:);  y = S.theta(:);  dt = S.dt; %#ok<NASGU>

% --- 前処理: トレンド除去（振り子の平均ドリフト成分を除く） ---
z = iddata(y, u, dt);
z = detrend(z, 0);            % 直流除去
% z = detrend(z, 1);         % 直線トレンドも除きたい場合
data = z;                    % 出力引数 data（使用した iddata）

% --- 推定用 / 検証用に分割 ---
n  = size(z,1);
ne = round((1-opt.val_frac)*n);
ze = z(1:ne);                 % estimation
zv = z(ne+1:end);             % validation

% --- 同定: 伝達関数 tfest ---
Opt = tfestOptions("Display","off","InitMethod","all");
sys_tf = tfest(ze, opt.np, opt.nz, Opt);

% --- 検証（validation データでの当てはまり率） ---
fit_tf = get_fit(zv, sys_tf);
fit = struct("tfest", fit_tf);
fprintf("Fit (validation):  tfest = %.1f %%\n", fit_tf);

Pid = tf(sys_tf);
fprintf("採用モデル: tfest\n");
Pid

% --- 解析モデルと Bode 比較 ---
Pana = susp_load_linear_model([]);   % 既定パラメータの解析モデル

figure("Name","validation: measured vs model");
compare(zv, sys_tf); grid on;

figure("Name","Bode: identified vs analytical");
w = logspace(-1, 2.3, 800);
bode(Pana, "k--", sys_tf, "b", w); grid on;
legend("解析モデル (物理)", "tfest", "Location","southwest");
title("トルク \tau_y \rightarrow 揺れ角 \theta");

% --- 同定モデルから共振の抽出（参考表示のみ：ノッチのパラメータ設定の目安に使う） ---
pc = pole(Pid);  pc = pc(imag(pc) > 1e-6);            % 振動極（上側）
if ~isempty(pc)
  wn_c   = abs(pc);
  zeta_c = -real(pc)./abs(pc);
  cand   = zeta_c > 0 & zeta_c < 0.9;                 % 揺れらしい軽減衰モード
  if ~any(cand), cand = true(size(zeta_c)); end
  [~, idx] = min(zeta_c(cand));
  ic = find(cand); ic = ic(idx);
  fprintf("同定された揺れ共振:  wn = %.3f rad/s (%.3f Hz),  zeta = %.3f  （ノッチ設定の参考）\n", ...
          wn_c(ic), wn_c(ic)/2/pi, zeta_c(ic));
end

% --- ノッチフィルタ設計（パラメータは opt で自分で指定：トルク指令 \tau_y に適用する想定） ---
notch = tf([1 2*opt.zeta_z*opt.wn opt.wn^2], [1 2*opt.zeta_p*opt.wn opt.wn^2]);

fprintf("ノッチフィルタ設計:  wn = %.3f rad/s (%.3f Hz),  zeta_z = %.3f,  zeta_p = %.3f\n", ...
        opt.wn, opt.wn/2/pi, opt.zeta_z, opt.zeta_p);

% --- ノッチフィルタ自体の周波数特性をプロット ---
figure("Name","Bode: notch filter (torque command)");
bode(notch, w); grid on;
title(sprintf("トルク指令用ノッチフィルタ  (wn = %.2f rad/s = %.2f Hz)", opt.wn, opt.wn/2/pi));

% --- ノッチを適用した結果（\tau_y に notch を通してから Pid に入れる = Pid*notch）をプロット ---
Pid_notched = Pid * notch;

figure("Name","Bode: Pid vs Pid with notch on torque input");
bode(Pid, "b", Pid_notched, "r", w); grid on;
legend("Pid（ノッチ無し）", "Pid（\tau_y にノッチ適用後）", "Location","southwest");
title("ノッチフィルタ適用前後の \tau_y \rightarrow \theta 特性");

save("identified_model.mat", "Pid", "notch", "Pid_notched", "sys_tf", "fit");
end

% ---- 補助: compare の当てはまり率をスカラーで取り出す ----
function f = get_fit(z, sys)
[~, f] = compare(z, sys);
if iscell(f), f = f{1}; end        % モデル複数なら cell
f = f(1);                          % 出力チャネルの先頭
end