%% main_sysid_hinf.m
%  単機牽引（吊り下げ負荷）ドローンの「トルク→揺れ角」
%  システム同定 → ボード線図 → H∞（混合感度）による揺れ抑制  の一連の流れ.
%
%  common_matlab (acsl-tcu) のルートで実行する単独スクリプト.
%  （通常運用の入口は mainGUI だが、これは同定/設計用の解析スクリプトで、
%    experiment/test_*.m と同様に mainGUI とは独立に直接実行する。
%    下の addpath でリポジトリのクラス群にパスを通す。）
%  必要 Toolbox: Control System, System Identification, Robust Control
%
%  STEP0 だけ（解析モデルで設計まで通す）でも一通り体験できる.
%  実データ/シミュ同定を使う場合は STEP1→STEP2 を有効化する.

clear; clc;
addpath(genpath(pwd));   % common_matlab 本体に加えて sysid_hinf も見えるように

% 注意: 既定（STEP0+STEP3）は「解析線形モデル」だけで完結し、common_matlab の
%       非線形モデル（with_load_model_euler_for_HL）は使いません。リポジトリの
%       ダイナミクスで同定したい場合は STEP1/STEP2 のコメントを外してください。

%% ---- STEP0: 解析モデル（物理）でボード線図 ----
[Pana, info] = susp_load_linear_model([]);         % 既定 DIATONE パラメータ
fprintf("[使用モデル] 解析線形モデル（common_matlab 非依存）: wn=%.3f rad/s (%.3f Hz)\n", ...
        info.wn, info.fn);
figure("Name","analytical Bode"); bode(Pana); grid on;
title("解析モデル  \tau_y \rightarrow \theta");

%% ---- STEP1: 励振シミュレーションで同定データ生成（任意） ----
% run_sysid_experiment;         % → sysid_data.mat / .csv を作成

%% ---- STEP2: データから同定（STEP1 実行後 or 実機データがある場合） ----
usedModel = Pana;               % 既定は解析モデルで設計
% if isfile("sysid_data.mat")
%   [Pid, ~, fit] = identify_sway_tf("sysid_data.mat");
%   usedModel = Pid;            % 同定モデルで設計に切替
% end

%% ---- STEP3: H∞（混合感度）で揺れ抑制コントローラ設計 ----
if isequal(usedModel, Pana)
  disp("[設計対象] 解析線形モデル（同定していません）");
else
  disp("[設計対象] 同定モデル Pid（common_matlab の非線形モデル/実データ由来）");
end
[K, CL, gam, dinfo] = design_hinf_sway(usedModel, "x0deg", 10);

fprintf("\n==== 完了 ====\n");
fprintf("gamma = %.3f, GM = %.1f dB, PM = %.1f deg\n", ...
        gam, dinfo.GainMargin_dB, dinfo.PhaseMargin_deg);
fprintf("コントローラ K の次数: %d\n", order(K));
disp("K =");  tf(K)
