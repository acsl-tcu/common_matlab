function p = SwayRefMod_Param(dt)
% SWAY_REF_INTERRUPT 用パラメータ（一次遅れ＋CBFゲート）

% dt（必須）
p.dt = dt;

% ---- 揺れ判定（ヒステリシス） ----
p.S_on  = 0.25;
p.S_off = 0.15;

% 揺れ指標：S = ||v_xy|| + sr*||r_xy||
p.sr = 0.3;

% ---- 目標修正（水平位置） ----
% c* = alpha*(kv*Tv*v_xy + kr*r_xy)
p.Tv = 0.5;     %0.2~0.5sくらい　速度項を時間スケールで位置へ変換
p.kv = 0.6;     % 相対速度（主）
p.kr = 0.05;    % 相対位置（補助、小さめ推奨）

% ---- CBFゲート（水平距離で角度制約を代理） ----
p.theta_max = deg2rad(15);
p.h_gate    = 0.02;     % 危険域判定（m^2）
p.print_interval = 0.5;   % [s] 割り込み中の表示間隔


% 危険域での補正強化
p.gain_boost = 2.0;             % alpha=2.0 など
p.force_on_when_danger = true;  % dangerなら強制ON（安全側）
p.hold_on_when_danger  = false;  % dangerならOFFに戻さない（安全側）

% ---- 一次遅れの時定数 ----
% ON時は速い（小さいtau）、OFF時はゆっくり（大きいtau）
p.tau_on  = 0.15;   % 0.1〜0.3 s 推奨
p.tau_off = 0.40;   % 0.3〜1.0 s 程度でOK

% ---- 速度目標への適用（任意）----
p.apply_to_vref = false; % 基本falseでOK（まず位置だけスムーズに）
p.kv_vref = 0.0;         % apply_to_vref=trueにするなら小さく（例0.2）
end
