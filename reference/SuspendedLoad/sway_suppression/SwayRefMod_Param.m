function p = SwayRefMod_Param(dt)
% SWAY_REF_INTERRUPT 用パラメータ（一次遅れ＋CBFゲート）--------------------

% dt（必須）
p.dt = dt;

% === 角度＋相対速度で判定する設定 ===
p.use_theta_vr_switch = true;
% --- 角度しきい値（まずは固定値でOK。後でログから調整） ---
p.theta_on  = deg2rad(5); %５度  % 8〜10° 推奨8
p.theta_off = deg2rad(3); %3度 % 4〜6° 推奨（theta_onの0.5〜0.7倍）5
% --- 相対速度しきい値（ログから決めるのが理想。まずは暫定値） ---
p.vr_on  = 0.20;            % [m/s] まず0.2〜0.35あたり  0.25
p.vr_off = 0.1;            % 0.6〜0.8 * vr_on  0.15

% ---- CBFゲート（水平距離で角度制約を代理） ----
p.theta_max = deg2rad(15);%許容角　15度
p.h_gate    = 0.02;     % 危険域判定（m^2）危険域判定のマージン（**hは[m^2]**なのでここ重要）
p.print_interval = 0.5;   % [s] 割り込み中の表示間隔

% ---- danger gate (angle domain) ----（ケーブルL使わない版）
p.theta_gate = deg2rad(2.0);   % theta_max に近づいたら danger 扱い

% 危険域での補正強化
p.gain_boost = 1.5;             % alpha=2.0 など  危険域のゲインスケジューリング倍率（※CBFではない）
p.force_on_when_danger = true;  % dangerなら強制ON（安全側）  安全側に倒す論理
p.hold_on_when_danger  = true;  % dangerならOFFに戻さない（安全側）

% ---- 一次遅れの時定数 ----補正量 $\mathbf{c}$ の平滑化（ON/OFFで時定数を変える）
% ON時は速い（小さいtau）、OFF時はゆっくり（大きいtau）
p.tau_on  = 0.1;   % 0.1〜0.3 s 推奨 0.3
p.tau_off = 0.50;   % 0.3〜1.0 s 程度でOK

% ---- 速度目標への適用----
p.apply_to_vref = true; % 基本falseでOK（まず位置だけスムーズに）
p.kv_vref = 0.6; %0.6 %0.3~1.0 % apply_to_vref=trueにするなら小さく（例0.2）0.8
p.dv_max  = 0.4; %0.6 %0.3~0.8 0.25
p.dv_lpf_tau= 0.05;  % 0.03〜0.08
p.vr_track_tau = 0.4;        % 追従(低周波)分離用 [s] 0.3〜0.8
p.dv_delay_sec = 0.4;     % [s] 追従開始直後は dv=0（0.2〜1.0で調整）
p.dv_ramp_sec  = 0.6;     % [s] その後ゆっくり効かせる（0.3〜1.0）
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p.c_max = 0.3;   % [m] 補正の最大値（まずは10cm）目標の「追従不能」を避ける安全上限（重要）0.15
p.softstart_sec = 0.1;  % ON後0.3秒はゆっくり  ON直後だけゆっくり入れる（位相遅れ対策）0.3
p.tau_on_soft   = 0.2;  % ON直後の時定数（大きめ）0.45
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p.min_on_time = 0.5;   % [s] 一度ONになったら最低0.5秒は保持  ON保持時間（チャタリング抑制）
% ---- real flight robustness ----実機向けのLPF時定数
p.vr_lpf_tau     = 0.02;  % [s] vrxy LPF (軽め) 0.03〜0.08
p.S_smooth_tau   = 0.05;  % [s] S平滑 0.1〜0.3 0.15
%-------------------------------------------------------------------------

end
