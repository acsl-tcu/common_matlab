% 案1：スイング減衰型 Backup-CBF ジェネレータ
% 論文②(FastBridge)の「安全に止まれるか？」を懸架荷物系に拡張
clear; clc;

disp('Generating Backup-CBF equations...');

% --- 1. 状態変数とパラメータの定義 ---
syms px py pz vx vy vz real
syms ax ay real % 荷物の揺れ角
syms dax day real % 揺れ角速度
syms ux uy uz real % コントロール入力 (仮想加速度)

% --- 2. バックアップ制御則 (Brake-to-Hover) の定義 ---
% 緊急時に「ドローンを停止させつつ、荷物の揺れも止める」仮想の制動入力を定義
% 例として、単純なPD制御による制動加速度を仮定
K_v = 2.0; K_a = 5.0; K_da = 2.0;

% ブレーキ時の仮想入力
u_brake_x = -K_v * vx + K_a * ax + K_da * dax;
u_brake_y = -K_v * vy + K_a * ay + K_da * day;
u_brake_z = -K_v * vz;

% 制動にかかる時間と停止位置（近似的なForward Reachable Set）
T_brake = 1.5; % 停止までにかかる推定時間
p_stop_x = px + vx * T_brake + 0.5 * u_brake_x * T_brake^2;
p_stop_y = py + vy * T_brake + 0.5 * u_brake_y * T_brake^2;
p_stop_z = pz + vz * T_brake + 0.5 * u_brake_z * T_brake^2;

% --- 3. Backup CBF の定義 ---
syms obs_px obs_py obs_pz obs_r real
% 停止位置が障害物と衝突していないか（Terminal CBF）
h_backup = (p_stop_x - obs_px)^2 + (p_stop_y - obs_py)^2 + (p_stop_z - obs_pz)^2 - obs_r^2;

% matlabFunction(h_backup, 'File', 'autogen_CBF_Backup', 'Vars', [px, py, pz, vx, vy, vz, ax, ay, dax, day, obs_px, obs_py, obs_pz, obs_r]);
disp('Backup-CBF setup complete.');
