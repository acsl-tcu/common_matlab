% 案2：ゼロダイナミクス励起最小化 APF-CBF ジェネレータ
% 論文①および一般ロボティクスの手法。参照軌道側で滑らかに避ける
clear; clc;

disp('Generating APF-CBF equations...');

syms px py pz real
syms obs_px obs_py obs_pz obs_r real
syms k_rep d_inf real

dist_sq = (px - obs_px)^2 + (py - obs_py)^2 + (pz - obs_pz)^2;
dist = sqrt(dist_sq);

% 斥力関数の定義 (距離が d_inf 以下になると発動する滑らかな斥力)
F_rep_x = k_rep * (1/(dist - obs_r) - 1/d_inf) * (1/(dist - obs_r)^2) * ((px - obs_px)/dist);
F_rep_y = k_rep * (1/(dist - obs_r) - 1/d_inf) * (1/(dist - obs_r)^2) * ((py - obs_py)/dist);
F_rep_z = k_rep * (1/(dist - obs_r) - 1/d_inf) * (1/(dist - obs_r)^2) * ((pz - obs_pz)/dist);

matlabFunction(F_rep_x, F_rep_y, F_rep_z, 'File', 'autogen_APF_Force', 'Vars', [px, py, pz, obs_px, obs_py, obs_pz, obs_r, k_rep, d_inf]);
disp('APF Force function generated: autogen_APF_Force.m');
