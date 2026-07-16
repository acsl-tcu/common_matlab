clear;
clc;

%SOLVE_KOOPMAN_ALTERNATING
% 交互最適化によって安定性制約付きKoopman作用素 U=[A B] を求める
%
% 入力
%   Psi        : p × q
%                [観測量; 入力]を並べたデータ行列
%
%   Theta_plus : p_theta × q
%                次時刻の観測量データ行列
%
%   rho_bar    : 安定性の上限
%                省略時 0.99
%
%   max_iter   : 最大反復回数
%                省略時 30
%
%   tolerance  : 収束判定値
%                省略時 1e-5
%
% 出力
%   U_val      : Koopman作用素 U=[A B]
%   A_val      : 状態・観測量部分
%   B_val      : 入力部分
%   P_val      : Lyapunov行列
%   result     : 計算結果を格納した構造体


rho_bar = 0.99;
max_iter = 30;
tolerance = 1e-5;

[U_val, A_val, B_val, P_val, result] = ...
    solve_koopman_alternating( ...
        Psi, Theta_plus, rho_bar, max_iter, tolerance);