clear;
clc;

load("LYKL_1124.mat");
% load("koopman_common_z_.mat")


A = est.A;
B = est.B;
C = est.C;

n = size(A, 1);

%% 可制御性・可観測性行列
Mc = ctrb(A, B);
Mo = obsv(A, C);

%% ランク
rank_Mc = rank(Mc);
rank_Mo = rank(Mo);

%% 可制御部分空間の正規直交基底
ImMc_orth = orth(Mc);

%% 判定
is_controllable = (rank_Mc == n);
is_observable   = (rank_Mo == n);

if is_controllable && is_observable

    disp('このシステムは「可制御・可観測」です。');

elseif is_controllable && ~is_observable

    disp('このシステムは「可制御・不可観測」です。');

elseif ~is_controllable && is_observable

    disp('このシステムは「不可制御・可観測」です。');

else

    disp('このシステムは「不可制御・不可観測」です。');

end

%% 次元も表示
fprintf('状態数                 : %d\n', n);
fprintf('可制御部分空間の次元   : %d\n', rank_Mc);
fprintf('不可制御部分空間の次元 : %d\n', n - rank_Mc);
fprintf('可観測部分空間の次元   : %d\n', rank_Mo);
fprintf('不可観測部分空間の次元 : %d\n', n - rank_Mo);

[eigs_a, eigs_b, eigs_c, eigs_d, info] = ...
    kalman_mode_eigs(est.A, est.B, est.C, true);




%%
% Q = diag([1,1,1,1,1,1,1,1,1,1,1,1,ones(1,size(est.A,1)-12)]);
Q = diag([ones(1,size(est.A,1))]);
Rc = diag([1;0.1;0.1;1]);
% 
% size(est.A)
% size(est.B)
% size(Q)
% size(Rc)

K_full = dlqr(est.A, est.B, Q, Rc );

%%
% A_deco = est.A - est.B * K_full;
% fprintf("元のＡ行列のeig");
% disp(eig(est.A));
% fprintf("ゲイン使ったときのＡ－ＢＫのeig");
% disp(eig(A_deco));

%%
save('kalman_gainたち\LYKL_1124_byKD_LYKL.mat','K_full');
fprintf("ゲインをkalman_gain_all_LYKL_sec.matとして保存しました");