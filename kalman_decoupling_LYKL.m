clear;
clc;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load("alldata_LYKL.mat");

Q = diag([1,1,1,1,1,1,1,1,1,1,1,1,ones(1,size(est.A,1)-12)]);
Rc = diag([1;0.1;0.1;1]);
K_full = dlqr(est.A,est.B,Q,Rc);
A_deco = est.A - est.B * K_full;
disp(eig(A_deco))
save('kalman_gain_LYKL.mat','K_full');
fprintf("ゲインをkalman_gain_LYKL.matとして保存しました");