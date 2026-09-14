clear;
clc;

%%
loadFileName = input('読み込むファイル名(.mat不要)：','s');
load([loadFileName '.mat']);

A = est.A;
B = est.B;
C = est.C;
n = size(A, 1);

Q = eye(26);
R  = diag([1; 0.1; 0.1; 1]);

 K_full = dlqr(A, B, Q, R);
%% 保存
saveFolder = fullfile(pwd, 'kalman_gainたち');
if ~exist(saveFolder, 'dir'), mkdir(saveFolder); end
saveFileName = fullfile(saveFolder, [loadFileName '_gain_by_only_LQR.mat']);
save(saveFileName, 'K_full');
fprintf('"%s" として保存しました。\n', saveFileName);