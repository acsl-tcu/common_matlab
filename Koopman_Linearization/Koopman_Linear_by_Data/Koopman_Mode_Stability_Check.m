clear;
clc;
load("without_w1.mat");
% load("koopman_model_first.mat",'est');
% load("koopman_common_z_.mat");
% load("EstimationResult_12state_2_7_Exp_sprine+zsprine+P2Pz_torque_incon_150data_vzからz算出.mat",'est');
% 固有値と固有ベクトル
[V, D] = eig(est.A);

% 可制御性行列
Uc = ctrb(est.A, est.B);

% 許容誤差
tol = 1e-6;
n=size(est.A,1);
rank_Uc=rank(Uc);
fprintf('システムの次数（状態数）: %d\n', n);
fprintf('可制御性行列 Uc のランク: %d\n', rank_Uc);
fprintf('=== 不安定かつ不可制御なモードのチェック ===\n\n');

for k = 1:size(est.A,1)
    lambda = D(k,k);
    vk = V(:,k);

    % vk を可制御性空間に射影
    proj = Uc * (Uc \ vk);
    err = norm(vk - proj);

    if err < tol
        fprintf('[%2d]固有値 λ = %.4f は可制御\n',k, lambda);
    else
        fprintf('[%2d]固有値 λ = %.4f は不可制御',k, lambda);
        if abs(lambda) >= 1
            fprintf('かつ不安定\n');
        else
            fprintf('ただし安定\n');
        end
    end
end
fprintf('\n=== モード方向への制御感度（B^T * v） ===\n');

control_sensitivity = zeros(n, 1);

for k = 1:n
    vk = V(:,k);
    sensitivity = norm(est.B' * vk);  % 感度（スカラー）
    control_sensitivity(k) = sensitivity;
    fprintf('[%2d] |B^T * v| = %.4e\n', k, sensitivity);
end
figure;
stem(1:n, control_sensitivity, 'filled');
xlabel('モード番号（固有値のインデックス）');
ylabel('制御感度 |B^T v|');
title('各モードに対する制御感度');
grid on;