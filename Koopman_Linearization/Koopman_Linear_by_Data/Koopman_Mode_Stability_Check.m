clear;
clc;
% load("without_w1.mat");
% load("koopman_model_first.mat",'est');
load("koopman_common_z_.mat");
% load("EstimationResult_12state_2_7_Exp_sprine+zsprine+P2Pz_torque_incon_150data_vzからz算出.mat",'est');
% 固有値と固有ベクトル
[V, D] = eig(est.A);

% 可制御性行列
Uc = ctrb(est.A, est.B);
k=rank(Uc);
% A, B：元のシステム行列
[Ac, Bc, Cc, Tc,P] = ctrbf(est.A, est.B,est.C);
% max(size(est.A))*eps(norm(est.A))
sum(P)
% 可制御部分（上の左上ブロック）を抽出
A_ctrl = Ac(1:k, 1:k);
B_ctrl = Bc(1:k, :);

% DLQRの設計
Q = eye(k);            % 状態重み
R = eye(size(est.B,2));    % 入力重み
K_ctrl = dlqr(A_ctrl, B_ctrl, Q, R);

% 可制御部分だけのゲイン → 全空間（26次元）に拡張
K_aug = [K_ctrl, zeros(size(K_ctrl,1), size(est.A,1) - k)];

% 変換行列 Tc を使って元の座標系に戻す
K_full = K_aug * inv(Tc);   % 最終的な状態フィードバックゲイン



% 許容誤差
tol = 1e-6;
n=size(est.A,1);
rank_Uc=rank(Uc);
fprintf('システムの次数（状態数）: %d\n', n);
fprintf('可制御性行列 Uc のランク: %d\n', rank_Uc);
fprintf('=== 不安定かつ不可制御なモードのチェック ===\n\n');
% eigvals = eig(est.A);
% if all(abs(eigvals) < 1) % 離散系の場合
%     disp('全体システムは安定');
% else
%     disp('不安定な極あり');
% end
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