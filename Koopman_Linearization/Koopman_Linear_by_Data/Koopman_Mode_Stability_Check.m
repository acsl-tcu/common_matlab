clear;
clc;
% load("without_w1.mat");
% load("koopman_model_first.mat",'est');
% load("second_model.mat",'est');
% load("koopman_common_z_.mat");
% load("EstimationResult_12state_2_7_Exp_sprine+zsprine+P2Pz_torque_incon_150data_vzからz算出.mat",'est');
% load("third_model.mat",'est');
% load("z.mat",'est');
% load("without1.mat");
load("znot.mat");

%αの値を変更したときの安定性チェック
% load('kalman_gain.mat','K_full');
% est.A = est.A-0.4*est.B*K_full;

%%%%%-----モードチェック-----%%%%%
% 固有値と固有ベクトル
[V, D] = eig(est.A);
% 許容誤差
tol = 1e-6;
n=size(est.A,1);
Mc = ctrb(est.A, est.B);
rank_Mc=rank(Mc);
fprintf('システムの次数（状態数）: %d\n', n);
fprintf('可制御性行列 Mc のランク: %d\n', rank_Mc);
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
    proj = Mc * (Mc \ vk);
    err = norm(vk - proj);

    if err < tol
        fprintf('[%2d]固有値 λ = %.4f は可制御\n',k, lambda);
        if abs(lambda) >= 1
            fprintf('かつ不安定！！！\n');
        else
            fprintf('ただし安定\n');
        end
    else
        fprintf('[%2d]固有値 λ = %.4f は不可制御',k, lambda);
        if abs(lambda) >= 1
            fprintf('かつ不安定！！！\n');
        else
            fprintf('ただし安定\n');
        end
    end
end