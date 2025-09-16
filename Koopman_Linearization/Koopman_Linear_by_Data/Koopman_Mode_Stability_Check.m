clear;
clc;
% load("without_w1.mat");
% load("koopman_model_first.mat",'est');
load("koopman_common_z_.mat");
% load("EstimationResult_12state_2_7_Exp_sprine+zsprine+P2Pz_torque_incon_150data_vzからz算出.mat",'est');
% 固有値と固有ベクトル
[V, D] = eig(est.A);
est.A = [1 0 0 0;0 -1 0 1;0 0 -1 0;2 0 -1 -1];
est.B = [-1;1;0;-1];
est.C = [1 0 1 0];

% 可制御性行列
n = size(est.A, 1);
tol = 1e-9; % 許容誤差
Mc = ctrb(est.A, est.B);
k=rank(Mc);
Mo = obsv(est.A,est.C);
KerMo_orth = null(Mo,'rational');
Rn = eye(size(est.A,1));
T_inv = [];
ImMc_orth = [1,0;0,1;0,0;1,0];

% Xa: 可制御かつ不可観測
% ImMc の基底から KerMo の空間に属するものを抽出
if ~isempty(ImMc_orth) && ~isempty(KerMo_orth)
    for i = 1:size(ImMc_orth, 2)
        v = ImMc_orth(:, i);
        % v が KerMo の列空間に含まれるか判定
        if norm(KerMo_orth * (KerMo_orth' * v) - v) < tol
            % 既に T_inv に含まれていないか確認
            if isempty(T_inv) || rank([T_inv, v]) > rank(T_inv)
                T_inv = [T_inv, v];
            end
        end
    end
end
Xa = T_inv; % ここまでが Xa

% Xb: 可制御かつ可観測
% ImMc の基底のうち、Xa と線形独立なものを抽出
for i = 1:size(ImMc_orth, 2)
    v = ImMc_orth(:, i);
    % T_inv に含まれていないか確認
    if isempty(T_inv) || rank([T_inv, v]) > rank(T_inv)
        T_inv = [T_inv, v];
    end
end
Xb = T_inv(:, size(Xa,2)+1:size(T_inv,2));

% Xc: 不可制御かつ可観測
% KerMo の基底のうち、Xa, Xb と線形独立なものを抽出
for i = 1:size(KerMo_orth, 2)
    v = KerMo_orth(:, i);
    % T_inv に含まれていないか確認
    if isempty(T_inv) || rank([T_inv, v]) > rank(T_inv)
        T_inv = [T_inv, v];
    end
end
Xc = T_inv(:, size(Xa,2)+size(Xb,2)+1:size(T_inv,2));

% Xd: 不可制御かつ不可観測
% T_inv に残りの次元を埋める基底を追加
if size(T_inv, 2) < n
    % T_invの列空間の直交補空間をnullで計算
    Xd = null(T_inv');
    T_inv = [T_inv, Xd];
end
% 最終的なXb, Xc, Xdの抽出
Xd = T_inv(:, size(Xa,2)+size(Xb,2)+size(Xc,2)+1:end);

% 3. 変換行列 T の作成と分解の実行
%--------------------------------------------------------------------------
% T_inv の列数がnになっているか確認
if size(T_inv, 2) ~= n
    error('変換行列の列数が状態空間の次元と一致しません。');
end

% 正則性を最終確認
if abs(det(T_inv)) < tol
    error('変換行列が特異行列です。');
end


T = inv(T_inv);
F = T*est.A*T_inv;
G = T*est.B;
H = est.C*T_inv;

% A, B：元のシステム行列
[Ac, Bc, Cc, Tc,P] = ctrbf(est.A, est.B,est.C);
% max(size(est.A))*eps(norm(est.A))
k = sum(P);
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
    else
        fprintf('[%2d]固有値 λ = %.4f は不可制御',k, lambda);
        if abs(lambda) >= 1
            fprintf('かつ不安定\n');
        else
            fprintf('ただし安定\n');
        end
    end
end