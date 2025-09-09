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
Mc = ctrb(est.A, est.B);
k=rank(Mc);
Mo = obsv(est.A,est.C);
% [R_Mc, p_Mc] = rref(Mc);
% ImMc = Mc(:, p_Mc);
% ImMc = orth(Mc);
[~, pivots] = rref(Mc);   % ピボット列の番号を取得
ImMc = Mc(:, pivots);
ImMc = [1,0;0,1;0,0;1,0];

KerMo = null(Mo,'rational');
Rn = eye(size(est.A,1));
tol = 1e-10;
Xa = [];
for i = 1:size(ImMc,2)
    v = ImMc(:,i);
    % v が KerMo の張る部分空間に含まれるかを判定
    coeff = KerMo \ v;
    if norm(KerMo*coeff - v) < tol
        Xa = [Xa, v];
    end
end
Xb = [];
for i = 1:size(ImMc,2)
    v = ImMc(:,i);
    % Xa の張る部分空間に含まれるか確認
    if isempty(Xa)
        inXa = false;
    else
        coeff = Xa \ v;
        inXa = (norm(Xa*coeff - v) < tol);
    end
    
    if ~inXa
        Xb = [Xb, v];
    end
end
Xc = [];
for i = 1:size(KerMo,2)
    v = KerMo(:,i);
    if isempty(Xa)
        inXa = false;
    else
        coeff = Xa \ v;
        inXa = (norm(Xa*coeff - v) < tol);
    end
    
    if ~inXa
        Xc = [Xc, v];   % そのまま追加
    end
end
Known = [Xa, Xb, Xc];
if ~isempty(Known)
    coln = vecnorm(Known,2,1);
    Known = Known(:, coln > tol);
end

n = size(Known,1);
if isempty(Known)
    % Known が空なら補空間は全空間
    Xd = eye(n);
else
    % 通常解: Known の列空間の直和補空間を求める
    Xd = null(Known', 'r');   % 再現性のある基底を返す
    % null が空（数値問題等）ならフォールバックで標準基底から組み立て
    if isempty(Xd)
        Xd = zeros(n,0);
        for i = 1:n
            e = zeros(n,1); e(i)=1;
            if rank([Known, Xd, e], tol) > rank([Known, Xd], tol)
                Xd = [Xd, e];
            end
            if rank([Known, Xd], tol) == n
                break;
            end
        end
        if isempty(Xd)
            % 補空間が存在しない場合（Known が全空間を張る）
            Xd = zeros(n,1);
        end
    end
end
T_inv = [Xa,Xb,Xc,Xd];
T = inv(T_inv);
F = T*est.A*T_inv;
G = T*est.B;
H = est.C*T_inv;

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