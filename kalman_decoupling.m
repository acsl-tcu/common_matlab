clear;
clc;
% load("without_w1.mat");
% load("koopman_model_first.mat",'est');
load("koopman_common_z_.mat");
% load("EstimationResult_12state_2_7_Exp_sprine+zsprine+P2Pz_torque_incon_150data_vzからz算出.mat",'est');

% 可制御性行列
n = size(est.A, 1);
tol = 1e-14; % 許容誤差
Mc = ctrb(est.A, est.B);
k=rank(Mc);
Mo = obsv(est.A,est.C);
ImMc_orth = orth(Mc);
KerMo_orth = null(Mo,'rational');
T_inv = [];


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
F = T_inv\est.A*T_inv;
G = T_inv\est.B;
H = est.C*T_inv;
U_uc = T(:,k+1:end);
contrib = abs(U_uc);
score = sum(contrib,2);

% 大きい順に並べる
[sorted, idx] = sort(score,'descend');
fprintf('不可制御部分に強く寄与する観測量:\n');
for j = 1:min(10,length(idx))
    fprintf('観測量 %d: %.3f\n', idx(j), sorted(j));
end

%可制御部分抜き出し
Ac = F(1:k, 1:k);
Bc = G(1:k, :);
%不可制御部分抜き出し
Acbar = F(k+1:end,k+1:end);
Bcbar = G(k+1:end,:);

Qc = diag([ones(1,size(Ac,1))]);
Rc = 1*eye(4);
Kc = dlqr(Ac, Bc, Qc, Rc);
K_all = [Kc,zeros(size(est.B,2),(size(est.A,1)-k))];
K_full = K_all/T_inv;
save('kalman_gain.mat','K_full');