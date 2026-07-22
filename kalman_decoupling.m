clear;
clc;
% load("without_w1.mat");
% load("koopman_model_first.mat",'est');
load("KL_1126.mat");
% load("koopman_common_z_.mat")
%速度から位置を積分して求める
% est.A = [zeros(3,3),eye(3,3),zeros(3,20);
%      zeros(23,3),est.A];
% est.B = [zeros(3,4);est.B];
% est.C = [est.C,zeros(12,3)];

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





%変換
T = inv(T_inv);
F = T_inv\est.A*T_inv;
G = T_inv\est.B;
H = est.C*T_inv;

%% 26次元システムのカルマン分解に基づく固有値分布プロット

% 1. 各部分空間の次元（状態数）を確認
na = size(Xa, 2);
nb = size(Xb, 2);
nc = size(Xc, 2);
nd = size(Xd, 2);

% 2. 変換後行列 F から各ブロックの固有値を抽出
% インデックスの累積和を用いて安全に切り出し
eigs_a = []; eigs_b = []; eigs_c = []; eigs_d = [];

if na > 0, eigs_a = eig(F(1:na, 1:na)); end
if nb > 0, eigs_b = eig(F(na+1 : na+nb, na+1 : na+nb)); end
if nc > 0, eigs_c = eig(F(na+nb+1 : na+nb+nc, na+nb+1 : na+nb+nc)); end
if nd > 0, eigs_d = eig(F(na+nb+nc+1 : end, na+nb+nc+1 : end)); end

% 3. プロットの作成
figure('Color', 'w', 'Name', 'カルマン分解 固有値解析');
hold on; grid on;

% 安定境界（単位円）の描画
theta = linspace(0, 2*pi, 300);
plot(cos(theta), sin(theta), 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');

% プロット設定用データ（データ、色、マーカー、ラベル）
plot_data = {
    eigs_a, [1 0 0], 'o', 'Xa: 可制御・不可観測';   % 赤
    eigs_b, [0 0 1], 's', 'Xb: 可制御・可観測';     % 青
    eigs_c, [0 0.7 0], 'd', 'Xc: 不可制御・不可観測'; % 緑
    eigs_d, [0.7 0 0.7], 'x', 'Xd: 不可制御・可観測'  % 紫
};

h_legend = [];
legend_labels = {};

% 各部分空間のプロット実行
for i = 1:size(plot_data, 1)
    current_eigs = plot_data{i,1};
    if ~isempty(current_eigs)
        p = plot(real(current_eigs), imag(current_eigs), ...
            'LineStyle', 'none', ...
            'Marker', plot_data{i,3}, ...
            'MarkerEdgeColor', plot_data{i,2}, ...
            'MarkerFaceColor', 'none', ...
            'LineWidth', 1.5, ...
            'MarkerSize', 9);
        
        h_legend = [h_legend, p];
        legend_labels{end+1} = sprintf('%s (%d個)', plot_data{i,4}, length(current_eigs));
    end
end

% 4. グラフの装飾
title(sprintf('26次元システムのカルマン分解 固有値配置 (全%d状態)', na+nb+nc+nd));
xlabel('実軸 (Real)');
ylabel('虚軸 (Imaginary)');
legend(h_legend, legend_labels, 'Location', 'northeastoutside', 'FontSize', 10);
axis equal;

% 表示範囲の自動調整（単位円＋固有値の最大値）
all_eigs = [eigs_a; eigs_b; eigs_c; eigs_d];
max_val = max([1.2; abs(all_eigs)]);
xlim([-max_val*1.1, max_val*1.1]);
ylim([-max_val*1.1, max_val*1.1]);

% 中心軸の描画
line([-max_val*1.1 max_val*1.1], [0 0], 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
line([0 0], [-max_val*1.1 max_val*1.1], 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');

hold off;


% U_uc = T(:,k+1:end);
% contrib = abs(U_uc);
% score = sum(contrib,2);

%可制御部分抜き出し
Ac = F(1:k, 1:k);
Bc = G(1:k, :);
%不可制御部分抜き出し
Acbar = F(k+1:end,k+1:end);
Bcbar = G(k+1:end,:);

%重みづけ
%pqvwの順
Q = diag([1,1,1,1,1,1,1,1,1,1,1,1,ones(1,size(est.A,1)-12)]);
I = T_inv\Q*T_inv;
Qc = I(1:k,1:k);

% Qc = diag([10,10,10,1,1,1,ones(1,size(Ac,1)-6)]);
% Qc = diag([ones(1,size(Ac,1))]);

%HLは[1;0.05;0.05;0.1]
% Rc = 1*eye(4);
Rc = diag([1;0.1;0.1;1]);
% Kc = dlqr(Ac, Bc, Qc, Rc);
% K_all = [Kc,zeros(size(est.B,2),(size(est.A,1)-k))];
% K_full = K_all/T_inv;

%%
K_full = dlqr(est.A,est.B,Q,Rc);


A_deco = est.A - est.B * K_full;
fprintf("元のＡ行列のeig");
disp(eig(est.A))
fprintf("ゲイン使ったときのＡ－ＢＫのeig")
disp(eig(A_deco))

save('kalman_gainたち\KL_1126_gain_byKD.mat','K_full');
fprintf("ゲインをkalman_gain_senpai_common.matとして保存しました");