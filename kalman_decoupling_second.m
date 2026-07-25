clear;
% clc;

%%
loadFileName = input('読み込むファイル名(.mat不要)：','s');
load([loadFileName '.mat']);

A = est.A;
B = est.B;
C = est.C;
n = size(A, 1);

% ★訂正1： tol=1e-14 は機械精度レベルで厳しすぎる。
%   ソルバー(SDPT3)の実際の収束精度(1e-8〜1e-11オーダー)に合わせて
%   もう少し緩い値を使う。
tol = 1e-6;

%% ★訂正2：定数方向(const_idx)は数値的に"発見"しない
%   rensyuuKLLY_fixed で厳密固定した const_idx / dyn_idx をそのまま使う。
%   理由：ctrb(A,B)はA^25まで掛けるため、本来ゼロのはずの
%   const_idx方向の数値誤差が蓄積・増幅し、rank(Mc)が常に26に
%   なってしまう（=数値的に信頼できない）。
%   なので「不可制御方向は1個、それはconst_idx」という
%   *解析的に保証された事実* をそのまま使い、
%   数値的に頑健な動的25次元部分だけをctrb/obsvにかける。

const_idx = est.const_idx;
dyn_idx   = est.dyn_idx;
n_dyn     = numel(dyn_idx);

A_dyn = A(dyn_idx, dyn_idx);
B_dyn = B(dyn_idx, :);
C_dyn = C(:, dyn_idx);

% サニティチェック（数値誤差レベルであることの確認）
fprintf('||B(const_idx,:)|| = %.3e (機械精度〜1e-8程度なら正常)\n', ...
    norm(B(const_idx,:)));

%% ★訂正3：Xa/Xb/Xc/Xd分解は動的25次元部分だけに対して行う
%   （A_dynの固有値は全てrho_bar未満なので、A_dyn^kが減衰し、
%    ctrb/obsvが数値的に頑健になる）

[Xa_dyn, Xb_dyn, Xc_dyn, Xd_dyn, k_dyn] = local_kalman_decompose(A_dyn, B_dyn, C_dyn, tol);

fprintf('---- 動的部分空間(%d次元)の分解結果 ----\n', n_dyn);
fprintf('Xa(可制御・不可観測) : %d\n', size(Xa_dyn,2));
fprintf('Xb(可制御・可観測)   : %d\n', size(Xb_dyn,2));
fprintf('Xc(不可制御・可観測) : %d\n', size(Xc_dyn,2));
fprintf('Xd(不可制御・不可観測): %d\n', size(Xd_dyn,2));

%% ★訂正4：dyn基底を26次元へゼロ埋め込み
embed = @(V) local_embed(V, dyn_idx, n);

Xa = embed(Xa_dyn);
Xb = embed(Xb_dyn);
Xc = embed(Xc_dyn);
Xd = embed(Xd_dyn);

%% ★訂正5：定数方向(λ=1モード)を正しい固有ベクトルで追加
%   e_const_idx をそのまま使うのではなく、A*v=v を満たす
%   正確な固有ベクトルが必要。
%   理由：rensyuuKLLY_fixedで固定したのは A(const_idx,:) という
%   "行"だけであり、A(dyn_idx,const_idx) という"列側の結合項"
%   （定数項が動的状態に与えるアフィン・オフセットの影響）はゼロと
%   決めつけていない。むしろ通常は非ゼロ。したがって
%   e_const_idx自体はAの厳密な固有ベクトルにはならない。
%
%   ブロック構造 A = [A_dyn, A_dc; 0, 1] （A_dc=A(dyn_idx,const_idx)）
%   から、固有値1のベクトルは以下を解くだけで直接求まる：
%       v_dyn   = -(A_dyn - I)^-1 * A_dc
%       v_const = 1
%   A_dynの固有値は全てrho_bar未満なので (A_dyn - I) は
%   (1-rho_bar)以上の余裕を持って非特異であり、この計算は
%   eig(A)を26次元全体にかけて"1に近いものを探す"よりも
%   直接的かつ数値的に安定している。

A_dc = A(dyn_idx, const_idx);
v_dyn = -(A_dyn - eye(n_dyn)) \ A_dc;

v = zeros(n, 1);
v(dyn_idx)   = v_dyn;
v(const_idx) = 1;
v = v / norm(v);

% 検算：A*v が本当に v に近いか確認（数値的な妥当性チェック）
residual = norm(A*v - v) / norm(v);
fprintf('λ=1固有ベクトルの検算 ||A*v - v||/||v|| = %.3e\n', residual);

% λ=1モードが可観測かどうかをPBHテストで判定（これも1回の掛け算のみ）
obs_score = norm(C * v) / max(1, norm(C,'fro'));
fprintf('λ=1モードの可観測性スコア ||C*v|| = %.3e\n', obs_score);

if obs_score > tol
    fprintf('λ=1モードは可観測 → Xc(不可制御・可観測)に分類\n');
    Xc = [Xc, v];
else
    fprintf('λ=1モードは不可観測 → Xd(不可制御・不可観測)に分類\n');
    Xd = [Xd, v];
end

%% 変換行列の組み立て
T_inv = [Xa, Xb, Xc, Xd];

if size(T_inv, 2) ~= n
    error('変換行列の列数が状態空間の次元と一致しません。(%d vs %d)', ...
        size(T_inv,2), n);
end
if abs(det(T_inv)) < 1e-30
    error('変換行列が特異行列です。');
end

T = inv(T_inv);
F = T_inv \ A * T_inv;
G = T_inv \ B;
H = C * T_inv;

%% 固有値プロット（Xa/Xb/Xc/Xdで色分け）
na = size(Xa,2); nb = size(Xb,2); nc = size(Xc,2); nd = size(Xd,2);

eigs_a = []; eigs_b = []; eigs_c = []; eigs_d = [];
if na > 0, eigs_a = eig(F(1:na, 1:na)); end
if nb > 0, eigs_b = eig(F(na+1:na+nb, na+1:na+nb)); end
if nc > 0, eigs_c = eig(F(na+nb+1:na+nb+nc, na+nb+1:na+nb+nc)); end
if nd > 0, eigs_d = eig(F(na+nb+nc+1:end, na+nb+nc+1:end)); end

figure('Color', 'w', 'Name', 'カルマン分解 固有値解析（修正版）');
hold on; grid on;
theta = linspace(0, 2*pi, 300);
plot(cos(theta), sin(theta), 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');

plot_data = {
    eigs_a, [1 0 0],   'o', 'Xa: 可制御・不可観測';
    eigs_b, [0 0 1],   's', 'Xb: 可制御・可観測';
    eigs_c, [0 0.7 0], 'd', 'Xc: 不可制御・可観測';
    eigs_d, [0.7 0 0.7],'x', 'Xd: 不可制御・不可観測'
};

h_legend = []; legend_labels = {};
for i = 1:size(plot_data,1)
    ce = plot_data{i,1};
    if ~isempty(ce)
        p = plot(real(ce), imag(ce), 'LineStyle','none', ...
            'Marker', plot_data{i,3}, 'MarkerEdgeColor', plot_data{i,2}, ...
            'MarkerFaceColor','none', 'LineWidth',1.5, 'MarkerSize',9);
        h_legend = [h_legend, p];
        legend_labels{end+1} = sprintf('%s (%d個)', plot_data{i,4}, length(ce));
    end
end

title(sprintf('%d次元システムのカルマン分解 固有値配置（修正版）', n));
xlabel('実軸 (Real)'); ylabel('虚軸 (Imaginary)');
legend(h_legend, legend_labels, 'Location','northeastoutside','FontSize',10);
axis equal;
all_eigs = [eigs_a; eigs_b; eigs_c; eigs_d];
max_val = max([1.2; abs(all_eigs)]);
xlim([-max_val*1.1, max_val*1.1]); ylim([-max_val*1.1, max_val*1.1]);
line([-max_val*1.1 max_val*1.1],[0 0],'Color',[.5 .5 .5],'HandleVisibility','off');
line([0 0],[-max_val*1.1 max_val*1.1],'Color',[.5 .5 .5],'HandleVisibility','off');
hold off;

%% ★訂正6：LQRは正しい k = na+nb（＝実際の可制御次元）で設計
k = na + nb;
fprintf('\n真の可制御次元 k = %d （従来コードは誤って%dを使用していた）\n', ...
    k, n);

Ac = F(1:k, 1:k);
Bc = G(1:k, :);

Q = diag(ones(1, n));
I = T_inv \ Q * T_inv;
Qc = I(1:k, 1:k);
Qc = (Qc + Qc')/2;   % 数値誤差による非対称性の除去

Rc = diag([1; 0.1; 0.1; 1]);

Kc = dlqr(Ac, Bc, Qc, Rc);
K_all = [Kc, zeros(size(B,2), n-k)];
K_full = K_all / T_inv;

%% 確認
fprintf('\n元のA行列のeig:\n'); disp(eig(A));
fprintf('A-B*K_full のeig:\n'); disp(eig(A - B*K_full));

%% 保存
saveFolder = fullfile(pwd, 'kalman_gainたち');
if ~exist(saveFolder, 'dir'), mkdir(saveFolder); end
saveFileName = fullfile(saveFolder, [loadFileName '_gain_KD_fixed.mat']);
save(saveFileName, 'K_full', 'T_inv', 'const_idx', 'dyn_idx');
fprintf('"%s" として保存しました。\n', saveFileName);


%% ===== ローカル関数 =====

function [Xa, Xb, Xc, Xd, k] = local_kalman_decompose(A, B, C, tol)
% 元コードのXa/Xb/Xc/Xd抽出アルゴリズムを関数化したもの。
% A,B,Cが数値的に頑健な（固有値がrho_bar未満に収まっている）
% 部分系である前提で使うこと。

    n = size(A,1);
    Mc = ctrb(A,B);
    Mo = obsv(A,C);

    k = rank(Mc, tol);

    ImMc_orth  = orth(Mc);
    KerMo_orth = null(Mo, 'rational');

    T_inv = [];

    % Xa: 可制御かつ不可観測
    if ~isempty(ImMc_orth) && ~isempty(KerMo_orth)
        for i = 1:size(ImMc_orth,2)
            v = ImMc_orth(:,i);
            if norm(KerMo_orth*(KerMo_orth'*v) - v) < tol
                if isempty(T_inv) || rank([T_inv, v]) > rank(T_inv)
                    T_inv = [T_inv, v];
                end
            end
        end
    end
    Xa = T_inv;

    % Xb: 可制御かつ可観測
    for i = 1:size(ImMc_orth,2)
        v = ImMc_orth(:,i);
        if isempty(T_inv) || rank([T_inv, v]) > rank(T_inv)
            T_inv = [T_inv, v];
        end
    end
    Xb = T_inv(:, size(Xa,2)+1:size(T_inv,2));

    % Xc: 不可制御かつ可観測
    for i = 1:size(KerMo_orth,2)
        v = KerMo_orth(:,i);
        if isempty(T_inv) || rank([T_inv, v]) > rank(T_inv)
            T_inv = [T_inv, v];
        end
    end
    Xc = T_inv(:, size(Xa,2)+size(Xb,2)+1:size(T_inv,2));

    % Xd: 不可制御かつ不可観測（残りの直交補空間）
    if size(T_inv,2) < n
        Xd_new = null(T_inv');
        T_inv = [T_inv, Xd_new];
    end
    Xd = T_inv(:, size(Xa,2)+size(Xb,2)+size(Xc,2)+1:end);
end

function V_full = local_embed(V_sub, idx, n)
% (numel(idx) x m) の行列 V_sub を、idx行だけ埋めた (n x m) 行列に
% ゼロ埋め込みする
    m = size(V_sub, 2);
    V_full = zeros(n, m);
    V_full(idx, :) = V_sub;
end