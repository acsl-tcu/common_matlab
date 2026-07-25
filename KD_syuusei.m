clear;


%%
loadFileName = input('読み込むファイル名(.mat不要)：','s');
load([loadFileName '.mat']);

A = est.A;
B = est.B;
C = est.C;
n = size(A, 1);

%% ★ステップ1：定数方向をAの構造から自動検出
%   const_idxが保存されていない場合でも、Aの各行を単位ベクトルと
%   比較すれば、定数観測量の行（自分自身にしか依存しない行）を
%   見つけられる。ハード制約なしで学習されていても、回帰的に
%   ほぼ正確にこの形になっているはず。

detect_tol = 1e-3;   % これくらい緩めでOK（見つけるためだけの閾値）
const_idx = [];

for i = 1:n
    e_i = zeros(1, n); e_i(i) = 1;
    row_err = norm(A(i,:) - e_i);
    b_norm  = norm(B(i,:));
    if row_err < detect_tol && b_norm < detect_tol
        const_idx(end+1) = i; %#ok<AGROW>
        fprintf('定数方向候補: index=%d  (||A(i,:)-e_i||=%.3e, ||B(i,:)||=%.3e)\n', ...
            i, row_err, b_norm);
    end
end

if isempty(const_idx)
    warning(['定数方向がAの構造から見つかりませんでした。' ...
             'detect_tolを緩めるか、モデルに定数観測量が' ...
             '含まれているか確認してください。']);
else
    fprintf('検出された定数方向: %s\n', mat2str(const_idx));
end

%% ★ステップ2：見つかった行を厳密にスナップ（丸め込み）
%   ここが今回のシリーズの核心。
%   A(i,:)を厳密にe_iに、B(i,:)を厳密に0に置き換えることで、
%   (A^k B)のその行が"丸め誤差"ではなく"本当にゼロ"になる。
%   これにより、ctrbを26次元フルにかけても誤差が増幅されず、
%   通常のtol(1e-14ではなく1e-6程度)でも正しくrankが出るようになる。

for i = const_idx
    e_i = zeros(1, n); e_i(i) = 1;
    A(i,:) = e_i;
    B(i,:) = 0;
end

%% 可制御性・可観測性行列
tol = 1e-6;   % ★1e-14から変更：ソルバー精度に見合った現実的な値

Mc = ctrb(A, B);
k = rank(Mc, tol);
Mo = obsv(A, C);
ImMc_orth  = orth(Mc);
KerMo_orth = null(Mo, 'rational');
T_inv = [];

fprintf('可制御次元 k = %d / %d\n', k, n);
if ~isempty(const_idx) && k ~= (n - numel(const_idx))
    warning(['可制御次元が期待値(n - 定数方向の数)と一致しません。' ...
             'スナップ処理やtolを見直してください。']);
end

% Xa: 可制御かつ不可観測
if ~isempty(ImMc_orth) && ~isempty(KerMo_orth)
    for i = 1:size(ImMc_orth, 2)
        v = ImMc_orth(:, i);
        if norm(KerMo_orth * (KerMo_orth' * v) - v) < tol
            if isempty(T_inv) || rank([T_inv, v], tol) > rank(T_inv, tol)
                T_inv = [T_inv, v];
            end
        end
    end
end
Xa = T_inv;

% Xb: 可制御かつ可観測
for i = 1:size(ImMc_orth, 2)
    v = ImMc_orth(:, i);
    if isempty(T_inv) || rank([T_inv, v], tol) > rank(T_inv, tol)
        T_inv = [T_inv, v];
    end
end
Xb = T_inv(:, size(Xa,2)+1:size(T_inv,2));

% Xc: 不可制御かつ可観測
for i = 1:size(KerMo_orth, 2)
    v = KerMo_orth(:, i);
    if isempty(T_inv) || rank([T_inv, v], tol) > rank(T_inv, tol)
        T_inv = [T_inv, v];
    end
end
Xc = T_inv(:, size(Xa,2)+size(Xb,2)+1:size(T_inv,2));

% Xd: 不可制御かつ不可観測
if size(T_inv, 2) < n
    Xd = null(T_inv');
    T_inv = [T_inv, Xd];
end
Xd = T_inv(:, size(Xa,2)+size(Xb,2)+size(Xc,2)+1:end);

%% 変換行列チェック
if size(T_inv, 2) ~= n
    error('変換行列の列数が状態空間の次元と一致しません。');
end
if abs(det(T_inv)) < tol
    error('変換行列が特異行列です。');
end

T = inv(T_inv);
F = T_inv \ A * T_inv;
G = T_inv \ B;
H = C * T_inv;

%% 固有値プロット
na = size(Xa,2); nb = size(Xb,2); nc = size(Xc,2); nd = size(Xd,2);

eigs_a = []; eigs_b = []; eigs_c = []; eigs_d = [];
if na > 0, eigs_a = eig(F(1:na, 1:na)); end
if nb > 0, eigs_b = eig(F(na+1:na+nb, na+1:na+nb)); end
if nc > 0, eigs_c = eig(F(na+nb+1:na+nb+nc, na+nb+1:na+nb+nc)); end
if nd > 0, eigs_d = eig(F(na+nb+nc+1:end, na+nb+nc+1:end)); end

figure('Color', 'w', 'Name', 'カルマン分解 固有値解析');
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

title(sprintf('%d次元システムのカルマン分解 固有値配置', n));
xlabel('実軸 (Real)'); ylabel('虚軸 (Imaginary)');
legend(h_legend, legend_labels, 'Location','northeastoutside','FontSize',10);
axis equal;
all_eigs = [eigs_a; eigs_b; eigs_c; eigs_d];
max_val = max([1.2; abs(all_eigs)]);
xlim([-max_val*1.1, max_val*1.1]); ylim([-max_val*1.1, max_val*1.1]);
line([-max_val*1.1 max_val*1.1],[0 0],'Color',[.5 .5 .5],'HandleVisibility','off');
line([0 0],[-max_val*1.1 max_val*1.1],'Color',[.5 .5 .5],'HandleVisibility','off');
hold off;

%% LQR設計（可制御部分のみ）
Ac = F(1:k, 1:k);
Bc = G(1:k, :);

Q = diag(ones(1, n));
I_ = T_inv \ Q * T_inv;
Qc = I_(1:k, 1:k);
Qc = (Qc + Qc') / 2;

Rc = diag([1; 0.1; 0.1; 1]);

Kc = dlqr(Ac, Bc, Qc, Rc);
K_all = [Kc, zeros(size(B,2), n-k)];
K_full = K_all / T_inv;

fprintf('\n元のA行列のeig:\n'); disp(eig(A));
fprintf('A-B*K_full のeig:\n'); disp(eig(A - B*K_full));

%% 保存
saveFolder = fullfile(pwd, 'kalman_gainたち');
if ~exist(saveFolder, 'dir'), mkdir(saveFolder); end
saveFileName = fullfile(saveFolder, [loadFileName '_gain_KD.mat']);
save(saveFileName, 'K_full', 'const_idx');
fprintf('"%s" として保存しました。\n', saveFileName);