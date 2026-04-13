%% =========================================================
%  Koopman K/B 行列 収束性分析スクリプト
%
%  目的:
%    1. K行列のスペクトル解析 (固有値分布・谱半径)
%    2. 長期予測の収束性判定 (K^H の挙動)
%    3. B行列の入力チャンネル分析
%    4. 開ループ多ステップ予測の誤差成長
%    5. 収束/発散の自動判定レポート
%
%  使い方:
%    1. mat_file に .mat ファイルのパスを指定
%    2. 実行すると分析図 + 判定レポートが出力される
% =========================================================
clear; clc; close all;

%% =========================================================
%  設定
%% =========================================================
mat_file    = 'koopman_v5_pure.mat';   % ← ここを変更
H_analyze   = 200;    % 分析する最大予測ステップ数
dt          = 0.01;   % サンプリング時間 [s]
rho_warn    = 1.00;   % この値を超えたら警告
rho_danger  = 1.05;   % この値を超えたら危険

fprintf('============================================================\n');
fprintf('  Koopman K/B 行列 収束性分析\n');
fprintf('  対象ファイル: %s\n', mat_file);
fprintf('============================================================\n');

%% =========================================================
%  データ読み込み
%% =========================================================
if ~exist(mat_file, 'file')
    error('ファイルが見つかりません: %s', mat_file);
end
kr   = load(mat_file);
kr   = kr.koopman_results;
K    = kr.K;
B    = kr.B;
C    = kr.C;
nz   = size(K, 1);

% input_scales の確認
if isfield(kr, 'input_scales')
    input_scales = kr.input_scales;
    B_phys = B .* input_scales';
    fprintf('  input_scales: [%.4f  %.4f  %.4f  %.4f]\n', input_scales);
else
    input_scales = ones(4,1);
    B_phys = B;
    fprintf('  [WARNING] input_scales が見つかりません。B をそのまま使用します。\n');
end

fprintf('  K: %dx%d   B: %dx%d   C: %dx%d\n', size(K,1),size(K,2),size(B,1),size(B,2),size(C,1),size(C,2));

%% =========================================================
%  STEP 1: スペクトル解析
%% =========================================================
fprintf('\n[Step 1] スペクトル解析...\n');
ev       = eig(K);
ev_abs   = abs(ev);
rho      = max(ev_abs);
rho_mean = mean(ev_abs);

fprintf('  谱半径 ρ(K)    = %.6f\n', rho);
fprintf('  固有値絶対値 平均 = %.6f\n', rho_mean);
fprintf('  固有値絶対値 最小 = %.6f\n', min(ev_abs));
fprintf('  実部が正の固有値  = %d / %d\n', sum(real(ev)>0), nz);
fprintf('  |λ| > 1.0 の個数  = %d / %d\n', sum(ev_abs>1.0), nz);
fprintf('  |λ| > 0.99 の個数 = %d / %d\n', sum(ev_abs>0.99), nz);

%% =========================================================
%  STEP 2: K^H の挙動 (フロベニウスノルムで追跡)
%% =========================================================
fprintf('\n[Step 2] K^H の長期挙動解析 (H=1~%d)...\n', H_analyze);
Kh_norm    = zeros(H_analyze, 1);
Kh_maxelem = zeros(H_analyze, 1);
Kh_cur     = eye(nz);

for h = 1:H_analyze
    Kh_cur        = Kh_cur * K;
    Kh_norm(h)    = norm(Kh_cur, 'fro');
    Kh_maxelem(h) = max(abs(Kh_cur(:)));
end

% 理論値: ρ^H
rho_theory = rho .^ (1:H_analyze)';

% 収束判定
h50_norm  = Kh_norm(min(50,  H_analyze));
h100_norm = Kh_norm(min(100, H_analyze));
h200_norm = Kh_norm(min(200, H_analyze));

fprintf('  ‖K^50‖_F  = %.4f\n', h50_norm);
fprintf('  ‖K^100‖_F = %.4f\n', h100_norm);
fprintf('  ‖K^200‖_F = %.4f\n', h200_norm);

%% =========================================================
%  STEP 3: B行列チャンネル分析
%% =========================================================
fprintf('\n[Step 3] B行列 入力チャンネル分析...\n');
ch_names = {'F_total [N]', 'tau_x [Nm]', 'tau_y [Nm]', 'tau_z [Nm]'};
for ch = 1:4
    b_col     = B_phys(:, ch);
    b_norm    = norm(b_col);
    b_max     = max(abs(b_col));
    fprintf('  ch%d %-16s: ‖b‖=%.4f  max|b|=%.4f  scale=%.4f\n', ...
        ch, ch_names{ch}, b_norm, b_max, input_scales(ch));
end

% 入力チャンネル間の相対影響力
b_norms = arrayfun(@(c) norm(B_phys(:,c)), 1:4);
fprintf('  相対影響力 (正規化): ');
fprintf('%.3f  ', b_norms / max(b_norms));
fprintf('\n');

%% =========================================================
%  STEP 4: ゼロ入力での状態減衰シミュレーション
%% =========================================================
fprintf('\n[Step 4] ゼロ入力 自由応答シミュレーション...\n');

% 初期状態: 典型的な偏差（位置0.1m, 速度0.1m/s）
% Ψ空間での初期値は C^+ * x0 (擬似逆行列)
x0_test = zeros(12, 1);
x0_test(3) = 0.1;   % z方向に0.1m偏差
x0_test(9) = 0.1;   % vz = 0.1m/s

% C の擬似逆行列で z0 を計算
C_pinv = pinv(C);
z0     = C_pinv * x0_test;

% 自由応答
z_free      = zeros(nz, H_analyze+1);
x_free      = zeros(12, H_analyze+1);
z_free(:,1) = z0;
x_free(:,1) = C * z0;

for h = 1:H_analyze
    z_free(:,h+1) = K * z_free(:,h);
    x_free(:,h+1) = C * z_free(:,h+1);
end

t_vec     = (0:H_analyze) * dt;
pos_decay = sqrt(sum(x_free(1:3,:).^2, 1));
vel_decay = sqrt(sum(x_free(7:9,:).^2, 1));

fprintf('  初期偏差: z=%.2fm  vz=%.2fm/s\n', x0_test(3), x0_test(9));
fprintf('  t=0.5s 後の位置ノルム: %.4f m\n', pos_decay(round(0.5/dt)+1));
fprintf('  t=1.0s 後の位置ノルム: %.4f m\n', pos_decay(round(1.0/dt)+1));
fprintf('  t=2.0s 後の位置ノルム: %.4f m\n', pos_decay(round(2.0/dt)+1));

%% =========================================================
%  STEP 5: ホバー入力での定常応答
%% =========================================================
fprintf('\n[Step 5] ホバー入力 定常応答チェック...\n');

% ホバー入力 (質量は params から取るか、デフォルト値)
if isfield(kr, 'params') && isfield(kr.params, 'm')
    mass = kr.params.m;
else
    mass = 0.75;  % デフォルト値 [kg]
    fprintf('  質量が見つかりません。デフォルト %.2f kg を使用\n', mass);
end

u_hover_phys   = [mass * 9.81; 0; 0; 0];
u_hover_scaled = u_hover_phys .* input_scales;

% 平衡点: (I - K)·z_e = B·u_hover_scaled
IK = eye(nz) - K;
if rank(IK) < nz
    fprintf('  [WARNING] (I-K) が特異です。平衡点計算をスキップします。\n');
else
    z_eq    = IK \ (B * u_hover_scaled);
    x_eq    = C * z_eq;
    fprintf('  平衡点 x_eq (C·z_eq):\n');
    fprintf('    位置  [x,y,z]:     %.4f  %.4f  %.4f m\n',   x_eq(1),x_eq(2),x_eq(3));
    fprintf('    姿態  [r,p,y]:     %.4f  %.4f  %.4f rad\n', x_eq(4),x_eq(5),x_eq(6));
    fprintf('    速度  [vx,vy,vz]:  %.4f  %.4f  %.4f m/s\n', x_eq(7),x_eq(8),x_eq(9));
    fprintf('    角速度[wx,wy,wz]:  %.4f  %.4f  %.4f rad/s\n',x_eq(10),x_eq(11),x_eq(12));

    % 理想的な平衡点からの乖離
    pos_eq_err = norm(x_eq(1:3));
    vel_eq_err = norm(x_eq(7:9));
    fprintf('  平衡点の位置ノルム: %.4f m  (理想=0)\n', pos_eq_err);
    fprintf('  平衡点の速度ノルム: %.4f m/s (理想=0)\n', vel_eq_err);
end

%% =========================================================
%  STEP 6: 自動判定レポート
%% =========================================================
fprintf('\n============================================================\n');
fprintf('  自動判定レポート\n');
fprintf('============================================================\n');

% 各判定
judge = struct();

% ① 谱半径
if rho < 0.99
    judge.rho = '✓ STABLE   ';
    judge.rho_detail = sprintf('ρ=%.4f < 0.99', rho);
elseif rho < rho_warn
    judge.rho = '△ MARGINAL ';
    judge.rho_detail = sprintf('ρ=%.4f 境界付近', rho);
elseif rho < rho_danger
    judge.rho = '✗ WARNING  ';
    judge.rho_detail = sprintf('ρ=%.4f > 1.0 発散リスク', rho);
else
    judge.rho = '✗ DANGER   ';
    judge.rho_detail = sprintf('ρ=%.4f >> 1.0 確実に発散', rho);
end

% ② K^100のノルム
if h100_norm < 0.1
    judge.kh = '✓ CONVERGE ';
    judge.kh_detail = sprintf('‖K^100‖=%.4f 十分収束', h100_norm);
elseif h100_norm < 1.0
    judge.kh = '△ SLOW     ';
    judge.kh_detail = sprintf('‖K^100‖=%.4f 収束は遅い', h100_norm);
elseif h100_norm < 10.0
    judge.kh = '✗ POOR     ';
    judge.kh_detail = sprintf('‖K^100‖=%.4f ほぼ減衰しない', h100_norm);
else
    judge.kh = '✗ DIVERGE  ';
    judge.kh_detail = sprintf('‖K^100‖=%.4f 発散している', h100_norm);
end

% ③ B行列のバランス
b_ratio = max(b_norms) / (min(b_norms) + 1e-12);
if b_ratio < 10
    judge.b = '✓ BALANCED ';
    judge.b_detail = sprintf('最大/最小比=%.1f 均衡', b_ratio);
elseif b_ratio < 50
    judge.b = '△ UNEVEN   ';
    judge.b_detail = sprintf('最大/最小比=%.1f やや不均衡', b_ratio);
else
    judge.b = '✗ SKEWED   ';
    judge.b_detail = sprintf('最大/最小比=%.1f 極度に不均衡', b_ratio);
end

% ④ 平衡点精度
if exist('pos_eq_err','var')
    if pos_eq_err < 0.01
        judge.eq = '✓ ACCURATE ';
        judge.eq_detail = sprintf('平衡点誤差=%.4fm 精度良好', pos_eq_err);
    elseif pos_eq_err < 0.1
        judge.eq = '△ FAIR     ';
        judge.eq_detail = sprintf('平衡点誤差=%.4fm やや大きい', pos_eq_err);
    else
        judge.eq = '✗ POOR     ';
        judge.eq_detail = sprintf('平衡点誤差=%.4fm B/C行列が不正確', pos_eq_err);
    end
else
    judge.eq = '? SKIP     ';
    judge.eq_detail = '計算スキップ';
end

fprintf('  ┌──────────────────────────────────────────────────────┐\n');
fprintf('  │  項目                  判定         詳細              │\n');
fprintf('  ├──────────────────────────────────────────────────────┤\n');
fprintf('  │  谱半径 ρ(K)           %s  %s\n', judge.rho, judge.rho_detail);
fprintf('  │  長期収束 ‖K^100‖      %s  %s\n', judge.kh,  judge.kh_detail);
fprintf('  │  B行列バランス         %s  %s\n', judge.b,   judge.b_detail);
fprintf('  │  ホバー平衡点精度      %s  %s\n', judge.eq,  judge.eq_detail);
fprintf('  └──────────────────────────────────────────────────────┘\n');

% 総合判定
n_danger = sum([rho >= rho_warn, h100_norm >= 1.0, b_ratio >= 50]);
if n_danger == 0
    fprintf('\n  総合判定: ✓ MPC に使用可能\n');
elseif n_danger == 1
    fprintf('\n  総合判定: △ 条件付き使用可 (上記警告を確認)\n');
else
    fprintf('\n  総合判定: ✗ 再学習を推奨 (%d 項目が警告水準)\n', n_danger);
end

%% =========================================================
%  STEP 7: 可視化
%% =========================================================
fig1 = figure('Name','K Matrix Spectral Analysis','Position',[50,50,1400,900]);

% ── 図1: 固有値分布 (複素平面)
subplot(2,3,1);
theta = linspace(0, 2*pi, 200);
plot(cos(theta), sin(theta), 'k--', 'LineWidth',0.8); hold on;
plot(0.98*cos(theta), 0.98*sin(theta), 'b:', 'LineWidth',0.8);
scatter(real(ev), imag(ev), 20, abs(ev), 'filled');
colorbar; colormap(gca, jet);
xlabel('実部'); ylabel('虚部');
title(sprintf('固有値分布  ρ=%.4f', rho));
axis equal; grid on;
legend('単位円','ρ=0.98','固有値','Location','best');

% ── 図2: 固有値絶対値の分布
subplot(2,3,2);
histogram(ev_abs, 30, 'FaceColor',[0.2 0.5 0.8]);
xline(1.0,  'r-',  '|λ|=1.0', 'LineWidth',1.5);
xline(0.98, 'b--', 'ρ=0.98',  'LineWidth',1.2);
xline(rho,  'r--', sprintf('ρ_{max}=%.4f',rho), 'LineWidth',1.5);
xlabel('|固有値|'); ylabel('個数');
title('固有値絶対値のヒストグラム');
grid on;

% ── 図3: K^H のフロベニウスノルム
subplot(2,3,3);
h_vec = 1:H_analyze;
semilogy(h_vec*dt, Kh_norm, 'b-', 'LineWidth',1.8); hold on;
semilogy(h_vec*dt, rho_theory * norm(eye(nz),'fro'), 'r--', 'LineWidth',1.2);
semilogy(h_vec*dt, ones(H_analyze,1), 'k:', 'LineWidth',0.8);
xlabel('時間 [s]'); ylabel('‖K^H‖_F (log)');
title('K^H のフロベニウスノルム');
legend('実測 ‖K^H‖_F', sprintf('理論 ρ^H (ρ=%.4f)',rho), '‖I‖=1', 'Location','best');
grid on;

% ── 図4: ゼロ入力 自由応答
subplot(2,3,4);
plot(t_vec, pos_decay, 'b-', 'LineWidth',1.8); hold on;
plot(t_vec, vel_decay, 'r-', 'LineWidth',1.8);
xlabel('時間 [s]'); ylabel('ノルム');
title('自由応答 (ゼロ入力)');
legend('位置ノルム ‖p‖','速度ノルム ‖v‖','Location','best');
grid on;

% ── 図5: B行列の各列ノルム (チャンネル別影響力)
subplot(2,3,5);
b_cols_norm = zeros(nz, 4);
for c = 1:4
    b_cols_norm(:,c) = abs(B_phys(:,c));
end
bar(1:nz, b_cols_norm, 'stacked');
xlabel('Koopman 次元 (観測量インデックス)');
ylabel('|B_{phys}| の大きさ');
title('B_{phys} 各行の各チャンネル寄与');
legend('F','τ_x','τ_y','τ_z','Location','best');
grid on;

% ── 図6: 各チャンネルの B ノルム比較
subplot(2,3,6);
b_ch_norms = arrayfun(@(c) norm(B_phys(:,c)), 1:4);
bar_colors = [0.2 0.5 0.8; 0.8 0.3 0.3; 0.3 0.8 0.3; 0.8 0.6 0.2];
b_bar = bar(b_ch_norms, 'FaceColor','flat');
for c = 1:4, b_bar.CData(c,:) = bar_colors(c,:); end
set(gca, 'XTickLabel', {'F','τ_x','τ_y','τ_z'});
ylabel('‖B_{phys}(:,ch)‖');
title('入力チャンネル別 B行列影響力');
for c = 1:4
    text(c, b_ch_norms(c)*1.02, sprintf('×%.1f', input_scales(c)), ...
        'HorizontalAlignment','center','FontSize',10);
end
grid on;

sgtitle(sprintf('Koopman K/B 行列分析  |  ρ(K)=%.4f  |  nz=%d', rho, nz), ...
    'FontSize',13,'FontWeight','bold');

saveas(fig1, 'koopman_matrix_analysis.png');
fprintf('\n  図を保存: koopman_matrix_analysis.png\n');

%% =========================================================
%  STEP 8: 予測ステップ数ごとの誤差成長率
%% =========================================================
fig2 = figure('Name','Prediction Error Growth','Position',[100,100,1000,600]);

% 複数の初期偏差に対してK^H·z0 の誤差成長を確認
test_cases = {
    'z偏差 0.1m',       [0;0;0.1; 0;0;0; 0;0;0;  0;0;0];
    'vz偏差 0.2m/s',    [0;0;0;   0;0;0; 0;0;0.2; 0;0;0];
    'roll偏差 10°',     [0;0;0;   deg2rad(10);0;0; 0;0;0; 0;0;0];
    'x偏差 0.5m',       [0.5;0;0; 0;0;0; 0;0;0;   0;0;0];
};
colors = {'b','r','g','m'};

subplot(1,2,1);
hold on;
for tc = 1:size(test_cases,1)
    x0_tc   = test_cases{tc,2};
    z0_tc   = pinv(C) * x0_tc;
    err_growth = zeros(H_analyze+1, 1);
    z_tc    = z0_tc;
    for h = 0:H_analyze
        x_tc = C * z_tc;
        err_growth(h+1) = norm(x_tc(1:3));   % 位置ノルム
        z_tc = K * z_tc;
    end
    plot(t_vec, err_growth, colors{tc}, 'LineWidth',1.5, ...
        'DisplayName', test_cases{tc,1});
end
xlabel('時間 [s]'); ylabel('位置ノルム ‖p‖ [m]');
title('各初期偏差に対する位置誤差成長 (ゼロ入力)');
legend('Location','best'); grid on;

subplot(1,2,2);
hold on;
for tc = 1:size(test_cases,1)
    x0_tc   = test_cases{tc,2};
    z0_tc   = pinv(C) * x0_tc;
    err_growth = zeros(H_analyze+1, 1);
    z_tc    = z0_tc;
    for h = 0:H_analyze
        x_tc = C * z_tc;
        err_growth(h+1) = norm(x_tc(7:9));   % 速度ノルム
        z_tc = K * z_tc;
    end
    plot(t_vec, err_growth, colors{tc}, 'LineWidth',1.5, ...
        'DisplayName', test_cases{tc,1});
end
xlabel('時間 [s]'); ylabel('速度ノルム ‖v‖ [m/s]');
title('各初期偏差に対する速度誤差成長 (ゼロ入力)');
legend('Location','best'); grid on;

sgtitle('予測誤差成長分析  (ゼロ入力 自由応答)', 'FontSize',12,'FontWeight','bold');
saveas(fig2, 'koopman_error_growth.png');
fprintf('  図を保存: koopman_error_growth.png\n');

fprintf('\n============================================================\n');
fprintf('  分析完了\n');
fprintf('============================================================\n');