%% check_koopman_model.m
% 構造化Koopmanモデルの制御特性を包括的に分析
% ======================================================
clear; clc;
load('structured_koopman_model.mat');

fprintf('══════════════════════════════════════════════════════\n');
fprintf('  構造化Koopmanモデル 制御特性分析\n');
fprintf('══════════════════════════════════════════════════════\n\n');

B_eff = B_k * diag(input_scales(:));

%% ═══════════════════════════════════════════════════════
%% 1. 基本情報
%% ═══════════════════════════════════════════════════════
fprintf('── 1. 基本情報 ──\n');
fprintf('  状態次元 n_z = %d\n', n_z);
fprintf('  物理状態 n_x = %d\n', n_x);
fprintf('  入力次元 n_u = %d\n', n_u);
fprintf('  サンプリング dt = %.3f s (%.0f Hz)\n', dt, 1/dt);
fprintf('  質量 = %.4f kg\n', mass);
fprintf('  観測量内訳: 物理%d + データ%d + RBF%d\n', n_phys, n_edmd, n_rbf);
fprintf('\n');

%% ═══════════════════════════════════════════════════════
%% 2. 行列のランク
%% ═══════════════════════════════════════════════════════
fprintf('── 2. 行列ランク ──\n');
rank_A = rank(A_k);
rank_B = rank(B_k);
rank_Beff = rank(B_eff);
rank_C = rank(C_k);
fprintf('  rank(A_k) = %d / %d\n', rank_A, n_z);
fprintf('  rank(B_k) = %d / %d (min(n_z,n_u)=%d)\n', rank_B, n_z, min(n_z,n_u));
fprintf('  rank(B_eff) = %d / %d\n', rank_Beff, n_z);
fprintf('  rank(C_k) = %d / %d (min(n_x,n_z)=%d)\n', rank_C, n_z, min(n_x,n_z));
if rank_A < n_z
    fprintf('  [警告] A_kがフルランクでない → %d個の零固有値\n', n_z - rank_A);
end
fprintf('\n');

%% ═══════════════════════════════════════════════════════
%% 3. 安定性分析（固有値）
%% ═══════════════════════════════════════════════════════
fprintf('── 3. 安定性分析 ──\n');
eig_A = eig(A_k);
rho = max(abs(eig_A));
fprintf('  谱半径 ρ(A_k) = %.6f', rho);
if rho < 1.0
    fprintf('  ✓ 安定\n');
else
    fprintf('  ✗ 不安定!\n');
end
fprintf('  |λ|最大5個: ');
[~,idx_sort] = sort(abs(eig_A),'descend');
for i = 1:min(5,n_z)
    fprintf('%.4f ', abs(eig_A(idx_sort(i))));
end
fprintf('\n');
fprintf('  |λ|最小5個: ');
for i = n_z:-1:max(1,n_z-4)
    fprintf('%.4f ', abs(eig_A(idx_sort(i))));
end
fprintf('\n');

% 固有値の分布
n_real = sum(abs(imag(eig_A)) < 1e-10);
n_complex = (n_z - n_real) / 2;
fprintf('  実固有値: %d個  複素ペア: %d組\n', n_real, n_complex);

% 不安定固有値
n_unstable = sum(abs(eig_A) > 1.0);
if n_unstable > 0
    fprintf('  [警告] |λ|>1 の固有値: %d個\n', n_unstable);
end
fprintf('\n');

%% ═══════════════════════════════════════════════════════
%% 4. 可制御性分析
%% ═══════════════════════════════════════════════════════
fprintf('── 4. 可制御性分析 ──\n');

% フル可制御性行列は n_z×(n_z*n_u) で巨大なので
% PBH テスト（固有値テスト）を使用
fprintf('  PBH可制御性テスト（各固有値で rank([λI-A, B]) = n_z か確認）\n');
n_uncontrollable = 0;
uncontrollable_eigs = [];
for i = 1:n_z
    lam = eig_A(i);
    M_pbh = [lam*eye(n_z) - A_k, B_eff];
    r = rank(M_pbh, 1e-6);
    if r < n_z
        n_uncontrollable = n_uncontrollable + 1;
        uncontrollable_eigs(end+1) = lam;
    end
end
fprintf('  不可制御モード: %d / %d\n', n_uncontrollable, n_z);
if n_uncontrollable > 0
    fprintf('  不可制御固有値: ');
    for i = 1:min(10, length(uncontrollable_eigs))
        fprintf('%.4f+%.4fi ', real(uncontrollable_eigs(i)), imag(uncontrollable_eigs(i)));
    end
    fprintf('\n');
    % 不可制御だが安定かチェック
    n_uc_unstable = sum(abs(uncontrollable_eigs) >= 1.0);
    n_uc_stable = n_uncontrollable - n_uc_unstable;
    fprintf('  うち安定(|λ|<1): %d  不安定(|λ|≥1): %d\n', n_uc_stable, n_uc_unstable);
    if n_uc_unstable > 0
        fprintf('  [危険] 不安定かつ不可制御なモードが存在!\n');
    else
        fprintf('  [OK] 不可制御モードは全て安定 → 実用上問題なし\n');
    end
else
    fprintf('  ✓ 完全可制御\n');
end

% Gramian ベースの可制御度（小規模な場合のみ）
if n_z <= 200
    try
        Wc = dlyap(A_k, B_eff*B_eff');
        sv_Wc = svd(Wc);
        fprintf('  可制御Gramian特異値:\n');
        fprintf('    最大: %.4e  最小: %.4e  条件数: %.2e\n', ...
            sv_Wc(1), sv_Wc(end), sv_Wc(1)/max(sv_Wc(end),1e-30));
        fprintf('    有効ランク(>1e-10): %d\n', sum(sv_Wc > 1e-10));
    catch ME
        fprintf('  Gramian計算失敗: %s\n', ME.message);
    end
end
fprintf('\n');

%% ═══════════════════════════════════════════════════════
%% 5. 可観測性分析
%% ═══════════════════════════════════════════════════════
fprintf('── 5. 可観測性分析 ──\n');

% PBH可観測性テスト
n_unobservable = 0;
unobservable_eigs = [];
for i = 1:n_z
    lam = eig_A(i);
    M_pbh = [lam*eye(n_z) - A_k; C_k];
    r = rank(M_pbh, 1e-6);
    if r < n_z
        n_unobservable = n_unobservable + 1;
        unobservable_eigs(end+1) = lam;
    end
end
fprintf('  PBH可観測性テスト:\n');
fprintf('  不可観測モード: %d / %d\n', n_unobservable, n_z);
if n_unobservable > 0
    fprintf('  不可観測固有値: ');
    for i = 1:min(10, length(unobservable_eigs))
        fprintf('%.4f ', abs(unobservable_eigs(i)));
    end
    fprintf('\n');
else
    fprintf('  ✓ 完全可観測\n');
end

% 観測Gramian
if n_z <= 200
    try
        Wo = dlyap(A_k', C_k'*C_k);
        sv_Wo = svd(Wo);
        fprintf('  可観測Gramian特異値:\n');
        fprintf('    最大: %.4e  最小: %.4e  条件数: %.2e\n', ...
            sv_Wo(1), sv_Wo(end), sv_Wo(1)/max(sv_Wo(end),1e-30));
        fprintf('    有効ランク(>1e-10): %d\n', sum(sv_Wo > 1e-10));
    catch ME
        fprintf('  Gramian計算失敗: %s\n', ME.message);
    end
end
fprintf('\n');

%% ═══════════════════════════════════════════════════════
%% 6. 入力増益分析
%% ═══════════════════════════════════════════════════════
fprintf('── 6. B_k入力増益分析 ──\n');

labels_u = {'推力F', 'roll τ_x', 'pitch τ_y', 'yaw τ_z'};
labels_x = {'px','py','pz','φ','θ','ψ','vx','vy','vz','ωx','ωy','ωz'};
theory_gains = [0, 0, dt/mass, 0, 0, 0, 0, 0, dt/mass, 0, 0, 0;    % 推力→各状態
                0, 0, 0, 0, 0, 0, 0, 0, 0, dt/Ixx, 0, 0;           % roll
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, dt/Iyy, 0;           % pitch
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, dt/Izz];          % yaw
% 主要増益のみ
primary_pairs = {1,9; 2,10; 3,11; 4,12};  % {入力idx, 主要状態idx}

fprintf('  ┌──────────────────────────────────────────────────────┐\n');
fprintf('  │ 入力      │ 主要状態  │ B_k増益   │ 理論値    │ 比率   │\n');
fprintf('  ├──────────────────────────────────────────────────────┤\n');
for k = 1:4
    u_test = zeros(4,1); u_test(k) = 1;
    dx = C_k * B_eff * u_test;
    ui = primary_pairs{k,1};
    xi = primary_pairs{k,2};
    gain_bk = abs(dx(xi));
    gain_th = abs(theory_gains(ui, xi));
    ratio = gain_bk / max(gain_th, 1e-10) * 100;
    fprintf('  │ %-9s │ %-9s │ %.6f  │ %.6f  │ %5.1f%% │\n', ...
        labels_u{k}, labels_x{xi}, gain_bk, gain_th, ratio);
end
fprintf('  └──────────────────────────────────────────────────────┘\n');

% 全入力→全状態のクロスカップリング
fprintf('\n  クロスカップリング行列 (C_k * B_eff):\n');
CB = C_k * B_eff;
fprintf('  %10s', '');
for j = 1:4; fprintf(' %9s', labels_u{j}); end
fprintf('\n');
for i = [1,2,3,7,8,9,10,11,12]  % 主要状態のみ
    fprintf('  %10s', labels_x{i});
    for j = 1:4
        v = CB(i,j);
        if abs(v) > 0.001
            fprintf(' %+9.4f', v);
        else
            fprintf('     ~0   ');
        end
    end
    fprintf('\n');
end
fprintf('\n');

%% ═══════════════════════════════════════════════════════
%% 7. hover平衡点分析
%% ═══════════════════════════════════════════════════════
fprintf('── 7. hover平衡点分析 ──\n');

if exist('b_k','var')
    fprintf('  b_k補正: あり  |b_k| = %.6f\n', norm(b_k));
    
    % hover状態でのlifting（簡易版）
    x_hov = [0,0,0.6,0,0,0,0,0,0,0,0,0];
    % 最近傍を使用
    fprintf('  hover一歩予測（b_k補正なし）:\n');
    
    % b_kの物理空間への影響
    dx_bk = C_k * b_k;
    fprintf('  b_kの物理空間影響:\n');
    for i = 1:12
        if abs(dx_bk(i)) > 1e-6
            fprintf('    %s: %+.6f\n', labels_x{i}, dx_bk(i));
        end
    end
else
    fprintf('  b_k補正: なし\n');
end
fprintf('\n');

%% ═══════════════════════════════════════════════════════
%% 8. C_k復元品質
%% ═══════════════════════════════════════════════════════
fprintf('── 8. C_k復元品質 ──\n');
fprintf('  C_k サイズ: [%d × %d]\n', size(C_k,1), size(C_k,2));
fprintf('  rank(C_k) = %d\n', rank(C_k));
sv_C = svd(C_k);
fprintf('  C_k特異値: 最大=%.4f  最小=%.4e  条件数=%.2e\n', ...
    sv_C(1), sv_C(end), sv_C(1)/max(sv_C(end),1e-30));

% C_kの各行（各物理状態の復元）のノルム
fprintf('  各状態の復元係数ノルム:\n  ');
for i = 1:12
    fprintf('%s=%.3f ', labels_x{i}, norm(C_k(i,:)));
end
fprintf('\n\n');

%% ═══════════════════════════════════════════════════════
%% 9. MPC予測品質の推定
%% ═══════════════════════════════════════════════════════
fprintf('── 9. MPC予測品質（理論推定）──\n');

% H歩先の予測行列の条件数
H_test = 12;
ExA_test = zeros(n_z*H_test, n_z);
A_pow = eye(n_z);
for k = 1:H_test
    A_pow = A_k * A_pow;
    ExA_test((k-1)*n_z+1:k*n_z, :) = A_pow;
end
fprintf('  予測ホライゾン H = %d\n', H_test);
fprintf('  ||A_k^H|| = %.4e （H歩の伝播倍率）\n', norm(A_k^H_test));
fprintf('  A_k^H 谱半径 = %.6f\n', max(abs(eig(A_k^H_test))));

% 定常ゲイン: (I-A)^{-1}B の検査
if rho < 1.0
    try
        G_ss = (eye(n_z) - A_k) \ B_eff;  % 定常ゲイン
        G_ss_phys = C_k * G_ss;  % 物理空間
        fprintf('  定常ゲイン C(I-A)^{-1}B (推力列):\n');
        dx_ss = G_ss_phys(:,1);
        for i = [3,9]  % pz, vz
            fprintf('    %s: %.4f\n', labels_x{i}, dx_ss(i));
        end
    catch
        fprintf('  定常ゲイン計算失敗（I-Aが特異）\n');
    end
end
fprintf('\n');

%% ═══════════════════════════════════════════════════════
%% 10. 総合評価
%% ═══════════════════════════════════════════════════════
fprintf('══════════════════════════════════════════════════════\n');
fprintf('  総合評価\n');
fprintf('══════════════════════════════════════════════════════\n');

issues = 0;
if rho >= 1.0
    fprintf('  ✗ 不安定（ρ=%.4f）\n', rho); issues = issues+1;
else
    fprintf('  ✓ 安定（ρ=%.6f）\n', rho);
end

if n_uncontrollable == 0
    fprintf('  ✓ 完全可制御\n');
elseif sum(abs(uncontrollable_eigs) >= 1.0) == 0
    fprintf('  △ 部分可制御（不可制御モードは安定）\n');
else
    fprintf('  ✗ 不安定な不可制御モードあり\n'); issues = issues+1;
end

if n_unobservable == 0
    fprintf('  ✓ 完全可観測\n');
else
    fprintf('  △ 部分可観測（%d個の不可観測モード）\n', n_unobservable);
end

% B_k増益
gains_ratio = zeros(4,1);
for k = 1:4
    u_test = zeros(4,1); u_test(k) = 1;
    dx = C_k * B_eff * u_test;
    xi = primary_pairs{k,2};
    gains_ratio(k) = abs(dx(xi)) / abs(theory_gains(k,xi));
end
mean_gain = mean(gains_ratio) * 100;
if mean_gain > 70
    fprintf('  ✓ B_k増益: 平均%.0f%%（良好）\n', mean_gain);
elseif mean_gain > 40
    fprintf('  △ B_k増益: 平均%.0f%%（要補償）\n', mean_gain);
else
    fprintf('  ✗ B_k増益: 平均%.0f%%（大幅不足）\n', mean_gain); issues = issues+1;
end

if exist('b_k','var') && norm(b_k) > 0
    fprintf('  ✓ hover b_k補正: あり（|b_k|=%.4f）\n', norm(b_k));
else
    fprintf('  △ hover b_k補正: なし\n');
end

fprintf('\n  問題数: %d\n', issues);
if issues == 0
    fprintf('  → MPC実装に適したモデル\n');
else
    fprintf('  → 上記の問題を対処してからMPC実装\n');
end
fprintf('══════════════════════════════════════════════════════\n');