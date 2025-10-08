function judge()
% =========================================================================
% 初期設定とデータロード
% =========================================================================
% 必要な変数が定義されているMATファイルからデータをロード
% (例: A, B, factor_alpha, est構造体など)
load('2025-10-02_exp_renew_code00_randompp.mat');


A = est.A;
B = est.B;


n_model = size(A, 1);
n_physical = 12;
stability_tolerance = 1e-6; % 安定性判定用の許容誤差

% --- 結果保存用の構造体を初期化 ---
results = struct();
results.part1 = struct();
results.part2 = struct();
results.summary = struct();
results.A_tilde_generated = false; % 疑似安定化モデル生成フラグ


% --- 1.1 システムの基本情報 ---
results.part1.n_model = n_model;
results.part1.n_physical = n_physical;
results.part1.m = size(B, 2);

% --- 1.2 可制御性の判定 ---
Co = ctrb(A, B);
tolerance = n_model * eps(norm(A, 'fro'));
results.part1.controllableRank = rank(Co, tolerance);

if results.part1.controllableRank == n_model
    results.summary.controllability = '完全可制御 (Fully Controllable)';
    results.summary.stabilizability = '可安定化 (Stabilizable)';
    results.part1.is_controllable = true;
else
    results.summary.controllability = '不可制御 (Not Fully Controllable)';
    results.part1.is_controllable = false;
    
    % --- 1.3 可安定化の解析 ---
    r = results.part1.controllableRank;
    Tc = orth(Co);
    Tuc = null(Tc');
    T = [Tc, Tuc];
    T_inv = inv(T); 
    A_bar = T_inv * A * T;
    
    A_uc = A_bar(r+1:end, r+1:end);
    eig_uc = eig(A_uc);
    
    max_mag_uc = 0;
    if ~isempty(eig_uc)
        max_mag_uc = max(abs(eig_uc));
    end
    results.part1.uncontrollable_max_eig = max_mag_uc;
    
    if max_mag_uc < 1 - stability_tolerance
        results.summary.stabilizability = '可安定化 (Stabilizable)';
        results.part1.is_stabilizable = true;
    else
        results.summary.stabilizability = '不可安定化 (Not Stabilizable)';
        results.part1.is_stabilizable = false;
        
        % --- 1.4 疑似安定な設計用モデル A_tilde の生成 ---
        results.A_tilde_generated = true;
        results.part1.factor_alpha = factor_alpha;
        
        A_c = A_bar(1:r, 1:r);
        A_12 = A_bar(1:r, r+1:end);
        A_uc_tilde = factor_alpha * A_uc;
        A_bar_tilde = [A_c, A_12; zeros(size(A_uc, 1), size(A_c, 2)), A_uc_tilde];
        
        % 新しい設計用モデル A_tilde を生成
        A_tilde = T * A_bar_tilde * inv(T);
        
        % 元のA行列を、修正されたA_tildeで上書き
        A = A_tilde; 
    end
end


% --- 2.1 固有値の計算 ---
eigenvalues = eig(A);
magnitudes = abs(eigenvalues);
max_magnitude = max(magnitudes);

results.part2.eigenvalues = eigenvalues;
results.part2.magnitudes = magnitudes;
results.part2.max_magnitude = max_magnitude;

% --- 2.2 開ループ安定性の判定 ---
if max_magnitude < 1 - stability_tolerance
    results.summary.stability = '漸近安定 (Asymptotically Stable)';
elseif max_magnitude > 1 + stability_tolerance
    results.summary.stability = '不安定 (Unstable)';
else
    results.summary.stability = '臨界安定 (Critically Stable)';
end


figure('Name', 'Koopman 設計用モデル 安定性解析', 'NumberTitle', 'off');
hold on; grid on; axis equal;
plot(cos(linspace(0, 2*pi, 200)), sin(linspace(0, 2*pi, 200)), 'r--', 'LineWidth', 1.5);
plot(real(eigenvalues), imag(eigenvalues), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
title('設計用A行列の固有値分布'); xlabel('実部 (Real Part)'); ylabel('虚部 (Imaginary Part)');
legend('単位円 (安定境界)', 'システムの固有値', 'Location', 'best');
lim_val = max(1.2, max_magnitude * 1.1);
xlim([-lim_val, lim_val]); ylim([-lim_val, lim_val]);
hold off;



fprintf('============================================================\n');
fprintf('         パート1：Koopmanモデル 可制御性・可安定化解析\n');
fprintf('============================================================\n');
fprintf('1. システムの基本情報:\n');
fprintf('   - Koopmanモデルの総状態次元 (n_model) : %d\n', results.part1.n_model);
fprintf('   - 物理状態の次元 (n_physical)         : %d\n', results.part1.n_physical);
fprintf('   - 入力の次元 (m)                      : %d\n', results.part1.m);

fprintf('\n2. 可制御性の判定:\n');
fprintf('   - 可制御性行列のランク (r)              : %d\n', results.part1.controllableRank);
fprintf('   - モデルの総状態次元 (n_model)          : %d\n', results.part1.n_model);

if results.part1.is_controllable
    fprintf('   - 結論: r = n_model のため、システムは完全可制御です。\n');
    fprintf('   - (解釈: システムは当然「可安定化」です。)\n');
else
    fprintf('   - 結論: r < n_model のため、システムは不可制御です。\n');
    fprintf('\n3. 可安定化の解析:\n');
    fprintf('   - 不可制御部分系の最大固有値の絶対値: %f\n', results.part1.uncontrollable_max_eig);
    if results.part1.is_stabilizable
        fprintf('   >>> 最終結論: システムは可安定化 (Stabilizable) です。\n');
        fprintf('       (解釈: 全ての不安定モードが制御可能であり、制御器設計により安定化が可能です。)\n');
    else
        fprintf('   >>> 最終結論: システムは可安定化ではありません (Not Stabilizable)。\n');
        fprintf('\n------------------------------------------------------------\n');
        fprintf('   警告！ 制御不能な不安定モードが検出されました。\n');
        fprintf('   MPCの最適化計算を可能にするため、提案手法に基づき、\n');
        fprintf('   不可制御部分系のダイナミクスを係数 alpha = %f でスケール倍し、\n', results.part1.factor_alpha);
        fprintf('   疑似的に安定な設計用モデル A_tilde を生成します。\n');
        fprintf('   注意：これは実システムとの間に意図的な誤差を導入する操作です。\n');
        fprintf('   新しい設計用モデル A_tilde が生成され、以降の計算で使用されます。\n');
        fprintf('------------------------------------------------------------\n');
    end
end
fprintf('\n');

% --- パート2：開ループ安定性解析の結果表示 ---
fprintf('============================================================\n');
fprintf('         パート2：Koopmanモデル 開ループ安定性解析\n');
fprintf('============================================================\n');
if results.A_tilde_generated
    fprintf('(注意：システムが不可安定化のため、この解析は疑似的に安定化された設計用モデルに基づきます。)\n');
end
fprintf('1. (設計用)モデル全体の固有値情報:\n');
fprintf('   - A行列の最大固有値の絶対値: %f\n', results.part2.max_magnitude);

fprintf('\n2. (設計用)モデルの開ループ安定性の判定:\n');
if strcmp(results.summary.stability, '漸近安定 (Asymptotically Stable)')
    fprintf('   - 結論: 設計用モデルは漸近安定です。\n');
elseif strcmp(results.summary.stability, '不安定 (Unstable)')
    fprintf('   - 結論: 設計用モデルは不安定です。\n');
else
    fprintf('   - 結論: 設計用モデルは臨界安定です。\n');
end

fprintf('\n3. 固有値分布図を作成しています...\n');
fprintf('   - プロットが作成されました。\n\n');

% --- パート2.5：最終サマリーと全固有値リストの表示 ---
fprintf('============================================================\n');
fprintf('               解析結果サマリーと全固有値リスト\n');
fprintf('============================================================\n');
fprintf('1. システムの可制御性・安定性の概要:\n');
fprintf('   - Koopmanモデルの可制御性: %s\n', results.summary.controllability);
fprintf('   - Koopmanモデルの可安定化: %s\n', results.summary.stabilizability);
fprintf('   - 開ループの安定性    : %s (最大固有値絶対値: %.6f)\n', results.summary.stability, results.part2.max_magnitude);

fprintf('\n2. Koopmanモデル A行列の全固有値リスト:\n');
fprintf('------------------------------------------------------------------\n');
fprintf('  固有値 No. | 固有値 (Real + Imag)      | 固有値の絶対値 | 安定性\n');
fprintf('------------------------------------------------------------------\n');

for j = 1:n_model
    lambda_j = results.part2.eigenvalues(j);
    mag_j = results.part2.magnitudes(j);
    
    if mag_j < 1 - stability_tolerance
        status = '安定 (Stable)';
    elseif mag_j > 1 + stability_tolerance
        status = '不安定 (Unstable)';
    else
        status = '臨界安定 (Critical)';
    end
    
    if abs(imag(lambda_j)) < stability_tolerance 
         lambda_str = sprintf('%10.6f           ', real(lambda_j));
    else 
         lambda_str = sprintf('%10.6f + %10.6fi', real(lambda_j), imag(lambda_j));
    end
    
    fprintf('     %03d       | %s |   %10.6f | %s\n', j, lambda_str, mag_j, status);
end
fprintf('------------------------------------------------------------------\n');

end