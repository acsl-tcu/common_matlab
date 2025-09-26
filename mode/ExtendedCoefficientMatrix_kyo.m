%-- ベクトル化するための関数
function [ResultA,ResultB] = ExtendedCoefficientMatrix_kyo(Param)

 factor_alpha =1; 
    % ECM:Extended Coeifficient Matrix
    A = Param{1};
    B = Param{2};
    
    Horizon = Param{3};
    Xnum = Param{4};
     
% --- 次元の定義 ---
n_model = size(A, 1);
n_physical = Param{4};


%% ========================================================================
%  パート1：可制御性および可安定化の解析
% =========================================================================
fprintf('============================================================\n');
fprintf('         パート1：Koopmanモデル 可制御性・可安定化解析\n');
fprintf('============================================================\n');

% --- 1.1 システムの基本情報 ---
fprintf('1. システムの基本情報:\n');
fprintf('   - Koopmanモデルの総状態次元 (n_model) : %d\n', n_model);
fprintf('   - 物理状態の次元 (n_physical)         : %d\n', n_physical);
fprintf('   - 入力の次元 (m)                      : %d\n', size(B, 2));

% --- 1.2 可制御性の判定 ---
Co = ctrb(A, B);
tolerance = n_model * eps(norm(A, 'fro'));
controllableRank = rank(Co, tolerance);

fprintf('\n2. 可制御性の判定:\n');
fprintf('   - 可制御性行列のランク (r)              : %d\n', controllableRank);
fprintf('   - モデルの総状態次元 (n_model)          : %d\n', n_model);

if controllableRank == n_model
    fprintf('   - 結論: r = n_model のため、システムは完全可制御です。\n');
    fprintf('   - (解釈: システムは当然「可安定化」です。)\n');
else
    fprintf('   - 結論: r < n_model のため、システムは不可制御です。\n');
    
    % --- 1.3 可安定化の解析 ---
    fprintf('\n3. 可安定化の解析:\n');
    r = controllableRank;
    Tc = orth(Co);
    Tuc = null(Tc');
    T = [Tc, Tuc];
    T_inv = inv(T); 
    A_bar = T_inv * A * T;
    
    A_c = A_bar(1:r, 1:r);
    A_12 = A_bar(1:r, r+1:end);
    A_uc = A_bar(r+1:end, r+1:end);
    
    eig_uc = eig(A_uc);
    max_mag_uc = 0;
    if ~isempty(eig_uc)
        max_mag_uc = max(abs(eig_uc));
    end
    
    fprintf('   - 不可制御部分系の最大固有値の絶対値: %f\n', max_mag_uc);
    stability_tolerance = 1e-6;
    if max_mag_uc < 1-stability_tolerance
        fprintf('   >>> 最終結論: システムは可安定化 (Stabilizable) です。\n');
        fprintf('       (解釈: 全ての不安定モードが制御可能であり、制御器設計により安定化が可能です。)\n');
    else
        fprintf('   >>> 最終結論: システムは可安定化ではありません (Not Stabilizable)。\n');
        
        % --- ★★★ 疑似安定化処理 (ARTIFICIAL STABILIZATION) ★★★ ---
        fprintf('\n------------------------------------------------------------\n');
        fprintf('   警告！ 制御不能な不安定モードが検出されました。\n');
        fprintf('   MPCの最適化計算を可能にするため、提案手法に基づき、\n');
        fprintf('   不可制御部分系のダイナミクスを係数 alpha = %f でスケール倍し、\n', factor_alpha);
        fprintf('   疑似的に安定な設計用モデル A_tilde を生成します。\n');
        fprintf('   注意：これは実システムとの間に意図的な誤差を導入する操作です。\n');
        
        A_uc_tilde = factor_alpha * A_uc;
        A_bar_tilde = [A_c, A_12; zeros(size(A_uc, 1), size(A_c, 2)), A_uc_tilde];
        A_tilde = T * A_bar_tilde * inv(T);
        
        % 元のA行列を、修正されたA_tildeで上書きする
        % A = A_tilde;
        
        fprintf('   新しい設計用モデル A_tilde が生成され、以降の計算で使用されます。\n');
        fprintf('------------------------------------------------------------\n');
    end
end
fprintf('\n');

%% ========================================================================
%  パート2：モデル全体の開ループ安定性解析
% =========================================================================
fprintf('============================================================\n');
fprintf('         パート2：Koopmanモデル 開ループ安定性解析\n');
fprintf('============================================================\n');
fprintf('(注意：システムが不可安定化の場合、この解析は疑似的に安定化された設計用モデルに基づきます。)\n');

% --- 2.1 固有値の計算 ---
eigenvalues = eig(A);
magnitudes = abs(eigenvalues);
max_magnitude = max(magnitudes);

fprintf('1. (設計用)モデル全体の固有値情報:\n');
fprintf('   - A行列の最大固有値の絶対値: %f\n', max_magnitude);

% --- 2.2 開ループ安定性の判定 ---
fprintf('\n2. (設計用)モデルの開ループ安定性の判定:\n');
if max_magnitude < 1
    fprintf('   - 結論: 設計用モデルは漸近安定です。\n');
elseif max_magnitude > 1
    fprintf('   - 結論: 設計用モデルは不安定です。\n');
else
    fprintf('   - 結論: 設計用モデルは臨界安定です。\n');
end

% --- 2.3 固有値の可視化 ---
fprintf('\n3. 固有値分布図を作成しています...\n');
figure('Name', 'Koopman 設計用モデル 安定性解析', 'NumberTitle', 'off');
hold on; grid on; axis equal;
plot(cos(linspace(0, 2*pi, 200)), sin(linspace(0, 2*pi, 200)), 'r--', 'LineWidth', 1.5);
plot(real(eigenvalues), imag(eigenvalues), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
title('設計用A行列の固有値分布'); xlabel('実部 (Real Part)'); ylabel('虚部 (Imaginary Part)');
legend('単位円 (安定境界)', 'システムの固有値', 'Location', 'best');
lim_val = max(1.2, max_magnitude * 1.1);
xlim([-lim_val, lim_val]); ylim([-lim_val, lim_val]);
hold off;
fprintf('   - プロットが作成されました。\n\n');

%% ========================================================================
%  パート3：MPC用拡張行列の計算
% =========================================================================
fprintf('============================================================\n');
fprintf('         パート3：MPC用拡張行列の計算\n');
fprintf('============================================================\n');
fprintf('全ての解析が完了しました。これより、主処理である拡張行列の計算を開始します。\n');

    S = zeros(Horizon*Xnum, Horizon*length(B(1,:)));

    % ホライズンの値によらない
    % A行列
    Am = [];
    for i = 1:Horizon
        Am = [Am; A^i]; %A
    end
    % B行列
    for i  = 1:Horizon
        for j = 1:Horizon
            if j <= i
                S(1+length(B(:,1))*(i-1):length(B(:,1))*i,1+length(B(1,:))*(j-1):length(B(1,:))*j) = A^(i-j)*B;
            end
        end
    end
    ResultA = Am;
    ResultB = S;
end