function [U_val, A_val, B_val, P_val, result] = ...
    rensyuuKLLY(X,U,Y,F,~)
%SOLVE_KOOPMAN_ALTERNATING_FIXED
% 交互最適化によって安定性制約付きKoopman作用素 U=[A B] を求める
%
% ★このバージョンでの修正点（元コードからの変更）
%   元コードでは、Koopman観測量に定数項(=1)を含めているため、
%   Aは必ず固有値 λ=1 を一つ持つ。これは
%     ・定数方向は入力uで励起できない（本質的に不可制御）
%     ・にもかかわらず安定性LMI（rho_bar<1のSchur補行列条件）を
%       "全次元"のAに課すと、定数方向にも rho_bar<1 を要求してしまい、
%       数値的に矛盾／悪条件（リアプノフ方程式の退化、Gramianが
%       見かけ上フルランクになる等）を引き起こす。
%
%   対策：
%     1. Xlift（観測量）の中から「全データで値が1で一定」の行を
%        自動検出し const_idx とする。
%     2. Step1のSDPで、Uのconst_idx行を
%          U(const_idx, :) = [0,...,0, 1, 0,...,0]  (const_idx列だけ1)
%        に「厳密に固定」する等式制約を追加。
%        → 定数観測量は「次の時刻でも定数のまま・入力の影響を受けない」
%          という物理的に正しい構造をハードコードし、
%          最適化変数から実質的に外す。
%     3. 安定性LMI（Step1のmatrix_P、Step2のStab_P）は
%        定数方向を除いた (p_theta-1) 次元の動的部分空間
%        A_dyn = A(dyn_idx, dyn_idx) にのみ課す。
%        P も (p_theta-1)×(p_theta-1) の縮約版になる。
%     4. これにより固有値1の方向はLMIの外に置かれるので、
%        「rho_bar<1を要求しつつ実は1が混じっている」という矛盾が解消。
%        最終的なAは「厳密に1の定数モード」＋「rho_bar未満に安定化
%        された25次元の動的モード」という、ユーザーの分析
%        （ctrbfで25、定数方向は不可制御）と整合する構造になる。
%
% 入力
%   X, U, Y    : 状態・入力・次時刻状態のデータ
%   F          : 観測量へのリフト関数 (dx) -> Xlift(:,i)
%
% 出力
%   U_val      : Koopman作用素 U=[A B]（const_idx行は厳密に固定済み）
%   A_val, B_val, P_val : 状態・入力部分、Lyapunov行列（動的部分のみ）
%   result     : 計算結果を格納した構造体
%                （result.const_idx, result.dyn_idx を追加）

    %% 初期値
    rho_bar   = 0.99;
    max_iter  = 30;
    tolerance = 1e-5;

    %% リフト
    remi = round(size(X,2) / 5);
    j = 0;
    for i = 1:size(X,2)
        dx = [X(:,i);U(:,i)];
        dy = [Y(:,i);U(:,i)];
        Xlift(:,i) = F(dx);
        Ylift(:,i) = F(dy);
        if rem(i, remi) == 0
            j = j+1;
            fprintf('convert %d times observables \n', remi*j);
            toc
        end
    end

    psi        = [Xlift ; U];
    Theta_plus = Ylift;

    %% サイズの取得
    [p, q]             = size(psi);
    [p_theta, q_theta] = size(Theta_plus);
    n_u = p - p_theta;

    if q ~= q_theta
        error(['PsiとTheta_plusのデータ数が一致していません。\n' ...
               'Psiの列数          : %d\n' ...
               'Theta_plusの列数   : %d'], q, q_theta);
    end
    if n_u < 0
        error(['Psiの行数がTheta_plusの行数より小さくなっています。\n' ...
               'p = %d, p_theta = %d'], p, p_theta);
    end

    fprintf('====================================\n');
    fprintf('p         = %d\n', p);
    fprintf('p_theta   = %d\n', p_theta);
    fprintf('n_u       = %d\n', n_u);
    fprintf('q         = %d\n', q);
    fprintf('rho_bar   = %.4f\n', rho_bar);
    fprintf('====================================\n');

    %% ★定数観測量（λ=1方向）の自動検出
    const_tol = 1e-8;
    is_const_row = all(abs(Xlift - 1) < const_tol, 2);
    const_idx = find(is_const_row);

    if isempty(const_idx)
        warning(['定数観測量が見つかりませんでした。' ...
                 '安定性LMIは従来通り全次元に課します。']);
        has_const = false;
        const_idx = [];
        dyn_idx   = 1:p_theta;
    else
        if numel(const_idx) > 1
            warning(['定数観測量が複数見つかりました。先頭のみ' ...
                     '固定し、残りは通常の状態として扱います。']);
            const_idx = const_idx(1);
        end
        has_const = true;
        dyn_idx = setdiff(1:p_theta, const_idx);
        fprintf('定数観測量を検出: index = %d（λ=1に固定）\n', const_idx);
        fprintf('動的部分空間の次元 = %d\n', numel(dyn_idx));
    end

    p_theta_dyn = numel(dyn_idx);

    %% G, H, c
    G = (1/q) * Theta_plus * psi';
    H = (1/q) * psi * psi';
    c = (1/q) * trace(Theta_plus * Theta_plus');

    %% H = L*L'を満たすL
    [Usvd, S, ~] = svd(psi, 'econ');
    L = (1/sqrt(q)) * Usvd * S;
    L_error = norm(H - L*L', 'fro');

    fprintf('size(G) = %d × %d\n', size(G,1), size(G,2));
    fprintf('size(H) = %d × %d\n', size(H,1), size(H,2));
    fprintf('size(L) = %d × %d\n', size(L,1), size(L,2));
    fprintf('||H - LL''||_F = %.6e\n\n', L_error);

    %% YALMIP
    yalmip('clear');
    options = sdpsettings('solver', 'sdpt3', 'verbose', 0);
    epsilon = 1e-6;

    %% Pの初期値（★動的部分空間のみのサイズ）
    P_val = eye(p_theta_dyn);

    %% 収束判定用
    U_previous          = [];
    objective_previous  = [];
    objective_history   = nan(max_iter,1);
    U_change_history    = nan(max_iter,1);
    P_change_history    = nan(max_iter,1);
    max_eig_history      = nan(max_iter,1);
    eta_history          = nan(max_iter,1);   % ★元コードで未初期化だったバグを修正

    converged = false;

    %% 交互最適化
    for iter = 1:max_iter

        fprintf('---------- iteration %d ----------\n', iter);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Step 1：Pを固定して U, W, nu を最適化
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        U  = sdpvar(p_theta, p, 'full');
        nu = sdpvar(1,1);
        W  = sdpvar(p_theta, p_theta, 'symmetric');

        A = U(:,1:p_theta);

        Constraints_U = [];

        Constraints_U = [Constraints_U, trace(W) <= nu];
        Constraints_U = [Constraints_U, W >= epsilon * eye(p_theta)];

        matrix_M = [W,       U*L;
                    (U*L)', eye(size(L,2))];
        Constraints_U = [Constraints_U, ...
            matrix_M >= epsilon * eye(size(matrix_M,1))];

        % ★定数観測量の行を厳密に固定
        %   （次時刻でも定数のまま、かつ入力・他状態に依存しない）
        if has_const
            e_const = zeros(1, p);
            e_const(const_idx) = 1;
            Constraints_U = [Constraints_U, ...
                U(const_idx, :) == e_const];
        end

        % ★安定性LMIは動的部分空間 A_dyn = A(dyn_idx,dyn_idx) のみに課す
        A_dyn = A(dyn_idx, dyn_idx);
        matrix_P = [rho_bar * P_val,        A_dyn' * P_val;
                    P_val' * A_dyn,         rho_bar * P_val];
        Constraints_U = [Constraints_U, ...
            matrix_P >= epsilon * eye(2*p_theta_dyn)];

        Objective_U = c - 2*trace(U*G') + nu;

        diagnostics_U = optimize(Constraints_U, Objective_U, options);

        if diagnostics_U.problem ~= 0
            error(['Uの最適化に失敗しました。\n' ...
                   'iteration : %d\n' ...
                   'problem   : %d\n' ...
                   'info      : %s'], ...
                   iter, diagnostics_U.problem, diagnostics_U.info);
        end

        U_val = value(U);
        A_val = U_val(:,1:p_theta);

        if n_u > 0
            B_val = U_val(:,p_theta+1:end);
        else
            B_val = [];
        end

        objective_val = value(Objective_U);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Step 2：Aを固定して P を最適化（★動的部分空間のみ）
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        P   = sdpvar(p_theta_dyn, p_theta_dyn, 'symmetric');
        eta = sdpvar(1,1);

        Constraints_P = [];
        Constraints_P = [Constraints_P, P >= epsilon * eye(p_theta_dyn)];
        Constraints_P = [Constraints_P, trace(P) == p_theta_dyn];

        A_val_dyn = A_val(dyn_idx, dyn_idx);

        Stab_P = [rho_bar * P,        A_val_dyn' * P;
                  P * A_val_dyn,      rho_bar * P];

        Constraints_P = [Constraints_P, Stab_P >= eta * eye(2*p_theta_dyn)];
        Constraints_P = [Constraints_P, Stab_P >= 0];

        diagnostics_P = optimize(Constraints_P, -eta, options);

        if diagnostics_P.problem ~= 0
            error(['Pの最適化に失敗しました。\n' ...
                   'iteration : %d\n' ...
                   'problem   : %d\n' ...
                   'info      : %s'], ...
                   iter, diagnostics_P.problem, diagnostics_P.info);
        end

        P_new = value(P);
        P_new = (P_new + P_new') / 2;
        eta_val = value(eta);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 収束判定
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        if isempty(U_previous)
            U_change = inf;
            objective_change = inf;
        else
            U_change = norm(U_val - U_previous, 'fro') ...
                / max(1, norm(U_previous, 'fro'));
            objective_change = abs(objective_val - objective_previous) ...
                / max(1, abs(objective_previous));
        end

        P_change = norm(P_new - P_val, 'fro') / max(1, norm(P_val, 'fro'));

        eigenvalues  = eig(A_val);
        max_abs_eig  = max(abs(eigenvalues));

        objective_history(iter) = objective_val;
        U_change_history(iter)  = U_change;
        P_change_history(iter)  = P_change;
        max_eig_history(iter)   = max_abs_eig;
        eta_history(iter)       = eta_val;

        fprintf('Objective          = %.8e\n', objective_val);
        fprintf('max|eig(A)|        = %.8f\n', max_abs_eig);
        fprintf('U change           = %.6e\n', U_change);
        fprintf('P change           = %.6e\n', P_change);
        fprintf('objective change   = %.6e\n', objective_change);
        fprintf('stability margin   = %.6e\n\n', eta_val);

        U_previous         = U_val;
        objective_previous = objective_val;
        P_val = P_new;

        if iter >= 2 && U_change < tolerance && objective_change < tolerance
            converged = true;
            fprintf('収束しました。\n');
            fprintf('iteration = %d\n\n', iter);
            break;
        end
    end

    if ~converged
        fprintf(['最大反復回数までに指定した収束条件を' ...
                 '満たしませんでした。\n']);
        fprintf('iteration = %d\n\n', iter);
    end

    %% 最終結果
    eigenvalues = eig(A_val);
    max_abs_eig = max(abs(eigenvalues));

    fprintf('====================================\n');
    fprintf('最終結果\n');
    fprintf('size(U)       = %d × %d\n', size(U_val,1), size(U_val,2));
    fprintf('size(A)       = %d × %d\n', size(A_val,1), size(A_val,2));
    fprintf('size(B)       = %d × %d\n', size(B_val,1), size(B_val,2));
    fprintf('max|eig(A)|   = %.8f\n', max_abs_eig);
    fprintf('iteration     = %d\n', iter);
    fprintf('converged     = %d\n', converged);
    if has_const
        fprintf('定数方向(index=%d)は厳密に固有値1に固定、\n', const_idx);
        fprintf('動的部分(次元=%d)はrho_bar=%.4f未満に安定化\n', ...
            p_theta_dyn, rho_bar);
    end
    fprintf('====================================\n');

    fprintf('\neig(A)\n');
    disp(eigenvalues);

    %% 結果を構造体に保存
    result = struct;

    result.p       = p;
    result.p_theta = p_theta;
    result.n_u     = n_u;
    result.q       = q;

    result.G = G;
    result.H = H;
    result.C = X*pinv(Xlift);
    result.L = L;
    result.L_error = L_error;

    result.U = U_val;
    result.A = A_val;
    result.B = B_val;
    result.P = P_val;              % ★動的部分空間のみのサイズ

    result.has_const = has_const;
    result.const_idx = const_idx;  % ★追加：定数(λ=1)観測量の行番号
    result.dyn_idx   = dyn_idx;    % ★追加：安定化された動的部分空間の行番号
    result.A_dyn     = A_val(dyn_idx, dyn_idx); % ★追加：動的部分のみのA

    result.eigenvalues  = eigenvalues;
    result.max_abs_eig  = max_abs_eig;

    result.iteration = iter;
    result.converged = converged;

    result.objective_history = objective_history(1:iter);
    result.U_change_history  = U_change_history(1:iter);
    result.P_change_history  = P_change_history(1:iter);
    result.max_eig_history   = max_eig_history(1:iter);
    result.eta_history       = eta_history(1:iter);

    %% 目的関数の値をプロット
    valid_idx = ~isnan(objective_history);

    figure;
    plot(find(valid_idx), objective_history(valid_idx), '-o', 'LineWidth', 1.5);
    xlabel('反復回数');
    ylabel('目的関数の値');
    title('目的関数の履歴');
    grid on;
end