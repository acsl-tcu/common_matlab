
function [U_val, A_val, B_val, P_val, result] = ...
    rensyuuKLLY(X,U,Y,F,~)
%SOLVE_KOOPMAN_ALTERNATING
% 交互最適化によって安定性制約付きKoopman作用素 U=[A B] を求める
%
% 入力
%   Psi        : p × q
%                [観測量; 入力]を並べたデータ行列
%
%   Theta_plus : p_theta × q
%                次時刻の観測量データ行列
%
%   rho_bar    : 安定性の上限
%                省略時 0.99
%
%   max_iter   : 最大反復回数
%                省略時 30
%
%   tolerance  : 収束判定値
%                省略時 1e-5
%
% 出力
%   U_val      : Koopman作用素 U=[A B]
%   A_val      : 状態・観測量部分
%   B_val      : 入力部分
%   P_val      : Lyapunov行列
%   result     : 計算結果を格納した構造体

    %% 初期値
    
        rho_bar = 0.99;
        max_iter = 30;
        tolerance = 1e-5;
    
%%

remi = round(size(X,2) / 5);
j = 0;
for i = 1:size(X,2)%1:Data.num
    
        dx = [X(:,i);U(:,i)]; % hermite
        dy = [Y(:,i);U(:,i)];
   
    Xlift(:,i) = F(dx); 
    Ylift(:,i) = F(dy);
    if rem(i, remi) == 0
        j = j+1;
        fprintf('convert %d times observables \n', remi*j);
        toc
    end
end

%%
psi=[Xlift ; U];
Theta_plus = Ylift;

    %% サイズの取得
    %   p       ：観測量+入力の個数，
    %   p_theta ：観測量
    %qを持つものはデータの個数

    [p, q] = size(psi);
    [p_theta, q_theta] = size(Theta_plus);

    % 入力数
    n_u = p - p_theta;

    if q ~= q_theta
        error(['PsiとTheta_plusのデータ数が一致していません。\n' ...
               'Psiの列数          : %d\n' ...
               'Theta_plusの列数   : %d'], ...
               q, q_theta);
    end

    if n_u < 0
        error(['Psiの行数がTheta_plusの行数より小さくなっています。\n' ...
               'p = %d, p_theta = %d'], ...
               p, p_theta);
    end

    fprintf('====================================\n');
    fprintf('p         = %d\n', p);
    fprintf('p_theta   = %d\n', p_theta);
    fprintf('n_u       = %d\n', n_u);
    fprintf('q         = %d\n', q);
    fprintf('rho_bar   = %.4f\n', rho_bar);
    fprintf('====================================\n');

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
    %%%%%%%%%%%%%%%%%%%%ここから最小化問題%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    yalmip('clear');

    options = sdpsettings( ...
        'solver', 'sdpt3', ...
        'verbose', 0);
    %'verbose',0で途中経過を非表示，１で表示

    epsilon = 1e-6;     %Uの変化量がこれより小さくなったら収束とみなす

    %% Pの初期値
    P_val = eye(p_theta);       %Pの最初の初期設定

    %% 収束判定用
    %前回の値を保存する
    U_previous = [];
    objective_previous = [];

%max_iterは何回反復で交互最適化を行うか
    %目的関数の結果の履歴保存
    objective_history = nan(max_iter,1);
    %Uがどれくらい変化したかの履歴保存，ちゃんと収束の方向に動いているか確認    
    U_change_history = nan(max_iter,1);
    %Pの変化量の保存
    P_change_history = nan(max_iter,1);
    %Aの最大固有値の保存，安定性の確認が一旦できる
    max_eig_history = nan(max_iter,1);
    % %角反復で得たηの保存，目的関数とか収束状況との関係確認，いらないかも
    % eta_history = nan(max_iter,1);
    
    %収束したか
    converged = false;

    %% 交互最適化
    %Pを固定してUを求める，Uから得たAからPを求める
    for iter = 1:max_iter

        fprintf('---------- iteration %d ----------\n', iter);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Step 1：Pを固定して U, W, nu を最適化
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        U = sdpvar(p_theta, p, 'full');
        nu = sdpvar(1,1);
        W = sdpvar(p_theta, p_theta, 'symmetric');
        

        %Ｕは（観測量）×（観測量＋入力）のサイズのはず
        A = U(:,1:p_theta);

        %Uを求めるための制約の入れ物
        Constraints_U = [];


        Constraints_U = [Constraints_U, ...
            trace(W) <= nu];

        Constraints_U = [Constraints_U, ...
            W >= epsilon * eye(p_theta)];

        % W >= U*L*L'*U' のSchur補行列
        matrix_M = [W,       U*L;
             (U*L)', eye(size(L,2))];

        Constraints_U = [Constraints_U, ...
            matrix_M >= epsilon * eye(size(matrix_M,1))];

        % P_valはていすうだからAに関してLMIになる
        
        matrix_P = [rho_bar * P_val, A' * P_val;
                  P_val' * A,       rho_bar * P_val];

        Constraints_U = [Constraints_U, ...
            matrix_P >= epsilon * eye(2*p_theta)];

        Objective_U = c - 2*trace(U*G') + nu;

        diagnostics_U = optimize( ...
            Constraints_U, Objective_U, options);

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
        % Step 2：Aを固定して P を最適化
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %   Pはリアプノふ方程式の行列，正定行列である必要あり
        P = sdpvar(p_theta, p_theta, 'symmetric');
        %etaを入れると安定性を半正定値境界ぎりぎり責めなくて良くなる
        %   etaを使うか使わないか問題
        %   eta無し条件を満たすPなら何でもよいので、どれか1個返してください
        %   数値誤差の影響を受けやすいらしい・・・？
        eta = sdpvar(1,1);

        Constraints_P = [];
        
        %   P>0の制約
        Constraints_P = [Constraints_P, ...
            P >= epsilon * eye(p_theta)];

        % Pの定数倍による不定性を除く
        %   最初はeye(26)だからtraceは26になるから，とりあえず，そのままでいいのかなと
        Constraints_P = [Constraints_P, ...
            trace(P) == p_theta];

        % Constraints_P = [Constraints_P, ...
            % eta >= 0];

        % A_valは数値なので、Pに関してLMIになる
        % stab_Pはあのいつもの最後のやつ
        % 2倍の観測量×観測量のサイズになるはず
        Stab_P = [rho_bar * P, A_val' * P;
                  P * A_val,   rho_bar * P];

        % 安定性制約の余裕etaを最大化
        Constraints_P = [Constraints_P, ...
            Stab_P >= eta * eye(2*p_theta)];

        
        % 安定性条件
        Constraints_P = [Constraints_P, ...
                        Stab_P >= 0];


        diagnostics_P = optimize( ...
            Constraints_P, -eta, options);

        if diagnostics_P.problem ~= 0
            error(['Pの最適化に失敗しました。\n' ...
                   'iteration : %d\n' ...
                   'problem   : %d\n' ...
                   'info      : %s'], ...
                   iter, diagnostics_P.problem, diagnostics_P.info);
        end

        P_new = value(P);
        % 数値誤差による非対称成分を除く（これは何？）
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

            objective_change = abs( ...
                objective_val - objective_previous) ...
                / max(1, abs(objective_previous));
        end

        P_change = norm(P_new - P_val, 'fro') ...
            / max(1, norm(P_val, 'fro'));

        eigenvalues = eig(A_val);
        max_abs_eig = max(abs(eigenvalues));

        objective_history(iter) = objective_val;
        U_change_history(iter) = U_change;
        P_change_history(iter) = P_change;
        max_eig_history(iter) = max_abs_eig;
        eta_history(iter) = eta_val;

        fprintf('Objective          = %.8e\n', objective_val);
        fprintf('max|eig(A)|        = %.8f\n', max_abs_eig);
        fprintf('U change           = %.6e\n', U_change);
        fprintf('P change           = %.6e\n', P_change);
        fprintf('objective change   = %.6e\n', objective_change);
        fprintf('stability margin   = %.6e\n\n', eta_val);

        % 次の反復に使用
        U_previous = U_val;
        objective_previous = objective_val;
        P_val = P_new;

        % Uと目的関数が両方ほぼ変化しなくなったら終了
        if iter >= 2 && ...
                U_change < tolerance && ...
                objective_change < tolerance

            converged = true;

            fprintf('収束しました。\n');
            fprintf('iteration = %d\n\n', iter);
            break;
        end
    end

    %% 最大反復回数まで収束しなかった場合
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
    fprintf('size(U)       = %d × %d\n', ...
        size(U_val,1), size(U_val,2));
    fprintf('size(A)       = %d × %d\n', ...
        size(A_val,1), size(A_val,2));
    fprintf('size(B)       = %d × %d\n', ...
        size(B_val,1), size(B_val,2));
    fprintf('max|eig(A)|   = %.8f\n', max_abs_eig);
    fprintf('iteration     = %d\n', iter);
    fprintf('converged     = %d\n', converged);
    fprintf('====================================\n');

    fprintf('\neig(A)\n');
    disp(eigenvalues);

    %% 結果を構造体に保存
    result = struct;

    result.p = p;
    %普通のクープマンで求めてる
    result.p_theta = p_theta;
    result.n_u = n_u;
    result.q = q;

    result.G = G;
    result.H = H;
    % result.c = c;
    result.C = X*pinv(Xlift);
    result.L = L;
    result.L_error = L_error;

    result.U = U_val;
    result.A = A_val;
    result.B = B_val;
    result.P = P_val;

    result.eigenvalues = eigenvalues;
    result.max_abs_eig = max_abs_eig;

    result.iteration = iter;
    result.converged = converged;

    result.objective_history = ...
        objective_history(1:iter);

    result.U_change_history = ...
        U_change_history(1:iter);

    result.P_change_history = ...
        P_change_history(1:iter);

    result.max_eig_history = ...
        max_eig_history(1:iter);

    result.eta_history = ...
        eta_history(1:iter);

    
end