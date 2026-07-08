function output = KL(X,U,Y,F,flg)
%KL クープマン線形化によって線形アフィン系状態方程式の係数行列ABCを求める
%   output = KoopmanLinear(X,U,Y)
%   outuput.A .B  観測量空間における線形アフィン系の係数行列 Z[k+1] = A*Z[k]+Bu[k]
%          .C     観測量空間から状態空間に送る線形状態方程式の係数行列 X[k] = C*Z[k]
%   X, U, Y       観測する状態Xに入力Uを与えた際の出力Yを集めたデータセット
%                 列：データ, 行：時系列
%   F             観測量 関数ハンドル
tic
%Xlift,Yliftを計算する
remi = round(size(X,2) / 5); j = 0;
for i = 1:size(X,2)%1:Data.num
    % if flg.hermite
        dx = [X(:,i);U(:,i)]; % hermite
        dy = [Y(:,i);U(:,i)];
    % else
        % dx = X(:,i); % ふつう
        % dy = Y(:,i);
    % end
    Xlift(:,i) = F(dx); 
    Ylift(:,i) = F(dy);
    if rem(i, remi) == 0
        j = j+1;
        fprintf('convert %d times observables \n', remi*j);
        toc
    end
end

numX = size(Xlift, 1); %[numX, ~]=size(Xlift): Xliftのサイズ=(A行,B列)のとき，A行の値をnumXに入れ，B列の値は使わない(~:notの意味)
numU = size(U);

% %ABをまとめて計算する 参考資料記載のやりかた
% M = Ylift * pinv([Xlift; U]);
% A = M(1 : numX, 1 : numX);
% B = M(1 : numX, numX + 1:numX + numU);
% C = X*pinv(Xlift);

%% A,Bをまとめて計算するデータ数が多い場合のやりかた
if flg.weight 
    % 磯部とエルミートで重みわける code23
    % Q_isobe = blkdiag(flg.weight_Qisobe, eye(26-12));
    % Q_hermite_k1_k2 = eye(256); % 状態以外の観測量部分は1とする
    % Q_hermite_k3_k4 = eye(256) * flg.weight_Qhermite;
    % Q_hermite = blkdiag(Q_hermite_k1_k2, Q_hermite_k3_k4);
    % Q = blkdiag(Q_isobe, Q_hermite);

    Q = blkdiag(flg.weight_Qisobe, eye(26-12)); % 00
    % Q = blkdiag(flg.weight_Qisobe, eye(36-12)); % 02
    % Q = blkdiag(flg.weight_Qisobe(4:end,4:end), eye(23-9)); % 10

    % code22
    % Q = blkdiag(flg.weight_Qisobe, eye(4), eye(256), eye(256)*flg.weight_Qhermite);

    % code26, code27
    % Q = blkdiag(flg.weight_Qisobe, eye(14), eye(256)*flg.weight_Qhermite);
    
    % code28
    % Q = blkdiag(flg.weight_Qisobe, eye(14), eye(32)*flg.weight_Qhermite);

    % 磯部のうち、[回転行列,1]以外は重み
    % Q = blkdiag(flg.weight_Qisobe, eye(4), eye(10)*1.00001);

    % Q = blkdiag(flg.weight_Qp, flg.weight_Qq, flg.weight_Qv, flg.weight_Qw, eye(numX-12)); 
    % 汎用性のためにflgにweightを格納
    %%%%%これまでの先輩の方法%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % G = [Xlift ; U]*[Xlift ; U]'; fprintf('finished get G\n');           % size(G) = (numX+numU, numX+numU)
    % V = (Q*Ylift)*[(Q*Xlift) ; U]'; fprintf('finished get V\n');         % size(V) = (numX,      numX+numU)
    % M = V * pinv(G); fprintf('finished get V*pinv(G) \n');               % size(M) = (numX,      numX+numU)
    % output.A = M(1:numX, 1:numX); fprintf('finished get A \n');          % size(.A) = (numX, numX)
    % output.B = M(1:numX, numX+1:numX+numU); fprintf('finished get B \n');% size(.B) = (numX, numU)
    % if flg.without_pos
    %     output.C = (Q(1:9,1:9)*X)*pinv(Q*Xlift); fprintf('finished get C\n');
    % else
    %     output.C = (Q(1:12,1:12)*X)*pinv(Q*Xlift); fprintf('finished get C\n');
    % end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%リアプノフ入れてみる方法%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% パラメータ・既知の行列の定義（具体的なサイズや値は、お使いのシステムに合わせて調整してください）
    
    %GLHcを求めるのに必要な物
    %p=Pv+pv
    psi = [Xlift ; U];
    theta_plus = [Ylift];

    [p, q] = size(Psi); %26+4で30×得たデータ数になるはず
    [p_theta, q_theta] = size(Theta_plus); %Yliftのサイズ

    % 既知の定数行列GLHcを求める
    G = (1/q) * Theta_plus * Psi';
    H = (1/q) * Psi * Psi';
    c = (1/q) * trace(Theta_plus * Theta_plus');
    
    
    % 最大反復回数と収束判定の閾値
    max_iter = 50;
    tol = 1e-5;
    
    %% 1. 初期化
    % 初期値として P = I (単位行列) を設定
    P_val = eye(nx); 
    
    % 収束チェック用のコスト関数の履歴
    cost_history = zeros(max_iter, 1);
    
    fprintf('--- 交互最適化の開始 ---\n');
    
    for iter = 1:max_iter
        
        %% ==========================================
        %% ステップ1: P を固定し、U, nu_var, W を最適化 (LMI問題)
        %% ==========================================
        
        % YALMIPの変数をクリア・定義
        yalmip('clear');
        
        A = sdpvar(nx, nx, 'full');
        B = sdpvar(nx, nu, 'full');
        W = sdpvar(nx, nx, 'symmetric');
        nu_var = sdpvar(1, 1); % 画像の \nu 
        
        % U = [A, B] の構築
        U = [A, B];
        
        % 制約条件のリスト
        Constraints = [];
        
        % tr(W) < nu
        Constraints = [Constraints, trace(W) < nu_var];
        
        % W > 0
        Constraints = [Constraints, W >= 1e-6 * eye(nx)];
        
        % [W, U*L; L'*U', I] > 0
        Matrix1 = [W, U*L; (U*L)', eye(nx)]; 
        Constraints = [Constraints, Matrix1 >= 1e-6 * eye(2*nx)];
        
        % [rho_bar*P, A'*P; P*A, rho_bar*P] > 0  (Pは定数 P_val として扱う)
        % ※画像中の A'*P は転置の関係から、ブロック行列が対称になるよう配置します
        Matrix2 = [rho_bar * P_val, A' * P_val; ...
                   P_val * A,       rho_bar * P_val];
        Constraints = [Constraints, Matrix2 >= 1e-6 * eye(2*nx)];
        
        % 目的関数: c - 2*tr(U*G') + nu
        Objective = c - 2 * trace(U * G') + nu_var;
        
        % ソルバーの設定 (SDPT3を指定)
        options = sdpsettings('solver', 'sdpt3', 'verbose', 0);
        
        % 最適化の実行
        diagnostics1 = optimize(Constraints, Objective, options);
        
        if diagnostics1.problem ~= 0
            error('ステップ1のLMIが解けませんでした。履歴を確認してください。');
        end
        
        % 最適化された値の抽出
        A_val = value(A);
        B_val = value(B);
        U_val = [A_val, B_val];
        W_val = value(W);
        nu_val = value(nu_var);
        current_cost = value(Objective);
        
        cost_history(iter) = current_cost;
        fprintf('Iter %d: Cost = %.6f\n', iter, current_cost);
        
        % 収束判定 (コスト関数の変化が少なくなったら終了)
        if iter > 1 && abs(cost_history(iter) - cost_history(iter-1)) < tol
            fprintf('コスト関数が収束しました。\n');
            break;
        end
        
        %% ==========================================
        %% ステップ2: U (A, B) を固定し、P の生存（実現可能性）を確認/更新
        %% ==========================================
        
        yalmip('clear');
        
        % P を変数として再定義
        P = sdpvar(nx, nx, 'symmetric');
        
        Constraints = [];
        
        % P > 0
        Constraints = [Constraints, P >= 1e-6 * eye(nx)];
        
        % [rho_bar*P, A'*P; P*A, rho_bar*P] > 0 (Aは先ほど求めた A_val として扱う)
        Matrix2_P = [rho_bar * P, A_val' * P; ...
                     P * A_val,   rho_bar * P];
        Constraints = [Constraints, Matrix2_P >= 1e-6 * eye(2*nx)];
        
        % ここは実現可能性問題（Feasibility problem）なので、目的関数は空にするかダミー
        Objective_P = 0; 
        
        diagnostics2 = optimize(Constraints, Objective_P, options);
        
        if diagnostics2.problem ~= 0
            warning('ステップ2のPに関するFeasibility問題が解けませんでした。最適化を中断します。');
            break;
        end
        
        % Pの値を更新して次のループへ
        P_val = value(P);
    
    end
    
    %% 最終結果の表示
    fprintf('\n--- 最適化完了 ---\n');
    disp('最良の A:'); disp(A_val);
    disp('最良の B:'); disp(B_val);
    disp('最良の P:'); disp(P_val);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


else
    G = [Xlift ; U]*[Xlift ; U]'; % size(G) = (numX+numU, numX+numU)
    V = Ylift*[Xlift ; U]';       % size(V) = (numX,      numX+numU)
    M = V * pinv(G);              % size(M) = (numX,      numX+numU)
    output.A = M(1:numX, 1:numX); % size(.A) = (numX, numX)
    output.B = M(1:numX, numX+1:numX+numU); % size(.B) = (numX, numU)
    output.C = X*pinv(Xlift); % C: Z->X の厳密な求め方 pinv: Moore-Penrose疑似逆行列  size(.C) = (size(X), numX)
end
toc