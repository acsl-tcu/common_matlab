%　MECの部分，eからΔuまでの近似線形化の部分
%  % e[k+1] = A*e[k] + B*delta_u[k]
%
% E      : e[k]        (26 x N)
% dU     : delta_u[k]  (4 x N)
% E_next : e[k+1]      (26 x N)

function [U_val, A_val, B_val, P_val, result] = rensyuuLYKL_error(E, dU, E_next, flg)


%% 初期値
rho_bar   = 0.99;   %固有値の最大をどこまでにするか
% max_iter  = 30;   %何回繰り返し線形化するか
max_iter = 14;
tolerance = 1e-5;   %

%% リフトが扶養になった
%　元々eはリフトした状態の差だから

psi        = [E ; dU];
Theta_plus = E_next;

%% サイズの取得
[p, q]             = size(psi);
[p_theta, q_theta] = size(Theta_plus);
n_u = p - p_theta;

%%サイズの確認
N = size(E,2);

assert(N > 0, '学習データが空です');

assert(size(dU,2) == N, ...
    'EとdUのデータ数が一致しません');

assert(isequal(size(E),size(E_next)), ...
    'EとE_nextのサイズが一致しません');

assert(all(isfinite([E(:);dU(:);E_next(:)])), ...
    'NaNまたはInfが含まれています');

fprintf('====================================\n');
fprintf('p         = %d\n', p);
fprintf('p_theta   = %d\n', p_theta);
fprintf('n_u       = %d\n', n_u);
fprintf('q         = %d\n', q);
fprintf('rho_bar   = %.4f\n', rho_bar);
fprintf('====================================\n');

dyn_idx = 1:p_theta;
p_theta_dyn = p_theta;


%% 数値状態の確認
fprintf('\n===== Numerical diagnostics =====\n');

fprintf('max|E|      = %.3e\n', max(abs(E(:))));
fprintf('max|dU|     = %.3e\n', max(abs(dU(:))));
fprintf('max|E_next| = %.3e\n', max(abs(E_next(:))));

% 特異値からランクを確認
s = svd(psi, 'econ');
tol_rank = max(size(psi)) * eps(s(1));
rank_psi = sum(s > tol_rank);

fprintf('rank(psi) = %d / %d\n', rank_psi, size(psi,1));
fprintf('max singular value = %.3e\n', s(1));
fprintf('min singular value = %.3e\n', s(end));

% 行ごとの大きさを確認
row_rms = sqrt(mean(psi.^2, 2));

fprintf('max row RMS = %.3e\n', max(row_rms));
fprintf('min row RMS = %.3e\n', min(row_rms));

% ほぼゼロになっている行
zero_idx = find(row_rms < 1e-8 * max(row_rms));
fprintf('Nearly zero rows: ');
disp(zero_idx');


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

%% Pの初期値（動的部分空間のみのサイズ）
P_val = eye(p_theta_dyn);

%% 収束判定用
U_previous          = [];
objective_previous  = [];
objective_history   = nan(max_iter,1);
U_change_history    = nan(max_iter,1);
P_change_history    = nan(max_iter,1);
max_eig_history      = nan(max_iter,1);
eta_history          = nan(max_iter,1);   % 元々未初期化だったバグを修正

converged = false;

% 固有値を記録するiteration
eig_save_iters = [1, 14];
%固有値って，毎回計算し直してるから，どれがどう動いてるかは分からないんだ，多分

% 各iterationの固有値
eigenvalue_history = complex(nan(p_theta, length(eig_save_iters)));

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

    % eigenvalues  = eig(A_val);
    % max_abs_eig  = max(abs(eigenvalues));


    eigenvalues  = eig(A_val);
    max_abs_eig  = max(abs(eigenvalues));

    % 指定したiterationだけ固有値を保存
    save_idx = find(eig_save_iters == iter, 1);

    if ~isempty(save_idx)
        eigenvalue_history(:, save_idx) = eigenvalues;
    end

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

%% 固有値の対応付け
eig_traj = eigenvalue_history;

for j = 2:length(eig_save_iters)

    prev_eig = eig_traj(:, j-1);
    curr_eig = eigenvalue_history(:, j);

    matched_eig = nan(size(curr_eig));
    remaining = 1:length(curr_eig);

    for k = 1:length(prev_eig)

        % 前回の固有値との距離
        [~, idx] = min(abs(curr_eig(remaining) - prev_eig(k)));

        % 最も近い固有値を対応させる
        matched_eig(k) = curr_eig(remaining(idx));

        % 使用済み固有値を除外
        remaining(idx) = [];
    end

    eig_traj(:, j) = matched_eig;
end
%%固有値の変化プロット
for k = 1:p_theta

    % iter = 1 → iter = 30 の軌道
    plot(real(eig_traj(k,:)), ...
        imag(eig_traj(k,:)), ...
        '-', 'LineWidth', 1.5, ...
        'HandleVisibility', 'off');

    % iter = 30 → ○
    plot(real(eig_traj(k,2)), ...
        imag(eig_traj(k,2)), ...
        'o', 'MarkerSize', 6, ...
        'HandleVisibility', 'off');

    % iter = 1 → ×
    plot(real(eig_traj(k,1)), ...
        imag(eig_traj(k,1)), ...
        'x', 'MarkerSize', 9, ...
        'LineWidth', 2, ...
        'HandleVisibility', 'off');

end

xlabel('Real');
ylabel('Imaginary');
title('Eigenvalue trajectories of A');

xline(0, 'k:');
yline(0, 'k:');
%
% % 実際に実行されたiteration数だけ取り出す
% eig_hist = eigenvalue_history(:, 1:iter);
%
% figure;
% hold on;
% grid on;
% axis equal;
%
% % 単位円
% theta = linspace(0, 2*pi, 500);
% plot(cos(theta), sin(theta), 'k--', 'LineWidth', 1.2);
%
% % 各固有値のiterationごとの軌跡
% for k = 1:p_theta
%     plot(real(eig_hist(k,:)), ...
%          imag(eig_hist(k,:)), ...
%          '-o', 'LineWidth', 1.2, 'MarkerSize', 4);
% end
%
% xlabel('Real');
% ylabel('Imaginary');
% title('Eigenvalue trajectories of A');


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
% result.C = X*pinv(Xlift);
% リフトされた状態からxに戻す必要が無くてそのまま誤差状態を出力するから1かな？
result.C = eye(p_theta);
result.L = L;
result.L_error = L_error;

result.U = U_val;
result.A = A_val;
result.B = B_val;
result.P = P_val;              % 動的部分空間のみのサイズ

result.dyn_idx   = dyn_idx;    % 追加：安定化された動的部分空間の行番号
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