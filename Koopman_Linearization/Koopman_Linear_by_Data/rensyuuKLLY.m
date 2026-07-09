clear;clc;

p = 8;          % Psiの行数(観測量+入力の数)
p_theta = 6;    % Theta_plusの行数(観測量の数)
q = 100;        % データ数

Psi = randn(p, q);
Theta_plus = randn(p_theta, q);

%%
G = (1/q) * Theta_plus * Psi';
H = (1/q) * Psi * Psi';
% c = (1/q) * Theta_plus * Theta_plus';
c = (1/q) * trace(Theta_plus * Theta_plus');

disp('G');
disp(size(G));
disp('H');
disp(size(H));
disp('c');
disp(c);

%%
% 特異値分解して同じだけど？小さいサイズのLを得ることができる
%psi=USV^Tと置くと，psipsi^T=US(US)^Tになり，psi=USを得る．
% L=1/\sqrt(q) USになる
%だからvは不要だから以下の特異値分解ではU,S,~になってる

[U,S,~] = svd(Psi, 'econ');     %econはエコノミーサイズで分解．分解制度を損なわずにできるらしい
L = (1/sqrt(q)) * U * S;

% L = (1/sqrt(q)) * Psi;    %文章通りに作るならこっち，ただ(観測量＋入力)×(データ数)行列になる
disp('L')
disp(size(L))

check_L=norm(H - L*L', 'fro');

fprintf('Lが適しているか');
disp(check_L);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
yalmip('clear');
%Uは求めたいクープマン作用素U=[A B]=観測量X(観測量+入力)行列
%WはLMIの補助関数
%nu_varは||UL||_F^2の上限を示す補助関数

%sdpvarはこの変数を最適化問題の未知関数として扱ってねって関数
%sdpvar(行数，列数，行列の種類)

U = sdpvar(p_theta, p, 'full');
W = sdpvar(p_theta, p_theta, 'symmetric');
nu = sdpvar(1,1);

%%
%制約をいれるリスト作り
Constraints = [];

Constraints = [Constraints, trace(W) <= nu];
Constraints = [Constraints, W >= 1e-6*eye(p_theta)];

M = [W, U*L;
     (U*L)', eye(size(L,2))];

Constraints = [Constraints, M >= 1e-6*eye(size(M,1))];

%%

Objective = c - 2*trace(U*G') + nu;

%ソルバーはsdpt3を使って，計算中の情報をコマンドウィンドウに表示する設定
options = sdpsettings('solver','sdpt3','verbose',0);
%constraintsを満たす範囲でobjectiveを最小にするそれぞれを求める
diagnostics = optimize(Constraints, Objective, options);

% options = sdpsettings('verbose',1);
% diagnostics = optimize(Constraints, Objective, options);

disp(diagnostics.problem)
disp(diagnostics.info)

U_val = value(U);
disp(size(U_val))