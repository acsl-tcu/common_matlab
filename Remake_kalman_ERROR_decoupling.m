%% Koopmanモデル: カルマン正準分解 -> 可制御部分だけにdlqr

% 固定ゼロ・定数特徴を手動で除去しないで，全状態を分解して，可制御部分のみに対してdlqr


clear;
clc;
if exist('ctrbf', 'file') == 0 || exist('dlqr', 'file') == 0
    error('koopman:MissingControlToolbox', 'Control System Toolboxが必要です。');
end

%% 1. MATファイルの読み込み
loadFileName = strtrim(input('読み込むファイル名(.mat可): ', 's'));
if isempty(loadFileName)
    error('MATファイル名を入力してください。');
end
[~, baseName, extension] = fileparts(loadFileName);
if isempty(extension)
    loadFileName = [loadFileName '.mat'];
elseif ~strcmpi(extension, '.mat')
    error('MATファイルを指定してください。');
end

loaded = load(loadFileName);
if isfield(loaded, 'est')
    est = loaded.est;
elseif all(isfield(loaded, {'A', 'B', 'C'}))
    est = struct('A', loaded.A, 'B', loaded.B, 'C', loaded.C);
else
    error('MATファイルに est または A,B,C がありません。');
end

%% 元の26状態・4入力のモデルを使用

if ~all(isfield(est, {'A_original', 'B_original'}))
    error('estにA_originalまたはB_originalがありません。');
end

A = double(est.A_original);
B = double(est.B_original);

n = size(A, 1);
m = size(B, 2);

% 全誤差状態を出力として扱う
C = eye(n);

%% サイズ確認
fprintf('\n===== モデルのサイズ =====\n');
fprintf('A : %d x %d\n', size(A));
fprintf('B : %d x %d\n', size(B));
fprintf('C : %d x %d\n', size(C));

assert(n == 26 && m == 4, ...
    '元の26状態・4入力に復元されていません');
%%
if n == 0 || size(A, 2) ~= n || size(B, 1) ~= n || ...
        size(C, 2) ~= n || m == 0
    error('Aは正方行列、Bはn行m列、Cはn列である必要があります。');
end
if ~isreal(A) || ~isreal(B) || ~isreal(C) || ...
        any(~isfinite(A(:))) || any(~isfinite(B(:))) || any(~isfinite(C(:)))
    error('A,B,Cは有限の実数行列で指定してください。');
end



%% 2. 状態重み行列 Q の設定

% ===== 各状態の重みを設定 =====

weight_P = 100;    % P1, P2, P3（位置）
weight_Q = 1;     % Q1, Q2, Q3
weight_V = 1;   % V1, V2, V3（速度）
weight_W = 1;    % W1, W2, W3（角速度）
weight_R = 1;     % R13, R23, R33
weight_other = 1; % 16～26番目の状態

% ===== 26状態の重みを設定 =====
q = weight_other * ones(n, 1);
q(1:3)   = weight_P;  % P1, P2, P3
q(4:6)   = weight_Q;  % Q1, Q2, Q3
q(7:9)   = weight_V;  % V1, V2, V3
q(10:12) = weight_W;  % W1, W2, W3
q(13:15) = weight_R;  % R13, R23, R33
% ===== 状態重み行列 =====

Qfull = diag(q);

% ===== 入力重み行列 =====

R =  eye(m);

%=================以下元のコード=====================
% % ctrbfのtolは行列の大きさに合わせて設定する必要あり
% % 推定モデルのスケールや誤差に応じてrelTolを調整する
% % この閾値以下の弱い可制御性に対しては、数値的に不可制御と判定される
% relTol = 1e-10;
% tol = relTol * max(1, norm([A, B], 1));
% % 元の状態座標での重みで重み変えたかったらここで
% Qfull = eye(n);
% 
% if m == 4
%     R = 1000*diag([1, 1, 1, 1]);
% else
%     R = eye(m);
% end
% 2. 重みと可制御性判定の許容誤差
%===========================================================

%% 可制御性判定の許容誤差

relTol = 1e-12;

% A,Bの大きさからスケールを取得
modelScale = max(norm(A,1), norm(B,1));

% 相対許容誤差から絶対許容誤差を計算
tol = relTol * max(modelScale, realmin);

fprintf('モデルスケール: %.3e\n', modelScale);
fprintf('可制御性判定tol: %.3e\n', tol);

%% 3. 可制御性に関するカルマン正準分解
% 可観測性による4分類って，結局可制御部分しか使ってないなら，
% LQR設計には不要な気がして，分類してない
[F, G, H, T, kSteps] = ctrbf(A, B, C, tol);
% [F, G, H, T, kSteps] = ctrbf(A, B, C);
nCont = sum(kSteps);
fprintf('既定閾値での可制御次元: %d / %d\n', nCont, n);

S = T';
T_inv = S;
nCont = sum(kSteps);
nUncont = n - nCont;
ucIdx = 1:nUncont;
cIdx = (nUncont + 1):n;

fprintf('\n可制御次元: %d / %d\n', nCont, n);
fprintf('不可制御次元: %d / %d\n', nUncont, n);
fprintf('ctrbfの許容誤差: %.3e\n', tol);
if nCont == 0
    error('koopman:NoControllableStates', ...
        '可制御部分が存在しないためdlqrを実行できません。');
end

Ac = F(cIdx, cIdx);
Bc = G(cIdx, :);
Auc = F(ucIdx, ucIdx);
Tc = T(cIdx, :);
residuals = struct( ...
    'uncontrollableInput', norm(G(ucIdx, :), 'fro'), ...
    'controllableToUncontrollable', norm(F(ucIdx, cIdx), 'fro'), ...
    'stateTransform', norm(F - T*A*S, 'fro'));
fprintf('||G(不可制御行,:)||F = %.3e\n', residuals.uncontrollableInput);
fprintf('||F(不可制御行,可制御列)||F = %.3e\n', ...
    residuals.controllableToUncontrollable);

%% 4. 可制御部分だけにdlqrを適用

Qc = Tc * Qfull * Tc';
Qc = (Qc + Qc') / 2;
[Kc, Pc, eigControlledClosed] = dlqr(Ac, Bc, Qc, R);
K_dyn = Kc;  % 可制御座標のゲインとして保存する互換名

% u=-Kc*z_c, z_c=Tc*x より元座標のゲインを復元する。
K_all = [zeros(m, nUncont), Kc];
K_full = K_all * T;  % = Kc*Tc

eigUncontrolled = eig(Auc);
eigClosed = eig(A - B*K_full);
fprintf('\n可制御部分の閉ループ最大固有値絶対値: %.6g\n', ...
    max(abs(eigControlledClosed)));
fprintf('全状態の閉ループ最大固有値絶対値: %.6g\n', ...
    max(abs(eigClosed)));
if ~isempty(eigUncontrolled)
    fprintf('不可制御部分の固有値');
    disp(eigUncontrolled);
    if any(abs(eigUncontrolled) >= 1)
        warning('koopman:UncontrollableModes', ...
            ['不可制御部分に|lambda|>=1のモードがあります。' ...
             '可制御部分のdlqrでは全状態を漸近安定化できません。']);
    end
end
% z_ucが非零の場合は、Acの状態方程式にF(cIdx,ucIdx)*z_ucが入る。
% このゲインは可制御部分のLQR解であり、全系のLQR最適解ではない。

%% 5. ゲインと分解結果の保存
saveFolder = fullfile(pwd, 'kalman_gainたち');
if ~exist(saveFolder, 'dir')
    mkdir(saveFolder);
end
timeStamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
saveFileName = fullfile(saveFolder, ...
    [baseName '_gain_controllable_dlqr_Q1_R1000_' timeStamp '.mat']);
save(saveFileName, ...
    'A', 'B', 'C', 'F', 'G', 'H', 'T', 'S', 'T_inv', 'kSteps', ...
    'nCont', 'nUncont', 'cIdx', 'ucIdx', 'Ac', 'Bc', 'Auc', ...
    'Qfull', 'Qc', 'R', 'relTol', 'tol', 'residuals', ...
    'Kc', 'K_dyn', 'K_all', 'K_full', 'Pc', ...
    'eigControlledClosed', 'eigUncontrolled', 'eigClosed');
fprintf('\nゲインと分解結果を保存しました。\n%s\n', saveFileName);


%% 6. 固有値プロット

% A行列の固有値
eigA = eig(A);

% 閉ループ行列 A-BK_full の固有値
eigABK = eigClosed;


theta = linspace(0, 2*pi, 500);
unitX = cos(theta);
unitY = sin(theta);

%% 6-1. A行列の固有値分布
figure('Name', 'Eigenvalues of A', 'Color', 'w');

hold on;
h1 = plot(unitX, unitY, 'k--', 'LineWidth', 1.2);
h2 = plot(real(eigA), imag(eigA), 'bx', ...
    'MarkerSize', 8, 'LineWidth', 1.5);

xline(0, 'k:');
yline(0, 'k:');

axis equal;
grid on;
xlabel('Real Part');
ylabel('Imaginary Part');
title('Eigenvalues of A');
legend([h2, h1], {'Eigenvalues', 'Unit Circle'}, ...
    'Location', 'best');
hold off;

%% 6-2. A-BK_full行列の固有値分布
figure('Name', 'Eigenvalues of A-BK', 'Color', 'w');

hold on;
h1 = plot(unitX, unitY, 'k--', 'LineWidth', 1.2);
h2 = plot(real(eigABK), imag(eigABK), 'rx', ...
    'MarkerSize', 8, 'LineWidth', 1.5);

xline(0, 'k:');
yline(0, 'k:');

axis equal;
grid on;
xlabel('Real Part');
ylabel('Imaginary Part');
title('Eigenvalues of A-BK_{full}');
legend([h2, h1], {'Eigenvalues', 'Unit Circle'}, ...
    'Location', 'best');
hold off;
