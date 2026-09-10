%% Koopmanモデル用 カルマン正準分解 + LQRゲイン計算

clear;
clc;

%% 1. MATファイルの読み込み
loadFileName = input('読み込むファイル名(.mat可): ', 's');
if isempty(regexp(loadFileName, '\.mat$', 'once'))
    loadFileName = [loadFileName '.mat'];
end

loaded = load(loadFileName);
if isfield(loaded, 'est')
    est = loaded.est;
elseif all(isfield(loaded, {'A','B','C'}))
    est.A = loaded.A;
    est.B = loaded.B;
    est.C = loaded.C;
else
    error('MATファイルに est または A,B,C がありません。');
end

A = est.A;
B = est.B;
C = est.C;
n = size(A, 1);
m = size(B, 2);

if size(A,2) ~= n
    error('Aは正方行列でなければなりません。');
end
if size(B,1) ~= n || size(C,2) ~= n
    error('A,B,Cの状態次元が一致していません。');
end

%% 2. 設定
% 定数特徴1の添字を、A,Bの行構造から自動検出する。
% 定数特徴の行は、理想的には A(i,:)=e_i' かつ B(i,:)=0 である。
% 推定誤差を考慮し、以下は相対残差の許容値とする。
constantStructureTol = 1e-6;
[const_idx, constDetect] = local_find_constant_idx(A, B, constantStructureTol);
fprintf('自動検出した定数特徴の状態添字 const_idx = %d\n', const_idx);
if isfield(est, 'const_idx') && est.const_idx ~= const_idx
    warning('est.const_idx=%d と自動検出結果=%d が一致しません。', ...
            est.const_idx, const_idx);
end

if numel(const_idx) ~= 1 || const_idx < 1 || const_idx > n || const_idx ~= round(const_idx)
    error('const_idxは1個の有効な整数添字で指定してください。');
end
const_idx = double(const_idx);
dyn_idx = setdiff(1:n, const_idx, 'stable');
nd = numel(dyn_idx);

% 推定モデルに対する相対特異値閾値。必要なら1e-6等に変更する。
relTol = 1e-10;

% LQR重み。必要ならここを変更する。
Qfull = eye(n);
if m == 4
    R = diag([1; 0.1; 0.1; 1]);
else
    R = eye(m);
end

%% 3. 定数特徴の構造チェック
econst = zeros(1,n);
econst(const_idx) = 1;
constRowResidual = norm(A(const_idx,:) - econst, 'fro');
constInputResidual = norm(B(const_idx,:), 'fro');

fprintf('\n===== 定数特徴の構造チェック =====\n');
fprintf('const_idx = %d\n', const_idx);
fprintf('||A(const_idx,:)-e_const|| = %.3e\n', constRowResidual);
fprintf('||B(const_idx,:)||         = %.3e\n', constInputResidual);
fprintf(['この条件はx_const(k+1)=x_const(k)かつ、入力が定数特徴を直接変化させないことを表す。\n' ...
         'A(dyn_idx,const_idx)が非零でも構造としては成立する。\n']);

if constRowResidual > constantStructureTol || constInputResidual > constantStructureTol
    warning('定数特徴の構造残差が大きいです。const_idxまたは推定モデルを確認してください。');
end

Ad  = A(dyn_idx, dyn_idx);
Bd  = B(dyn_idx, :);
Cd  = C(:, dyn_idx);
Adc = A(dyn_idx, const_idx);

rhoAd = max(abs(eig(Ad)));
if rcond(eye(nd)-Ad) < 100*eps
    error('A_dynに固有値1が近接しているため、定数モードを分離できません。');
end

fprintf('rho(A_dyn)       = %.6g\n', rhoAd);
fprintf('rcond(I-A_dyn)   = %.3e\n', rcond(eye(nd)-Ad));

%% 4. dynamic部分のカルマン4分解
% full 26次元のctrbでは、定数モードが数値誤差で可制御に見えることがある。
% そこで定数特徴を除いたdynamic部分で部分空間を計算する。
Mcd = ctrb(Ad, Bd);
Mod = obsv(Ad, Cd);

[Uc, svMc, rankMc] = local_range_svd(Mcd, relTol);
[N,  svMo, rankMo] = local_null_svd(Mod, relTol);

% Xa: 可制御∩不可観測
coeffXa = local_null_svd(Mod*Uc, relTol);
Xa_d = local_range_svd(Uc*coeffXa, relTol);

% Xb: 可制御だが不可観測ではない
Xb_d = local_range_svd(Uc - Xa_d*(Xa_d'*Uc), relTol);

% Xd: 不可観測だが可制御ではない
Xd_d = local_range_svd(N - Xa_d*(Xa_d'*N), relTol);

% Xc: 可制御部分空間と不可観測部分空間の和の補空間
Wsum = local_range_svd([Uc,N], relTol);
Xc_d = local_null_svd(Wsum', relTol);

fprintf('\n===== dynamic部分のカルマン分解 =====\n');
fprintf('rank(ctrb(A_dyn,B_dyn)) = %d / %d\n', rankMc, nd);
fprintf('rank(obsv(A_dyn,C_dyn)) = %d / %d\n', rankMo, nd);
fprintf('Xa: 可制御・不可観測 = %d\n', size(Xa_d,2));
fprintf('Xb: 可制御・可観測   = %d\n', size(Xb_d,2));
fprintf('Xc: 不可制御・可観測 = %d\n', size(Xc_d,2));
fprintf('Xd: 不可制御・不可観測 = %d\n', size(Xd_d,2));

%% 5. 固有値1の真の右固有ベクトルを追加
% A=[Ad Adc; 0 1] なので、v=[vd;1]についてAv=vとなるvdは
% (I-Ad)*vd=Adcを満たす。
vd = (eye(nd)-Ad) \ Adc;
v = zeros(n,1);
v(dyn_idx) = vd;
v(const_idx) = 1;
v = v/norm(v);

eigResidual = norm(A*v-v)/max(1,norm(v));
modeOutput = norm(C*v)/max(1,norm(C,'fro'));
modeIsObservable = modeOutput > relTol;

fprintf('\n===== 固有値1モード =====\n');
fprintf('||A*v-v||/||v||        = %.3e\n', eigResidual);
fprintf('||C*v||/max(1,||C||F)  = %.3e\n', modeOutput);

embed = @(V) local_embed(V, dyn_idx, n);
Xa = embed(Xa_d);
Xb = embed(Xb_d);
Xc = embed(Xc_d);
Xd = embed(Xd_d);

if modeIsObservable
    fprintf('固有値1モードは不可制御・可観測なのでXcへ追加します。\n');
    Xc = [Xc, v];
else
    fprintf('固有値1モードは不可制御・不可観測なのでXdへ追加します。\n');
    Xd = [Xd, v];
end

%% 6. 変換行列とカルマン分解の検証
% x=S*z, z=T*x
S = [Xa, Xb, Xc, Xd];
if size(S,2) ~= n
    error('分解基底の総列数がnと一致しません: %d vs %d', size(S,2), n);
end
if rcond(S) < 100*eps
    error('分解基底Sが数値的に特異です。relTolを確認してください。');
end

T = S \ eye(n);
F = T*A*S;
G = T*B;
H = C*S;

na = size(Xa,2);
nb = size(Xb,2);
nc = size(Xc,2);
nd4 = size(Xd,2);
k = na + nb;

ia = 1:na;
ib = na + (1:nb);
ic = na + nb + (1:nc);
id = na + nb + nc + (1:nd4);

if isempty(ia), ia = zeros(1,0); end
if isempty(ib), ib = zeros(1,0); end
if isempty(ic), ic = zeros(1,0); end
if isempty(id), id = zeros(1,0); end

% [Xa Xb Xd Xc]の順では、Aはブロック上三角になる。
ord = [ia, ib, id, ic];
Ford = F(ord,ord);
blockLowerResidual = local_block_lower_residual(Ford, [na nb nd4 nc]);

uncRows = [ic,id];
unobsCols = [ia,id];
if isempty(uncRows)
    inputResidual = 0;
else
    inputResidual = norm(G(uncRows,:), 'fro');
end
if isempty(unobsCols)
    outputResidual = 0;
else
    outputResidual = norm(H(:,unobsCols), 'fro');
end

fprintf('\n===== カルマン分解の検証 =====\n');
fprintf('rank(S)                         = %d / %d\n', rank(S), n);
fprintf('Aのブロック下三角残差           = %.3e\n', blockLowerResidual);
fprintf('G(不可制御行,:)の残差           = %.3e\n', inputResidual);
fprintf('H(:,不可観測列)の残差           = %.3e\n', outputResidual);

if blockLowerResidual > 100*relTol || inputResidual > 100*relTol || outputResidual > 100*relTol
    warning('カルマン分解の残差が大きいです。relTol、const_idx、推定誤差を確認してください。');
end

%% 7. 各カテゴリの固有値と安定性をplot
eigs_a = local_block_eigs(F,ia);
eigs_b = local_block_eigs(F,ib);
eigs_c = local_block_eigs(F,ic);
eigs_d = local_block_eigs(F,id);

fprintf('\n===== 各カテゴリの固有値 =====\n');
local_print_spectrum('Xa 可制御・不可観測', eigs_a);
local_print_spectrum('Xb 可制御・可観測', eigs_b);
local_print_spectrum('Xc 不可制御・可観測', eigs_c);
local_print_spectrum('Xd 不可制御・不可観測', eigs_d);

figure('Color','w','Name','Kalman canonical decomposition');
hold on; grid on; axis equal;
theta = linspace(0,2*pi,400);
plot(cos(theta),sin(theta),'k:','HandleVisibility','off');
local_plot_category(eigs_a,[1 0 0],'o','Xa: 可制御・不可観測');
local_plot_category(eigs_b,[0 0 1],'s','Xb: 可制御・可観測');
local_plot_category(eigs_c,[0 0.6 0],'d','Xc: 不可制御・可観測');
local_plot_category(eigs_d,[0.7 0 0.7],'x','Xd: 不可制御・不可観測');
xlabel('実部');
ylabel('虚部');
title(sprintf('カルマン正準分解 (%d状態)',n));
legend('Location','northeastoutside');

allEig = [eigs_a; eigs_b; eigs_c; eigs_d];
if ~isempty(allEig)
    lim = 1.1*max([1.2; abs(allEig)]);
    xlim([-lim lim]);
    ylim([-lim lim]);
end

%% 8. 可制御部分だけにLQRを設計
if k == 0
    error('可制御部分が存在しないためLQRを設計できません。');
end

Ac = F(1:k,1:k);
Bc = G(1:k,:);

% x=S*zなので、変換後重みはQz=S'*Qfull*S。
Qz = S'*Qfull*S;
Qc = Qz(1:k,1:k);
Qc = (Qc+Qc')/2;
R = (R+R')/2;

Kc = dlqr(Ac,Bc,Qc,R);
K_dyn = Kc;

% u=-Kc*z_c, z=T*x より、元座標のゲインは以下。
K_all = [Kc, zeros(m,n-k)];
K_full = K_all*T;

Acl = A-B*K_full;
eigClosed = eig(Acl);
eigControlledClosed = eig(Ac-Bc*Kc);
feedbackOnOneMode = norm(K_full*v);

fprintf('\n===== LQR結果 =====\n');
fprintf('可制御次元 k = %d / %d\n', k, n);
fprintf('max|eig(Ac-Bc*Kc)| = %.6g\n', max(abs(eigControlledClosed)));
fprintf('max|eig(A-B*K_full)| = %.6g\n', max(abs(eigClosed)));
fprintf('||K_full*v_(lambda=1)|| = %.3e\n', feedbackOnOneMode);
fprintf('注意: 不可制御の固有値1は閉ループにも残ります。\n');

%% 9. 保存
[~,baseName,~] = fileparts(loadFileName);
saveFolder = fullfile(pwd,'kalman_gainたち');
if ~exist(saveFolder,'dir')
    mkdir(saveFolder);
end

saveFileName = fullfile(saveFolder,[baseName '_gain_KCD_LQR_check.mat']);

% 既存コードとの互換性のため、SをT_invという名前でも保存する。
T_inv = S;
residuals = struct(...
    'constantRow',constRowResidual,...
    'constantInput',constInputResidual,...
    'lambda1Eigenvector',eigResidual,...
    'blockLower',blockLowerResidual,...
    'uncontrollableInput',inputResidual,...
    'unobservableOutput',outputResidual,...
    'feedbackOnLambda1',feedbackOnOneMode);
dims = [na nb nc nd4];

save(saveFileName, ...
    'K_full','K_dyn','Kc','K_all', ...
    'A','B','C','Ad','Bd','Cd','Ac','Bc','Qfull','Qc','R', ...
    'S','T','T_inv','F','G','H', ...
    'Xa','Xb','Xc','Xd','v','vd', ...
    'const_idx','dyn_idx','dims','relTol', ...
    'constDetect','constantStructureTol', ...
    'eigs_a','eigs_b','eigs_c','eigs_d', ...
    'eigClosed','eigControlledClosed','residuals', ...
    'rankMc','rankMo','svMc','svMo');

fprintf('\nゲインと分解結果を保存しました。\n%s\n',saveFileName);

%% ===== 以下はスクリプトから呼び出す補助関数 =====

function [idx, info] = local_find_constant_idx(A, B, tol)
% A(i,:)が単位行列のi行、かつB(i,:)が零に近い状態を探す。
% rank(ctrb(A,B))は使用しない。推定誤差で定数モードが可制御に
% 見える場合でも、定数特徴の座標行そのものを検出できる。

    n = size(A,1);
    scaleA = max(1, norm(A,'fro'));
    scaleB = max(1, norm(B,'fro'));
    rowResidualA = zeros(n,1);
    rowResidualB = zeros(n,1);

    for i = 1:n
        ei = zeros(1,n);
        ei(i) = 1;
        rowResidualA(i) = norm(A(i,:) - ei) / scaleA;
        rowResidualB(i) = norm(B(i,:)) / scaleB;
    end

    score = max(rowResidualA, rowResidualB);
    [bestScore, idx] = min(score);
    candidates = find(rowResidualA <= tol & rowResidualB <= tol);

    fprintf('\n===== 定数特徴の自動検出 =====\n');
    fprintf('最良候補: i=%d, A行残差=%.3e, B行残差=%.3e, score=%.3e\n', ...
        idx, rowResidualA(idx), rowResidualB(idx), bestScore);

    if isempty(candidates) || bestScore > tol
        [~, order] = sort(score, 'ascend');
        top = order(1:min(5,n));
        fprintf('候補上位のscore:\n');
        for j = 1:numel(top)
            q = top(j);
            fprintf('  i=%d: %.3e\n', q, score(q));
        end
        error(['定数特徴を自動検出できませんでした。' ...
               'constantStructureTolを調整するか、A/Bの定数行を確認してください。']);
    end

    if numel(candidates) > 1
        warning('定数特徴の条件を満たす候補が複数あります: %s。最良候補を使用します。', ...
            mat2str(candidates(:)'));
    end

    info = struct('candidates',candidates, ...
                  'rowResidualA',rowResidualA, ...
                  'rowResidualB',rowResidualB, ...
                  'score',score, ...
                  'bestScore',bestScore);
end

function [Q,s,r] = local_range_svd(M,relTol)
    nrow = size(M,1);
    ncol = size(M,2);
    if ncol == 0
        Q = zeros(nrow,0);
        s = zeros(0,1);
        r = 0;
        return;
    end
    [U,S,~] = svd(M,'econ');
    s = diag(S);
    if isempty(s) || max(s) == 0
        Q = zeros(nrow,0);
        r = 0;
        return;
    end
    r = sum(s > relTol*max(s));
    Q = U(:,1:r);
end

function [N,s,r] = local_null_svd(M,relTol)
    ncol = size(M,2);
    if ncol == 0
        N = zeros(0,0);
        s = zeros(0,1);
        r = 0;
        return;
    end
    % MATLABでは 'full' はsvdの有効なオプションではない。
    % オプションなしのsvd(M)が完全SVDを返す。
    [~,S,V] = svd(M);
    s = diag(S);
    if isempty(s) || max(s) == 0
        r = 0;
        N = V;
        return;
    end
    r = sum(s > relTol*max(s));
    N = V(:,r+1:end);
end

function V = local_embed(Vd,dyn_idx,n)
    V = zeros(n,size(Vd,2));
    V(dyn_idx,:) = Vd;
end

function e = local_block_eigs(F,idx)
    if isempty(idx)
        e = zeros(0,1);
    else
        e = eig(F(idx,idx));
    end
end

function residual = local_block_lower_residual(M,dims)
    residual = 0;
    starts = cumsum([1,dims(1:end-1)]);
    for i = 2:numel(dims)
        rows = starts(i):(starts(i)+dims(i)-1);
        if isempty(rows)
            continue;
        end
        for j = 1:(i-1)
            cols = starts(j):(starts(j)+dims(j)-1);
            if isempty(cols)
                continue;
            end
            residual = max(residual,norm(M(rows,cols),'fro'));
        end
    end
end

function local_print_spectrum(name,e)
    if isempty(e)
        fprintf('%s: なし\n',name);
        return;
    end
    if all(abs(e)<1)
        stableText = 'YES';
    else
        stableText = 'NO';
    end
    fprintf('%s: %d個, max|lambda|=%.6g, 単位円内=%s\n', ...
        name,numel(e),max(abs(e)),stableText);
    disp(e(:));
end

function local_plot_category(e,color,marker,labelText)
    if isempty(e)
        return;
    end
    plot(real(e),imag(e),'LineStyle','none','Marker',marker, ...
        'MarkerEdgeColor',color,'MarkerFaceColor','none', ...
        'LineWidth',1.5,'MarkerSize',9, ...
        'DisplayName',sprintf('%s (%d)',labelText,numel(e)));
end
