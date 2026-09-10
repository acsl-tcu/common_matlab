clear;
clc;

% load("retry_LYKL.mat");
loadFileName = input('読み込むファイル名(.mat不要)：','s');
load([loadFileName '.mat']);

A = est.A;
B = est.B;
C = est.C;
n = size(A, 1);

%% ★ステップ1：定数方向は「数値的に発見」せず、既知の構造情報を使う
%   rensyuuKLLY_fixedで厳密固定した const_idx / dyn_idx をそのまま使う。
%   これにより ctrb/rank の数値誤差に一切依存せず、
%   「定数方向=不可制御」を解析的に確定できる。

const_idx = est.const_idx;
dyn_idx   = est.dyn_idx;
n_dyn     = numel(dyn_idx);

%% ★ステップ2：本当に固定できているかのサニティチェック
%   （ここは数値誤差レベルであることの確認であって、判定には使わない）
b_const_norm = norm(B(const_idx, :));
a_const_row  = A(const_idx, :);
e_const      = zeros(1, n); e_const(const_idx) = 1;
a_const_err  = norm(a_const_row - e_const);

fprintf('---- 定数方向の固定チェック ----\n');
fprintf('||B(const_idx,:)||        = %.3e  (機械精度〜1e-8程度なら正常)\n', b_const_norm);
fprintf('||A(const_idx,:)-e_const|| = %.3e  (同上)\n', a_const_err);

if b_const_norm > 1e-4 || a_const_err > 1e-4
    warning(['定数方向の固定が想定より緩んでいます。' ...
             'rensyuuKLLY_fixedの等式制約・ソルバー精度を確認してください。']);
end

%% ★ステップ3：動的部分空間(25次元)だけでctrb/obsvを計算
%   → A_dynの固有値は全てrho_bar未満なので、A_dyn^kは減衰し、
%     数値誤差に埋もれない、頑健なランク判定ができる。

A_dyn = A(dyn_idx, dyn_idx);
B_dyn = B(dyn_idx, :);
C_dyn = C(:, dyn_idx);

Mc_dyn = ctrb(A_dyn, B_dyn);
Mo_dyn = obsv(A_dyn, C_dyn);

rank_Mc_dyn = rank(Mc_dyn);
rank_Mo_dyn = rank(Mo_dyn);

is_controllable_dyn = (rank_Mc_dyn == n_dyn);
is_observable_dyn   = (rank_Mo_dyn == n_dyn);

fprintf('\n---- 動的部分空間(%d次元)の可制御性・可観測性 ----\n', n_dyn);
if is_controllable_dyn && is_observable_dyn
    disp('動的部分空間は「可制御・可観測」です。');
elseif is_controllable_dyn && ~is_observable_dyn
    disp('動的部分空間は「可制御・不可観測」です。');
elseif ~is_controllable_dyn && is_observable_dyn
    disp('動的部分空間は「不可制御・可観測」です。');
else
    disp('動的部分空間は「不可制御・不可観測」です。');
end

fprintf('状態数(全体)             : %d\n', n);
fprintf('  内、定数(不可制御)モード: 1  (index=%d)\n', const_idx);
fprintf('  内、動的モード          : %d\n', n_dyn);
fprintf('動的部分の可制御次元      : %d / %d\n', rank_Mc_dyn, n_dyn);
fprintf('動的部分の可観測次元      : %d / %d\n', rank_Mo_dyn, n_dyn);

%% ★ステップ4：LQRは動的部分空間(25次元)だけに対して設計する
%   定数方向はそもそも入力で動かせないので、LQRに含めても無意味
%   （dlqrに26次元のまま渡すと、B(const_idx,:)=0付近の数値誤差のせいで
%   　条件数の悪いリカッチ方程式を解くことになり、Kが不必要に
%   　大きくなったり不安定になったりするリスクがある）。

Q_dyn  = eye(n_dyn);
Rc     = diag([1; 0.1; 0.1; 1]);   % 入力の重みは従来のものを流用

K_dyn = dlqr(A_dyn, B_dyn, Q_dyn, Rc);

%% ★ステップ5：全状態用のゲインK_fullを再構成
%   定数方向の列は0（そもそも制御できないので0のままでよい）
K_full = zeros(size(K_dyn,1), n);
K_full(:, dyn_idx) = K_dyn;
% K_full(:, const_idx) は 0 のまま
%   → 定数(バイアス)成分に対するフィードバックは行わない。
%     定常オフセットを補償したい場合は、この段では扱わず、
%     別途、積分動作(サーボ系)やフィードフォワードで対応すること。

%% 確認
A_deco = A - B * K_full;

fprintf('\n元のA行列のeig（定数モード含む全26次元）:\n');
disp(eig(A));

fprintf('A-BK_full のeig（動的モードのみ移動、定数モードは1のまま）:\n');
disp(eig(A_deco));

fprintf('\n定数モードの固有値が閉ループでも厳密に1のままなら、\n');
fprintf('「定数方向は入力で動かせない」という解析的事実と整合しています。\n');

%% 保存
saveFolder = fullfile(pwd, 'kalman_gainたち');
if ~exist(saveFolder, 'dir')
    mkdir(saveFolder);
end
saveFileName = fullfile(saveFolder, [loadFileName '_gain_KD_LYKL_fixed2.mat']);
save(saveFileName, 'K_full', 'K_dyn', 'const_idx', 'dyn_idx');
fprintf('"%s" として保存しました。\n', saveFileName);