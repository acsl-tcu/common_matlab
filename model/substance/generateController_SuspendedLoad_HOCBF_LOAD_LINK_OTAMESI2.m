% このスクリプトが置いてあるフォルダの絶対パスを取得して移動
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

%% Define the nonlinear physical model of a quadrotor
syms p1 p2 p3 dp1 dp2 dp3 ddp1 ddp2 ddp3 q0 q1 q2 q3 o1 o2 o3 real
syms u u1 u2 u3 u4 T1 T2 T3 T4 real
syms m Lx Ly lx ly  jx jy jz gravity km1 km2 km3 km4 k1 k2 k3 k4 real
syms mL l Length cableL rotor_r real % LengthとcableLは同じ意味,lは機体の長さ正方形を仮定
syms pl1 pl2 pl3 dpl1 dpl2 dpl3 ol1 ol2 ol3 real
syms pT1 pT2 pT3 real
syms obj % dummy
%% Controller design
clc
syms xd1(t) xd2(t) xd3(t) xd4(t) v1(t) % 牽引物体の位置＋機体のyaw角
syms t real
xd = [xd1(t),xd2(t),xd3(t),xd4(t)]; % reference を後で決める時はこっち
%% 
p	= [  p1;  p2;  p3];             % Position　：xb : 進行方向，zb ：ホバリング時に上向き
% dp	= [ dp1; dp2; dp3];             % Velocity
% ddp	= [ddp1;ddp2;ddp3];             % Accelaletion
% q	= [  q0;  q1;  q2;  q3];        % Quaternion
% ob	= [  o1;  o2;  o3];             % Angular velocity
pl  = [ pl1; pl2; pl3];             % Load position
dpl = [dpl1;dpl2;dpl3];             % Load velocity
ol  = [ ol1; ol2; ol3];             % Load angular velocity
pT  = [ pT1; pT2; pT3];             % String position
% x=[q;ob;pl;dpl;pT;ol];
x = [pl; dpl; pT; ol];
% physicalParam = [m, Lx, Ly lx ly, jx, jy, jz, gravity, km1, km2, km3, km4, k1, k2, k3, k4, rotor_r, mL, cableL];
% f = FL(x,physicalParam);
% g = GL(x,physicalParam);
% physicalParam = [m, jx, jy, jz, gravity,mL,cableL];

syms dstx dsty real
physicalParam = [m, Lx, Ly lx ly, jx, jy, jz, gravity, km1, km2, km3, km4, k1, k2, k3, k4, rotor_r, mL, cableL, dstx, dsty];

f = FLxyDst_CBF(x,physicalParam);
g = GLxyDst_CBF(x,physicalParam);
physicalParam = [m, jx, jy, jz, gravity,mL,cableL,dstx, dsty];

% %% =========================================================================
% %% 【追加】高次制御バリア関数z方向 (HOCBF) の自動導出と関数エクスポート
% %% =========================================================================
% disp("Start: 高次CBF(HOCBF)の導出と関数化を開始します。");
% FG_z = simplify(f+g*[u1;u2;u3;u4]);
% g_z = simplify(MyCoeff(FG_z,[u1;u2;u3;u4]));
% f_z = subs(FG_z,[u1,u2,u3,u4],[0,0,0,0]);
% simplify(FG_z-(f_z+g_z*[u1;u2;u3;u4]))
% 
% % 1. 障害物安全関数の定義 (真球障害物 [xo, yo, zo] と半径 ro)
% syms xo yo zo ro real
% syms rl real
% 
% % physicalParam からケーブル長 L を参照
% L_cable = physicalParam(7);
% 
% % 🌟 【核心部】 安全関数の基準位置を「ロープ（リンク）の中心点 p_mid」に設定
% p_mid = pl - 0.5 * L_cable * pT;
% 
% % 荷物の位置 pl = [pl1; pl2; pl3] と障害物の距離の2乗から安全を定義
% h_cbf_z = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2;
% cbf1 = h_cbf_z;
% % 2. クラスK関数のゲイン（チューニングパラメータ：実際のシミュレーション側で変更可能にシンボリック化）
% syms gamma1 gamma2 real
% g1_z = g_z(:, 1);% ★ $u_1$（推力）に掛かる列を抽出
% 
% % 3. 2階の高次CBFの導出（相対次数2）
% % cbf1 (h)   >= 0 
% % cbf2 (dot_h + gamma1 * h) >= 0
% cbf2 = LieD(cbf1, f_z, x) + diff(cbf1, t) + gamma1 * cbf1; %h2
% 
% 
% % 4. 最適化問題（QP）へのマッピング
% % cbf2_dot + gamma2 * cbf2 >= 0  を導出
% % ここで L_g_cbf2 * u1 + L_f_cbf2 + gamma2 * cbf2 >= 0
% L_f_cbf2  = LieD(cbf2, f_z, x) + diff(cbf2, t);
% L_g_cbf2  = LieD(cbf2, g1_z, x);  % [1 x 1] のシンボリックスカラー（u1 の係数）
% 
% % A_qp * u1 <= b_qp の形に変形（符号反転）
% A_cbf_sym = -L_g_cbf2; 
% b_cbf_sym = L_f_cbf2 + gamma2 * cbf2;
% 
% % 5. 実数値シミュレーション用の置換 (xdRef -> XDf, vInput1f -> V1vf)
% A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 6. Mファイルとして関数エクスポート
% gamma_params = [gamma1; gamma2];
% obs_params = [xo; yo; zo; ro];
% sys_params = rl;
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:); % 縦ベクトル化
% 
% disp("Exporting: CBF_Constraints_zlink.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_zlink.m', ...
%     'vars', {obj, x, XD_sym, cell2sym(V1v), obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp'});
% disp("Done: CBF関数の生成が完了しました！");
% %% =========================================================================
% %% 【仮想入力 F = [Fx, Fy, Fz] 対応】CBF 制約自動導出セクション (相対次数 2)
% %% =========================================================================
% disp("Start: 仮想入力 F に対応した CBF (相対次数 2) の導出と関数化を開始します...");
% %% =========================================================================
% disp("Start: 高次CBF(HOCBF)の導出と関数化を開始します。");
% FG = simplify(f+g*[u1;u2;u3]);
% g = simplify(MyCoeff(FG,[u1;u2;u3]));
% f = subs(FG,[u1,u2,u3],[0,0,0]);
% simplify(FG-(f+g*[u1;u2;u3]))
% 
% % 1. 障害物安全関数の定義 (真球障害物 [xo, yo, zo] と半径 ro)
% syms xo yo zo ro real
% syms rl real
% 
% % physicalParam からケーブル長 L を参照
% L_cable = physicalParam(7);
% 
% % 🌟 【核心部】 安全関数の基準位置を「ロープ（リンク）の中心点 p_mid」に設定
% p_mid = pl - 0.5 * L_cable * pT;
% 
% % 荷物の位置 pl = [pl1; pl2; pl3] と障害物の距離の2乗から安全を定義
% h = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2;
% cbf1 = h;
% % 2. クラスK関数のゲイン（チューニングパラメータ：実際のシミュレーション側で変更可能にシンボリック化）
% syms gamma1 gamma2 real
% 
% % 3. 2階の高次CBFの導出（相対次数2）
% % cbf1 (h)   >= 0 
% % cbf2 (dot_h + gamma1 * h) >= 0
% % cbf2 = LieD(cbf1, f, x) + diff(cbf1, t) + gamma1 * cbf1; %h2
% cbf2 = LieD(cbf1, f, x) + gamma1 * cbf1; %h2
% 
% 
% % 4. 最適化問題（QP）へのマッピング
% % cbf2_dot + gamma2 * cbf2 >= 0  を導出
% % ここで L_g_cbf2 * u1 + L_f_cbf2 + gamma2 * cbf2 >= 0
% L_f_cbf2  = LieD(cbf2, f, x); %+ diff(cbf2, t);
% L_g_cbf2  = LieD(cbf2, g, x);  % [1 x 1] のシンボリックスカラー（u1 の係数）
% 
% % A_qp * u1 <= b_qp の形に変形（符号反転）
% A_cbf_sym = -L_g_cbf2; 
% b_cbf_sym = L_f_cbf2 + gamma2 * cbf2;
% 
% % % 5. 実数値シミュレーション用の置換 (xdRef -> XDf, vInput1f -> V1vf)
% % A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% % b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 6. Mファイルとして関数エクスポート
% gamma_params = [gamma1; gamma2];
% obs_params = [xo; yo; zo; ro];
% sys_params = rl;
% % XD_sym = cell2sym(XD);
% % XD_sym = XD_sym(:); % 縦ベクトル化
% 
% disp("Exporting: CBF_forece.m を書き出しています...");
% matlabFunction(A_cbf_sym, b_cbf_sym, 'file', 'CBF_force.m', ...
%     'vars', {obj, x, obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp'});
% disp("Done: CBF関数の生成が完了しました！");
%% =========================================================================
%% 【仮想入力 F = [Fx, Fy, Fz] 対応】CBF 制約自動導出セクション (相対次数 2)
%% =========================================================================
disp("Start: 仮想入力 F に対応した CBF (相対次数 2) の導出と関数化を開始します...");

U_F = [u1; u2; u3]; % 3軸の全推力ベクトル F = [Fx; Fy; Fz]
FG  = simplify(f + g * U_F);

% ドリフト項 f と 入力行列 g の抽出
g = simplify(MyCoeff(FG, U_F));
f = subs(FG, U_F, [0; 0; 0]);

% 1. 障害物安全関数の定義 (真球障害物 [xo, yo, zo] と半径 ro, 保護半径 rl)
syms xo yo zo ro real
syms rl real

L_cable = physicalParam(7); % cableL
p_mid = pl - 0.5 * L_cable * pT; % ケーブル中心点

% 安全関数 h(x) >= 0
h = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2;
cbf1 = h;

% 2. クラスK関数のゲイン
syms gamma1 gamma2 real

% 3. 1階微分 (dot_h) と 2階微分用の展開
dot_h = LieD(cbf1, f, x);          % L_f h
cbf2  = dot_h + gamma1 * cbf1;     % h2 = dot_h + gamma1 * h

% 4. 最適化問題（QP）へのマッピング
L_f_cbf2 = LieD(cbf2, f, x);       % L_f^2 h + gamma1 * L_f h
L_g_cbf2 = LieD(cbf2, g, x);       % L_g L_f h (入力行列 G に掛かる項) [1 x 3]

% A_qp * F <= b_qp の形に変形
A_cbf_sym = -L_g_cbf2; 
b_cbf_sym =  L_f_cbf2 + gamma2 * cbf2;

% =========================================================================
% 🔍【デバッグ・検証用セクション】数式の整合性チェック (未置換変数自動補填版)
% =========================================================================
fprintf('\n----------------- [ CBF 数式チェック ] -----------------\n');
fprintf('1. L_g_cbf2 (入力行列への感度) のランク・零ベクトルチェック:\n');
disp(simplify(L_g_cbf2));

if isequal(simplify(L_g_cbf2), sym([0, 0, 0]))
    warning('⚠️ 【エラー】L_g_cbf2 が完全に 0 になっています！仮想入力 F が CBF に伝わっていません！');
else
    fprintf('✅ L_g_cbf2 に入力 U_F への依存性が正しく存在します。\n');
end

% --- 数値テスト（障害物に近づけた仮想テストデータの代入） ---
fprintf('\n2. 数値テスト（障害物直前に機体・荷物がある場合の A_qp, b_qp 評価）:\n');

% 基本物性値 (m=1.0kg, mL=0.5kg, L=1.0m, g=9.81)
P_test = [1.0, 0.01, 0.01, 0.02, 9.81, 0.5, 1.0]; 
obs_test = [1.0; 0.0; -1.0; 0.3]; % 障害物 [x, y, z, r]
rl_test  = 0.2;
gamma_test = [5.0; 10.0];

% 荷物が障害物前 0.3m に迫り、1m/s で接近中の状態
x_test = [
    0.7; 0.0; -1.0;   % pl (荷物位置)
    1.0; 0.0;  0.0;   % dpl (荷物速度)
    0.0; 0.0; -1.0;   % pT (紐方向)
    0.0; 0.0;  0.0    % ol (紐角速度)
];

% 基本変数の定義
sym_vars_base = [x; xo; yo; zo; ro; rl; gamma1; gamma2; physicalParam(1:7).'];
val_vars_base = [x_test; obs_test; rl_test; gamma_test; P_test(:)];

% 【エラー回避】式中に残っている未代入の物理パラメータ等を全自動検索してゼロで置換
sub_A_sym = subs(A_cbf_sym, sym_vars_base, val_vars_base);
rem_vars_A = symvar(sub_A_sym);
if ~isempty(rem_vars_A)
    sub_A_sym = subs(sub_A_sym, rem_vars_A, zeros(size(rem_vars_A)));
end
A_val = double(sub_A_sym);

sub_b_sym = subs(b_cbf_sym, sym_vars_base, val_vars_base);
rem_vars_b = symvar(sub_b_sym);
if ~isempty(rem_vars_b)
    sub_b_sym = subs(sub_b_sym, rem_vars_b, zeros(size(rem_vars_b)));
end
b_val = double(sub_b_sym);

sub_h_sym = subs(cbf1, [x(1:3); x(7:9); xo; yo; zo; ro; rl; physicalParam(7)], [x_test(1:3); x_test(7:9); obs_test; rl_test; P_test(7)]);
h_val = double(sub_h_sym);

sub_doth_sym = subs(dot_h, sym_vars_base, val_vars_base);
rem_vars_doth = symvar(sub_doth_sym);
if ~isempty(rem_vars_doth)
    sub_doth_sym = subs(sub_doth_sym, rem_vars_doth, zeros(size(rem_vars_doth)));
end
dot_h_val = double(sub_doth_sym);

fprintf('   ・安全関数値 h(x)   = %.4f  (正の値なら安全域)\n', h_val);
fprintf('   ・時間変化率 dot_h  = %.4f  (負の値なら障害物に接近中)\n', dot_h_val);
fprintf('   ・A_qp (制約の勾配) = [%.4f, %.4f, %.4f]\n', A_val(1), A_val(2), A_val(3));
fprintf('   ・b_qp (制約の上限) = %.4f\n', b_val);

% 公称入力 F_nom = [0, 0, (m_Q + m_L)*g] = [0, 0, 14.715 N] (ホバリング力) が制約を満たすかテスト
F_nom_test = [0; 0; 14.715];
lh_side = A_val * F_nom_test;
fprintf('   ・公称入力時の左辺 A_qp * F_nom = %.4f  (<= b_qp = %.4f であるべき)\n', lh_side, b_val);

if lh_side > b_val
    fprintf('   🚨 【判定】公称入力 F_nom では制約違反になります！CBF が介入して回避推力 F* を算出します。\n');
else
    fprintf('   ℹ️ 【判定】公称入力 F_nom のままで安全制約を満たしています。\n');
end
fprintf('--------------------------------------------------------\n\n');

% 5. Mファイルとして関数エクスポート
gamma_params = [gamma1; gamma2];
obs_params   = [xo; yo; zo; ro];
sys_params   = rl;

disp("Exporting: CBF_force.m を書き出しています...");
matlabFunction(A_cbf_sym, b_cbf_sym, 'file', 'CBF_force.m', ...
    'vars', {obj, x, obs_params, gamma_params, sys_params, physicalParam}, ...
    'outputs', {'A_qp', 'b_qp'});
disp("Done: CBF関数の生成が完了しました！");

%% =========================================================================
%% 【追加】 ケーブル傾き制約 (<= 15 deg) の HOCBF 自動導出 (相対次数 2)
%% =========================================================================
disp("Start: ケーブル傾き角制約 (<= 15 deg) の HOCBF 導出を開始します...");

% 1. ケーブル傾き安全関数: -pT3 - cos(15 deg) >= 0
syms theta_max_deg real
syms gamma_c1 gamma_c2 real

theta_max_rad = theta_max_deg * (pi / 180);
h_cable = -pT(3) - cos(theta_max_rad);

% 2. 仮想入力 F = [u1; u2; u3] に対する相対次数 2 の Lie 微分
cbf1_cb = h_cable;
cbf2_cb = LieD(cbf1_cb, f, x) + gamma_c1 * cbf1_cb;

L_f_cbf_cb = LieD(cbf2_cb, f, x);
L_g_cbf_cb = LieD(cbf2_cb, g, x); % [1 x 3] 行列

% A_qp_cb * F <= b_qp_cb
A_cb_sym = -L_g_cbf_cb;
b_cb_sym =  L_f_cbf_cb + gamma_c2 * cbf2_cb;

% 3. Mファイルとして関数エクスポート
gamma_cable_params = [gamma_c1; gamma_c2];

disp("Exporting: CBF_cable_angle.m を書き出しています...");
matlabFunction(A_cb_sym, b_cb_sym, 'file', 'CBF_cable_angle.m', ...
    'vars', {obj, x, theta_max_deg, gamma_cable_params, physicalParam}, ...
    'outputs', {'A_qp_cb', 'b_qp_cb'});

disp("Done: ケーブル傾き制約 CBF 関数の生成が完了しました！");

% %% =========================================================================
% %% 【追加】高次制御バリア関数 (HOCBF) の自動導出と関数エクスポート
% %% =========================================================================
% disp("Start: 高次CBF(HOCBF)の導出と関数化を開始します。");
% 
% % 1. 障害物安全関数の定義 (真球障害物 [xo, yo, zo] と半径 ro)
% syms xo yo zo ro real
% syms rl real
% 
% % physicalParam からケーブル長 L を参照
% L_cable = physicalParam(7);
% 
% % 🌟 【核心部】 安全関数の基準位置を「ロープ（リンク）の中心点 p_mid」に設定
% p_mid = pl - 0.5 * L_cable * pT;
% 
% % 荷物の位置 pl = [pl1; pl2; pl3] と障害物の距離の2乗から安全を定義
% h_cbf_xy = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2;
% cbf1 = h_cbf_xy;
% 
% 
% % 2. クラスK関数のゲイン（チューニングパラメータ：実際のシミュレーション側で変更可能にシンボリック化）
% syms gamma1 gamma2 gamma3 gamma4 real
% 
% % 3. 6階の高次CBFをリー微分(LieD)を用いて順次計算
% % f1, g1 は 2nd layer で求めた [u2; u3; u4] に対するシステム方程式
% % 今回は u4 (yaw) の項は既知（理想値）として扱うため、g1の3列目を取り出して処理します
% g1_xy = g1(:, 1:2); % u2, u3 に掛かる列だけを抽出
% g1_yaw = g1(:, 3);  % u4 に掛かる列
% 
% % 階層的なCBFの導出
% % ※ diff(..., t) は目標軌道 xd(t) などの時間微分をカバーするために維持します
% cbf2 = LieD(cbf1, f1, x) + diff(cbf1, t) + gamma1 * cbf1; %h2
% cbf3 = LieD(cbf2,  f1, x) + diff(cbf2,  t) + gamma2 * cbf2; %h3
% cbf4 = LieD(cbf3,  f1, x) + diff(cbf3,  t) + gamma3 * cbf3; %h4
% 
% % 最上階（6階）の計算：ここに u2, u3 が現れる
% % cbf6_dot = L_f1(cbf5) + L_g1_xy(cbf5)*[u2; u3] + L_g1_yaw(cbf5)*u4 + diff(cbf5, t)
% L_f_cbf4  = LieD(cbf4, f1, x) + diff(cbf4, t);
% L_g_cbf4  = LieD(cbf4, g1_xy, x);  % [1 x 2] のベクトル（u2, u3 の係数 β）
% L_gy_cbf4 = LieD(cbf4, g1_yaw, x); % (u4 の係数)
% 
% % これを QP用の形式 「 A_qp * [u2; u3] <= b_qp 」に整理します。
% % 不等号を反転させるため、符号をマイナスにします。
% 
% A_cbf_sym = -L_g_cbf4; % [1 x 2] のシンボリック行ベクトル
% b_cbf_sym = L_f_cbf4 + L_gy_cbf4 * u4 + gamma4 * cbf4; % シンボリックスカラー
% 
% % 4. 実際の数値シミュレーション側で代入しやすいよう、変数を置き換え (xdRef -> XDf, vInput1f -> V1vf)
% % 2nd layerの入力 u4（yaw用）には、コントローラが後で計算する実入力の4番目の要素を指定できるように V4 を代入
% syms V4 real
% A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f, u4], [XDf, V1vf, V4]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f, u4], [XDf, V1vf, V4]);
% 
% % 5. 高速計算用に関数ファイル (Mファイル) としてエクスポート
% % クラスK関数のゲインも外部から与えられるように引数に含めます
% gamma_params = [gamma1; gamma2; gamma3; gamma4];
% obs_params = [xo; yo; zo; ro];
% sys_params = rl;
% 
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:); % 強制的に縦ベクトル化
% 
% disp("Exporting: CBF_Constraints_xylink.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_xylink.m', ...
%                'vars', {obj, x, XD_sym, cell2sym(V1v), V4, obs_params, gamma_params, sys_params, physicalParam}, ...
%                'outputs', {'A_qp', 'b_qp'});
% 
% disp("Done: CBF関数の生成が完了しました！");
% %% =========================================================================
% %% 【決定版】外部確定入力 u1 をそのままドリフト項に保持する HOCBF 自動導出
% %% =========================================================================
% disp("Start: 実入力 u1 を含むダイナミクス f_xy による HOCBF の導出を開始します。");
% 
% % 1. 全ダイナミクス FG_xy の定義
% FG_xy = simplify(f + g * [u1; u2; u3; u4]);
% 
% % 2. u2, u3, u4 を 0 とした実効ドリフト項 f_xy
% %    🌟 u1（推力）は 0 にせず、シンボリック変数 u1 のまま f_xy に残ります！
% f_xy = subs(FG_xy, [u2, u3, u4], [0, 0, 0]);
% 
% % 2nd layer の操作入力 [u2; u3; u4] に対する入力行列 g_xy
% g_xy   = simplify(MyCoeff(FG_xy, [u2; u3; u4]));
% g1_xy  = g_xy(:, 1:2); % u2 (roll), u3 (pitch) に掛かる列
% g1_yaw = g_xy(:, 3);   % u4 (yaw) に掛かる列
% 
% % 3. 障害物安全関数の定義 (ロープ中心 p_mid)
% syms xo yo zo ro real
% syms rl real
% L_cable = physicalParam(7);
% p_mid = pl - 0.5 * L_cable * pT;
% 
% h_cbf_xy = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2;
% cbf1 = h_cbf_xy;
% 
% % 4. クラスK関数のゲイン
% syms gamma1 gamma2 gamma3 gamma4 real
% 
% % 5. f_xy を用いた Lie 微分の計算（相対次数 4）
% % ※ f_xy の中に u1 が入っているため、Lie 微分の中に u1 が自然な形で組み込まれます
% cbf2 = LieD(cbf1, f_xy, x) + diff(cbf1, t) + gamma1 * cbf1; % h2
% cbf3 = LieD(cbf2, f_xy, x) + diff(cbf2, t) + gamma2 * cbf2; % h3
% cbf4 = LieD(cbf3, f_xy, x) + diff(cbf3, t) + gamma3 * cbf3; % h4
% 
% % 最上階での展開（u2, u3 が現れる階層）
% L_f_cbf4  = LieD(cbf4, f_xy, x) + diff(cbf4, t);
% L_g_cbf4  = LieD(cbf4, g1_xy, x);  % [1 x 2] 行列
% L_gy_cbf4 = LieD(cbf4, g1_yaw, x); % スカラー
% 
% % A_qp * [u2; u3] <= b_qp の形に整理（符号反転）
% A_cbf_sym = -L_g_cbf4;
% b_cbf_sym =  L_f_cbf4 + L_gy_cbf4 * u4 + gamma4 * cbf4;
% 
% % 6. 実数値シミュレーション用の変数置換
% %    u1 を外部入力記号 U1_val に、u4 を V4 に、目標軌道微分を XDf に置換
% syms U1_val V4 real
% A_cbf_subs = subs(A_cbf_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);
% 
% % 7. Mファイルとしてエクスポート
% gamma_params = [gamma1; gamma2; gamma3; gamma4];
% obs_params   = [xo; yo; zo; ro];
% sys_params   = rl;
% XD_sym       = cell2sym(XD);
% XD_sym       = XD_sym(:); 
% 
% disp("Exporting: CBF_Constraints_xyotamesi.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, 'file', 'CBF_Constraints_xyotamesi.m', ...
%                'vars', {obj, x, XD_sym, U1_val, V4, obs_params, gamma_params, sys_params, physicalParam}, ...
%                'outputs', {'A_qp', 'b_qp'});
% disp("Done: CBF関数の生成が完了しました！");
% %% =========================================================================
% %% 【追加】 ケーブル傾き制約 (<= 10 deg) の HOCBF 自動導出
% %% =========================================================================
% disp("Start: ケーブル傾き角制約 (<= 10 deg) の HOCBF 導出を開始します...");
% 
% % 1. ケーブル傾き安全関数: p_z <= -cos(10 deg)  <=>  h_cable = -p_z - cos(10 deg) >= 0
% theta_max = 10 * (pi / 180); % 10度をラジアン変換
% h_cable = -pT(3) - cos(theta_max);
% 
% % 2. Lie 微分の展開 (f_xy を使用, 相対次数 2 で展開)
% syms gamma_c1 gamma_c2 real
% cbf_c1 = h_cable;
% cbf_c2 = LieD(cbf_c1, f_xy, x) + diff(cbf_c1, t) + gamma_c1 * cbf_c1;
% 
% L_f_c2  = LieD(cbf_c2, f_xy, x) + diff(cbf_c2, t);
% L_g_c2  = LieD(cbf_c2, g1_xy, x);  % [1 x 2] 行列
% L_gy_c2 = LieD(cbf_c2, g1_yaw, x); % スカラー
% 
% % A_qp_cable * [u2; u3] <= b_qp_cable
% A_cable_sym = -L_g_c2;
% b_cable_sym =  L_f_c2 + L_gy_c2 * u4 + gamma_c2 * cbf_c2;
% 
% % 3. 実数値シミュレーション用の置換
% A_cable_subs = subs(A_cable_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);
% b_cable_subs = subs(b_cable_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);
% 
% % 4. 関数ファイルとして書き出し
% gamma_cable_params = [gamma_c1; gamma_c2];
% disp("Exporting: CBF_Constraints_cable_angle.m を書き出しています...");
% matlabFunction(A_cable_subs, b_cable_subs, 'file', 'CBF_Constraints_cable_angle.m', ...
%     'vars', {obj, x, XD_sym, U1_val, V4, gamma_cable_params, physicalParam}, ...
%     'outputs', {'A_qp_cable', 'b_qp_cable'});
% disp("Done: ケーブル傾き制約 CBF 関数の生成が完了しました！");

%% Local functions
function tmp = DeleteCommentLine(fname)
% % DeleteCommentLine : Remove comments and blank lines in functions
% % % Load file
	fp = fopen(strcat(fname,'.m'),'r');
	str = textscan(fp,'%s','delimiter','\n');
	str = str{1};
	fclose(fp);
	tmp = length(str);
% % % Check comment line and blank line
	str = str(~strncmp(str,'%',1));
	str = str(~strncmp(str,'',1));
	tmp2 = length(str);
% % % Update file
	if (tmp~=tmp2) && (tmp2 ~= 0)
		fp = fopen(strcat(fname,'.m'),'w');
		cellfun(@(a) fprintf(fp,'%s\n',a),str,'UniformOutput',false);
		fclose(fp);
	end
end
function pd = pdiff(flist, vars)
% % flistをvarsで微分したもののシンボリック配列を返す？
    flist = flist(:);
    vars = vars(:);
%    pd = arrayfun(@(x) diff(flist(1), x), vars)';
    pdiff_tmp = @(flist, vars) arrayfun(@(func) arrayfun(@(x) diff(func, x), vars)',flist,'UniformOutput',false);
    pd = cell2sym(pdiff_tmp(flist,vars));
end
function dh = LieD(h,f,x)
    x = x(:);
    dh = pdiff(h,x)*f;
end
function mat = MyCoeff(M,vars)
    vars = vars(:);
    mat = coder.nullcopy(sym(zeros(length(M),length(vars))));
    for i = 1:length(M)
        for j = 1:length(vars)
            mat(i,j) = subs(M(i)-subs(M(i),vars(j),0),vars(j),1);
        end
    end
end
function [eul,th,psi] = Quat2Eul(q,varargin)
% % phi...roll, th...pitch, psi...yaw
    switch nargin
        case 1
            q0 = q(1);
            q1 = q(2);
            q2 = q(3);
            q3 = q(4);
        case 4
            q0 = q;
            q1 = varargin{1};
            q2 = varargin{2};
            q3 = varargin{3};
    end
	phi = atan2((2*(q0*q1 + q2*q3)),(q0^2 - q1^2 - q2^2 + q3^2));
	th  = asin(2*(q0*q2 - q1*q3));
	psi = atan2((2*(q0*q3 + q1*q2)),(q0^2 + q1^2 - q2^2 - q3^2));
	switch  nargout
        case 1
		eul = [phi, th, psi];
        case 3
		eul = phi;
	end
end
function R=Rodrigues(u,th)
    u = u(:);
    u = u/norm(u);
    R=u*u'+cos(th)*(eye(3)-u*u')+sin(th)*Skew(u);
end
function Om = Skew(o)
% % Skew : Skew symmetric matrix
    o  = o(:);
    o1 = o(1);
    o2 = o(2);
    o3 = o(3);
    Om = [  0,-o3, o2;
		   o3,  0,-o1;
		  -o2, o1,  0];
end
function R = RodriguesQuaternion(q)
% % RodriguesQuaternion : R is the rotation matrix
    q  = q(:);
    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);
	E = [-q1, q0,-q3, q2;
		 -q2, q3, q0,-q1;
		 -q3,-q2, q1, q0];
	L = [-q1, q0, q3,-q2;
		 -q2,-q3, q0, q1;
		 -q3, q2,-q1, q0];
	R = E*L';
end
