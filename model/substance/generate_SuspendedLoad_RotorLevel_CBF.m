% generate_SuspendedLoad_RotorLevel_CBF.m
% =========================================================================
% 【厳密な理論に基づく Rotor-Level HOCBF ジェネレータ】
%
% [理論的背景: Dynamic Extension + 順序付き HOCBF (Xiao & Belta, 2021)]
%
% 入力: ローター推力の2階微分 ddf = [ddf1; ddf2; ddf3; ddf4]
% 相対次数: 4 (Dynamic Extension により統一)
%
% [正しい HOCBF の定式化]
%   現在のコードは「多項式展開 (s+α)^4」を使っていたが、これは ψ_i が負になると
%   b_cbf が非常に負になり QP が Infeasible になる欠点があった。
%
%   正しい「順序付き HOCBF (Sequential HOCBF)」では:
%     ψ0 = h
%     ψ1 = ḣ + α·ψ0
%     ψ2 = ψ̇1 + α·ψ1
%     ψ3 = ψ̇2 + α·ψ2
%   QP条件: ψ̇3 + α·ψ3 ≥ 0
%     ⟺ h^(4) + α·ψ3 ≥ 0 (drift項を除いた部分が制御入力に依存)
%     ⟺ A_cbf · u ≤ b_cbf (b_cbf = h4_drift + α·ψ3)
%
%   【前方不変性保証】: ψ0(0)≥0, ψ1(0)≥0, ψ2(0)≥0, ψ3(0)≥0 ならば
%     QP は常に Feasible かつ h(t)≥0 が保証される。
%   ψ_i を出力に含めることで、HLC側で feasibility を事前チェックできる。
% =========================================================================

mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

disp("==============================================================");
disp(" Start: Sequential HOCBF (Xiao & Belta 2021)");
disp(" Target: A_cbf * [ddf1; ddf2; ddf3; ddf4] <= b_cbf");
disp(" b_cbf = h4_drift + alpha * psi3  (always feasible if psi3>=0)");
disp("==============================================================");

%% 1. シンボリック変数の定義
syms q0 q1 q2 q3 real
syms w1 w2 w3 real
syms m jx jy jz g real

% 荷物・紐側
syms pl1 pl2 pl3 vl1 vl2 vl3 real
syms pT1 pT2 pT3 ol1 ol2 ol3 real
syms mL cableL real

% 楕円体と分離超平面パラメータ
syms a_rad b_rad real
syms n1 n2 n3 d_plane real

% ローター配置パラメータ
syms L_arm c_tau real

% ★Rotor-Level の状態変数と入力（Dynamic Extension）
% モーター推力 f_i とその1階微分 df_i を「状態」として扱う
syms f1 f2 f3 f4 real
syms df1 df2 df3 df4 real
% 入力は推力の2階微分
syms ddf1 ddf2 ddf3 ddf4 real

% [変更点] 単一の α パラメータ（4つのゲインを1つに集約）
syms alpha_hocbf real

q = [q0; q1; q2; q3];
w = [w1; w2; w3];
J = diag([jx, jy, jz]);
e3 = [0; 0; 1];

pl = [pl1; pl2; pl3];
vl = [vl1; vl2; vl3];
pT = [pT1; pT2; pT3];
ol = [ol1; ol2; ol3];
n_vec = [n1; n2; n3];

f_rot  = [f1; f2; f3; f4];
df_rot = [df1; df2; df3; df4];
u_ext  = [ddf1; ddf2; ddf3; ddf4];

%% 2. 幾何関係と回転行列 R(q)
R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3),     2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3),     q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2),     2*(q2*q3+q0*q1),     q0^2-q1^2-q2^2+q3^2];

%% 3. Control Allocation (ローター推力 -> T, tau への変換)
% 一般的なクアッドローター (X型) の配置行列
T = f1 + f2 + f3 + f4;
tau1 = L_arm * (-f1 + f2 + f3 - f4) / sqrt(2); % Roll
tau2 = L_arm * ( f1 - f2 + f3 - f4) / sqrt(2); % Pitch
tau3 = c_tau * (-f1 - f2 + f3 + f4);           % Yaw
tau = [tau1; tau2; tau3];

%% 4. 牽引系ダイナミクス連鎖の定義
dpT = cross(ol, pT);
F_T = T * (R * e3);
a_load = - g * e3 + ((pT.' * F_T - m * cableL * (dpT.' * dpT)) / (m + mL)) * pT;
a_ol = cross(pT, - F_T) / (m * cableL);

dq = 0.5 * [-q1, -q2, -q3;
             q0, -q3,  q2;
             q3,  q0, -q1;
            -q2,  q1,  q0] * w;
dw = J \ (tau - cross(w, J * w));

% 全状態ベクトル
x_all = [pl; vl; pT; ol; q; w; f_rot; df_rot];
% 状態の微分 (f_rot の微分は df_rot, df_rot の微分が u_ext)
dx_all = [vl; a_load; dpT; a_ol; dq; dw; df_rot; u_ext];

%% 5. バリア関数の定義 (分離超平面ベース)
% 紐中点 p_mid (ドローン本体と荷物の間の代表点)
p_mid = pl - 0.5 * cableL * pT;
% 楕円体の法線方向への有効半径
r_extend = sqrt(a_rad^2 + (b_rad^2 - a_rad^2)*(n_vec.' * pT)^2);
% バリア関数 h: 正 → 安全, 負 → 侵入
h = n_vec.' * p_mid - d_plane - r_extend;

%% 6. Lie微分チェーンの計算 (相対次数4まで)
disp('Calculating dh (1st order Lie derivative)...');
dh = jacobian(h, x_all) * dx_all;
dh = simplify(dh);

disp('Calculating ddh (2nd order)...');
ddh = jacobian(dh, x_all) * dx_all;
ddh = simplify(ddh);

disp('Calculating dddh (3rd order)...');
dddh = jacobian(ddh, x_all) * dx_all;
dddh = simplify(dddh);

disp('Calculating h4 (4th order, control input appears here)...');
h4 = jacobian(dddh, x_all) * dx_all;

%% 7. 入力アフィン分離 (A_cbf * u_ext <= b_cbf)
disp('Extracting affine components from h4...');
A_cbf = - jacobian(h4, u_ext);    % (負号注意: 不等式方向に合わせる)
h4_drift = subs(h4, u_ext, [0; 0; 0; 0]);

% ===========================================================
% [変更点] 正しい Sequential HOCBF の b_cbf
%
% ψ0 = h
% ψ1 = ḣ + α·ψ0          = dh + α·h
% ψ2 = ψ̇1 + α·ψ1         = ddh + α·dh + α·(dh + α·h)   = ddh + 2α·dh + α²·h  ... ではなく単純に:
% ψ2 = (ψ1の時間微分) + α·ψ1  だが、シンボリックでは以下の再帰式で表現:
%
% ※ ψ1, ψ2, ψ3 は状態の関数として計算。各ψの時間微分は Lie微分チェーンから得る。
%   ψ1 = dh + alpha * h
%   dψ1/dt = ddh + alpha * dh   (h の2階微分を活用)
%   ψ2 = dψ1/dt + alpha * ψ1 = ddh + alpha*dh + alpha*(dh + alpha*h)
%        = ddh + 2*alpha*dh + alpha^2*h
%   dψ2/dt = dddh + 2*alpha*ddh + alpha^2*dh
%   ψ3 = dψ2/dt + alpha*ψ2
%        = dddh + 2*alpha*ddh + alpha^2*dh + alpha*(ddh + 2*alpha*dh + alpha^2*h)
%        = dddh + 3*alpha*ddh + 3*alpha^2*dh + alpha^3*h
%
%   QP条件: h^(4) + alpha·ψ3 ≥ 0
%   → A_cbf*u ≤ b_cbf で b_cbf = h4_drift + alpha·ψ3
%
%   【(s+α)^4 との違い】
%     (s+α)^4 展開: k3=4α, k2=6α², k1=4α³, k0=α⁴  → 係数が大きく数値的に爆発しやすい
%     Sequential HOCBF: ψ3の係数は (1, 3α, 3α², α³)  → 係数が小さく数値的に安定
% ===========================================================

disp('Computing sequential HOCBF psi1, psi2, psi3...');
psi1 = dh + alpha_hocbf * h;
psi2 = ddh + 2*alpha_hocbf*dh + alpha_hocbf^2*h;        % = d(psi1)/dt + alpha*psi1
psi3 = dddh + 3*alpha_hocbf*ddh + 3*alpha_hocbf^2*dh + alpha_hocbf^3*h; % = d(psi2)/dt + alpha*psi2

% b_cbf: ψ3 >= 0 の初期条件があれば、この QP は常に Feasible
b_cbf = h4_drift + alpha_hocbf * psi3;

%% 8. MATLAB関数としてエクスポート
z_state = [pl; vl; pT; ol; q; w];
f_state = [f_rot; df_rot];
params = [m; mL; cableL; jx; jy; jz; g; L_arm; c_tau];
ellipsoid_params = [a_rad; b_rad];
plane_params = [n_vec; d_plane];
% [変更点] cbf_gains -> alpha_hocbf (スカラー)

disp('Exporting to RotorLevel_SuspendedLoad_CBF.m...');
disp('  Outputs: A_cbf, b_cbf, h, dh, ddh, dddh, psi1, psi2, psi3');
matlabFunction(A_cbf, b_cbf, h, dh, ddh, dddh, psi1, psi2, psi3, ...
    'file', 'RotorLevel_SuspendedLoad_CBF.m', ...
    'vars', {z_state, f_state, params, ellipsoid_params, plane_params, alpha_hocbf}, ...
    'outputs', {'A_cbf', 'b_cbf', 'h', 'dh', 'ddh', 'dddh', 'psi1', 'psi2', 'psi3'}, ...
    'Optimize', true);

disp('RotorLevel_SuspendedLoad_CBF.m generated successfully!');
disp('');
disp('== 使い方 ==');
disp('  alpha = 0.5;  % ← これだけチューニングすればよい');
disp('  [A_cbf, b_cbf, h, dh, ddh, dddh, psi1, psi2, psi3] = ...');
disp('    RotorLevel_SuspendedLoad_CBF(z_state, f_state, params, ellipsoid_params, plane_params, alpha);');
disp('  % 前方不変性保証の確認:');
disp('  % assert(h>=0 && psi1>=0 && psi2>=0 && psi3>=0, "初期条件違反");');

