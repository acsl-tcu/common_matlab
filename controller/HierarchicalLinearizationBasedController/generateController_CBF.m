% このスクリプトが置いてあるフォルダの絶対パスを取得して移動
mfile_path = fileparts(mfilename('fullpath'));
cd(mfile_path);

%% Define the nonlinear physical model of a quadrotor
syms p1 p2 p3 dp1 dp2 dp3 ddp1 ddp2 ddp3 q0 q1 q2 q3 o1 o2 o3 real
syms u u1 u2 u3 u4 T1 T2 T3 T4 real
syms m Lx Ly lx ly jx jy jz gravity km1 km2 km3 km4 k1 k2 k3 k4 real
%% Controller design
clc
syms xd1(t) xd2(t) xd3(t) xd4(t) v1(t)
syms t real
xd = [xd1(t),xd2(t),xd3(t),xd4(t)]; % reference を後で決める時はこっち
%% 
p	= [  p1;  p2;  p3];             % Position　：xb : 進行方向，zb ：ホバリング時に上向き
dp	= [ dp1; dp2; dp3];             % Velocity
ddp	= [ddp1;ddp2;ddp3];             % Accelaletion
q	= [  q0;  q1;  q2;  q3];        % Quaternion
ob	= [  o1;  o2;  o3];             % Angular velocity
x = [q;p;dp;ob];
physicalParam = [m, Lx, Ly, lx, ly, jx, jy, jz, gravity, km1, km2, km3, km4, k1, k2, k3, k4];
f = F(x,physicalParam);
g = G(x,physicalParam);
%g= [g1 g2 g3 g4];
%% 1st layer
clc
% % Define virtual output: h1
h1 = p3 - xd(3);
q	= [  q0;  q1;  q2;  q3];        % Quaternion
% % **************************************** % %
% LieD(h1,g,x) % For check 
dh1 = LieD(h1,f,x)+diff(h1,t);
alpha1 = LieD(dh1,f,x)+diff(dh1,t);
beta1 = simplify(LieD(dh1,g,x));
H = [1/beta1(1),-beta1(2)/beta1(1),-beta1(3)/beta1(1),-beta1(4)/beta1(1); zeros(3,1),eye(3)];
syms e1 e2 real
He = [1/(beta1(1)+e1),-beta1(2)/(beta1(1)+e1),-beta1(3)/(beta1(1)+e1),-beta1(4)/(beta1(1)+e1); zeros(3,1),eye(3)];
% H = pinv(beta1);
% nH = null(H');


% syms v1(t) dv1(t) ddv1(t) dddv1(t) ddddv1(t)
% dv1(t) = diff(v1(t),t);
% ddv1(t) = diff(v1(t),t,2);
% dddv1(t) = diff(v1(t),t,3);
% ddddv1(t) = diff(v1(t),t,4);
% Qz = diag([1.0,1.0]);               % Weight of state
% Rz = 0.1;                           % Weight of input
% Nz = 0;
% Fz = lqr([0,1;0,0], [0;1], Qz, Rz, Nz);
 %v1 = -Fz * [h1;dh1]; % v1を事前に決めておく時はこっち
%% 2nd layer
syms v2 v3 v4
FG = simplify(f+g*H*[-alpha1+v1(t);u2;u3;u4]);	% v1を後で設計する時はこっち
g1 = simplify(MyCoeff(FG,[u2;u3;u4]));
f1 = subs(FG,[u2,u3,u4],[0,0,0]);
%simplify(FG-(f1+g1*[v2;v3;v4]))   % For check

% FG = simplify(f+g*(H*(-alpha1+v1(t))+nH*[v2;v3;v4]));	% v1を後で設計する時はこっち
% g1 = simplify(MyCoeff(FG,[v2;v3;v4]));
% f1 = subs(FG,[v2,v3,v4],[0,0,0]);

%%
% % Define virtual output: h2, h3, h4
h2 = p1 - xd(1);
h3 = p2 - xd(2);
[~,~,yaw] = Quat2Eul(q);
h4 = yaw - xd(4);
%h4 = 2*(q0*q3 + q1*q2) - xd(4); % これでは姿勢は変わらない
%% **************************************** % %
clc
dh2 = LieD(h2,f1,x)+diff(h2,t);
dh3 = LieD(h3,f1,x)+diff(h3,t);
dh4 = LieD(h4,f1,x)+diff(h4,t);
ddh2 = LieD(dh2,f1,x)+diff(dh2,t);
ddh3 = LieD(dh3,f1,x)+diff(dh3,t);
dddh2 = LieD(ddh2,f1,x)+diff(ddh2,t);
dddh3 = LieD(ddh3,f1,x)+diff(ddh3,t);
% % For check
% 	[LieD(h2,g1,x),LieD(h3,g1,x)]
% 	simplify([LieD(dh2,g1,x),LieD(dh3,g1,x)])
% 	simplify([LieD(ddh2,g1,x),LieD(ddh3,g1,x)])
% 	[LieD(h2,g1,x),LieD(h3,g1,x),LieD(h4,g1,x)]
% 	simplify([LieD(dh2,g1,x),LieD(dh3,g1,x)])
% 	simplify([LieD(ddh2,g1,x),LieD(ddh3,g1,x)])
%% 
% % Derive 2nd layer controller
alpha2 = [LieD(dddh2,f1,x)+diff(dddh2,t); LieD(dddh3,f1,x)+diff(dddh3,t); LieD(dh4,f1,x)+diff(dh4,t)];
beta2 = [LieD(dddh2,g1,x); LieD(dddh3,g1,x); LieD(dh4,g1,x)];
% Qxy = diag([1.0, 1.0, 0.1, 0.01]);	% Weight of state
% Rxy = 1.0;                          % Weight of input
% Nxy = 0;
% Fxy = lqr([0,1,0,0;0,0,1,0;0,0,0,1;0,0,0,0], [0;0;0;1], Qxy, Rxy, Nxy);
% Qr = diag([1.0, 1.0]);              % Weight of state
% Rr = 1;                             % Weight of input
% Nr = 0;
% Fr = lqr([0,1;0,0], [0;1], Qr, Rr, Nr);
% if flag.predefinedReference
%     v2 = [-F4*[h2;dh2;ddh2;dddh2]; -Fxy*[h3;dh3;ddh3;dddh3]; -Fr*[h4;dh4]];
%     U2 = inv(beta2)*(-alpha2+v2);   % v2 を事前に設計しておく時はこっち
% else
    syms v2(t) v3(t) v4(t)
    U2 = inv(beta2)*(-alpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
    %U2 = beta2/(-alpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
    % U2e = (adjoint(beta2)/(det(beta2)+e2))*(-alpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
%end
%% Initialize xd as an unspecified function of t
% % If regenerate Uf, Us or Xd functions, evaluate this section.
    xd = [xd1(t),xd2(t),xd3(t),xd4(t)];
    dxd = diff(xd,t);
    ddxd = diff(xd,t,2);
    dddxd = diff(xd,t,3);
    ddddxd = diff(xd,t,4);
%% Set variables for output functions
    syms Xd1 Xd2 Xd3 Xd4 dXd1 dXd2 dXd3 dXd4 ddXd1 ddXd2 ddXd3 ddXd4 dddXd1 dddXd2 dddXd3 dddXd4 ddddXd1 ddddXd2 ddddXd3 ddddXd4 real
    syms V1 V2 V3 V4 dV1 ddV1 dddV1 real
    XD = {Xd1 Xd2 Xd3 Xd4 dXd1 dXd2 dXd3 dXd4 ddXd1 ddXd2 ddXd3 ddXd4 dddXd1 dddXd2 dddXd3 dddXd4 ddddXd1 ddddXd2 ddddXd3 ddddXd4};
    XDf = fliplr(XD);
    V1v = {V1 dV1 ddV1 dddV1};
    V1vf = fliplr(V1v);
    xdRef = fliplr([xd dxd ddxd dddxd ddddxd]);
    vInput1 = fliplr([v1(t) diff(v1(t),t) diff(v1(t),t,2) diff(v1(t),t,3)]);

%%
% CBF作成 (FastBridge Non-cascaded ECBF: Optimized Chain-Rule)
%%
%% ================= C3BF Symbolic Derivation ================= %%
fprintf('--- 高次 C3BF (ECBF) シンボリック導出開始 (メモリ最適化版) ---\n');

syms px_obs py_obs pz_obs vx_obs vy_obs vz_obs robs lambda_cbf real
syms f_T_curr real 

p_obs = [px_obs; py_obs; pz_obs];
v_obs = [vx_obs; vy_obs; vz_obs];

p_rel = p_obs - p;
v_rel = v_obs - dp;

norm_p = sqrt(p_rel.' * p_rel);
norm_v = sqrt(v_rel.' * v_rel + 1e-6);

% シンボリック上は滑らかな代数式として展開 (diff を正常に通すため max は使用しない)
cos_phi = sqrt(norm_p^2 - robs^2) / norm_p;
h0 = (p_rel.' * v_rel) + norm_p * norm_v * cos_phi;

% クアッドコプター姿勢・幾何
R_mat = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2-q3^2];

z_B = R_mat(:, 3); % 推力方向ベクトル
w_I = R_mat * ob;  % 慣性系角速度

% 1. 各階層の物理運動
% 加速度 (u1: 総推力)
ddp_sym = [0; 0; -gravity] + (1/m) * z_B * u1;

% Jerk (角速度連成)
jerk_sym = (1/m) * f_T_curr * cross(w_I, z_B);

% Snap のトルク寄与行列 G_tau (3x3)
J_mat = diag([jx, jy, jz]);
S_zB = [   0,    -z_B(3),  z_B(2);
         z_B(3),    0,    -z_B(1);
        -z_B(2),  z_B(1),    0   ];

% トルク tau = [u2; u3; u4] に対する Snap の伝達行列 (3x3)
G_tau_snap = -(1/m) * f_T_curr * S_zB * R_mat * inv(J_mat);

% 2. 衝突円錐の勾配ベクトル (Chain Rule)
dh0_dp = pdiff(h0, p);
dh0_dv = pdiff(h0, dp); % 制御方向ベクトル w^T

% リー微分 (時間微分の近似展開: ドリフト成分)
Lfh0  = dh0_dp * dp + dh0_dv * [0; 0; -gravity];
Lf2h0 = pdiff(Lfh0, p) * dp + pdiff(Lfh0, dp) * jerk_sym;

% 3. 入力ゲインベクトル A_cbf (-Lg) の構築
% u1 (推力) に対する制約
A_u1 = -dh0_dv * ( (1/m) * z_B );

% u2, u3, u4 (トルク) に対する制約 (Snap 経由)
A_tau = -dh0_dv * G_tau_snap; % 1x3 行列

A_cbf_sym = [A_u1, A_tau];

% 4. 境界値ベクトル b_cbf
% ECBF 極配置: b_cbf = 3*lambda*Lf2h0 + 3*lambda^2*Lfh0 + lambda^3*h0
b_cbf_sym = 3*lambda_cbf*Lf2h0 + 3*(lambda_cbf^2)*Lfh0 + (lambda_cbf^3)*h0;

% 5. .m ファイル出力
fprintf('Step 4: 関数ファイルを出力中...\n');
obsParam = [px_obs, py_obs, pz_obs, vx_obs, vy_obs, vz_obs, robs];

matlabFunction(h0,        'file', 'ECBF_h.m',    'vars', {x, obsParam, physicalParam}, 'outputs', {'h'});
matlabFunction(A_cbf_sym, 'file', 'ECBF_Acbf.m', 'vars', {x, obsParam, physicalParam, f_T_curr}, 'outputs', {'A_cbf'});
matlabFunction(b_cbf_sym, 'file', 'ECBF_bcbf.m', 'vars', {x, obsParam, physicalParam, f_T_curr, lambda_cbf}, 'outputs', {'b_cbf'});

clear ECBF_h ECBF_Acbf ECBF_bcbf;

%% ================= 生成後ファイル動作・影響度チェック ================= %%
fprintf('\n========================================\n');
fprintf('  生成済み ECBF 関数の実動作テスト\n');
fprintf('========================================\n');

P_test = [0.027, 0.046, 0.046, 0.046, 0.046, 1.657e-5, 1.657e-5, 2.926e-5, 9.81, 1, 1, 1, 1, 1, 1, 1, 1];
f_T_test = 0.027 * 9.81; % ホバリング推力 (約 0.265 N)
lambda_test = 2.0;

% テスト状態: 姿勢傾斜（ロール・ピッチ15度）で接近中
q_tilt = Eul2Quat([deg2rad(15); deg2rad(10); 0]);
x_test = [q_tilt;   0; 0; 1;   1.0; 0.2; 0;   0.1; 0.1; 0];
obs_test = [1.5, 0.0, 1.0,   0.0, 0.0, 0.0,   0.3];

h_val = ECBF_h(x_test, obs_test, P_test);
A_val = ECBF_Acbf(x_test, obs_test, P_test, f_T_test);
b_val = ECBF_bcbf(x_test, obs_test, P_test, f_T_test, lambda_test);

fprintf('  h_val = % .4f\n', real(h_val));
fprintf('  b_cbf = % .4f\n', real(b_val));
fprintf('  A_cbf = [% .4e,  % .4e,  % .4e,  % .4e]\n\n', real(A_val(1)), real(A_val(2)), real(A_val(3)), real(A_val(4)));

input_labels = {'u1 (総推力)', 'u2 (ロールトルク)', 'u3 (ピッチトルク)', 'u4 (ヨートルク)'};
for i = 1:4
    if abs(real(A_val(i))) > 1e-6
        fprintf('  [OK] %s : 有効 (A_cbf(%d) = % .4e)\n', input_labels{i}, i, real(A_val(i)));
    else
        fprintf('  [確認] %s : 0\n', input_labels{i}, i);
    end
end
fprintf('========================================\n\n');

%%
% 姿勢制限 CBF 作成 (Attitude Limit ECBF: Roll & Pitch <= max_tilt)
%%
%% ================= Attitude CBF Symbolic Derivation ================= %%
fprintf('--- 姿勢角制限 ECBF (ロール・ピッチ制限) シンボリック導出開始 ---\n');

syms max_tilt gamma_att real
% max_tilt : 許容最大姿勢角 [rad] (例: deg2rad(15))
% gamma_att: 姿勢CBF極配置ゲイン (例: 3.0)

% 1. オイラー角 (ロール phi, ピッチ th) の導出 (スクリプト内定義と同一)
phi_sym = atan2((2*(q0*q1 + q2*q3)), (q0^2 - q1^2 - q2^2 + q3^2));
th_sym  = asin(max(-1.0, min(1.0, 2*(q0*q2 - q1*q3))));

% 2. バリア関数定義 h = [max_tilt - phi; phi + max_tilt; max_tilt - th; th + max_tilt]
h_att = [
    max_tilt - phi_sym;
    phi_sym + max_tilt;
    max_tilt - th_sym;
    th_sym + max_tilt
    ];

% 3. オイラー角速度のキネマティクス (角速度 ob = [o1; o2; o3] との関係)
dot_phi_sym = o1 + o2*sin(phi_sym)*tan(th_sym) + o3*cos(phi_sym)*tan(th_sym);
dot_th_sym  = o2*cos(phi_sym) - o3*sin(phi_sym);

dot_h_att = [
    -dot_phi_sym;
    dot_phi_sym;
    -dot_th_sym;
    dot_th_sym
    ];

% 4. 角加速度 (クアッドコプターのオイラー方程式)
% J * dot_ob = tau - ob x (J * ob)
% dot_ob1 (ロール軸)  = (u2 - (jz - jy)*o2*o3) / jx
% dot_ob2 (ピッチ軸)  = (u3 - (jx - jz)*o1*o3) / jy
% トルク入力 u = [u1; u2; u3; u4] に対するアフィン項 A_att (-Lg) の構築
% d2_phi/dt2 に対する u2 の係数: 1 / jx
% d2_th/dt2  に対する u3 の係数: cos(phi) / jy
A_att_sym = [
    % u1,      u2,                           u3,                     u4
    0,    1/jx,                            0,                      0; % phi <= max_tilt
    0,   -1/jx,                            0,                      0; % phi >= -max_tilt
    0,       0,          cos(phi_sym)/jy,                      0; % th  <= max_tilt
    0,       0,         -cos(phi_sym)/jy,                      0  % th  >= -max_tilt
    ];

% 境界値ベクトル b_att の構築 (ECBF 極配置: ddot_h + 2*gamma*dot_h + gamma^2*h >= 0)
% A_att * u <= b_att
b_att_sym = 2*gamma_att*dot_h_att + (gamma_att^2)*h_att;

% 5. .m ファイル出力
fprintf('姿勢制約関数ファイルを出力中...\n');
matlabFunction(h_att,     'file', 'Attitude_CBF_h.m',    'vars', {x, max_tilt}, 'outputs', {'h_att'});
matlabFunction(A_att_sym, 'file', 'Attitude_CBF_Acbf.m', 'vars', {x, physicalParam}, 'outputs', {'A_att'});
matlabFunction(b_att_sym, 'file', 'Attitude_CBF_bcbf.m', 'vars', {x, physicalParam, max_tilt, gamma_att}, 'outputs', {'b_att'});

clear Attitude_CBF_h Attitude_CBF_Acbf Attitude_CBF_bcbf;

%% ================= 生成後ファイル動作チェック ================= %%
fprintf('\n========================================\n');
fprintf('  生成済み 姿勢制限 CBF 関数の実動作テスト\n');
fprintf('========================================\n');

max_tilt_test = deg2rad(15); % 15度
gamma_att_test = 3.0;

% テスト状態: ロール12度、ピッチ10度で傾斜中
q_test = Eul2Quat([deg2rad(12); deg2rad(10); 0]);
x_test_att = [q_test;  0;0;1;  0;0;0;  0.1;0.1;0];

h_att_val = Attitude_CBF_h(x_test_att, max_tilt_test);
A_att_val = Attitude_CBF_Acbf(x_test_att, P_test);
b_att_val = Attitude_CBF_bcbf(x_test_att, P_test, max_tilt_test, gamma_att_test);

fprintf('  h_att (ロール上限余力) : % .4f rad (約 %0.2f deg)\n', h_att_val(1), rad2deg(h_att_val(1)));
fprintf('  h_att (ピッチ上限余力) : % .4f rad (約 %0.2f deg)\n', h_att_val(3), rad2deg(h_att_val(3)));
fprintf('  A_att サイズ           : [%d x %d]\n', size(A_att_val, 1), size(A_att_val, 2));
fprintf('  [OK] 姿勢制限関数 正常生成完了\n');
fprintf('========================================\n\n');

% %% Make functions of z
% % % If either model, virtual output or parameters is changed, then evaluate this section.
%     disp("Start: make functions of virtual states.");
%     matlabFunction(subs([h1;dh1], [xdRef], [XDf]),'file','Z1.m','vars',{x cell2sym(XD) physicalParam},'outputs',{'cZ1'});
%     matlabFunction(subs([h2;dh2;ddh2;dddh2], [xdRef vInput1], [XDf V1vf]),'file','Z2.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ2'});
%     matlabFunction(subs([h3;dh3;ddh3;dddh3], [xdRef vInput1], [XDf V1vf]),'file','Z3.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ3'});
%     matlabFunction(subs([h4;dh4], [xdRef vInput1], [XDf V1vf]),'file','Z4.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ4'});
% 
% %% Make functions of virtual inputs
% clc
%     disp("Start: make functions of virtual inputs.");
%     clear dt
%     syms f11 f12 f21 f22 f23 f24 f31 f32 f33 f34 f41 f42 dt k real
%     F1 = [f11 f12];
%     F2 = [f21 f22 f23 f24];
%     F3 = [f31 f32 f33 f34];
%     F4 = [f41 f42];
%     A1=[0,1;0,0]-[0;1]*F1; % closed loop : continuous
%     matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1*A1*[h1;dh1],-F1*A1*A1*A1*[h1;dh1]], [xdRef], [XDf]),'file','Vf.m','vars',{x cell2sym(XD) physicalParam F1},'outputs',{'V1'});
%     matlabFunction(subs([-F2*[h2;dh2;ddh2;dddh2],-F3*[h3;dh3;ddh3;dddh3],-F4*[h4;dh4]], [xdRef vInput1], [XDf V1vf]),'file','Vs.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam F2 F3 F4},'outputs',{'cV2'});
%     %%
% 
%     A1 = expm([0,1;0,0]*dt)-int(expm([0,1;0,0]*(dt-k))*[0;1],k,[0,dt])*F1; % closed loop discrete
%     matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1*A1*[h1;dh1],-F1*A1*A1*A1*[h1;dh1]], [xdRef], [XDf]),'file','Vfd.m','vars',{dt x cell2sym(XD) physicalParam F1},'outputs',{'V1'});
%     A2 = expm(diag([1,1,1],1)*dt)-int(expm(diag([1,1,1],1)*(dt-k))*[0;0;0;1],k,[0,dt])*F2; % closed loop discrete
%     A3 = expm(diag([1,1,1],1)*dt)-int(expm(diag([1,1,1],1)*(dt-k))*[0;0;0;1],k,[0,dt])*F3; % closed loop discrete
%     A4 = expm([0 1;0 0]*dt)-int(expm([0 1;0 0]*(dt-k))*[0;1],k,[0,dt])*F4; % closed loop discrete
%     matlabFunction(subs([-F2*[h2;dh2;ddh2;dddh2],-F3*[h3;dh3;ddh3;dddh3],-F4*[h4;dh4]], [xdRef vInput1], [XDf V1vf]),'file','Vsd.m','vars',{dt x cell2sym(XD) cell2sym(V1v) physicalParam F2 F3 F4},'outputs',{'cV2'});
% 
%     % % For check
% %     Vf(0,x0,Xd(0))
% %     Vs(0,x0,Xd(0),Vf(0,x0,Xd(0)))
% %% Make functions of actual inputs taking t, x, xd, v1 and v2 as arguments
% % % If either model, virtual output or parameters is changed, then evaluate this section. It'll take few minutes.
% % % Usage: u = Uf(...) + Us(...)
%     matlabFunction(subs(H(:,1)*(-alpha1+v1(t)), [xdRef vInput1], [XDf V1vf]),'file','Uf.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'U1'});
%     matlabFunction(subs(H(:,2:4)*U2, [xdRef vInput1 v2(t) v3(t) v4(t)], [XDf V1vf [V2 V3 V4]]),'file','Us.m','vars',{x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'U2'});
%     %matlabFunction(subs(He(:,1)*(-alpha1+v1(t)), [xdRef vInput1], [XDf V1vf]),'file','Ufe.m','vars',{t x cell2sym(XD) cell2sym(V1v) [physicalParam,e1,e2]},'outputs',{'cU1'});
%     %matlabFunction(subs(He(:,2:4)*U2e, [xdRef vInput1 v2(t) v3(t) v4(t)], [XDf V1vf [V2 V3 V4]]),'file','Use.m','vars',{t x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] [physicalParam,e1,e2]},'outputs',{'cU2'});
% % % For check
% %     Uf(0,x0,Xd(0),Vf(0,x0,Xd(0)))
% %     Us(0,x0,Xd(0),Vf(0,x0,Xd(0)),Vs(0,x0,Xd(0),Vf(0,x0,Xd(0))))
% % %%
% % matlabFunction(subs(alpha1, [xdRef], [XDf]),'file','alpha1.m','vars',{cell2sym(XD) physicalParam},'outputs',{'al1'});
% % matlabFunction(subs(alpha2, [xdRef vInput1], [XDf V1vf]),'file','alpha2.m','vars',{t x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'al2'});
% % %%
% % matlabFunction(subs(beta2, [xdRef vInput1], [XDf V1vf]),'file','beta2.m','vars',{t x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'be2'});
% % %%
% % matlabFunction(subs(He, [xdRef vInput1], [XDf V1vf]),'file','He.m','vars',{t x cell2sym(XD) cell2sym(V1v) e1 physicalParam},'outputs',{'mat'});
% % %%
% % matlabFunction(beta1,'file','beta1.m','vars',{t x physicalParam},'outputs',{'beta1'});

%% Local functions
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