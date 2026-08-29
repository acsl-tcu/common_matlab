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
dp	= [ dp1; dp2; dp3];             % Velocity
ddp	= [ddp1;ddp2;ddp3];             % Accelaletion
q	= [  q0;  q1;  q2;  q3];        % Quaternion
ob	= [  o1;  o2;  o3];             % Angular velocity
pl  = [ pl1; pl2; pl3];             % Load position
dpl = [dpl1;dpl2;dpl3];             % Load velocity
ol  = [ ol1; ol2; ol3];             % Load angular velocity
pT  = [ pT1; pT2; pT3];             % String position
x=[q;ob;pl;dpl;pT;ol];
% physicalParam = [m, Lx, Ly lx ly, jx, jy, jz, gravity, km1, km2, km3, km4, k1, k2, k3, k4, rotor_r, mL, cableL];
% f = FL(x,physicalParam);
% g = GL(x,physicalParam);
% physicalParam = [m, jx, jy, jz, gravity,mL,cableL];

syms dstx dsty real
physicalParam = [m, Lx, Ly lx ly, jx, jy, jz, gravity, km1, km2, km3, km4, k1, k2, k3, k4, rotor_r, mL, cableL, dstx, dsty];

f = FLxyDst(x,physicalParam);
g = GLxyDst(x,physicalParam);
physicalParam = [m, jx, jy, jz, gravity,mL,cableL,dstx, dsty];
%% 1st layer
clc
% % Define virtual output: h1
h1 = pl3 - xd(3);
dh1 = LieD(h1,f,x)+diff(h1,t);
alpha1 = LieD(dh1,f,x)+diff(dh1,t);
beta1 = simplify(LieD(dh1,g,x));
H = [1/beta1(1),-beta1(2)/beta1(1),-beta1(3)/beta1(1),-beta1(4)/beta1(1); zeros(3,1),eye(3)];
%%
% syms e1 e2 real
% He = [1/(beta1(1)+e1),-beta1(2)/(beta1(1)+e1),-beta1(3)/(beta1(1)+e1),-beta1(4)/(beta1(1)+e1); zeros(3,1),eye(3)];
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
simplify(FG-(f1+g1*[u2;u3;u4]))   % For check

% FG = simplify(f+g*(H*(-alpha1+v1(t))+nH*[v2;v3;v4]));	% v1を後で設計する時はこっち
% g1 = simplify(MyCoeff(FG,[v2;v3;v4]));
% f1 = subs(FG,[v2,v3,v4],[0,0,0]);

%%
% % Define virtual output: h2, h3, h4
h2 = pl1 - xd(1);
h3 = pl2 - xd(2);
[~,~,yaw] = Quat2Eul(q);
h4 = yaw - xd(4);

%% **************************************** % %
clc
dh4 = LieD(h4,f1,x)+diff(h4,t);
dh2 = LieD(h2,f1,x)+diff(h2,t);
dh3 = LieD(h3,f1,x)+diff(h3,t);
d2h2 = LieD(dh2,f1,x)+diff(dh2,t);
d2h3 = LieD(dh3,f1,x)+diff(dh3,t);
d3h2 = LieD(d2h2,f1,x)+diff(d2h2,t);
d3h3 = LieD(d2h3,f1,x)+diff(d2h3,t);
d4h2 = LieD(d3h2,f1,x)+diff(d3h2,t);
d4h3 = LieD(d3h3,f1,x)+diff(d3h3,t);
d5h2 = LieD(d4h2,f1,x)+diff(d4h2,t);
d5h3 = LieD(d4h3,f1,x)+diff(d4h3,t);
%% % For check
	[LieD(h2,g1,x),LieD(h3,g1,x),LieD(h4,g1,x)]
	simplify([LieD(dh2,g1,x),LieD(dh3,g1,x)])
	simplify([LieD(d2h2,g1,x),LieD(d2h3,g1,x)])
  	simplify([LieD(d3h2,g1,x),LieD(d3h3,g1,x)])
 	simplify([LieD(d4h2,g1,x),LieD(d4h3,g1,x)])
 	% simplify([LieD(d5h2,g1,x),LieD(d5h3,g1,x)])
%% trial
% simplify してからmatlabFunctionは意味がない（というか遅くなった）
% a2_1=simplify(LieD(d5h2,f1,x)+diff(d5h2,t));
% matlabFunction(subs(a2_1,[xdReff vInput1f], [XDf V1vf]),"file",'test1.m','Vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'v2_alpha2'})
% matlabFunction(subs(LieD(d5h2,f1,x)+diff(d5h2,t),[xdReff vInput1f], [XDf V1vf]),"file",'test2.m','Vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'v2_alpha2'})
%%
% 2+6+6+2
% % Derive 2nd layer controller
% alpha2 = [LieD(d5h2,f1,x)+diff(d5h2,t); LieD(d5h3,f1,x)+diff(d5h3,t)];
% beta2 = [LieD(d5h2,g1,x); LieD(d5h3,g1,x)];
% H2 = pinv(beta2);
% nb2 = null(beta2);
alpha2 = [LieD(d5h2,f1,x)+diff(d5h2,t); LieD(d5h3,f1,x)+diff(d5h3,t); LieD(dh4,f1,x)+diff(dh4,t)];
beta2 = [LieD(d5h2,g1,x); LieD(d5h3,g1,x); LieD(dh4,g1,x)];
%%
% FG2 = simplify(f1+g1*(H2*(-alpha2+[v2(t);v3(t)])+nb2*[v4]));	% v1を後で設計する時はこっち
% g2 = simplify(MyCoeff(FG2,[v4]));
% f2 = subs(FG,v4,0);
% syms v2(t) v3(t) v4(t)
    % salpha = simplify(alpha2);
    % sbeta2 = simplify(beta2);
    % U2 = sbeta2\(-salpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
    % U2 = inv(beta2)\(-alpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
%     U2e = (adjoint(beta2)/(det(beta2)+e2))*(-alpha2+[v2(t);v3(t);v4(t)]);  % v2を後で設計する時はこっち
%end
%% Initialize xd as an unspecified function of t
% % If regenerate Uf, Us or Xd functions, evaluate this section.
    xd = [xd1(t),xd2(t),xd3(t),xd4(t)];
    dxd = diff(xd,t);
    d2xd = diff(dxd,t);
    d3xd = diff(d2xd,t);
    d4xd = diff(d3xd,t);
    d5xd = diff(d4xd,t);
    d6xd = diff(d5xd,t);
%% Set variables for output functions
%fix
    syms Xd1 Xd2 Xd3 Xd4 dXd1 dXd2 dXd3 dXd4 d2Xd1 d2Xd2 d2Xd3 d2Xd4 d3Xd1 d3Xd2 d3Xd3 d3Xd4 d4Xd1 d4Xd2 d4Xd3 d4Xd4 d5Xd1 d5Xd2 d5Xd3 d5Xd4 d6Xd1 d6Xd2 d6Xd3 d6Xd4 real
    syms V1 V2 V3 V4 dV1 d2V1 d3V1 d4V1 d5V1 real
    XD = {Xd1 Xd2 Xd3 Xd4 dXd1 dXd2 dXd3 dXd4 d2Xd1 d2Xd2 d2Xd3 d2Xd4 d3Xd1 d3Xd2 d3Xd3 d3Xd4 d4Xd1 d4Xd2 d4Xd3 d4Xd4 d5Xd1 d5Xd2 d5Xd3 d5Xd4 d6Xd1 d6Xd2 d6Xd3 d6Xd4};
    V1v = {V1 dV1 d2V1 d3V1 d4V1 d5V1};
    xdRef = [xd dxd d2xd d3xd d4xd d5xd d6xd];
    vInput1 = [v1(t) diff(v1(t),t) diff(v1(t),t,2) diff(v1(t),t,3) diff(v1(t),t,4) diff(v1(t),t,5)];
    XDf = flip(XD);
    V1vf = flip(V1v);
    xdReff = flip(xdRef);
    vInput1f = flip(vInput1);
 %% Make functions of virtual output
% % If either model, virtual output or parameters is changed, then evaluate this section.
    disp("Start: make functions of virtual states.");
    % matlabFunction(subs([h1;dh1], [xdReff], [XDf]),'file','Z1_SuspendedLoad.m','vars',{obj x cell2sym(XD) physicalParam},'outputs',{'cZ1'});
    % matlabFunction(subs([h2;dh2;d2h2;d3h2;d4h2;d5h2], [xdReff vInput1f], [XDf V1vf]),'file','Z2_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ2'});
    % matlabFunction(subs([h3;dh3;d2h3;d3h3;d4h3;d5h3], [xdReff vInput1f], [XDf V1vf]),'file','Z3_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ3'});
    % matlabFunction(subs([h4;dh4], [xdReff vInput1f], [XDf V1vf]),'file','Z4_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ4'});
    %xydst用
    matlabFunction(subs([h1;dh1], [xdReff], [XDf]),'file','Z1_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) physicalParam},'outputs',{'cZ1'});
    matlabFunction(subs([h2;dh2;d2h2;d3h2;d4h2;d5h2], [xdReff vInput1f], [XDf V1vf]),'file','Z2_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ2'});
    matlabFunction(subs([h3;dh3;d2h3;d3h3;d4h3;d5h3], [xdReff vInput1f], [XDf V1vf]),'file','Z3_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ3'});
    matlabFunction(subs([h4;dh4], [xdReff vInput1f], [XDf V1vf]),'file','Z4_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'cZ4'});
%% Make functions of virtual inputs
clc
    disp("Start: make functions of virtual inputs.");
    clear dt
    syms f11 f12 f21 f22 f23 f24 f25 f26 f31 f32 f33 f34 f35 f36 f41 f42 dt k real
    F1 = [f11 f12];
    F2 = [f21 f22 f23 f24 f25 f26];
    F3 = [f31 f32 f33 f34 f35 f36];
    F4 = [f41 f42];
    A1=[0,1;0,0]-[0;1]*F1; % closed loop : continuous
    % matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1^2*[h1;dh1],-F1*A1^3*[h1;dh1],-F1*A1^4*[h1;dh1],-F1*A1^5*[h1;dh1]], [xdRef], [XD]),'file','Vf_SuspendedLoad.m','vars',{x cell2sym(XD) physicalParam F1},'outputs',{'V1'});
    % matlabFunction(subs([-F2*[h2;dh2;d2h2;d3h2;d4h2;d5h2],-F3*[h3;dh3;d2h3;d3h3;d4h3;d5h3],-F4*[h4;dh4]], [xdRef vInput1], [XD V1v]),'file','Vs_SuspendedLoad.m','vars',{x cell2sym(XD) cell2sym(V1v) physicalParam F2 F3 F4},'outputs',{'cV2'});

    A1 = expm([0,1;0,0]*dt)-int(expm([0,1;0,0]*(dt-k))*[0;1],k,[0,dt])*F1; % closed loop discrete
    % matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1^2*[h1;dh1],-F1*A1^3*[h1;dh1],-F1*A1^4*[h1;dh1],-F1*A1^5*[h1;dh1]], [xdRef], [XD]),'file','Vfd_SuspendedLoad.m','vars',{dt x cell2sym(XD) physicalParam F1},'outputs',{'Vd1'});
    A2 = expm(diag([1,1,1,1,1],1)*dt)-int(expm(diag([1,1,1,1,1],1)*(dt-k))*[0;0;0;0;0;1],k,[0,dt])*F2; % closed loop discrete
    A3 = expm(diag([1,1,1,1,1],1)*dt)-int(expm(diag([1,1,1,1,1],1)*(dt-k))*[0;0;0;0;0;1],k,[0,dt])*F3; % closed loop discrete
    A4 = expm([0 1;0 0]*dt)-int(expm([0 1;0 0]*(dt-k))*[0;1],k,[0,dt])*F4; % closed loop discrete
    
    %xydst用
    A1=[0,1;0,0]-[0;1]*F1; % closed loop : continuous
    matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1^2*[h1;dh1],-F1*A1^3*[h1;dh1],-F1*A1^4*[h1;dh1],-F1*A1^5*[h1;dh1]], [xdReff], [XDf]),'file','Vf_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) F1},'outputs',{'V1'});
    A1 = expm([0,1;0,0]*dt)-int(expm([0,1;0,0]*(dt-k))*[0;1],k,[0,dt])*F1; % closed loop discrete
    matlabFunction(subs([-F1*[h1;dh1],-F1*A1*[h1;dh1],-F1*A1^2*[h1;dh1],-F1*A1^3*[h1;dh1],-F1*A1^4*[h1;dh1],-F1*A1^5*[h1;dh1]], [xdReff], [XDf]),'file','Vfd_SuspendedLoadxyDst.m','vars',{obj dt x cell2sym(XD) F1},'outputs',{'Vd1'});
    matlabFunction(subs([-F2*[h2;dh2;d2h2;d3h2;d4h2;d5h2],-F3*[h3;dh3;d2h3;d3h3;d4h3;d5h3],-F4*[h4;dh4]], [xdReff vInput1f], [XDf V1vf]),'file','Vs_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam F2 F3 F4},'outputs',{'cV2'});

    % % For check
%     Vf(0,x0,Xd(0))
%     Vs(0,x0,Xd(0),Vf(0,x0,Xd(0)))

%% =========================================================================
%% 【C3-ECBF】単機牽引モデル（ドローン機体本体）3次 Exponential 衝突円錐 CBF 自動導出
%% =========================================================================
disp("Start: 単機牽引モデルにおける機体本体 C3-ECBF の導出を開始します。");

% 1. シンボリック変数の定義
syms u1 u2 u3 u4 real          % u1: 推力 f, [u2, u3, u4]: トルク M
syms xo yo zo vxo vyo vzo real % 障害物の位置・速度
syms ro rq d_safe real         % 半径パラメータ (ro: 障害物, rq: クアッドロータ, d_safe: 安全マージン)
syms lambda real               % 3重極ゲイン (論文 λ)
syms U1_ref real               % 動作点推力 (トルク感度保持用)

% 2. 物理パラメータの抽出 (状態方程式との対応付け)
% physicalParam = [m, jx, jy, jz, gravity, mL, cableL, dstx, dsty];
m_Q     = physicalParam(1);
J_Q_vec = physicalParam(2:4);
g_acc   = physicalParam(5);
m_L     = physicalParam(6);
L_cable = physicalParam(7);

% 3. 入力列の厳密な分離 (19x1, 19x2, 19x1)
g1  = g(:, 1);    % 推力 f に対する入力列 (19x1)
g23 = g(:, 2:3);  % Roll/Pitch トルク [Mx, My] に対する入力列 (19x2)
g4  = g(:, 4);    % Yaw トルク Mz に対する入力列 (19x1)

% 4. 機体本体の位置 p_drone (pQ) および 速度 v_drone (vQ) の幾何関係
% 状態配置: pl = x(8:10), dpl = x(11:13), pT = x(14:16), ol = x(17:19)
p_drone = pl(:) - L_cable * pT(:);
v_drone = dpl(:) - L_cable * Skew(ol(:)) * pT(:);

p_obs   = [xo; yo; zo];
v_obs   = [vxo; vyo; vzo];
r_safe  = ro + rq + d_safe;

% 5. 衝突円錐バリア関数 h(x) の定義 (ψ0 = h)
p_rel = p_obs - p_drone;
v_rel = v_obs - v_drone;

norm_p_sq = p_rel.' * p_rel;
norm_v_sq = v_rel.' * v_rel;
inner_pv  = p_rel.' * v_rel;
norm_v    = sqrt(norm_v_sq);

rad_expr = sqrt(max(1e-4, norm_p_sq - r_safe^2));
h_0      = inner_pv + norm_v * rad_expr; % ψ0

% 6. 1階微分 h_dot (ψ1) の自律項 L_f_h
eta   = p_rel + v_rel * (rad_expr / max(1e-4, norm_v));
sigma = norm_v_sq + (norm_v * inner_pv) / rad_expr;

L_f_vdrone = LieD(v_drone, f, x);
h_1        = simplify(- eta.' * L_f_vdrone + sigma); % L_f h (ψ1)

% 7. 2階微分 h_ddot (ψ2) の自律項 L_f^2_h
h_2 = simplify(LieD(h_1, f, x)); % L_f^2 h (ψ2)

% 8. 3階微分 h_dddot (ψ3) の自律項および入力感度 (Snap レベル)
L_f3_h = simplify(LieD(h_2, f, x));

% 4入力に対する各 Lie 微分
L_g1_raw  = simplify(LieD(h_2, g1, x));   % 推力 u1 係数 (1x1)
L_g23_raw = simplify(LieD(h_2, g23, x));  % Roll/Pitch [u2, u3] 係数 (1x2)
L_g4_raw  = simplify(LieD(h_2, g4, x));   % Yaw u4 係数 (1x1)

% 動作点推力 U1_ref の代入によるトルク感度の確定
L_g1_eval  = simplify(subs(L_g1_raw, u1, U1_ref));
L_g23_eval = simplify(subs(L_g23_raw, u1, U1_ref));
L_g4_eval  = simplify(subs(L_g4_raw, u1, U1_ref));

% 4入力結合感度行ベクトル (1x4)
L_g_all = [L_g1_eval, L_g23_eval, L_g4_eval];

%% =========================================================================
%% 9. 3次 ECBF-QP 制約の構築 (A_qp * u <= b_qp)
%% =========================================================================
% 制約: L_f^3 h + L_g L_f^2 h * u + 3*lambda*h_2 + 3*lambda^2*h_1 + lambda^3*h_0 >= 0[cite: 6]
A_c3ecbf_sym = - L_g_all; % (1x4)
b_c3ecbf_sym = simplify(L_f3_h + 3*lambda*h_2 + 3*(lambda^2)*h_1 + (lambda^3)*h_0);

%% =========================================================================
%% 10. 構造診断
%% =========================================================================
disp("------------------------------------------------------------");
disp("【単機牽引・機体本体 C3-ECBF 構造診断】");
fprintf("size(A_qp) = [%d, %d] (期待値: [1, 4])\n", size(A_c3ecbf_sym, 1), size(A_c3ecbf_sym, 2));
disp("A_qp (推力 u1 係数)          = "); disp(A_c3ecbf_sym(1));
disp("A_qp (Roll/Pitch u2,u3 係数) = "); disp(A_c3ecbf_sym(2:3));
disp("A_qp (Yaw u4 係数)           = "); disp(A_c3ecbf_sym(4));
disp("------------------------------------------------------------");

%% =========================================================================
%% 11. Mファイルとしてエクスポート
%% =========================================================================
obs_params = [xo; yo; zo; vxo; vyo; vzo; ro];
sys_params = [rq; d_safe; lambda];

if exist('XD', 'var')
    XD_sym = cell2sym(XD);
    XD_sym = XD_sym(:);
else
    XD_sym = sym('dummy_xd', [1 1]);
end

export_fname = 'CBF_Constraints_C3ECBF_SuspendedLoadDroneBody';
disp(['Exporting: ', export_fname, '.m を書き出しています...']);
matlabFunction(A_c3ecbf_sym, b_c3ecbf_sym, h_0, h_1, h_2, ...
    'file', strcat(export_fname, '.m'), ...
    'vars', {obj, x, XD_sym, U1_ref, obs_params, sys_params, physicalParam}, ...
    'outputs', {'A_qp', 'b_qp', 'h0_val', 'h1_val', 'h2_val'});

% 不要コメント行・空行のクリーンアップ
DeleteCommentLine(export_fname);

disp("Done: 単機牽引モデル（機体本体）C3-ECBF 関数の生成が完了しました！");


% %% =========================================================================
% %% 【Non-cascaded ECBF】完全展開・構造診断・アフィン化 HOCBF 自動導出
% %% =========================================================================
% disp("Start: 完全展開・構造診断による Non-cascaded ECBF 導出を開始します。");
% 
% syms u1 u2 u3 u4 real
% syms U1_val real
% syms xo yo zo ro rl real
% syms gamma1 gamma2 gamma3 gamma4 real
% 
% % 1. 入力行列の列分割
% g1    = g(:, 1);     % 推力 u1 に対する列 (19x1)
% g23 = g(:, 2:3);   % トルク u2; u3 に対する列 (19x2)
% g4 = g(:, 4);   % トルク u4 に対する列 (19x1)
% 
% % 2. 障害物安全関数の定義 (牽引紐の中点 p_mid)
% L_cable = physicalParam(7);
% p_mid = pl - 0.5 * L_cable * pT;
% 
% % 1階 (安全関数)
% h1 = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2;
% %% =========================================================================
% %% ステップ 1: 1階微分 (h2) の導出
% %% =========================================================================
% % h1 のドリフト項に対する Lie 微分
% h1_dot   = LieD(h1, f, x);
% L_g1_h1  = LieD(h1, g1, x);
% L_g23_h1 = LieD(h1, g23, x);
% L_g4_h1  = LieD(h1, g4, x);
% 
% % 1階の式 h2 の構築
% h2 = simplify(h1_dot + gamma1 * h1);
% 
% % 1階における入力非依存性の確認 (すべて 0 であるべき)
% disp("------------------------------------------------------------");
% disp("【1階の構造診断】(期待値: すべてゼロ)");
% disp("L_g1_h1  = "); disp(simplify(L_g1_h1));
% disp("L_g23_h1 = "); disp(simplify(L_g23_h1));
% disp("L_g4_h1  = "); disp(simplify(L_g4_h1));
% disp("------------------------------------------------------------");
% 
% %% =========================================================================
% %% ステップ 2: 2階微分 (h3) の導出
% %% =========================================================================
% % h2 の各ベクトル場に対する Lie 微分
% h2_dot   = LieD(h2, f, x);
% L_g1_h2  = LieD(h2, g1, x);    % 推力 u1 に対する感度 (スカラー)
% L_g23_h2 = LieD(h2, g23, x);   % Roll/Pitch [u2; u3] に対する感度 (1x2 行列)
% L_g4_h2  = LieD(h2, g4, x);    % Yaw u4 に対する感度 (スカラー)
% 
% % ドリフト項 C2 と 入力係数の抽出
% C2     = simplify(h2_dot + gamma2 * h2);
% A2_u1  = simplify(L_g1_h2);
% A2_u23 = simplify(L_g23_h2);
% A2_u4  = simplify(L_g4_h2);
% 
% % 2階の完全式 (h3_full)
% h3 = simplify(C2 + A2_u1 * u1 + A2_u23 * [u2; u3] + A2_u4 * u4);
% 
% % 2階における構造診断
% disp("------------------------------------------------------------");
% disp("【2階の構造診断】(期待値: A2_u1 は非ゼロ, トルク項はすべてゼロ)");
% disp("A2_u1  (推力 u1 係数)       = "); disp(A2_u1);
% disp("A2_u23 (Roll/Pitch 係数)   = "); disp(A2_u23);
% disp("A2_u4  (Yaw 係数)          = "); disp(A2_u4);
% disp("------------------------------------------------------------");
% isZero_A2_u23 = all(isAlways(A2_u23 == 0));
% isZero_A2_u4  = isAlways(A2_u4 == 0);
% 
% fprintf("A2_u23 = 0 : %d\n", isZero_A2_u23);
% fprintf("A2_u4  = 0 : %d\n", isZero_A2_u4);
% if isZero_A2_u23 && isZero_A2_u4
%     disp("✓ h3 は C2 + A2_u1*u1 の構造です。");
% else
%     disp("⚠ h3 にトルク項が存在するため、一般形で h4 を展開します。");
% end
% %% =========================================================================
% %% ステップ 3: 3階微分 (h4) の導出 (dot{u1} = 0 の適用)
% %% =========================================================================
% % 1. C2 (状態ドリフト項) に対する各ベクトル場の Lie 微分
% L_f_C2   = LieD(C2, f, x);
% L_g1_C2  = LieD(C2, g1, x);
% L_g23_C2 = LieD(C2, g23, x);  % [1 x 2] 行列
% L_g4_C2  = LieD(C2, g4, x);   % スカラー
% 
% % 2. A2_u1 (推力係数項) に対する各ベクトル場の Lie 微分
% L_f_A2   = LieD(A2_u1, f, x);
% L_g1_A2  = LieD(A2_u1, g1, x);
% L_g23_A2 = LieD(A2_u1, g23, x); % [1 x 2] 行列
% L_g4_A2  = LieD(A2_u1, g4, x);  % スカラー
% 
% % 3. 各成分の整理 (クラスK項 gamma3 * h3 を含む)
% C3      = simplify(L_f_C2 + gamma3 * C2);
% A3_u1   = simplify(L_g1_C2 + L_f_A2 + gamma3 * A2_u1);
% A3_u23  = simplify(L_g23_C2);
% A3_u4   = simplify(L_g4_C2);
% B3_u1u1 = simplify(L_g1_A2);
% B3_u123 = simplify(L_g23_A2);
% B3_u14  = simplify(L_g4_A2);
% 
% % 3階の完全式 (h4)
% h4 = simplify(C3 + A3_u1 * u1 + A3_u23 * [u2; u3] + A3_u4 * u4 + ...
%     B3_u1u1 * u1^2 + u1 * (B3_u123 * [u2; u3] + B3_u14 * u4));
% 
% % 4. 3階における構造診断
% disp("------------------------------------------------------------");
% disp("【3階の構造診断】(期待値: A3_u1 は非ゼロ, トルク項・2次項はすべてゼロ)");
% disp("A3_u1   (推力 u1 係数)       = "); disp(A3_u1);
% disp("A3_u23  (Roll/Pitch 直接項)  = "); disp(A3_u23);
% disp("A3_u4   (Yaw 直接項)         = "); disp(A3_u4);
% disp("B3_u1u1 (u1^2 二次項)        = "); disp(B3_u1u1);
% disp("B3_u123 (u1*Roll/Pitch 積)   = "); disp(B3_u123);
% disp("B3_u14  (u1*Yaw 積)          = "); disp(B3_u14);
% disp("------------------------------------------------------------");
% % --- 各項がゼロかを厳密に判定 ---
% isZero_A3_u1 = isAlways(A3_u1 == 0);
% isZero_A3_u23 = all(isAlways(A3_u23 == 0));
% isZero_A3_u4 = isAlways(A3_u4 == 0);
% isZero_B3_u1u1 = isAlways(B3_u1u1 == 0);
% isZero_B3_u123 = all(isAlways(B3_u123 == 0));
% isZero_B3_u14 = isAlways(B3_u14 == 0);
% 
% disp("【3階 構造ゼロ判定結果】");
% 
% fprintf("A3_u1 = 0 : %d\n", isZero_A3_u1);
% fprintf("A3_u23 = 0 : %d\n", isZero_A3_u23);
% fprintf("A3_u4 = 0 : %d\n", isZero_A3_u4);
% fprintf("B3_u1u1 = 0 : %d\n", isZero_B3_u1u1);
% fprintf("B3_u123 = 0 : %d\n", isZero_B3_u123);
% fprintf("B3_u14 = 0 : %d\n", isZero_B3_u14);
% 
% disp("------------------------------------------------------------");
% if ~isZero_A3_u1 && ...
%         isZero_A3_u23 && ...
%         isZero_A3_u4 && ...
%         isZero_B3_u1u1 && ...
%         isZero_B3_u123 && ...
%         isZero_B3_u14
% 
%     disp("✓ 論文と同じ3階入力構造を確認しました。");
%     disp("  h4 = C3 + A3_u1*u1");
% end
% %% =========================================================================
% %% ステップ 4: 4階微分 (h5) の導出 (dot{u1} = 0 の適用)
% %% =========================================================================
% % 1. C3 (状態ドリフト項) に対する各ベクトル場の Lie 微分
% L_f_C3   = LieD(C3, f, x);
% L_g1_C3  = LieD(C3, g1, x);
% L_g23_C3 = LieD(C3, g23, x);  % [1 x 2] 行列
% L_g4_C3  = LieD(C3, g4, x);   % スカラー
% 
% % 2. A3_u1 (推力係数項) に対する各ベクトル場の Lie 微分
% L_f_A3   = LieD(A3_u1, f, x);
% L_g1_A3  = LieD(A3_u1, g1, x);
% L_g23_A3 = LieD(A3_u1, g23, x); % [1 x 2] 行列 (Roll/Pitch 双線形項)
% L_g4_A3  = LieD(A3_u1, g4, x);  % スカラー (Yaw 双線形項)
% 
% % 3. 4階ベース成分の整理 (クラスK項 gamma4 * h4 を含む)
% C4_base     = simplify(L_f_C3 + gamma4 * C3);
% A4_u1_base  = simplify(L_g1_C3 + L_f_A3 + gamma4 * A3_u1);
% A4_u23_dir  = simplify(L_g23_C3);
% A4_u4_dir   = simplify(L_g4_C3);
% B4_u1u1     = simplify(L_g1_A3);
% B4_u123     = simplify(L_g23_A3);
% B4_u14      = simplify(L_g4_A3);
% 
% % 4階の完全式 (h5_full)
% h5_full = simplify(C4_base + A4_u1_base * u1 + A4_u23_dir * [u2; u3] + A4_u4_dir * u4 + ...
%     B4_u1u1 * u1^2 + u1 * (B4_u123 * [u2; u3] + B4_u14 * u4));
% 
% % 4. 4階における構造診断
% disp("------------------------------------------------------------");
% disp("【4階の構造診断】(期待値: Roll/Pitch双線形項 B4_u123 が非ゼロ)");
% disp("A4_u1_base  (推力 u1 1次係数)      = "); disp(A4_u1_base);
% disp("A4_u23_dir  (Roll/Pitch 直接項)   = "); disp(A4_u23_dir);
% disp("A4_u4_dir   (Yaw 直接項)          = "); disp(A4_u4_dir);
% disp("B4_u123     (u1*Roll/Pitch 双線形) = "); disp(B4_u123);
% disp("B4_u14      (u1*Yaw 双線形)       = "); disp(B4_u14);
% disp("B4_u1u1     (u1^2 二次項)         = "); disp(B4_u1u1);
% disp("------------------------------------------------------------");
% % --- 各項がゼロかを厳密に判定 ---
% isZero_A4_u1 = isAlways(A4_u1_base == 0);
% isZero_A4_u23 = all(isAlways(A4_u23_dir == 0));
% isZero_A4_u4 = isAlways(A4_u4_dir == 0);
% isZero_B4_u1u1 = isAlways(B4_u1u1 == 0);
% isZero_B4_u123 = all(isAlways(B4_u123 == 0));
% isZero_B4_u14 = isAlways(B4_u14 == 0);
% 
% disp("【4階 構造ゼロ判定結果】");
% 
% fprintf("A4_u1_base = 0 : %d\n", isZero_A4_u1);
% fprintf("A4_u23_dir = 0 : %d\n", isZero_A4_u23);
% fprintf("A4_u4_dir = 0 : %d\n", isZero_A4_u4);
% fprintf("B4_u1u1 = 0 : %d\n", isZero_B4_u1u1);
% fprintf("B4_u123 = 0 : %d\n", isZero_B4_u123);
% fprintf("B4_u14 = 0 : %d\n", isZero_B4_u14);
% 
% disp("------------------------------------------------------------");
% if isZero_A4_u23 && ...
%         isZero_A4_u4 && ...
%         isZero_B4_u1u1 && ...
%         ~isZero_B4_u123 && ...
%         isZero_B4_u14
% 
%     disp("✓ 論文と同じ4階入力構造を確認しました。");
%     disp("  Roll/Pitch トルクが");
%     disp("  u1 * B4_u123 * [u2; u3]");
%     disp("  の双線形項として出現しています。");
% end
% %% =========================================================================
% %% ステップ 5: ノミナル入力周りでの 1次 Taylor アフィン化 & スタック制約構成
% %% =========================================================================
% syms U1_nom U2_nom U3_nom U4_nom real
% u_sym = [u1; u2; u3; u4];
% u_nom = [U1_nom; U2_nom; U3_nom; U4_nom];
% 
% % --- 2階制約 (h3 >= 0) のアフィン化 ---
% h3_nom = subs(h3, u_sym, u_nom);
% J3     = subs(jacobian(h3, u_sym), u_sym, u_nom);
% A_v2   = -J3;
% b_v2   = simplify(h3_nom - J3 * u_nom);
% 
% % --- 3階制約 (h4 >= 0) のアフィン化 ---
% h4_nom = subs(h4, u_sym, u_nom);
% J4     = subs(jacobian(h4, u_sym), u_sym, u_nom);
% A_v3   = -J4;
% b_v3   = simplify(h4_nom - J4 * u_nom);
% 
% % --- 4階制約 (h5_full >= 0) のアフィン化 ---
% h5_nom = subs(h5_full, u_sym, u_nom);
% J5     = subs(jacobian(h5_full, u_sym), u_sym, u_nom);
% A_v4   = -J5;
% b_v4   = simplify(h5_nom - J5 * u_nom);
% 
% % --- 3段スタック制約行列 [3 x 4] ---
% A_cbf_sym = [A_v2; A_v3; A_v4];
% b_cbf_sym = [b_v2; b_v3; b_v4];
% 
% % 係数行列のサイズと構造確認
% disp("------------------------------------------------------------");
% disp("【制約行列 A_qp の構造確認】(サイズ: 3行 x 4列)");
% disp("A_v2 (2階制約行) = "); disp(A_v2);
% disp("A_v3 (3階制約行) = "); disp(A_v3);
% disp("A_v4 (4階制約行) = "); disp(A_v4);
% disp("------------------------------------------------------------");
% %% =========================================================================
% %% ステップ 6: Mファイル関数として書き出し
% %% =========================================================================
% gamma_params = [gamma1; gamma2; gamma3; gamma4];
% obs_params   = [xo; yo; zo; ro];
% sys_params   = rl;
% u_nom_params = u_nom;
% 
% % 監視用出力値のシンボリック置換 (未代入変数を排除)
% h3_eval = h3_nom;
% h4_eval = h4_nom;
% h5_eval = h5_nom;
% 
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:);
% 
% disp("Exporting: CBF_Constraints_NonCascaded_Obstacle_Stacked.m を書き出しています...");
% matlabFunction(A_cbf_sym, b_cbf_sym, h1, h2, h3_eval, h4_eval, h5_eval, ...
%     'file', 'CBF_Constraints_NonCascaded_Obstacle_Stacked.m', ...
%     'vars', {obj, x, XD_sym, u_nom_params, obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp', 'h1', 'h2', 'h3', 'h4', 'h5'});
% 
% disp("Done: 完全展開 Non-cascaded CBF 関数の生成が完了しました！");

% %% =========================================================================
% %% 【C3BF】衝突円錐バリア関数のシンボリック導出（サイズ・列分離 完全整合版）
% %% =========================================================================
% disp("Start: 単機牽引ドローン用 C3BF の自動導出を開始します。");
% 
% syms u1 u2 u3 u4 real          % u1: 推力 f, [u2, u3, u4]: トルク M
% syms xo yo zo vxo vyo vzo real % 障害物の位置・速度
% syms ro rl d_safe real         % 半径パラメータ
% syms alpha real                % クラスKゲイン
% 
% % 1. 入力ベクトルの列分割 (明示的に列ベクトル・部分行列化)
% g1  = g(:, 1);     % 19 x 1
% g23 = g(:, 2:3);   % 19 x 2
% g4  = g(:, 4);     % 19 x 1
% 
% % 2. 牽引紐中点 p_mid (3x1) と 速度 v_mid (3x1)
% L_cable = physicalParam(7);
% p_mid   = pl(:) - 0.5 * L_cable * pT(:);
% 
% v_mid_raw = LieD(p_mid, f, x);
% v_mid     = v_mid_raw(:);  % 3x1 に整形
% 
% % 3. 障害物の位置・速度ベクトル (3x1)
% p_obs = [xo; yo; zo];
% v_obs = [vxo; vyo; vzo];
% 
% % 4. 相対位置 p_rel (3x1) と 相対速度 v_rel (3x1)
% p_rel  = p_obs - p_mid;
% v_rel  = v_obs - v_mid;
% r_safe = ro + rl + d_safe;
% 
% % 5. C3BF バリア関数 h(x) の定義
% norm_p_sq = p_rel.' * p_rel;
% norm_v_sq = v_rel.' * v_rel;
% inner_pv  = p_rel.' * v_rel;
% norm_v    = sqrt(norm_v_sq);
% 
% % rad_expr = sqrt(norm_p_sq - r_safe^2);
% % h_c3bf   = inner_pv + norm_v * rad_expr;
% % 
% % % 6. 幾何重みベクトル eta (3x1) と スカラー sigma の定義
% % eta   = p_rel + v_rel * (rad_expr / norm_v);
% % eta   = eta(:);
% % sigma = norm_v_sq + (norm_v * inner_pv) / rad_expr;
% % 根号項の保護 (負値による複素数化を防止)
% rad_expr = sqrt(max(1e-4, norm_p_sq - r_safe^2));
% h_c3bf   = inner_pv + norm_v * rad_expr;
% 
% % 幾何重みベクトル eta と スカラー sigma の保護
% eta   = p_rel + v_rel * (rad_expr / max(1e-4, norm_v));
% sigma = norm_v_sq + (norm_v * inner_pv) / rad_expr;
% 
% %% =========================================================================
% %% ステップ 3: ヤコビアン直接計算による Lie 微分導出
% %% =========================================================================
% % v_mid (3x1) の状態 x (19x1) に対するヤコビアン (3x19 行列)
% J_vmid = jacobian(v_mid, x);
% 
% % 加速度成分の計算
% L_f_vmid   = J_vmid * f;      % beta(x) (3x1)
% L_g1_vmid  = J_vmid * g1;     % Gamma(x) (3x1)
% L_g23_vmid = J_vmid * g23;    % (3x2)
% L_g4_vmid  = J_vmid * g4;     % (3x1)
% 
% % Lie 微分 (スカラーおよび行ベクトル)
% L_f_h   = simplify(- eta.' * L_f_vmid + sigma);       % スカラー (1x1)
% L_g1_h  = simplify(- eta.' * L_g1_vmid);              % スカラー (1x1)
% L_g23_h = simplify(- eta.' * L_g23_vmid);              % 行ベクトル (1x2)
% L_g4_h  = simplify(- eta.' * L_g4_vmid);              % スカラー (1x1)
% 
% % 4入力に対する結合感度行ベクトル (1x4)
% L_g_h_all = [L_g1_h, L_g23_h, L_g4_h];
% 
% %% =========================================================================
% %% 4. 構造診断
% %% =========================================================================
% disp("------------------------------------------------------------");
% disp("【C3BF の構造診断 (修正後)】");
% fprintf("size(L_g1_h)  = [%d, %d] (期待値: [1, 1])\n", size(L_g1_h, 1), size(L_g1_h, 2));
% fprintf("size(L_g23_h) = [%d, %d] (期待値: [1, 2])\n", size(L_g23_h, 1), size(L_g23_h, 2));
% fprintf("size(L_g4_h)  = [%d, %d] (期待値: [1, 1])\n", size(L_g4_h, 1), size(L_g4_h, 2));
% disp("L_g1_h (推力 u1 係数) = "); disp(L_g1_h);
% disp("------------------------------------------------------------");
% 
% %% =========================================================================
% %% 5. QP 制約の構成 (dot{h} + alpha * h >= 0  ==>  A_qp * u <= b_qp)
% %% =========================================================================
% A_c3bf_sym = - L_g_h_all;                      % (1x4)
% b_c3bf_sym = simplify(L_f_h + alpha * h_c3bf); % (1x1)
% 
% %% =========================================================================
% %% 6. Mファイルとしてエクスポート
% %% =========================================================================
% obs_params = [xo; yo; zo; vxo; vyo; vzo; ro];
% sys_params = [rl; d_safe; alpha];
% 
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:);
% 
% disp("Exporting: CBF_Constraints_C3BF_SuspendedLoad.m を書き出しています...");
% matlabFunction(A_c3bf_sym, b_c3bf_sym, h_c3bf, ...
%     'file', 'CBF_Constraints_C3BF_SuspendedLoad.m', ...
%     'vars', {obj, x, XD_sym, obs_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp', 'h_val'});
% 
% disp("Done: C3BF 関数の生成が完了しました！");
% %% =========================================================================
% %% 【C3BF】マルチポイント (機体オフセット点 p_Q + 牽引紐中点 p_mid) 自動導出
% %% =========================================================================
% disp("Start: 機体+中点マルチポイント C3BF の自動導出を開始します。");
% 
% syms u1 u2 u3 u4 real          % u1: 推力 f, [u2, u3, u4]: トルク M
% syms xo yo zo vxo vyo vzo real % 障害物の位置・速度
% syms ro rl d_safe real         % 半径パラメータ
% syms alpha_Q alpha_mid real    % クラスKゲイン
% syms l_off real                % ドローン機体オフセット長 [m]
% 
% % 1. 入力列ベクトルの厳密な分離 (19x1, 19x2, 19x1)
% g1  = g(:, 1);
% g23 = g(:, 2:3);
% g4  = g(:, 4);
% 
% % 2. ドローン機体姿勢回転行列 R の構築 (クォータニオン q = [q0; q1; q2; q3] より)
% q0 = x(1); q1 = x(2); q2 = x(3); q3 = x(4);
% R_mat = [ 1 - 2*(q2^2 + q3^2),     2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2);
%               2*(q1*q2 + q0*q3), 1 - 2*(q1^2 + q3^2),     2*(q2*q3 - q0*q1);
%               2*(q1*q3 - q0*q2),     2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2) ];
% 
% % 3. 代表点の定義
% L_cable = physicalParam(7);
% p_drone_base = pl(:) - L_cable * pT(:);             % ドローン結合点
% p_Q   = p_drone_base + R_mat * [0; 0; l_off];        % 機体オフセット点 (トルク感度用)
% p_mid = pl(:) - 0.5 * L_cable * pT(:);              % 牽引紐中点
% 
% % 4. 各代表点の速度ベクトル (Lie 微分により導出)
% v_Q_raw   = LieD(p_Q, f, x);   v_Q   = v_Q_raw(:);
% v_mid_raw = LieD(p_mid, f, x); v_mid = v_mid_raw(:);
% 
% % 5. 障害物定義
% p_obs = [xo; yo; zo];
% v_obs = [vxo; vyo; vzo];
% r_safe = ro + rl + d_safe;
% 
% %% -------------------------------------------------------------------------
% %% 共通サブルーチン: C3BF 式構築用インライン関数
% %% -------------------------------------------------------------------------
% % (A) 機体オフセット点 p_Q に対する C3BF
% p_rel_Q = p_obs - p_Q;
% v_rel_Q = v_obs - v_Q;
% norm_p_sq_Q = p_rel_Q.' * p_rel_Q;
% norm_v_sq_Q = v_rel_Q.' * v_rel_Q;
% inner_pv_Q  = p_rel_Q.' * v_rel_Q;
% norm_v_Q    = sqrt(norm_v_sq_Q);
% rad_expr_Q  = sqrt(max(1e-4, norm_p_sq_Q - r_safe^2));
% h_c3bf_Q    = inner_pv_Q + norm_v_Q * rad_expr_Q;
% 
% eta_Q   = p_rel_Q + v_rel_Q * (rad_expr_Q / max(1e-4, norm_v_Q));
% sigma_Q = norm_v_sq_Q + (norm_v_Q * inner_pv_Q) / rad_expr_Q;
% 
% J_vQ = jacobian(v_Q, x);
% L_f_vQ   = J_vQ * f;
% L_g1_vQ  = J_vQ * g1;
% L_g23_vQ = J_vQ * g23;
% L_g4_vQ  = J_vQ * g4;
% 
% L_f_h_Q   = simplify(- eta_Q.' * L_f_vQ + sigma_Q);
% L_g1_h_Q  = simplify(- eta_Q.' * L_g1_vQ);
% L_g23_h_Q = simplify(- eta_Q.' * L_g23_vQ);
% L_g4_h_Q  = simplify(- eta_Q.' * L_g4_vQ);
% L_g_h_Q   = [L_g1_h_Q, L_g23_h_Q, L_g4_h_Q];
% 
% A_Q_sym = - L_g_h_Q;
% b_Q_sym = simplify(L_f_h_Q + alpha_Q * h_c3bf_Q);
% 
% % (B) 牽引紐中点 p_mid に対する C3BF
% p_rel_M = p_obs - p_mid;
% v_rel_M = v_obs - v_mid;
% norm_p_sq_M = p_rel_M.' * p_rel_M;
% norm_v_sq_M = v_rel_M.' * v_rel_M;
% inner_pv_M  = p_rel_M.' * v_rel_M;
% norm_v_M    = sqrt(norm_v_sq_M);
% rad_expr_M  = sqrt(max(1e-4, norm_p_sq_M - r_safe^2));
% h_c3bf_M    = inner_pv_M + norm_v_M * rad_expr_M;
% 
% eta_M   = p_rel_M + v_rel_M * (rad_expr_M / max(1e-4, norm_v_M));
% sigma_M = norm_v_sq_M + (norm_v_M * inner_pv_M) / rad_expr_M;
% 
% J_vM = jacobian(v_mid, x);
% L_f_vM   = J_vM * f;
% L_g1_vM  = J_vM * g1;
% L_g23_vM = J_vM * g23;
% L_g4_vM  = J_vM * g4;
% 
% L_f_h_M   = simplify(- eta_M.' * L_f_vM + sigma_M);
% L_g1_h_M  = simplify(- eta_M.' * L_g1_vM);
% L_g23_h_M = simplify(- eta_M.' * L_g23_vM);
% L_g4_h_M  = simplify(- eta_M.' * L_g4_vM);
% L_g_h_M   = [L_g1_h_M, L_g23_h_M, L_g4_h_M];
% 
% A_mid_sym = - L_g_h_M;
% b_mid_sym = simplify(L_f_h_M + alpha_mid * h_c3bf_M);
% 
% %% -------------------------------------------------------------------------
% %% 6. スタック行列の構成 (2制約 x 4入力)
% %% -------------------------------------------------------------------------
% A_c3bf_dual = [A_Q_sym; A_mid_sym];     % (2 x 4)
% b_c3bf_dual = [b_Q_sym; b_mid_sym];     % (2 x 1)
% h_c3bf_dual = [h_c3bf_Q; h_c3bf_M];     % (2 x 1)
% 
% %% -------------------------------------------------------------------------
% %% 7. 構造診断
% %% -------------------------------------------------------------------------
% disp("------------------------------------------------------------");
% disp("【マルチポイント C3BF 構造診断】");
% disp("機体側   A_Q   (推力 + トルク) = "); disp(A_Q_sym);
% disp("中点側   A_mid (推力のみ)      = "); disp(A_mid_sym);
% disp("------------------------------------------------------------");
% 
% %% -------------------------------------------------------------------------
% %% 8. Mファイルとしてエクスポート
% %% -------------------------------------------------------------------------
% obs_params = [xo; yo; zo; vxo; vyo; vzo; ro];
% sys_params = [rl; d_safe; alpha_Q; alpha_mid; l_off];
% 
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:);
% 
% disp("Exporting: CBF_Constraints_C3BF_MultiPoint.m を書き出しています...");
% matlabFunction(A_c3bf_dual, b_c3bf_dual, h_c3bf_dual, ...
%     'file', 'CBF_Constraints_C3BF_MultiPoint.m', ...
%     'vars', {obj, x, XD_sym, obs_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp', 'h_val'});
% 
% disp("Done: マルチポイント C3BF 関数の生成が完了しました！");
% %% =========================================================================
% %% 【C3-ECBF】3次 Exponential 衝突円錐 CBF 自動導出 (全4入力結合版)
% %% =========================================================================
% disp("Start: 3次 Exponential Collision Cone CBF (全4入力) の導出を開始します。");
% 
% syms u1 u2 u3 u4 real          % u1: 推力 f, [u2, u3, u4]: トルク M
% syms xo yo zo vxo vyo vzo real % 障害物の位置・速度
% syms ro rl d_safe real         % 半径パラメータ
% syms lambda real               % 3重極ゲイン (論文 λ)
% syms U1_ref real               % 動作点推力 (トルク感度保持用)
% 
% % 1. 入力列の厳密な分離 (19x1, 19x2, 19x1)
% g1  = g(:, 1);
% g23 = g(:, 2:3);
% g4  = g(:, 4);
% 
% % 2. 牽引紐中点 p_mid (3x1) と 障害物幾何
% L_cable = physicalParam(7);
% p_mid   = pl(:) - 0.5 * L_cable * pT(:);
% p_obs   = [xo; yo; zo];
% v_obs   = [vxo; vyo; vzo];
% r_safe  = ro + rl + d_safe;
% 
% % 3. 速度 v_mid の導出 (Lie 微分)
% v_mid_raw = LieD(p_mid, f, x);
% v_mid     = v_mid_raw(:);
% 
% % 4. 衝突円錐バリア関数 h(x) の定義 (ψ0 = h)
% p_rel = p_obs - p_mid;
% v_rel = v_obs - v_mid;
% 
% norm_p_sq = p_rel.' * p_rel;
% norm_v_sq = v_rel.' * v_rel;
% inner_pv  = p_rel.' * v_rel;
% norm_v    = sqrt(norm_v_sq);
% 
% rad_expr = sqrt(max(1e-4, norm_p_sq - r_safe^2));
% h_0      = inner_pv + norm_v * rad_expr; % ψ0
% 
% % 5. 1階微分 h_dot (ψ1) の自律項 L_f_h
% eta   = p_rel + v_rel * (rad_expr / max(1e-4, norm_v));
% sigma = norm_v_sq + (norm_v * inner_pv) / rad_expr;
% 
% J_vmid   = jacobian(v_mid, x);
% L_f_vmid = J_vmid * f;
% h_1      = simplify(- eta.' * L_f_vmid + sigma); % L_f h (ψ1)
% 
% % 6. 2階微分 h_ddot (ψ2) の自律項 L_f^2_h
% J_h1 = jacobian(h_1, x);
% h_2  = simplify(J_h1 * f); % L_f^2 h (ψ2)
% 
% % 7. 3階微分 h_dddot (ψ3) の自律項および入力感度 (Snap レベル)
% J_h2   = jacobian(h_2, x);
% L_f3_h = simplify(J_h2 * f);
% 
% % 4入力に対する各 Lie 微分
% L_g1_raw  = simplify(J_h2 * g1);   % 推力 u1 係数 (1x1)
% L_g23_raw = simplify(J_h2 * g23);  % Roll/Pitch [u2, u3] 係数 (1x2)
% L_g4_raw  = simplify(J_h2 * g4);   % Yaw u4 係数 (1x1)
% 
% % 動作点推力 U1_ref の代入によるトルク感度の確定
% L_g1_eval  = simplify(subs(L_g1_raw, u1, U1_ref));
% L_g23_eval = simplify(subs(L_g23_raw, u1, U1_ref));
% L_g4_eval  = simplify(subs(L_g4_raw, u1, U1_ref));
% 
% % 4入力結合感度行ベクトル (1x4)
% L_g_all = [L_g1_eval, L_g23_eval, L_g4_eval];
% 
% %% =========================================================================
% %% 8. 論文式 (15) に基づく 3次 ECBF-QP 制約の構築
% %% =========================================================================
% % 制約: L_f^3 h + L_g L_f^2 h * u + 3*lambda*h_2 + 3*lambda^2*h_1 + lambda^3*h_0 >= 0
% %  ==>  A_qp * u <= b_qp
% A_c3ecbf_sym = - L_g_all; % (1x4)
% b_c3ecbf_sym = simplify(L_f3_h + 3*lambda*h_2 + 3*(lambda^2)*h_1 + (lambda^3)*h_0);
% 
% %% =========================================================================
% %% 9. 構造診断
% %% =========================================================================
% disp("------------------------------------------------------------");
% disp("【3次 Exponential C3BF 構造診断 (全4入力)】");
% fprintf("size(A_qp) = [%d, %d] (期待値: [1, 4])\n", size(A_c3ecbf_sym, 1), size(A_c3ecbf_sym, 2));
% disp("A_qp (推力 u1 係数)          = "); disp(A_c3ecbf_sym(1));
% disp("A_qp (Roll/Pitch u2,u3 係数) = "); disp(A_c3ecbf_sym(2:3));
% disp("A_qp (Yaw u4 係数)           = "); disp(A_c3ecbf_sym(4));
% disp("------------------------------------------------------------");
% 
% %% =========================================================================
% %% 10. Mファイルとしてエクスポート
% %% =========================================================================
% obs_params = [xo; yo; zo; vxo; vyo; vzo; ro];
% sys_params = [rl; d_safe; lambda];
% 
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:);
% 
% disp("Exporting: CBF_Constraints_C3ECBF_SuspendedLoad.m を書き出しています...");
% matlabFunction(A_c3ecbf_sym, b_c3ecbf_sym, h_0, h_1, h_2, ...
%     'file', 'CBF_Constraints_C3ECBF_SuspendedLoad.m', ...
%     'vars', {obj, x, XD_sym, U1_ref, obs_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp', 'h0_val', 'h1_val', 'h2_val'});
% 
% disp("Done: 全4入力 C3-ECBF 関数の生成が完了しました！");
% %% =========================================================================
% %% 3. 2階・2階の展開
% %% =========================================================================
% % --- 1階 (安全関数) ---
% % h1 = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2;
% 
% % --- 2階 (h2) ---
% % ※ 0階式 h1 には入力が現れないため、f による Lie 微分のみ
% h1_dot = LieD(h1, f, x);
% h2     = simplify(h1_dot + gamma1 * h1);
% 
% % --- 3階 (h3) の完全展開 ---
% % h2 の各ベクトル場に対する Lie 微分
% L_f_h2   = LieD(h2, f, x);
% L_g1_h2  = LieD(h2, g1, x);    % 推力 u1 に対する感度 (スカラー)
% L_g23_h2 = LieD(h2, g23, x);   % Roll/Pitch [u2; u3] に対する感度 (1x2 行列)
% L_g4_h2  = LieD(h2, g4, x);    % Yaw u4 に対する感度 (スカラー)
% 
% % 2階のドリフト項 C2 と 入力分離
% C2 = simplify(L_f_h2 + gamma2 * h2);
% A2_u1  = simplify(L_g1_h2);
% A2_u23 = simplify(L_g23_h2);
% A2_u4  = simplify(L_g4_h2);
% 
% % 3階の完全式 (h3_full)
% h3_full = simplify(C2 + A2_u1 * u1 + A2_u23 * [u2; u3] + A2_u4 * u4);
% disp("------------------------------------------------------------");
% disp("【2階の構造診断】(期待値: u1 は非ゼロ, トルクはすべてゼロ)");
% disp("------------------------------------------------------------");
% disp("A2_u1  (推力 u1 係数)       = "); disp(A2_u1);
% disp("A2_u12 (Roll/Pitch 係数)   = "); disp(A2_u23);
% disp("A2_u4  (Yaw 係数)          = "); disp(A2_u4);
% disp("------------------------------------------------------------");
% 
% %% =========================================================================
% %% 4. 3階の完全展開 (dot{u1}=0 を適用)
% %% =========================================================================
% % C2 の状態微分
% L_f_C2    = LieD(C2, f, x);
% L_g1_C2   = LieD(C2, g1, x);
% L_gtau_C2 = LieD(C2, g_tau, x);
% 
% % A2 の状態微分
% L_f_A2    = LieD(A2, f, x);
% L_g1_A2   = LieD(A2, g1, x);
% L_gtau_A2 = LieD(A2, g_tau, x);
% 
% % 3階の各成分
% C3      = simplify(L_f_C2 + gamma3 * C2);
% A31     = simplify(L_g1_C2 + L_f_A2 + gamma3 * A2);
% A3M     = simplify(L_gtau_C2);
% B3_u1u1 = simplify(L_g1_A2);
% B3_u1M  = simplify(L_gtau_A2);
% 
% psi3_full = simplify(C3 + A31 * u1 + A3M * M + B3_u1u1 * u1^2 + u1 * (B3_u1M * M));
% 
% %% =========================================================================
% %% 5. 相対次数構造の厳密診断
% %% =========================================================================
% 
% disp("================================================");
% disp("【診断 1】2階・3階における入力構造");
% disp("================================================");
% 
% disp("A2M = ");      disp(A2M);
% disp("A3M = ");      disp(A3M);
% disp("B3_u1u1 = ");  disp(B3_u1u1);
% disp("B3_u1M = ");   disp(B3_u1M);
% 
% isZeroA2M = all(isAlways(A2M == 0));
% isZeroA3M = all(isAlways(A3M == 0));
% isZeroB311 = isAlways(B3_u1u1 == 0);
% isZeroB31M = all(isAlways(B3_u1M == 0));
% 
% fprintf("A2M = 0      : %d\n", isZeroA2M);
% fprintf("A3M = 0      : %d\n", isZeroA3M);
% fprintf("B3_u1u1 = 0  : %d\n", isZeroB311);
% fprintf("B3_u1M = 0   : %d\n", isZeroB31M);
% 
% if ~(isZeroA2M && isZeroA3M && isZeroB311 && isZeroB31M)
% 
%     error([ ...
%         "現在のモデルでは psi2/psi3 が論文の期待する ", ...
%         "C + A*u1 構造になっていません。", ...
%         "非ゼロ項を捨てず、一般形から再展開してください。"]);
% 
% end
% 
% disp("✓ 論文と同じ入力相対次数構造を確認しました。");
% 
% % この時点で初めて簡約
% psi2 = simplify(C2 + A2*u1);
% 
% A3 = simplify(A31);
% 
% psi3 = simplify(C3 + A3*u1);
% 
% %% =========================================================================
% %% 6. 4階の完全展開 (dot{u1}=0 を適用)
% %% =========================================================================
% % C3 の状態微分
% L_f_C3    = LieD(C3, f, x);
% L_g1_C3   = LieD(C3, g1, x);
% L_gtau_C3 = LieD(C3, g_tau, x);
% 
% % A3 の状態微分
% L_f_A3    = LieD(A3, f, x);
% L_g1_A3   = LieD(A3, g1, x);
% L_gtau_A3 = LieD(A3, g_tau, x);
% 
% % 4階の完全ベース成分
% C4_base     = simplify(L_f_C3 + gamma4 * C3);
% A4_u1_base  = simplify(L_g1_C3 + L_f_A3 + gamma4 * A3);
% A4_M_direct = simplify(L_gtau_C3);
% B4_u1u1     = simplify(L_g1_A3);
% B4_u1M      = simplify(L_gtau_A3);
% 
% disp("================================================");
% disp("【診断 2】4階における非線形・トルク構造");
% disp("================================================");
% disp("A4_M_direct (C3由来のトルク項) = "); disp(A4_M_direct);
% disp("B4_u1M (u1*M 双線形トルク項)   = "); disp(B4_u1M);
% disp("B4_u1u1 (4階 u1^2 係数項)      = "); disp(B4_u1u1);
% disp("================================================");
% 
% %% =========================================================================
% %% 7. 4階の局所アフィン化 (if文を排除した統一数式展開)
% %% =========================================================================
% % u1^2 ≈ 2*U1_val*u1 - U1_val^2 を代数的にそのまま代入
% C4 = simplify(C4_base - B4_u1u1 * U1_val^2);
% A4_u1 = simplify(A4_u1_base + 2 * B4_u1u1 * U1_val);
% 
% % トルク係数: A4_M_direct + U1_val * B4_u1M
% A4_M_qp = simplify(A4_M_direct + U1_val * B4_u1M);
% 
% %% =========================================================================
% %% 8. v2, v3, v4 の 3段スタック制約の構成 (A_cbf * u <= b_cbf)
% %% =========================================================================
% % Constraint 1: psi2 >= 0
% A_v2 = [-A2, 0, 0, 0];
% b_v2 = C2;
% 
% % Constraint 2: psi3 >= 0
% A_v3 = [-A3, 0, 0, 0];
% b_v3 = C3;
% 
% % Constraint 3: psi4_qp >= 0
% A_v4 = -[A4_u1, A4_M_qp];
% b_v4 = C4;
% 
% % 全スタック制約 [3 x 4] 行列
% A_cbf_sym = [A_v2; A_v3; A_v4];
% b_cbf_sym = [b_v2; b_v3; b_v4];
% 
% %% =========================================================================
% %% 9. Mファイルとしてエクスポート
% %% =========================================================================
% % 出力監視用の psi2, psi3 に動作点 U1_val を代入して自由変数 u1 を消去
% psi2_subs = subs(psi2, u1, U1_val);
% psi3_subs = subs(psi3, u1, U1_val);
% 
% gamma_params = [gamma1; gamma2; gamma3; gamma4];
% obs_params   = [xo; yo; zo; ro];
% sys_params   = [rl; d_safe];
% 
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:);
% 
% disp("Exporting: CBF_Constraints_NonCascaded_Obstacle_Stacked.m を書き出しています...");
% matlabFunction(A_cbf_sym, b_cbf_sym, h0, psi1, psi2_subs, psi3_subs, ...
%     'file', 'CBF_Constraints_NonCascaded_Obstacle_Stacked.m', ...
%     'vars', {obj, x, XD_sym, U1_val, obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp', 'h0', 'psi1', 'psi2', 'psi3'});
% 
% disp("Done: 完全展開 Non-cascaded CBF 関数の生成が完了しました！");
% %% =========================================================================
% %% 【Non-cascaded ECBF】推力 u1 と トルク [u2, u3, u4] を同時決定する HOCBF 導出
% %% =========================================================================
% disp("Start: 4入力同時最適化 (Non-cascaded ECBF) の導出を開始します。");
% 
% % 1. ダイナミクスの定義 (状態 x, 入力 u = [u1; u2; u3; u4])
% syms u1 u2 u3 u4 real
% u = [u1; u2; u3; u4];
% 
% % モデル入力行列 g (FLxyDst, GLxyDst) の列分割
% g1     = g(:, 1);     % 推力 u1 に対する列 (19x1)
% g_tau  = g(:, 2:4);   % トルク [u2; u3; u4] に対する列 (19x3)
% 
% % 3. 障害物安全関数の定義 (ロープ中心 p_mid)
% syms xo yo zo ro real
% syms rl real
% L_cable = physicalParam(7);
% p_mid = pl - 0.5 * L_cable * pT;
% 
% h_cbf = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro + rl)^2;
% % h_cbf = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - (ro)^2;
% cbf1 = h_cbf;
% 
% % 4. クラスK関数のゲイン
% syms gamma1 gamma2 gamma3 gamma4 real
% 
% % 4. 各階層の Lie 微分計算 (論文の仮定: dot{u1} = 0)
% % --- 1階微分 (相対次数 1: 入力は現れない) ---
% cbf1_dot = LieD(cbf1, f, x);
% cbf2     = cbf1_dot + gamma1 * cbf1;
% 
% % --- 2階微分 (相対次数 2: ここで u1 が現れる) ---
% % dot{h1} = LieD(h1, f) + LieD(h1, g1)*u1 (※トルク g_tau は 0)
% L_f_cbf2  = LieD(cbf2, f, x);
% L_g1_cbf2 = LieD(cbf2, g1, x);
% cbf3      = (L_f_cbf2 + L_g1_cbf2 * u1) + gamma2 * cbf2;
% 
% % --- 3階微分 (相対次数 3: dot{u1} = 0 として x のみで偏微分) ---
% L_f_cbf3  = LieD(cbf3, f, x);
% L_g1_cbf3 = LieD(cbf3, g1, x);
% cbf4      = (L_f_cbf3 + L_g1_cbf3 * u1) + gamma3 * cbf3;
% 
% % --- 4階微分 (相対次数 4: ここで トルク [u2, u3, u4] が現れる) ---
% L_f_cbf4   = LieD(cbf4, f, x);
% L_g1_cbf4  = LieD(cbf4, g1, x);
% L_gtau_cbf4 = LieD(cbf4, g_tau, x); % [1 x 3] 行列
% 
% % 全体微分式: dot{cbf3} + gamma4 * cbf3 >= 0
% dot_cbf4_total = (L_f_cbf4 + L_g1_cbf4 * u1 + L_gtau_cbf4 * [u2; u3; u4]) + gamma4 * cbf4;
% 
% %% =========================================================================
% %% 5. 動作点 U1_val 周りでの厳密なアフィン分離 (A_qp * u <= b_qp)
% %% =========================================================================
% syms U1_val real
% 
% % 基準動作点 u_ref = [U1_val; 0; 0; 0]
% u_ref = [U1_val; 0; 0; 0];
% 
% % 4入力に対するヤコビアン（勾配ベクトル 1x4）
% % L_g_all = [dL/du1, dL/du2, dL/du3, dL/du4]
% L_g_all = jacobian(dot_cbf4_total, [u1; u2; u3; u4]);
% 
% % 基準点 u_ref における勾配および関数値の評価
% A_cbf_eval = subs(L_g_all, [u1; u2; u3; u4], u_ref);
% h4_ref_eval = subs(dot_cbf4_total, [u1; u2; u3; u4], u_ref);
% 
% % 不等号反転: dot_cbf4_total >= 0  ==>  A_qp * u <= b_qp
% % 1次近似: h4_ref + A_eval * (u - u_ref) >= 0
% %       -A_eval * u <= h4_ref - A_eval * u_ref
% A_cbf_sym = -A_cbf_eval;                               % [1 x 4] 行列
% b_cbf_sym = simplify(h4_ref_eval + A_cbf_sym * u_ref); % スカラー
% 
% %% =========================================================================
% %% 6. 各階層モニタリング関数の整形
% %% =========================================================================
% % 中間階層の u1 にも動作点 U1_val を代入
% h1_subs = cbf1;
% h2_subs = cbf2;
% h3_subs = subs(cbf3, u1, U1_val);
% h4_subs = subs(cbf4, u1, U1_val);
% 
% A_cbf_subs = A_cbf_sym;
% b_cbf_subs = b_cbf_sym;
% 
% %% =========================================================================
% %% 7. Mファイルとしてエクスポート
% %% =========================================================================
% gamma_params = [gamma1; gamma2; gamma3; gamma4];
% obs_params   = [xo; yo; zo; ro];
% sys_params   = rl;
% 
% XD_sym = cell2sym(XD);
% XD_sym = XD_sym(:);
% 
% disp("Exporting: CBF_Constraints_NonCascaded_Obstacle.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, h1_subs, h2_subs, h3_subs, h4_subs, ...
%     'file', 'CBF_Constraints_NonCascaded_Obstacle.m', ...
%     'vars', {obj, x, XD_sym, U1_val, obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp', 'h1', 'h2', 'h3', 'h4'});
% 
% disp("Done: トルク効果を保持した CBF 関数の生成が完了しました！");

% %% =========================================================================
% %% 診断・整合性チェックセクション
% %% =========================================================================
% disp("------------------------------------------------------------");
% disp("【検証 1】各入力 u1〜u4 の出現階層（相対次数の確認）");
% disp("------------------------------------------------------------");
% 
% % 各階層における入力 u1, [u2, u3, u4] の感度（偏微分）を確認
% check_u1 = [ ...
%     ~isequal(jacobian(cbf1, u1), sym(0));
%     ~isequal(jacobian(cbf2, u1), sym(0));
%     ~isequal(jacobian(cbf3, u1), sym(0));
%     ~isequal(jacobian(cbf4, u1), sym(0));
%     ~isequal(jacobian(dot_cbf4_total, u1), sym(0)) ...
%     ];
% 
% check_tau = [ ...
%     ~isequal(jacobian(cbf1, [u2; u3; u4]), sym([0, 0, 0]));
%     ~isequal(jacobian(cbf2, [u2; u3; u4]), sym([0, 0, 0]));
%     ~isequal(jacobian(cbf3, [u2; u3; u4]), sym([0, 0, 0]));
%     ~isequal(jacobian(cbf4, [u2; u3; u4]), sym([0, 0, 0]));
%     ~isequal(jacobian(dot_cbf4_total, [u2; u3; u4]), sym([0, 0, 0])) ...
%     ];
% 
% fprintf('cbf1 (0階):  u1 出現 = %d,  トルク 出現 = %d\n', check_u1(1), check_tau(1));
% fprintf('cbf2 (1階):  u1 出現 = %d,  トルク 出現 = %d\n', check_u1(2), check_tau(2));
% fprintf('cbf3 (2階):  u1 出現 = %d,  トルク 出現 = %d\n', check_u1(3), check_tau(3));
% fprintf('cbf4 (3階):  u1 出現 = %d,  トルク 出現 = %d\n', check_u1(4), check_tau(4));
% fprintf('dot_cbf4 (4階): u1 出現 = %d,  トルク 出現 = %d\n', check_u1(5), check_tau(5));
% 
% disp("------------------------------------------------------------");
% disp("【検証 1.5】入力に対するアフィン性の確認");
% disp("------------------------------------------------------------");
% 
% H_uu = simplify(hessian(dot_cbf4_total, [u1; u2; u3; u4]));
% 
% if isequal(H_uu, sym(zeros(4,4)))
%     disp("✓ dot_cbf4_total は [u1,u2,u3,u4] に対して厳密にアフィンです。");
% else
%     disp("⚠ dot_cbf4_total は入力に対して非線形項を含みます。");
%     disp("入力 Hessian:");
%     disp(H_uu);
% end
% 
% % 特に u1 の2次項
% u1_second = simplify(diff(dot_cbf4_total, u1, 2));
% 
% if isequal(u1_second, sym(0))
%     disp("✓ u1 に関する2次以上の項はありません。");
% else
%     disp("⚠ u1 に非線形項があります:");
%     disp(u1_second);
% end
% 
% % 入力間の積 u1*u2 等がないか
% cross_u = sym(zeros(4,4));
% 
% for i = 1:4
%     for j = i+1:4
%         cross_u(i,j) = simplify(diff( ...
%             dot_cbf4_total, ...
%             u(i), ...
%             u(j)));
%     end
% end
% 
% disp("入力間クロス項の診断:");
% disp(cross_u);
% 
% disp("------------------------------------------------------------");
% disp("【検証 2】制約式 A_qp * u <= b_qp の線形展開整合性");
% disp("------------------------------------------------------------");
% 
% % 基準点 u_ref において、再構成した式 (b_qp - A_qp*u_ref) と h4_ref が一致するか確認
% reconstructed_val_at_ref = simplify(b_cbf_sym - A_cbf_sym * u_ref);
% diff_check = simplify(h4_ref_eval - reconstructed_val_at_ref);
% 
% if isequal(diff_check, sym(0))
%     disp("✓ 符号・展開整合性 OK: 動作点 U1_val において完全一致しています。");
% else
%     disp("⚠ 符号警告: 式の展開に不一致があります。残差式:");
%     disp(diff_check);
% end
% 
% disp("------------------------------------------------------------");
% disp("【検証 3】数値代入による QP 制約の動作テスト (横オフセットあり)");
% disp("------------------------------------------------------------");
% 
% % テスト用パラメータ
% m_Q_num = 1.5; m_L_num = 0.5; L_num = 1.0; g_num = 9.81;
% J_num = [0.039, 0.051, 0.102];
% p_obs_num = [0; 0; 2]; r_obs_num = 0.5; r_L_num = 0.1;
% 
% % 障害物の斜め上から接近（横オフセット pl_test = [0.3; 0.2; 2.8]）
% q_test = [1; 0; 0; 0];
% ob_test = [0; 0; 0];
% pl_test = [0.3; 0.2; 2.8];
% dpl_test = [-0.5; -0.3; -0.8];
% pT_test = [0; 0; -1];
% ol_test = [0; 0; 0];
% x_test_val = [q_test; ob_test; pl_test; dpl_test; pT_test; ol_test];
% 
% phys_vars = [m, Lx, Ly, lx, ly, jx, jy, jz, gravity, km1, km2, km3, km4, k1, k2, k3, k4, rotor_r, mL, cableL, dstx, dsty];
% phys_vals = [m_Q_num, 0.2, 0.2, 0.2, 0.2, J_num(1), J_num(2), J_num(3), g_num, 1, 1, 1, 1, 1, 1, 1, 1, 0.1, m_L_num, L_num, 0, 0];
% 
% u1_hover = (m_Q_num + m_L_num) * g_num;
% 
% A_num = double(subs(A_cbf_sym, ...
%     [x; U1_val; xo; yo; zo; ro; rl; gamma1; gamma2; gamma3; gamma4; phys_vars.'], ...
%     [x_test_val; u1_hover; p_obs_num; r_obs_num; r_L_num; 2; 2; 2; 2; phys_vals.']));
% 
% b_num = double(subs(b_cbf_sym, ...
%     [x; U1_val; xo; yo; zo; ro; rl; gamma1; gamma2; gamma3; gamma4; phys_vars.'], ...
%     [x_test_val; u1_hover; p_obs_num; r_obs_num; r_L_num; 2; 2; 2; 2; phys_vals.']));
% 
% disp("数値計算結果:");
% disp("  A_qp (1x4) = "); disp(A_num);
% disp("  b_qp (1x1) = "); disp(b_num);
% 
% val_hov = A_num * [u1_hover; 0; 0; 0] - b_num;
% fprintf('  ホバリング入力での評価値 (A*u - b <= 0 であるべき): %f\n', val_hov);
% disp("------------------------------------------------------------");
% 
% disp("------------------------------------------------------------");
% disp("【検証 2.5】元のCBF制約とQP再構成式の厳密一致確認");
% disp("------------------------------------------------------------");
% 
% u_all = [u1; u2; u3; u4];
% 
% % QP制約 A*u <= b を元の >=0形式に戻す
% cbf_reconstructed = simplify(b_cbf_sym - A_cbf_sym*u_all);
% 
% exact_diff = simplify(dot_cbf4_total - cbf_reconstructed);
% 
% if isequal(exact_diff, sym(0))
%     disp("✓ 元の dot_cbf4_total と QP再構成式は全入力領域で完全一致。");
%     disp("  → 厳密なアフィンQP制約です。");
% else
%     disp("⚠ 動作点以外では一致しません。");
%     disp("  → 現在のQP制約は局所線形化です。");
%     disp("残差:");
%     disp(factor(exact_diff));
% end
% %% =========================================================================
% %% 【決定版】外部確定入力 u1 をそのままドリフト項に保持する HOCBF 自動導出
% %% =========================================================================
% disp("Start: f_z による HOCBF の導出を開始します。");
% 
% % 1. 全ダイナミクス FG_xy の定義
% FG_z = simplify(f + g * [u1; u2; u3; u4]);
% 
% % 2. u2, u3, u4 を 0 とした実効ドリフト項 f_xy
% %    🌟 u1（推力）は 0 にせず、シンボリック変数 u1 のまま f_xy に残ります！
% f_z = subs(FG_z, [u1, u2, u3, u4], [0, 0, 0, 0]);
% 
% % 2nd layer の操作入力 [u2; u3; u4] に対する入力行列 g_xy
% g_z  = simplify(MyCoeff(FG_z, [u1; u2; u3; u4]));
% g1_z  = g_z(:, 1);
% g1_yaw  = g_z(:, 4);
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
% syms gamma1 gamma2 real
% 
% % 5. f_xy を用いた Lie 微分の計算（相対次数 4）
% % ※ f_xy の中に u1 が入っているため、Lie 微分の中に u1 が自然な形で組み込まれます
% cbf2 = LieD(cbf1, f_z, x) + diff(cbf1, t) + gamma1 * cbf1; % h2
% 
% % 最上階での展開（u2, u3 が現れる階層）
% L_f_cbf2  = LieD(cbf2, f_z, x) + diff(cbf2, t);
% L_g_cbf2  = LieD(cbf2, g1_z, x);  % [1 x 2] 行列
% L_gy_cbf2 = LieD(cbf2, g1_yaw, x); % スカラー
% 
% % A_qp * [u2; u3] <= b_qp の形に整理（符号反転）
% A_cbf_sym = -L_g_cbf2;
% b_cbf_sym =  L_f_cbf2 + L_gy_cbf2 * u4 + gamma2 * cbf2;
% 
% % 5. 実数値シミュレーション用の置換 (xdRef -> XDf, vInput1f -> V1vf)
% A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 🌟 各階層の関数（h1, h2, h3, h4）を代入用に置換処理
% h1_subs = subs(cbf1, [xdReff, vInput1f], [XDf, V1vf]);
% h2_subs = subs(cbf2, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 7. Mファイルとしてエクスポート
% gamma_params = [gamma1; gamma2];
% obs_params   = [xo; yo; zo; ro];
% sys_params   = rl;
% XD_sym       = cell2sym(XD);
% XD_sym       = XD_sym(:); 
% 
% disp("Exporting: CBF_Constraints_HOCBF_zlink_xyz.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, h1_subs, h2_subs, ...
%     'file', 'CBF_Constraints_HOCBF_zlink_xyz.m', ...
%     'vars', {obj, x, XD_sym, obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp', 'h1', 'h2'});
% disp("Done: CBF関数の生成が完了しました！");
% 
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
% % 🌟 各階層の関数（h1, h2, h3, h4）を代入用に置換処理
% h1_subs = subs(cbf1, [xdReff, u1, u4], [XDf, U1_val, V4]);
% h2_subs = subs(cbf2, [xdReff, u1, u4], [XDf, U1_val, V4]);
% h3_subs = subs(cbf3, [xdReff, u1, u4], [XDf, U1_val, V4]);
% h4_subs = subs(cbf4, [xdReff, u1, u4], [XDf, U1_val, V4]);
% 
% % 7. Mファイルとしてエクスポート
% gamma_params = [gamma1; gamma2; gamma3; gamma4];
% obs_params   = [xo; yo; zo; ro];
% sys_params   = rl;
% XD_sym       = cell2sym(XD);
% XD_sym       = XD_sym(:); 
% 
% disp("Exporting: CBF_Constraints_HOCBF_xylink_xyz.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, h1_subs, h2_subs, h3_subs, h4_subs, ...
%     'file', 'CBF_Constraints_HOCBF_xylink_xyz.m', ...
%     'vars', {obj, x, XD_sym, U1_val, V4, obs_params, gamma_params, sys_params, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp', 'h1', 'h2', 'h3', 'h4'});
% disp("Done: CBF関数の生成が完了しました！");
% %% =========================================================================
% %% 【決定版】姿勢角＆紐振れ角 HOCBF 自動導出（各階層出力・制約値完備）
% %% =========================================================================
% disp("Start: 姿勢角＆紐振れ角 HOCBF（各階層監視付き）の導出を開始します。");
% 
% % 1. 全ダイナミクス FG_xy の定義
% FG_xy = simplify(f + g * [u1; u2; u3; u4]);
% 
% % 2. u2, u3, u4 を 0 とした実効ドリフト項 f_xy（u1 はシンボリックのまま保持）
% f_xy = subs(FG_xy, [u2, u3, u4], [0, 0, 0]);
% 
% % 2nd layer の操作入力 [u2; u3; u4] に対する入力行列 g_xy
% g_xy   = simplify(MyCoeff(FG_xy, [u2; u3; u4]));
% g1_xy  = g_xy(:, 1:2); % u2 (roll), u3 (pitch)
% g1_yaw = g_xy(:, 3);   % u4 (yaw)
% 
% %% -------------------------------------------------------------------------
% %% (A) 機体姿勢角制約 (Roll / Pitch): 相対次数 2
% %% -------------------------------------------------------------------------
% [~, phi_sym, th_sym] = Quat2Eul(q);
% 
% syms phi_max th_max real
% syms gamma_roll1 gamma_roll2 gamma_pitch1 gamma_pitch2 real
% 
% % 1. Roll 角制約
% h_roll_1  = phi_max^2 - phi_sym^2;
% h_roll_2  = LieD(h_roll_1, f_xy, x) + diff(h_roll_1, t) + gamma_roll1 * h_roll_1;
% 
% L_f_roll2  = LieD(h_roll_2, f_xy, x) + diff(h_roll_2, t);
% L_g_roll2  = LieD(h_roll_2, g1_xy, x);  % [1 x 2]
% L_gy_roll2 = LieD(h_roll_2, g1_yaw, x);
% 
% A_roll = -L_g_roll2;
% b_roll =  L_f_roll2 + L_gy_roll2 * u4 + gamma_roll2 * h_roll_2;
% 
% % 2. Pitch 角制約
% h_pitch_1 = th_max^2 - th_sym^2;
% h_pitch_2 = LieD(h_pitch_1, f_xy, x) + diff(h_pitch_1, t) + gamma_pitch1 * h_pitch_1;
% 
% L_f_pitch2  = LieD(h_pitch_2, f_xy, x) + diff(h_pitch_2, t);
% L_g_pitch2  = LieD(h_pitch_2, g1_xy, x); % [1 x 2]
% L_gy_pitch2 = LieD(h_pitch_2, g1_yaw, x);
% 
% A_pitch = -L_g_pitch2;
% b_pitch =  L_f_pitch2 + L_gy_pitch2 * u4 + gamma_pitch2 * h_pitch_2;
% 
% %% -------------------------------------------------------------------------
% %% (B) 牽引紐振れ角制約 (鉛直傾斜角): 相対次数 4
% %% -------------------------------------------------------------------------
% syms cos_cb_max real
% syms gamma_cb1 gamma_cb2 gamma_cb3 gamma_cb4 real
% 
% h_cb_1 = -pT(3) - cos_cb_max; % 真下(pT3=-1)で最大値
% h_cb_2 = LieD(h_cb_1, f_xy, x) + diff(h_cb_1, t) + gamma_cb1 * h_cb_1;
% h_cb_3 = LieD(h_cb_2, f_xy, x) + diff(h_cb_2, t) + gamma_cb2 * h_cb_2;
% h_cb_4 = LieD(h_cb_3, f_xy, x) + diff(h_cb_3, t) + gamma_cb3 * h_cb_3;
% 
% L_f_cb4  = LieD(h_cb_4, f_xy, x) + diff(h_cb_4, t);
% L_g_cb4  = LieD(h_cb_4, g1_xy, x);  % [1 x 2]
% L_gy_cb4 = LieD(h_cb_4, g1_yaw, x);
% 
% A_cable = -L_g_cb4;
% b_cable =  L_f_cb4 + L_gy_cb4 * u4 + gamma_cb4 * h_cb_4;
% 
% %% -------------------------------------------------------------------------
% %% 統合制約および各階層値の構築
% %% -------------------------------------------------------------------------
% A_cbf_all = [A_roll; A_pitch; A_cable]; % [3 x 2] 行列
% b_cbf_all = [b_roll; b_pitch; b_cable]; % [3 x 1] ベクトル
% 
% % 各階層の関数値まとめ
% h_layers_sym = [ ...
%     h_roll_1;  h_roll_2;  ... % Roll: 1階層, 2階層
%     h_pitch_1; h_pitch_2; ... % Pitch: 1階層, 2階層
%     h_cb_1;    h_cb_2;    h_cb_3;    h_cb_4 ... % Cable: 1階層〜4階層
% ];
% 
% % 実数値代入用の置換
% syms U1_val V4 real
% A_cbf_subs    = subs(A_cbf_all,    [xdReff, u1, u4], [XDf, U1_val, V4]);
% b_cbf_subs    = subs(b_cbf_all,    [xdReff, u1, u4], [XDf, U1_val, V4]);
% h_layers_subs = subs(h_layers_sym, [xdReff, u1, u4], [XDf, U1_val, V4]);
% 
% %% -------------------------------------------------------------------------
% %% Mファイルとしてエクスポート
% %% -------------------------------------------------------------------------
% gamma_roll_p  = [gamma_roll1; gamma_roll2];
% gamma_pitch_p = [gamma_pitch1; gamma_pitch2];
% gamma_cb_p    = [gamma_cb1; gamma_cb2; gamma_cb3; gamma_cb4];
% gamma_all     = [gamma_roll_p; gamma_pitch_p; gamma_cb_p];
% 
% limit_params  = [phi_max; th_max; cos_cb_max];
% XD_sym        = cell2sym(XD);
% XD_sym        = XD_sym(:);
% 
% disp("Exporting: CBF_Constraints_Attitude_Cable_Layers_xyz.m を書き出しています...");
% matlabFunction(A_cbf_subs, b_cbf_subs, h_layers_subs, ...
%     'file', 'CBF_Constraints_Attitude_Cable_Layers_xyz.m', ...
%     'vars', {obj, x, XD_sym, U1_val, V4, limit_params, gamma_all, physicalParam}, ...
%     'outputs', {'A_qp', 'b_qp', 'h_layers'});
% disp("Done: 生成が完了しました！");
%% =========================================================================
%% u1の微分の導出
%% =========================================================================

% %% =========================================================================
% %% Make functions of actual inputs taking t, x, xd, v1 and v2 as arguments
% % % If either model, virtual output or parameters is changed, then evaluate this section. It'll take few minutes.
% % % Usage: u = Uf(...) + Us(...)
%     % matlabFunction(subs(H(:,1)*(-alpha1+v1(t)), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','Uf_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'U1'});
%     % matlabFunction(subs(H(:,2:4), [xdRef vInput1], [XD V1v]),'file','H234_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'H234'});
% 
%     %以下二つはとても重い
%     % matlabFunction(subs(inv(beta2), [xdRef vInput1], [XD V1v]),'file','inv_beta2_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'inv_beta2'});
% 
%     % matlabFunction(subs((-alpha2+[v2(t);v3(t);v4(t)]), [flip(xdRef) flip(vInput1) v2(t) v3(t) v4(t)], [flip(XD) flip(V1v) V2 V3 V4]),'file','vs_alpha2_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'vs_alpha2'});
%     % matlabFunction(subs(alpha2(1), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','alpha21_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'alpha21'});
%     % matlabFunction(subs(alpha2(2), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','alpha22_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'alpha22'});
%     % matlabFunction(subs(alpha2(3), [flip(xdRef) flip(vInput1)], [flip(XD) flip(V1v)]),'file','alpha23_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'alpha23'});
% 
%     % a2_v2 = subs((-alpha2+[v2(t);v3(t);v4(t)]), [flip(xdRef) flip(vInput1) v2(t) v3(t) v4(t)], [flip(XD) flip(V1v) V2 V3 V4]);
%     % matlabFunction(a2_v2,'file','vs_alpha2_SuspendedLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'vs_alpha2'});
% 
%     % xyDst
%     matlabFunction(subs(H(:,1)*(-alpha1+v1(t)), [xdReff vInput1f], [XDf V1vf]),'file','Uf_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'U1'});
%     matlabFunction(subs(H(:,2:4), [xdReff vInput1f], [XDf V1vf]),'file','H234_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'H234'});
%     matlabFunction(subs(beta2, [xdReff vInput1f], [XDf V1vf]),'file','Beta2_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'beta2'});
%     %以下はとても重い
%     v2_a2 = subs([V2;V3;V4] - alpha2, [xdReff vInput1f], [XDf V1vf]);
%     matlabFunction(v2_a2,'file','V2_alpha2_SuspendedLoadxyDst.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'v2_alpha2'});
% 
% %理想のfunctionだけどUsが重すぎるので分割している．
%     % matlabFunction(subs(H(:,1)*(-alpha1+v1(t)), [xdRef vInput1], [XD V1v]),'file','Uf_SuspededLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'U1'});
%     % matlabFunction(subs(H(:,2:4)*U2, [xdRef vInput1 v2(t) v3(t) v4(t)], [XD V1v [V2 V3 V4]]),'file','Us_SuspededLoad.m','vars',{obj x cell2sym(XD) cell2sym(V1v) [V2;V3;V4] physicalParam},'outputs',{'U2'});
% 
% % % For check
% %     Uf(0,x0,Xd(0),Vf(0,x0,Xd(0)))
% %     Us(0,x0,Xd(0),Vf(0,x0,Xd(0)),Vs(0,x0,Xd(0),Vf(0,x0,Xd(0))))
% % %%
% % matlabFunction(subs(alpha1, [xdRef], [XD]),'file','alpha1.m','vars',{obj cell2sym(XD) physicalParam},'outputs',{'al1'});
% % matlabFunction(subs(alpha2, [xdRef vInput1], [XD V1v]),'file','alpha2.m','vars',{obj t x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'al2'});
% % %%
% % matlabFunction(subs(beta2, [xdRef vInput1], [XD V1v]),'file','beta2.m','vars',{obj t x cell2sym(XD) cell2sym(V1v) physicalParam},'outputs',{'be2'});
% % %%
% % matlabFunction(subs(He, [xdRef vInput1], [XD V1v]),'file','He.m','vars',{obj t x cell2sym(XD) cell2sym(V1v) e1 physicalParam},'outputs',{'mat'});
% % %%
% % matlabFunction(beta1,'file','beta1.m','vars',{obj t x physicalParam},'outputs',{'beta1'});


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
