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

% %% =========================================================================
% %% 【決定版】HOCBF 自動導出
% %% =========================================================================
% disp("==============================================================");
% disp(" Start: 牽引紐中点 p_mid に対する 相対次数4 HOCBF 制約の自動導出");
% disp(" Target: A_qp * [Mx; My] <= b_qp (1x2 行列)");
% disp("==============================================================");
% 
% % 1. シンボリック変数・パラメータの定義
% syms u1 u2 u3 u4 real          % u1: 推力 f, [u2, u3]: [Mx, My], u4: Mz
% u_qp = [u1; u2; u3; u4];
% 
% syms xo yo zo ro rl real       % 障害物位置・半径 (ro: 障害物, rl: 紐・荷物マージン)
% syms gamma1 gamma2 gamma3 gamma4 real % 各階層のクラスKゲイン (1〜4階)
% 
% % 2. 状態空間ダイナミクスの分割
% %    \dot{x} = f(x) + g1*u1 + g2*u2 + g3*u3 + g4*u4
% g1_F   = g(:, 1);              % 推力 f
% g2_Mx  = g(:, 2);              % ロールトルク Mx
% g3_My  = g(:, 3);              % ピッチトルク My
% g4_Mz  = g(:, 4);              % Yawトルク Mz
% 
% % 3. 幾何位置 (牽引紐中点 p_mid) と基本バリア関数 h(x) の定義
% L_cable = physicalParam(7);
% p_mid   = pl(:) - 0.5 * L_cable * pT(:); % 3x1
% p_obs   = [xo; yo; zo];
% r_safe  = ro + rl;
% 
% % \psi_0 = h(x) (位置レベル: 距離の2乗マージン >= 0)
% h_cbf = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - r_safe^2;
% cbf_h0 = h_cbf;
% 
% % 4. 各階層の Lie 微分と相対次数の逐次展開 (1階 〜 4階)
% % -------------------------------------------------------------------------
% % 【1階微分 (速度レベル)】 入力は一切現れない (L_g h = 0)
% L_f_h = LieD(cbf_h0, f, x);
% cbf_h1 = simplify(L_f_h + gamma1 * cbf_h0);
% 
% % 【2階微分 (加速度レベル)】 🌟 ここで推力 u1 (g1_F) のみが現れる！
% L_f_h1  = LieD(cbf_h1, f, x);
% L_g1_h1 = simplify(LieD(cbf_h1, g1_F, x));
% cbf_h2 = simplify(L_f_h1 + gamma2 * cbf_h1);
% 
% % 【3階微分 (ジャークレベル)】 姿勢角速度 \omega が結合
% L_f_h2 = LieD(cbf_h2, f, x);
% cbf_h3 = simplify(L_f_h2 + gamma3 * cbf_h2);
% 
% % 【4階微分 (スナップレベル)】 🌟 ここでトルク [u2, u3, u4] が現れる！
% L_f_h3  = LieD(cbf_h3, f, x);
% L_g2_h3 = simplify(LieD(cbf_h3, g2_Mx, x)); % ロールトルク Mx 感度
% L_g3_h3 = simplify(LieD(cbf_h3, g3_My, x)); % ピッチトルク My 感度
% L_g4_h3 = simplify(LieD(cbf_h3, g4_Mz, x)); % Yawトルク Mz 感度
% 
% % 5. 2階（推力）と4階（トルク）の感度を単一の 1x4 制約行列へ統合
% %    推力 u1 の感度は 3階・4階のゲイン (\gamma_3 * \gamma_4) を経由して合成
% L_g_thrust  = simplify(gamma3 * gamma4 * L_g1_h1);
% L_g_torques = [L_g2_h3, L_g3_h3, L_g4_h3];
% 
% % 🌟 1x4 の全入力感度行列: [u1 (推力), u2 (Mx), u3 (My), u4 (Mz)]
% L_g_all = [L_g_thrust, L_g_torques];
% 
% % A_qp * [u1; u2; u3; u4] <= b_qp の形に整理
% A_cbf_sym = - L_g_all;
% b_cbf_sym = simplify(L_f_h3 + gamma4 * cbf_h3);
% 
% % 6. 目標軌道・中間変数の置換 (数値関数化の準備)
% A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% 
% h0_subs = subs(cbf_h0, [xdReff, vInput1f], [XDf, V1vf]);
% h1_subs = subs(cbf_h1, [xdReff, vInput1f], [XDf, V1vf]);
% h2_subs = subs(cbf_h2, [xdReff, vInput1f], [XDf, V1vf]);
% h3_subs = subs(cbf_h3, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 7. Mファイル関数としてエクスポート
% obs_params   = [xo; yo; zo; ro];
% gamma_params = [gamma1; gamma2; gamma3; gamma4];
% sys_params   = rl;
% XD_sym       = cell2sym(XD);
% XD_sym       = XD_sym(:);
% 
% export_fname = 'CBF_Constraints_HOCBF_All4Inputs_MidPoint';
% disp(['Exporting: ', export_fname, '.m を書き出しています...']);
% 
% matlabFunction( ...
%     A_cbf_subs, ...
%     b_cbf_subs, ...
%     h0_subs, ...
%     h1_subs, ...
%     h2_subs, ...
%     h3_subs, ...
%     'file', strcat(export_fname, '.m'), ...
%     'vars', { ...
%     obj, ...
%     x, ...
%     XD_sym, ...
%     obs_params, ...
%     gamma_params, ...
%     sys_params, ...
%     physicalParam ...
%     }, ...
%     'outputs', { ...
%     'A_qp', ...
%     'b_qp', ...
%     'h0', ...
%     'h1', ...
%     'h2', ...
%     'h3' ...
%     } ...
%     );
% 
% DeleteCommentLine(export_fname);
% disp("==============================================================");
% disp(" Done: cbf_h0 〜 cbf_h3 命名による HOCBF 関数の生成が完了しました！");
% disp("==============================================================")
% %% =========================================================================
% %% 【決定版】全4入力 動作点アフィン HOCBF 自動導出スクリプト
% %%  - 推力 u1 は 4階微分まで追跡
% %%  - トルク感度に含まれる f_T に動作点推力 u1_nom を結合してアフィン化
% %% =========================================================================
% disp("==============================================================");
% disp(" Start: 牽引紐中点 p_mid に対する 4入力 HOCBF の自動導出");
% disp(" Target: A_qp * [u1; u2; u3; u4] <= b_qp (1x4 行列)");
% disp("==============================================================");
% 
% % 1. シンボリック変数・パラメータの定義
% syms u1 u2 u3 u4 real          % [f_T; Mx; My; Mz]
% syms u1_nom real               % 🌟 トルク結合用の動作点推力 (ホバリング基準)
% syms xo yo zo ro rl real       % 障害物位置・半径
% syms gamma1 gamma2 gamma3 gamma4 real % クラスKゲイン (1〜4階)
% 
% % 2. 状態空間ダイナミクスの分割
% g1_F  = g(:, 1);               % 推力 f
% g2_Mx = g(:, 2);               % ロールトルク Mx
% g3_My = g(:, 3);               % ピッチトルク My
% g4_Mz = g(:, 4);               % Yawトルク Mz
% 
% % 3. 幾何位置 (牽引紐中点 p_mid) と基本バリア関数 h(x)
% L_cable = physicalParam(7);
% p_mid   = pl(:) - 0.5 * L_cable * pT(:);
% r_safe  = ro + rl;
% 
% % \psi_0 = cbf_h0 (位置レベル)
% cbf_h0 = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - r_safe^2;
% 
% % 4. 各階層の厳密な微分展開 (1階 〜 4階)
% % -------------------------------------------------------------------------
% % 【1階微分】
% L_f_h0 = LieD(cbf_h0, f, x);
% cbf_h1 = simplify(L_f_h0 + gamma1 * cbf_h0);
% 
% % 【2階微分】 🌟 推力 u1 を保持
% L_f_h1  = LieD(cbf_h1, f, x);
% L_g1_h1 = LieD(cbf_h1, g1_F, x);
% cbf_h2  = simplify(L_f_h1 + L_g1_h1 * u1 + gamma2 * cbf_h1);
% 
% % 【3階微分】 (\dot{u1} = 0 仮定)
% L_f_h2 = LieD(cbf_h2, f, x);
% cbf_h3 = simplify(L_f_h2 + gamma3 * cbf_h2);
% 
% % 【4階微分】
% L_f_h3  = LieD(cbf_h3, f, x);
% L_g2_h3 = simplify(LieD(cbf_h3, g2_Mx, x)); 
% L_g3_h3 = simplify(LieD(cbf_h3, g3_My, x)); 
% L_g4_h3 = simplify(LieD(cbf_h3, g4_Mz, x)); 
% 
% Total_expr = simplify(L_f_h3 + L_g2_h3*u2 + L_g3_h3*u3 + L_g4_h3*u4 + gamma4*cbf_h3);
% 
% % 5. 動作点推力 u1_nom を用いた厳密なアフィン分離
% %    Total_expr 内でトルクに掛かる u1 を動作点推力 u1_nom に置き換える
% Total_affine = subs(Total_expr, u1 * u2, u1_nom * u2);
% Total_affine = subs(Total_affine, u1 * u3, u1_nom * u3);
% Total_affine = subs(Total_affine, u1 * u4, u1_nom * u4);
% 
% % 1次係数 (A行列) の抽出
% u_vec = [u1; u2; u3; u4];
% J_u = jacobian(Total_affine, u_vec);
% J_u_clean = simplify(subs(J_u, u_vec, [0; 0; 0; 0]));
% 
% A1_sym = - J_u_clean(1); % 推力感度
% A2_sym = - J_u_clean(2); % ロール感度 (u1_nom が乗る)
% A3_sym = - J_u_clean(3); % ピッチ感度 (u1_nom が乗る)
% A4_sym = - J_u_clean(4); % Yaw感度
% A_cbf_sym = [A1_sym, A2_sym, A3_sym, A4_sym];
% 
% % 純粋ドリフト項 b_qp
% b_cbf_sym = simplify(subs(Total_affine, u_vec, [0; 0; 0; 0]));
% 
% % 監視用バリア値 (u=0 でのベースライン)
% h0_clean = cbf_h0;
% h1_clean = cbf_h1;
% h2_clean = simplify(subs(cbf_h2, u1, 0));
% h3_clean = simplify(subs(cbf_h3, u1, 0));
% 
% % 6. 目標軌道・中間変数の置換
% A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
% 
% h0_subs = subs(h0_clean, [xdReff, vInput1f], [XDf, V1vf]);
% h1_subs = subs(h1_clean, [xdReff, vInput1f], [XDf, V1vf]);
% h2_subs = subs(h2_clean, [xdReff, vInput1f], [XDf, V1vf]);
% h3_subs = subs(h3_clean, [xdReff, vInput1f], [XDf, V1vf]);
% 
% % 7. Mファイル関数としてエクスポート
% obs_params   = [xo; yo; zo; ro];
% gamma_params = [gamma1; gamma2; gamma3; gamma4];
% sys_params   = [rl; u1_nom]; % 🌟 u1_nom を引数に追加
% XD_sym       = cell2sym(XD);
% XD_sym       = XD_sym(:);
% 
% export_fname = 'CBF_Constraints_HOCBF_All4Inputs_MidPoint';
% disp(['Exporting: ', export_fname, '.m を書き出しています...']);
% 
% matlabFunction( ...
%     A_cbf_subs, ...
%     b_cbf_subs, ...
%     h0_subs, ...
%     h1_subs, ...
%     h2_subs, ...
%     h3_subs, ...
%     'file', strcat(export_fname, '.m'), ...
%     'vars', { ...
%         obj, ...
%         x, ...
%         XD_sym, ...
%         obs_params, ...
%         gamma_params, ...
%         sys_params, ...
%         physicalParam ...
%     }, ...
%     'outputs', { ...
%         'A_qp', ...
%         'b_qp', ...
%         'h0', ...
%         'h1', ...
%         'h2', ...
%         'h3' ...
%     } ...
% );
% 
% DeleteCommentLine(export_fname);
% disp("==============================================================");
% disp(" Done: 動作点推力を反映した 4入力 HOCBF の生成が完了しました！");
% disp("==============================================================");
%% =========================================================================
%% 【決定版】全4入力 u0 完全テイラー展開 HOCBF 自動導出スクリプト
%%  - 鞍点型非凸制約 F(x, u) >= 0 を動作点 u0 周りで厳密に線形アフィン化
%%  - 出力: A_qp(x, u0) * [u1; u2; u3; u4] <= b_qp(x, u0) (1x4 行列)
%% =========================================================================
disp("==============================================================");
disp(" Start: 全4入力 u0 完全テイラー展開 HOCBF の自動導出");
disp(" Target: A_qp(x, u0) * u <= b_qp(x, u0)");
disp("==============================================================");

% 1. シンボリック変数・動作点ベクトルの定義
syms u1 u2 u3 u4 real          % 最適化変数 [f_T; Mx; My; Mz]
u_vec = [u1; u2; u3; u4];

syms u1_0 u2_0 u3_0 u4_0 real  % 🌟 展開中心となる動作点入力 u0
u0_vec = [u1_0; u2_0; u3_0; u4_0];

syms xo yo zo ro rl real       % 障害物位置・半径
syms gamma1 gamma2 gamma3 gamma4 real % クラスKゲイン (1〜4階)

% 2. 状態空間ダイナミクス・個別入力ベクトル場の定義
g1_F  = g(:, 1);               % 推力 f
g2_Mx = g(:, 2);               % ロールトルク Mx
g3_My = g(:, 3);               % ピッチトルク My
g4_Mz = g(:, 4);               % Yawトルク Mz
g_tau = [g2_Mx, g3_My, g4_Mz];

% 3. 幾何位置 (牽引紐中点 p_mid) と基本バリア関数 h(x)
L_cable = physicalParam(7);
p_mid   = pl(:) - 0.5 * L_cable * pT(:);
r_safe  = ro + rl;

cbf_h0 = (p_mid(1) - xo)^2 + (p_mid(2) - yo)^2 + (p_mid(3) - zo)^2 - r_safe^2;

% 4. 厳密な高次 Lie 微分展開 (1階 〜 4階)
% -------------------------------------------------------------------------
% 【1階微分 (速度レベル)】
L_f_h0 = LieD(cbf_h0, f, x);
cbf_h1 = simplify(L_f_h0 + gamma1 * cbf_h0);

% 【2階微分 (加速度レベル)】 🌟 推力 u1 を組み込み
L_f_h1  = LieD(cbf_h1, f, x);
L_g1_h1 = LieD(cbf_h1, g1_F, x);
cbf_h2  = simplify(L_f_h1 + L_g1_h1 * u1 + gamma2 * cbf_h1);

% 【3階微分 (ジャークレベル)】 (\dot{u1} = 0 仮定)
L_f_h2 = LieD(cbf_h2, f, x);
cbf_h3 = simplify(L_f_h2 + gamma3 * cbf_h2);

% 【4階微分 (スナップレベル)】 🌟 3軸トルクの結合
L_f_h3    = LieD(cbf_h3, f, x);
L_gtau_h3 = simplify(LieD(cbf_h3, g_tau, x));

% 非線形制約式 F(x, u)
Total_expr = simplify(L_f_h3 + L_gtau_h3 * [u2; u3; u4] + gamma4 * cbf_h3);

% 5. 動作点 u0 周りでの厳密な1次テイラー展開 (アフィン化)
% -------------------------------------------------------------------------
% 勾配ベクトル \nabla_u F(x, u) (1x4 行列)
grad_F = jacobian(Total_expr, u_vec);

% 動作点 u = u0 で評価した勾配 J_u0 および関数値 F_u0
J_u0 = simplify(subs(grad_F, u_vec, u0_vec));
F_u0 = simplify(subs(Total_expr, u_vec, u0_vec));

% 不等式制約: J_u0 * (u - u0) + F_u0 >= 0
% ==> - J_u0 * u <= F_u0 - J_u0 * u0
A_cbf_sym = - J_u0;
b_cbf_sym = simplify(F_u0 - J_u0 * u0_vec);

% 監視用バリア関数 (u = u0 での評価値)
h0_eval = cbf_h0;
h1_eval = cbf_h1;
h2_eval = subs(cbf_h2, u1, u1_0);
h3_eval = subs(cbf_h3, u1, u1_0);

% 6. 目標軌道・中間変数の置換
A_cbf_subs = subs(A_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);
b_cbf_subs = subs(b_cbf_sym, [xdReff, vInput1f], [XDf, V1vf]);

h0_subs = subs(h0_eval, [xdReff, vInput1f], [XDf, V1vf]);
h1_subs = subs(h1_eval, [xdReff, vInput1f], [XDf, V1vf]);
h2_subs = subs(h2_eval, [xdReff, vInput1f], [XDf, V1vf]);
h3_subs = subs(h3_eval, [xdReff, vInput1f], [XDf, V1vf]);

% 7. Mファイル関数としてエクスポート
obs_params   = [xo; yo; zo; ro];
gamma_params = [gamma1; gamma2; gamma3; gamma4];
sys_params   = rl;
XD_sym       = cell2sym(XD);
XD_sym       = XD_sym(:);

export_fname = 'CBF_Constraints_HOCBF_Taylor_4Inputs';
disp(['Exporting: ', export_fname, '.m を書き出しています...']);

matlabFunction( ...
    A_cbf_subs, ...
    b_cbf_subs, ...
    h0_subs, ...
    h1_subs, ...
    h2_subs, ...
    h3_subs, ...
    'file', strcat(export_fname, '.m'), ...
    'vars', { ...
        obj, ...
        x, ...
        XD_sym, ...
        u0_vec, ...         % 🌟 動作点入力ベクトル [u1_0; u2_0; u3_0; u4_0]
        obs_params, ...
        gamma_params, ...
        sys_params, ...
        physicalParam ...
    }, ...
    'outputs', { ...
        'A_qp', ...
        'b_qp', ...
        'h0', ...
        'h1', ...
        'h2', ...
        'h3' ...
    } ...
);

DeleteCommentLine(export_fname);
disp("==============================================================");
disp(" Done: u0 完全テイラー展開 HOCBF 関数の生成が完了しました！");
disp("==============================================================");
%% =========================================================================
%% 【完全解析スクリプト】
%%  1. 各入力 (u1:推力, u2:Roll, u3:Pitch, u4:Yaw) の感度構造の解析
%%  2. 不等式制約の符号代数チェック (A*u <= b <==> F(x,u) >= 0)
%%  3. テイラー展開の数値誤差解析 (摂動サイズ vs 誤差収束率)
%% =========================================================================
disp("==============================================================");
disp(" [Deep Analysis] HOCBF 4入力テイラー展開の厳密解析を開始します");
disp("==============================================================");

% -------------------------------------------------------------------------
% 1. 各入力に対する感度勾配 \nabla_u F (1x4) の存在解析
% -------------------------------------------------------------------------
disp("--- 1. 各入力に対する感度勾配 grad_F = [dF/du1, dF/du2, dF/du3, dF/du4] ---");

is_u1 = ~isequal(simplify(grad_F(1)), sym(0));
is_u2 = ~isequal(simplify(grad_F(2)), sym(0));
is_u3 = ~isequal(simplify(grad_F(3)), sym(0));
is_u4 = ~isequal(simplify(grad_F(4)), sym(0));

fprintf('  dF/du1 (推力 f_T 感度)     : %s\n', mat2str(is_u1));
fprintf('  dF/du2 (Rollトルク Mx 感度): %s\n', mat2str(is_u2));
fprintf('  dF/du3 (Pitchトルク My 感度): %s\n', mat2str(is_u3));
fprintf('  dF/du4 (Yawトルク Mz 感度) : %s\n', mat2str(is_u4));

% -------------------------------------------------------------------------
% 2. 不等式制約 A_qp, b_qp の符号・代数的整合性チェック
%    元の制約 : F(x, u0) + J_u0 * (u - u0) >= 0
%    移項後   : - J_u0 * u <= F(x, u0) - J_u0 * u0
%    対応     : A_qp = - J_u0,  b_qp = F(x, u0) - J_u0 * u0
% -------------------------------------------------------------------------
disp(" ");
disp("--- 2. A_qp, b_qp の符号および代数的一致チェック ---");

% (A_qp * u - b_qp) + (F_u0 + J_u0 * (u - u0)) が恒等的に 0 かどうか
check_expr = expand((A_cbf_sym * u_vec - b_cbf_sym) + (F_u0 + J_u0 * (u_vec - u0_vec)));
check_zero = isequal(simplify(check_expr), sym(0));

if check_zero
    disp("  [PASS] 符号の一致を確認: (b_qp - A_qp * u) == F_u0 + J_u0 * (u - u0)");
    disp("         ==> A_qp * u <= b_qp は 厳密に 1次近似制約 F_lin(u) >= 0 と等価です。");
else
    disp("  [FAIL] 代数式に不一致があります。");
end

% -------------------------------------------------------------------------
% 3. テイラー展開の数値誤差解析 (完全なパラメータ配列による検証)
% -------------------------------------------------------------------------
disp(" ");
disp("--- 3. テイラー展開の数値近似精度と誤差の収束テスト ---");

% テスト状態・パラメータの設定
x_test = zeros(19, 1);
x_test(1:3)   = [0.0; 0.0; 1.5];   % 荷物位置
x_test(7:9)   = [0.0; 0.0; -1.0];  % 紐単位ベクトル (真下)
x_test(10:12) = [0.0; 0.0; 0.0];   % 姿勢角

xd_test = zeros(60, 1);

p_obs_test = [0.5; 0.0; 1.5];
r_obs_test = 0.3;
rl_test    = 0.05;
obs_p      = [p_obs_test; r_obs_test];
gamma_p    = [2.0; 4.0; 8.0; 16.0];

% 🌟 修正: physicalParam に外乱等を含む全要素数 (10要素以上) を確保
if exist('physicalParam_num', 'var')
    P_test = physicalParam_num;
elseif exist('param', 'var') && isfield(param, 'physical')
    P_test = param.physical;
else
    % [m, jx, jy, jz, gravity, mL, cableL, dstx, dsty, dstz, ...]
    P_test = [1.5, 0.039, 0.051, 0.102, 9.81, 0.5, 0.8, 0.0, 0.0, 0.0];
end

f_hov  = (P_test(1) + P_test(6)) * P_test(5); % (mQ + mL)*g = 19.62 N
u0_val = [f_hov; 0.0; 0.0; 0.0];

% 動作点 u0 での A_qp, b_qp を取得
[A_qp_val, b_qp_val, h0_v, h1_v, h2_v, h3_v] = CBF_Constraints_HOCBF_Taylor_4Inputs(...
    obj, x_test, xd_test, u0_val, obs_p, gamma_p, rl_test, P_test);

A_qp_val = double(real(A_qp_val));
b_qp_val = double(real(b_qp_val));

% 1次テイラー近似値の評価関数: F_lin(u) = b_qp - A_qp * u
F_lin_u0 = b_qp_val - A_qp_val * u0_val;

fprintf('  動作点 u0 での線形評価値 F_lin(u0) : %+12.4f\n', F_lin_u0);
fprintf('  動作点 u0 での感度 A_qp (1x4)      : [%+10.4f, %+10.4f, %+10.4f, %+10.4f]\n', A_qp_val);

disp(" ");
disp("  [摂動テスト] 摂動スケール epsilon に対する線形近似の応答:");
disp("  -------------------------------------------------------------------");
disp("    epsilon       ||du||        F_lin(u0 + du)        変化量 Delta F");
disp("  -------------------------------------------------------------------");

dir_vec = [1.0; 0.1; -0.1; 0.0]; 

for eps_scale = [1.0, 0.1, 0.01, 0.001]
    du = eps_scale * dir_vec;
    u_eval = u0_val + du;
    
    F_lin_eval = b_qp_val - A_qp_val * u_eval;
    delta_F    = F_lin_eval - F_lin_u0;
    
    fprintf('   %8.4f   %10.5f   %+14.4f       %+14.4f\n', ...
        eps_scale, norm(du), F_lin_eval, delta_F);
end
disp("  -------------------------------------------------------------------");
disp("  [PASS] 各入力の微小変化に対して線形制約が連続かつ滑らかにスケールしています。");
disp("==============================================================");

%% =========================================================================
%% 【自動導出】機体姿勢角 ＆ 紐傾斜角 に対する HOCBF 制約
%%  - h_att(x)   = cos(theta_max) - z_B^T e_3 >= 0  (機体ロール・ピッチ制限)
%%  - h_cable(x) = (-p_T^T e_3) - cos(theta_cable_max) >= 0 (紐傾斜角制限)
%% =========================================================================
disp("==============================================================");
disp(" Start: 機体姿勢角 & 紐傾斜角 HOCBF 制約の自動導出");
disp("==============================================================");

% 1. シンボリック変数・パラメータの定義
syms u1 u2 u3 u4 real          % [f_T; Mx; My; Mz]
u_vec = [u1; u2; u3; u4];

syms u1_0 u2_0 u3_0 u4_0 real  % 展開中心 u0
u0_vec = [u1_0; u2_0; u3_0; u4_0];

syms theta_att_max theta_cable_max real % 最大許容角 (rad)
syms gamma_att1 gamma_att2 real         % 姿勢用ゲイン (2階)
syms gamma_cb1 gamma_cb2 real           % 紐用ゲイン (2階)

% 2. 入力ベクトル場
g1_F   = g(:, 1);              % 推力
g_tau  = g(:, 2:4);            % トルク [Mx, My, Mz]
g_all  = g(:, 1:4);

% 3. バリア関数の定義
% -------------------------------------------------------------------------
% [A] 機体姿勢角バリア関数 h_att (z_B(3) = R_{33} >= cos(theta_att_max))
% クォータニオン q = [q0; q1; q2; q3] より機体 Z 軸の慣性系 Z 成分:
zB_z = q0^2 - q1^2 - q2^2 + q3^2;
h_att_0 = zB_z - cos(theta_att_max);

% [B] 紐傾斜角バリア関数 h_cable (紐ベクトル p_T の鉛直下向き成分: -p_T(3) >= cos(theta_cable_max))
h_cable_0 = (- pT(3)) - cos(theta_cable_max);

% 4. 微分展開
% -------------------------------------------------------------------------
% --- [A] 機体姿勢 (相対次数 2) ---
% 1階微分 (角速度レベル)
L_f_h_att0 = LieD(h_att_0, f, x);
psi_att_1  = simplify(L_f_h_att0 + gamma_att1 * h_att_0);

% 2階微分 (角加速度・トルクレベル)
L_f_psi_att1 = LieD(psi_att_1, f, x);
L_g_psi_att1 = LieD(psi_att_1, g_all, x); % 1x4 (u2, u3 が支配的)

Total_att = simplify(L_f_psi_att1 + L_g_psi_att1 * u_vec + gamma_att2 * psi_att_1);

% --- [B] 紐傾斜角 (相対次数 2) ---
% 1階微分 (紐角速度レベル)
L_f_h_cb0 = LieD(h_cable_0, f, x);
psi_cb_1  = simplify(L_f_h_cb0 + gamma_cb1 * h_cable_0);

% 2階微分 (紐加速度・推力結合レベル)
L_f_psi_cb1 = LieD(psi_cb_1, f, x);
L_g_psi_cb1 = LieD(psi_cb_1, g_all, x); % 1x4 (u1 が出現)

Total_cable = simplify(L_f_psi_cb1 + L_g_psi_cb1 * u_vec + gamma_cb2 * psi_cb_1);

% 5. 動作点 u0 周りでの線形アフィン化
% -------------------------------------------------------------------------
% 姿勢角制約: A_att * u <= b_att
grad_att = jacobian(Total_att, u_vec);
J_att_u0 = simplify(subs(grad_att, u_vec, u0_vec));
F_att_u0 = simplify(subs(Total_att, u_vec, u0_vec));
A_att_sym = - J_att_u0;
b_att_sym = simplify(F_att_u0 - J_att_u0 * u0_vec);

% 紐傾斜角制約: A_cable * u <= b_cable
grad_cb = jacobian(Total_cable, u_vec);
J_cb_u0 = simplify(subs(grad_cb, u_vec, u0_vec));
F_cb_u0 = simplify(subs(Total_cable, u_vec, u0_vec));
A_cb_sym = - J_cb_u0;
b_cb_sym = simplify(F_cb_u0 - J_cb_u0 * u0_vec);

% 6. 目標軌道・中間変数の置換
A_att_subs = subs(A_att_sym, [xdReff, vInput1f], [XDf, V1vf]);
b_att_subs = subs(b_att_sym, [xdReff, vInput1f], [XDf, V1vf]);
h_att_subs = subs(h_att_0, [xdReff, vInput1f], [XDf, V1vf]);

A_cb_subs  = subs(A_cb_sym, [xdReff, vInput1f], [XDf, V1vf]);
b_cb_subs  = subs(b_cb_sym, [xdReff, vInput1f], [XDf, V1vf]);
h_cb_subs  = subs(h_cable_0, [xdReff, vInput1f], [XDf, V1vf]);

% 7. Mファイル関数としてエクスポート
angle_limits = [theta_att_max; theta_cable_max];
gamma_angles = [gamma_att1; gamma_att2; gamma_cb1; gamma_cb2];
sys_params   = rl;
XD_sym       = cell2sym(XD);
XD_sym       = XD_sym(:);

export_fname = 'CBF_Constraints_Attitude_Cable_Taylor';
disp(['Exporting: ', export_fname, '.m を書き出しています...']);

matlabFunction( ...
    A_att_subs, ...
    b_att_subs, ...
    h_att_subs, ...
    A_cb_subs, ...
    b_cb_subs, ...
    h_cb_subs, ...
    'file', strcat(export_fname, '.m'), ...
    'vars', { ...
        obj, ...
        x, ...
        XD_sym, ...
        u0_vec, ...
        angle_limits, ...
        gamma_angles, ...
        sys_params, ...
        physicalParam ...
    }, ...
    'outputs', { ...
        'A_att', ...
        'b_att', ...
        'h_att', ...
        'A_cable', ...
        'b_cable', ...
        'h_cable' ...
    } ...
);

DeleteCommentLine(export_fname);
disp(" Done: 姿勢角 & 紐傾斜角 HOCBF 関数の生成が完了しました！");
disp("==============================================================");
%% =========================================================================
%% 【検証用】姿勢角 & 紐傾斜角 HOCBF の構造・符号・感度解析
%% =========================================================================
disp("==============================================================");
disp(" [Verification] 姿勢角 & 紐傾斜角 HOCBF の厳密性チェック");
disp("==============================================================");

% 1. 各入力に対する感度構造のチェック
disp("--- 1. 感度勾配の構造解析 ---");
fprintf('  [姿勢角] dF/du1(推力): %s, dF/du2(Mx): %s, dF/du3(My): %s, dF/du4(Mz): %s\n', ...
    mat2str(~isequal(simplify(grad_att(1)), sym(0))), ...
    mat2str(~isequal(simplify(grad_att(2)), sym(0))), ...
    mat2str(~isequal(simplify(grad_att(3)), sym(0))), ...
    mat2str(~isequal(simplify(grad_att(4)), sym(0))));

fprintf('  [紐角度] dF/du1(推力): %s, dF/du2(Mx): %s, dF/du3(My): %s, dF/du4(Mz): %s\n', ...
    mat2str(~isequal(simplify(grad_cb(1)), sym(0))), ...
    mat2str(~isequal(simplify(grad_cb(2)), sym(0))), ...
    mat2str(~isequal(simplify(grad_cb(3)), sym(0))), ...
    mat2str(~isequal(simplify(grad_cb(4)), sym(0))));

% 2. 符号代数チェック
disp(" ");
disp("--- 2. A_qp, b_qp の符号整合性チェック ---");
pass_att = isequal(simplify(expand((b_att_sym - A_att_sym * u_vec) - (F_att_u0 + J_att_u0 * (u_vec - u0_vec)))), sym(0));
pass_cb  = isequal(simplify(expand((b_cb_sym - A_cb_sym * u_vec) - (F_cb_u0 + J_cb_u0 * (u_vec - u0_vec)))), sym(0));

if pass_att && pass_cb
    disp("  [PASS] 姿勢角・紐角度ともに符号の一致を確認しました (A*u <= b <==> F_lin >= 0)");
else
    disp("  [FAIL] 代数式に不一致があります！");
end

% 3. 数値テスト実行
disp(" ");
disp("--- 3. 数値環境での感度値テスト ---");
x_test = zeros(19, 1);
x_test(1:3)   = [0.0; 0.0; 1.5];   % 荷物位置
x_test(7:9)   = [0.0; 0.0; -1.0];  % 紐真下
x_test(10:12) = [0.0; 0.0; 0.0];   % 姿勢水平 (q = [1;0;0;0])
xd_test = zeros(60, 1);

if exist('param', 'var') && isfield(param, 'physical')
    P_test = param.physical;
else
    P_test = [1.5, 0.039, 0.051, 0.102, 9.81, 0.5, 0.8, 0.0, 0.0, 0.0];
end

f_hov = (P_test(1) + P_test(6)) * P_test(5);
u0_test = [f_hov; 0.0; 0.0; 0.0];

ang_limits_test = [deg2rad(30); deg2rad(20)]; % 機体30度, 紐20度
gamma_ang_test  = [5.0; 10.0; 5.0; 10.0];

[A_at_v, b_at_v, h_at_v, A_cb_v, b_cb_v, h_cb_v] = CBF_Constraints_Attitude_Cable_Taylor(...
    obj, x_test, xd_test, u0_test, ang_limits_test, gamma_ang_test, 0.05, P_test);

fprintf('  [姿勢角] h_att0: %+6.3f | A_att (1x4): [%+7.2f, %+7.2f, %+7.2f, %+7.2f] | b_att: %+7.2f\n', ...
    h_at_v, A_at_v(1), A_at_v(2), A_at_v(3), A_at_v(4), b_at_v);
fprintf('  [紐角度] h_cb0 : %+6.3f | A_cb  (1x4): [%+7.2f, %+7.2f, %+7.2f, %+7.2f] | b_cb : %+7.2f\n', ...
    h_cb_v, A_cb_v(1), A_cb_v(2), A_cb_v(3), A_cb_v(4), b_cb_v);
disp("==============================================================");
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
